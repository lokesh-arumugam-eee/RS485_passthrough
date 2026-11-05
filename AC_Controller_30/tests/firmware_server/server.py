"""Chunked-OTA HTTP server for ESP32.

Endpoints:
  GET  /info?file=<name>                         -> {filename,size,sha256,min/max_packet_size}
  GET  /negotiate?file=<name>&packet_size=<N>    -> {packet_size,total_packets,size,sha256}
  GET  /firmware?file=<name>  + Range: bytes=a-b -> 206, body=bytes, header X-CRC16: <hhhh>
  POST /ota/report  body={mac,status,failure_reason,percentage,filename,...}
                                                 -> 200 {ok:true}; report appended to ota_reports.jsonl
  GET  /ota/reports                              -> JSON array of recent reports (newest last)

CRC-16/Modbus (poly 0xA001, init 0xFFFF) is computed over the *body actually sent*.
SHA-256 is computed once per file and cached on (path, size, mtime).

Run:
    python server.py --host 0.0.0.0 --port 8000 --dir ./firmware
"""

from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import json
import logging
import os
import re
import threading
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

MIN_PACKET_SIZE = 256
MAX_PACKET_SIZE = 8192
DEFAULT_PACKET_SIZE = 2048

RANGE_RE = re.compile(r"^bytes=(\d+)-(\d*)$")

_sha_cache: dict[str, tuple[float, int, str]] = {}

# Backwards-compat: maps the integer ota_t enum to its text name so older
# firmware that still sent integers continues to render correctly.
OTA_STATUS = {
    0: "default",
    1: "success",
    2: "downloading",   # OTA_STARTED
    3: "downloading",   # OTA_ONGOING
    4: "retrying",
    5: "failure",
    6: "flashing",
    7: "rebooting",
    8: "idle",
}

OTA_FAILURE = {
    0:  "none",
    1:  "invalid_argument",
    2:  "bad_json_info",
    3:  "bad_json_negotiate",
    4:  "mem_alloc_failed",
    5:  "ota_begin_failed",
    6:  "http_client_init_failed",
    7:  "download_packet_failed",
    8:  "write_packet_failed",
    9:  "sha_mismatch",
    10: "ota_end_failed",
    11: "set_boot_failed",
    12: "open_client_failed",
    13: "content_length_mismatch",
    14: "no_crc_provided",
    15: "crc_mismatch",
    16: "http_error",
}


def _to_status_text(raw) -> str:
    """Accept either a string (new firmware) or an int (legacy firmware)."""
    if isinstance(raw, str):
        return raw
    try:
        return OTA_STATUS.get(int(raw), f"unknown({raw})")
    except (TypeError, ValueError):
        return "unknown"


def _to_failure_text(raw) -> str:
    if isinstance(raw, str):
        return raw
    try:
        return OTA_FAILURE.get(int(raw), f"unknown({raw})")
    except (TypeError, ValueError):
        return "unknown"

REPORTS_FILE = "ota_reports.jsonl"
REPORTS_MAX_IN_MEM = 200
_reports_lock = threading.Lock()
_reports_recent: deque[dict] = deque(maxlen=REPORTS_MAX_IN_MEM)


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def file_sha256(path: str) -> tuple[int, str]:
    st = os.stat(path)
    cached = _sha_cache.get(path)
    if cached and cached[0] == st.st_mtime and cached[1] == st.st_size:
        return st.st_size, cached[2]
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    digest = h.hexdigest()
    _sha_cache[path] = (st.st_mtime, st.st_size, digest)
    return st.st_size, digest


def clamp_packet_size(n: int) -> int:
    if n < MIN_PACKET_SIZE:
        return MIN_PACKET_SIZE
    if n > MAX_PACKET_SIZE:
        return MAX_PACKET_SIZE
    return n


def _print_notification(r: dict) -> None:
    """One descriptive console line per received notification, keyed by status."""
    ts     = r.get("received_at", "?")
    mac    = r.get("mac", "?")
    fw     = r.get("firmware_version", "?")
    prev   = r.get("previous_firmware_version")
    fname  = r.get("filename", "?")
    pct    = r.get("percentage", "?")
    status = r.get("status_name", "?")
    reason = r.get("failure_name", "?")
    peer   = r.get("peer", "?")

    # Per-status tailored message. Match on the lowercase text the C side now
    # sends; the integer-decoded fallback in OTA_STATUS uses the same names.
    s = status.lower() if isinstance(status, str) else ""
    if s == "idle":
        msg = f"heartbeat - device alive, about to poll for OTA (fw={fw})"
    elif s == "downloading":
        msg = f"downloading {pct}% (fw={fw})"
    elif s == "retrying":
        msg = f"retrying whole-file from packet 0 (fw={fw})"
    elif s == "flashing":
        msg = f"SHA verified, flashing partition (fw={fw})"
    elif s == "rebooting":
        msg = f"about to reboot into new image (fw={fw})"
    elif s == "success":
        if prev:
            msg = f"SUCCESS, upgraded {prev} -> {fw} (post-reboot confirmation)"
        else:
            msg = f"SUCCESS, running fw={fw}"
    elif s == "failure":
        msg = f"FAILED at {pct}% reason={reason} (fw={fw})"
    else:
        msg = f"unrecognised status (raw={r.get('status')}, fw={fw})"

    line = (f"[{ts}] {status:<14} mac={mac} file={fname} peer={peer}\n"
            f"               -> {msg}")
    # Use print(flush=True) so the line appears immediately even when
    # stdout is piped (PowerShell Tee-Object, file redirection, etc.).
    print(line, flush=True)
    # Also drop a one-liner into the logging stream so existing log
    # aggregators / files still see it in their usual format.
    logging.info("OTA %s mac=%s file=%s pct=%s reason=%s fw=%s%s",
                 status, mac, fname, pct, reason, fw,
                 f" (prev={prev})" if prev else "")


def record_report(reports_dir: str, report: dict, peer: str) -> dict:
    """Decorate the raw report with server-side metadata, log it,
    append to JSONL, and stash in the in-memory ring buffer."""
    enriched = {
        "received_at": _dt.datetime.now().isoformat(timespec="seconds"),
        "peer": peer,
        "status_name":  _to_status_text(report.get("status")),
        "failure_name": _to_failure_text(report.get("failure_reason")),
        **report,
    }
    with _reports_lock:
        _reports_recent.append(enriched)
        try:
            with open(os.path.join(reports_dir, REPORTS_FILE), "a", encoding="utf-8") as fp:
                fp.write(json.dumps(enriched) + "\n")
        except OSError as e:
            logging.warning("could not append to %s: %s", REPORTS_FILE, e)

    _print_notification(enriched)
    return enriched


class FirmwareHandler(BaseHTTPRequestHandler):
    server_version = "FirmwareOTA/1.0"
    firmware_dir: str = "."

    def log_message(self, fmt: str, *args) -> None:
        logging.info("%s - %s", self.address_string(), fmt % args)

    def _resolve(self, filename: str) -> str | None:
        if not filename or "/" in filename or "\\" in filename or filename.startswith("."):
            return None
        path = os.path.join(self.firmware_dir, filename)
        return path if os.path.isfile(path) else None

    def _send_json(self, status: int, body: dict) -> None:
        payload = json.dumps(body).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        self.wfile.write(payload)

    def _send_error_json(self, status: int, message: str) -> None:
        self._send_json(status, {"error": message})

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        qs = parse_qs(parsed.query)
        filename = (qs.get("file") or [""])[0]

        if parsed.path == "/info":
            self._handle_info(filename)
        elif parsed.path == "/negotiate":
            self._handle_negotiate(filename, qs)
        elif parsed.path == "/firmware":
            self._handle_firmware(filename)
        elif parsed.path == "/ota/reports":
            self._handle_list_reports(qs)
        else:
            self._send_error_json(HTTPStatus.NOT_FOUND, "unknown endpoint")

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/ota/report":
            self._handle_post_report()
        elif parsed.path.endswith(".log"):
            # Tolerated for backward compatibility; PUT is the preferred verb.
            self._handle_post_log(parsed.path)
        else:
            self._send_error_json(HTTPStatus.NOT_FOUND, "unknown endpoint")

    def do_PUT(self) -> None:
        parsed = urlparse(self.path)
        # The URL prefix is whatever the device's httpserver_params.filepath
        # was configured to. We don't interpret it -- just take the basename
        # and save under <firmware_dir>/logs/.
        if parsed.path.endswith(".log"):
            self._handle_post_log(parsed.path)
        else:
            self._send_error_json(HTTPStatus.NOT_FOUND, "unknown endpoint")

    def do_HEAD(self) -> None:
        parsed = urlparse(self.path)
        qs = parse_qs(parsed.query)
        filename = (qs.get("file") or [""])[0]
        if parsed.path == "/firmware":
            path = self._resolve(filename)
            if not path:
                self.send_response(HTTPStatus.NOT_FOUND)
                self.end_headers()
                return
            size, _ = file_sha256(path)
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Length", str(size))
            self.send_header("Accept-Ranges", "bytes")
            self.end_headers()
        else:
            self.send_response(HTTPStatus.METHOD_NOT_ALLOWED)
            self.end_headers()

    def _handle_info(self, filename: str) -> None:
        path = self._resolve(filename)
        if not path:
            self._send_error_json(HTTPStatus.NOT_FOUND, "file not found")
            return
        size, sha = file_sha256(path)
        self._send_json(HTTPStatus.OK, {
            "filename": filename,
            "size": size,
            "sha256": sha,
            "min_packet_size": MIN_PACKET_SIZE,
            "max_packet_size": MAX_PACKET_SIZE,
            "default_packet_size": DEFAULT_PACKET_SIZE,
        })

    def _handle_negotiate(self, filename: str, qs: dict) -> None:
        path = self._resolve(filename)
        if not path:
            self._send_error_json(HTTPStatus.NOT_FOUND, "file not found")
            return
        try:
            requested = int((qs.get("packet_size") or [str(DEFAULT_PACKET_SIZE)])[0])
        except ValueError:
            self._send_error_json(HTTPStatus.BAD_REQUEST, "packet_size must be integer")
            return
        accepted = clamp_packet_size(requested)
        size, sha = file_sha256(path)
        total_packets = (size + accepted - 1) // accepted
        self._send_json(HTTPStatus.OK, {
            "packet_size": accepted,
            "total_packets": total_packets,
            "size": size,
            "sha256": sha,
        })

    def _handle_post_report(self) -> None:
        length_hdr = self.headers.get("Content-Length")
        if not length_hdr or not length_hdr.isdigit():
            self._send_error_json(HTTPStatus.LENGTH_REQUIRED, "Content-Length required")
            return
        length = int(length_hdr)
        if length <= 0 or length > 8192:
            self._send_error_json(HTTPStatus.BAD_REQUEST, "invalid body length")
            return
        raw = self.rfile.read(length)
        try:
            report = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as e:
            self._send_error_json(HTTPStatus.BAD_REQUEST, f"malformed JSON: {e}")
            return
        if not isinstance(report, dict):
            self._send_error_json(HTTPStatus.BAD_REQUEST, "body must be a JSON object")
            return

        enriched = record_report(self.firmware_dir, report, self.address_string())
        self._send_json(HTTPStatus.OK, {
            "ok": True,
            "status_name": enriched["status_name"],
            "failure_name": enriched["failure_name"],
            "received_at": enriched["received_at"],
        })

    def _handle_post_log(self, url_path: str) -> None:
        # Strict filename whitelist to keep the upload sandboxed inside logs/.
        raw_name = os.path.basename(url_path)
        if not re.match(r"^[A-Za-z0-9._-]+\.log$", raw_name):
            self._send_error_json(HTTPStatus.BAD_REQUEST, "invalid log filename")
            return

        length_hdr = self.headers.get("Content-Length")
        if not length_hdr or not length_hdr.isdigit():
            self._send_error_json(HTTPStatus.LENGTH_REQUIRED, "Content-Length required")
            return
        length = int(length_hdr)
        if length <= 0 or length > 1024 * 1024:        # 1 MB ceiling
            self._send_error_json(HTTPStatus.BAD_REQUEST, "invalid body length")
            return

        body = self.rfile.read(length)
        logs_dir = os.path.join(self.firmware_dir, "logs")
        try:
            os.makedirs(logs_dir, exist_ok=True)
        except OSError as e:
            self._send_error_json(HTTPStatus.INTERNAL_SERVER_ERROR, f"mkdir: {e}")
            return
        out_path = os.path.join(logs_dir, raw_name)
        try:
            with open(out_path, "wb") as fp:
                fp.write(body)
        except OSError as e:
            self._send_error_json(HTTPStatus.INTERNAL_SERVER_ERROR, f"write: {e}")
            return

        ts = _dt.datetime.now().isoformat(timespec="seconds")
        print(f"[{ts}] LOG_UPLOAD     mac={raw_name.replace('.log','')} "
              f"size={length}B peer={self.address_string()}\n"
              f"               -> saved to {out_path}", flush=True)
        logging.info("log uploaded: %s bytes=%d peer=%s -> %s",
                     raw_name, length, self.address_string(), out_path)
        self._send_json(HTTPStatus.OK, {
            "ok": True, "filename": raw_name, "bytes": length, "received_at": ts,
        })

    def _handle_list_reports(self, qs: dict) -> None:
        try:
            limit = int((qs.get("limit") or ["50"])[0])
        except ValueError:
            limit = 50
        limit = max(1, min(limit, REPORTS_MAX_IN_MEM))
        mac_filter = (qs.get("mac") or [""])[0].upper()
        with _reports_lock:
            items = list(_reports_recent)
        if mac_filter:
            items = [r for r in items if str(r.get("mac", "")).upper() == mac_filter]
        items = items[-limit:]
        self._send_json(HTTPStatus.OK, {"count": len(items), "reports": items})

    def _handle_firmware(self, filename: str) -> None:
        path = self._resolve(filename)
        if not path:
            self._send_error_json(HTTPStatus.NOT_FOUND, "file not found")
            return

        size = os.path.getsize(path)
        rng = self.headers.get("Range")
        if not rng:
            self._send_error_json(HTTPStatus.BAD_REQUEST, "Range header required")
            return
        m = RANGE_RE.match(rng.strip())
        if not m:
            self._send_error_json(HTTPStatus.BAD_REQUEST, "invalid Range syntax")
            return
        start = int(m.group(1))
        end = int(m.group(2)) if m.group(2) else size - 1
        if start >= size or end >= size or start > end:
            self.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
            self.send_header("Content-Range", f"bytes */{size}")
            self.end_headers()
            return

        length = end - start + 1
        with open(path, "rb") as f:
            f.seek(start)
            body = f.read(length)
        if len(body) != length:
            self._send_error_json(HTTPStatus.INTERNAL_SERVER_ERROR, "short read")
            return

        crc = crc16_modbus(body)
        self.send_response(HTTPStatus.PARTIAL_CONTENT)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Length", str(length))
        self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("X-CRC16", f"{crc:04x}")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        self.wfile.write(body)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--dir", default="./firmware",
                        help="directory holding firmware files served by name")
    args = parser.parse_args()

    args.dir = os.path.abspath(args.dir)
    os.makedirs(args.dir, exist_ok=True)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    logging.info("serving firmware from %s on %s:%d", args.dir, args.host, args.port)

    FirmwareHandler.firmware_dir = args.dir

    reports_path = os.path.join(args.dir, REPORTS_FILE)
    if os.path.isfile(reports_path):
        try:
            with open(reports_path, "r", encoding="utf-8") as fp:
                for line in fp:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        _reports_recent.append(json.loads(line))
                    except json.JSONDecodeError:
                        continue
            logging.info("loaded %d historical reports from %s",
                         len(_reports_recent), reports_path)
        except OSError as e:
            logging.warning("could not preload %s: %s", reports_path, e)

    httpd = ThreadingHTTPServer((args.host, args.port), FirmwareHandler)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        logging.info("shutting down")
    finally:
        httpd.server_close()


if __name__ == "__main__":
    main()
