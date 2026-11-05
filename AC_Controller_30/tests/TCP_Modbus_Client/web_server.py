"""
Web UI for the AC_Controller_30 ESP32's Modbus config server. Wraps
modbus_core.py's polling/write engine in a small local Flask server so live
register values can be watched in a browser instead of a console/log file.

Matches the firmware's register map in main/modbus/modbus_rtu.h: the config
server listens on port 502 (RTU-over-TCP framing) and only answers slave
ID 249. The device address is entirely config-file driven -- see
deviceconfig.json in this folder ("pull_ip"/"pull_port"/slave-id fields).
Nothing here is hardcoded; edit that file to point at a different unit.

Run (Linux):
    python3 -m venv .venv
    .venv/bin/pip install flask "pymodbus>=3.7"
    .venv/bin/python web_server.py
Run (Windows):
    .venv\\Scripts\\python.exe web_server.py
Then open http://127.0.0.1:5000/
"""

from flask import Flask, jsonify, request, Response

import modbus_core

app = Flask(__name__)
profile = modbus_core.load_ir_blaster_profile()

INDEX_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>AC Controller Modbus Client</title>
<style>
  :root {
    --bg: #f5f3ee; --surface: #fff; --border: #ddd7c9; --text: #1e1b16;
    --muted: #6b6459; --accent: #c2410c; --ok: #3f7d4e; --err: #a6291c;
  }
  @media (prefers-color-scheme: dark) {
    :root { --bg:#16130f; --surface:#1e1a15; --border:#3a332a; --text:#f0ebe3;
            --muted:#a89e90; --accent:#f0713a; --ok:#7ecb8d; --err:#e58074; }
  }
  * { box-sizing: border-box; }
  body { margin:0; background:var(--bg); color:var(--text); font-family: ui-sans-serif, system-ui, sans-serif; }
  header { padding: 1.2rem 1.5rem; border-bottom: 1px solid var(--border); display:flex; align-items:center; gap:1rem; flex-wrap:wrap; }
  header h1 { font-size: 1.2rem; margin: 0; }
  header .status { color: var(--muted); font-size: 0.85rem; }
  select, input, button {
    font: inherit; padding: 0.4rem 0.6rem; border-radius: 6px; border: 1px solid var(--border);
    background: var(--surface); color: var(--text);
  }
  button { cursor: pointer; }
  button.primary { background: var(--accent); color: #fff8f2; border-color: var(--accent); }
  button.danger { background: var(--err); color: #fff8f2; border-color: var(--err); }

  nav.tabs { display: flex; gap: 0.3rem; padding: 0 1.5rem; border-bottom: 1px solid var(--border); background: var(--bg); flex-wrap: wrap; }
  nav.tabs button {
    border: none; border-bottom: 2px solid transparent; border-radius: 0; background: transparent;
    padding: 0.7rem 0.9rem; color: var(--muted); font-weight: 500;
  }
  nav.tabs button.active { color: var(--accent); border-bottom-color: var(--accent); }
  .tab-content { display: none; }
  .tab-content.active { display: block; }

  main { padding: 1.5rem; max-width: 1100px; margin: 0 auto; display: flex; flex-direction: column; gap: 1.5rem; }
  .device { border: 1px solid var(--border); border-radius: 10px; background: var(--surface); overflow: hidden; }
  .device h2 { margin: 0; padding: 0.8rem 1rem; font-size: 1rem; border-bottom: 1px solid var(--border); display:flex; justify-content:space-between; align-items:center; flex-wrap: wrap; gap: 0.5rem; }
  .device h2 .meta { font-weight: 400; color: var(--muted); font-size: 0.8rem; font-family: monospace; }
  .pill { font-size: 0.72rem; padding: 0.1rem 0.5rem; border-radius: 999px; }
  .pill.ok { background: color-mix(in srgb, var(--ok) 18%, transparent); color: var(--ok); }
  .pill.bad { background: color-mix(in srgb, var(--err) 18%, transparent); color: var(--err); }
  table { border-collapse: collapse; width: 100%; font-size: 0.9rem; }
  th, td { text-align: left; padding: 0.5rem 1rem; border-bottom: 1px solid var(--border); }
  th { color: var(--muted); font-size: 0.75rem; text-transform: uppercase; letter-spacing: 0.04em; }
  tr.group td { color: var(--muted); font-size: 0.75rem; text-transform: uppercase; letter-spacing: 0.04em; background: var(--bg); }
  tr:last-child td { border-bottom: none; }
  td.val { font-family: monospace; font-weight: 600; }
  td.err { color: var(--err); font-size: 0.82rem; }
  td.addr { color: var(--muted); font-family: monospace; font-size: 0.8rem; }
  .writebar { padding: 0.8rem 1rem; display: flex; gap: 0.6rem; align-items: center; flex-wrap: wrap; border-top: 1px solid var(--border); }
  .writebar .result { font-size: 0.85rem; color: var(--muted); }
  .empty { padding: 1rem; color: var(--muted); font-size: 0.9rem; }

  section.panel { border: 1px solid var(--border); border-radius: 10px; background: var(--surface); margin: 1.5rem; max-width: 1100px; }
  section.panel > .head { padding: 0.8rem 1rem; border-bottom: 1px solid var(--border); display: flex; align-items: center; justify-content: space-between; flex-wrap: wrap; gap: 0.6rem; }
  section.panel > .head h2 { margin: 0; font-size: 1rem; }
  section.panel > .head .actions { display: flex; gap: 0.5rem; align-items: center; flex-wrap: wrap; }
  section.panel .body { padding: 1rem; overflow-x: auto; }
  section.panel .body p.hint { color: var(--muted); font-size: 0.82rem; margin: 0 0 0.8rem; }
  .result-msg { font-size: 0.85rem; margin-left: 0.6rem; }

  .fw-form { display: flex; flex-wrap: wrap; gap: 1rem; align-items: flex-end; margin-bottom: 1.2rem; }
  .fw-form .field { display: flex; flex-direction: column; gap: 0.25rem; }
  .fw-form .field label { font-size: 0.75rem; color: var(--muted); text-transform: uppercase; letter-spacing: 0.04em; }
  .fw-form .wide { width: 22rem; max-width: 100%; }
  .fw-form .mid { width: 14rem; max-width: 100%; }
  .fw-form .narrow { width: 6.5rem; }
  .fw-form .checkfield { flex-direction: row; align-items: center; gap: 0.4rem; }
  .fw-status { display: flex; gap: 1.5rem; flex-wrap: wrap; padding-top: 1rem; border-top: 1px solid var(--border); }
  .fw-status .stat { display: flex; flex-direction: column; gap: 0.2rem; }
  .fw-status .stat .k { font-size: 0.72rem; text-transform: uppercase; letter-spacing: 0.04em; color: var(--muted); }
  .fw-status .stat .v { font-family: monospace; font-weight: 600; }
  .cmd-row { display: flex; align-items: center; gap: 1rem; flex-wrap: wrap; padding: 0.6rem 0; }
  .cmd-row + .cmd-row { border-top: 1px solid var(--border); }
  .cmd-row .desc { color: var(--muted); font-size: 0.85rem; flex: 1; min-width: 14rem; }
</style>
</head>
<body>
<header>
  <h1>AC Controller &mdash; Modbus Config Client</h1>
  <button id="pollBtn" class="primary">Poll now</button>
  <label><input type="checkbox" id="autoPoll" checked> auto-poll every</label>
  <input id="interval" type="number" value="5" min="1" style="width:4rem"> s
  <span class="status" id="status"></span>
  <span style="flex-basis:100%;height:0"></span>
  <label for="targetIp">Target</label>
  <input id="targetIp" type="text" style="width:9rem" placeholder="10.179.42.1">
  <span>:</span>
  <input id="targetPort" type="number" min="1" max="65535" style="width:5.5rem" placeholder="502">
  <button id="applyTargetBtn">Apply</button>
  <span class="status" id="targetResult"></span>
</header>

<nav class="tabs">
  <button class="tab-btn active" data-tab="settings">Settings</button>
  <button class="tab-btn" data-tab="netcfg">Network Config</button>
  <button class="tab-btn" data-tab="firmware">Firmware Upgrade</button>
  <button class="tab-btn" data-tab="log">Debug Log</button>
  <button class="tab-btn" data-tab="maint">Maintenance</button>
</nav>

<div class="tab-content active" id="tab-settings">
  <main id="main"></main>
</div>

<div class="tab-content" id="tab-netcfg">
  <section class="panel">
    <div class="head">
      <h2>Network / TCP Config</h2>
      <div class="actions">
        <button id="loadNetcfgBtn">Load current</button>
        <button id="saveNetcfgBtn" class="primary">Save changed fields</button>
        <span class="result-msg" id="netcfgResult"></span>
      </div>
    </div>
    <div class="body">
      <p class="hint">Holding registers 18-28 and 130. The firmware takes one field per request, so only the
        fields you changed since "Load current" are written, one at a time. They are saved to the device's
        config file and <strong>take effect after a reboot</strong> (Maintenance tab). The IP/gateway/netmask
        shown on load are the <em>live</em> values, not the stored static config. The static IP is only used
        when enabled and IP, gateway and netmask are all non-zero.</p>
      <div class="fw-form">
        <div class="field checkfield">
          <label for="nc-static_en">Static IP enable</label>
          <input id="nc-static_en" type="checkbox">
        </div>
        <div class="field">
          <label for="nc-ip">IP address</label>
          <input id="nc-ip" class="mid" type="text" placeholder="192.168.8.200">
        </div>
        <div class="field">
          <label for="nc-gw">Gateway</label>
          <input id="nc-gw" class="mid" type="text" placeholder="192.168.8.1">
        </div>
        <div class="field">
          <label for="nc-nm">Netmask</label>
          <input id="nc-nm" class="mid" type="text" placeholder="255.255.255.0">
        </div>
      </div>
      <div class="fw-form">
        <div class="field">
          <label for="nc-mb_port">Modbus port</label>
          <input id="nc-mb_port" class="narrow" type="number" min="1" max="65535" placeholder="502">
        </div>
        <div class="field">
          <label for="nc-pt_port">Pass-through port</label>
          <input id="nc-pt_port" class="narrow" type="number" min="1" max="65535" placeholder="503">
        </div>
        <div class="field">
          <label for="nc-http_port">HTTP port</label>
          <input id="nc-http_port" class="narrow" type="number" min="1" max="65535" placeholder="80">
        </div>
        <div class="field">
          <label for="nc-srv_timeout">TCP idle timeout (s)</label>
          <input id="nc-srv_timeout" class="narrow" type="number" min="1" max="65535">
        </div>
        <div class="field">
          <label for="nc-pt_retry">Pass-through retries</label>
          <input id="nc-pt_retry" class="narrow" type="number" min="0" max="255">
        </div>
      </div>
    </div>
  </section>
</div>

<div class="tab-content" id="tab-firmware">
  <section class="panel">
    <div class="head">
      <h2>Firmware Upgrade (HTTP OTA)</h2>
      <div class="actions">
        <button id="otaSaveBtn" class="primary">Save &amp; Trigger Upgrade</button>
        <span class="result-msg" id="otaResult"></span>
      </div>
    </div>
    <div class="body">
      <p class="hint">Writes the server URL (reg 29), port (93) and file name (94) one at a time, then
        the trigger (126): 1 = http, 2 = https. Fields are pre-filled from the device on first poll.</p>
      <div class="fw-form">
        <div class="field checkfield">
          <label for="otaHttps">HTTPS</label>
          <input id="otaHttps" type="checkbox">
        </div>
        <div class="field">
          <label for="otaUrl">Server URL / host</label>
          <input id="otaUrl" class="wide" type="text" maxlength="127" placeholder="10.0.0.5 or ota.example.com">
        </div>
        <div class="field">
          <label for="otaPort">Port</label>
          <input id="otaPort" class="narrow" type="number" min="0" max="65535" value="8000">
        </div>
        <div class="field">
          <label for="otaFile">File name</label>
          <input id="otaFile" class="mid" type="text" maxlength="63" placeholder="firmware.bin">
        </div>
      </div>
      <div class="fw-status" id="fwStatus"></div>
    </div>
  </section>
</div>

<div class="tab-content" id="tab-log">
  <section class="panel">
    <div class="head">
      <h2>Debug Log Capture</h2>
      <div class="actions">
        <button id="logStartBtn" class="primary">Start capture</button>
        <span class="result-msg" id="logResult"></span>
      </div>
    </div>
    <div class="body">
      <p class="hint">Registers 131-134. Captures the device's log for the given window, then uploads it
        to the OTA server under the path prefix. Leave the path empty to keep the current one.
        Duration 0 means 300 s.</p>
      <div class="fw-form">
        <div class="field">
          <label for="logDuration">Duration (s)</label>
          <input id="logDuration" class="narrow" type="number" min="0" max="65535" value="300">
        </div>
        <div class="field">
          <label for="logPath">Upload path prefix</label>
          <input id="logPath" class="mid" type="text" maxlength="63" placeholder="fota/log">
        </div>
      </div>
      <div class="fw-status" id="logStatus"></div>
    </div>
  </section>
</div>

<div class="tab-content" id="tab-maint">
  <section class="panel">
    <div class="head">
      <h2>Maintenance</h2>
      <span class="result-msg" id="cmdResult"></span>
    </div>
    <div class="body">
      <div class="cmd-row">
        <button class="danger" data-cmd="reboot">Reboot device</button>
        <span class="desc">Writes 0x0A0A to register 566. The device restarts about 9 s later, which
          applies saved network/UART changes.</span>
      </div>
      <div class="cmd-row">
        <button class="danger" data-cmd="delete_config">Delete config file</button>
        <span class="desc">Writes 0x0A0A to register 567. Removes the stored configuration so the device
          falls back to defaults on the next boot.</span>
      </div>
    </div>
  </section>
</div>

<script>
let timer = null;
let currentDeviceKey = null;
let targetInputsInitialized = false;
let otaInputsInitialized = false;
let lastReads = {};
let netcfgLoaded = {};

// Read keys shown on their own tabs instead of the Settings table.
const OTA_KEYS = new Set(["ota_url","ota_port","ota_file","ota_trigger",
                           "fota_status","fota_percent","fota_fail"]);
const LOG_KEYS = new Set(["log_trigger","log_duration","log_status","log_path"]);
const NETCFG_KEYS = ["static_en","ip","gw","nm","mb_port","pt_port","http_port","srv_timeout","pt_retry"];

const GROUPS = [
  ["Device info (input regs)", ["fw_ver","rel_date","wifi_mac","ble_mac","wifi_rssi"]],
  ["RS-485 / UART", ["baud","parity","data_bits","stop_bits","retry_cnt","retry_prd"]],
  ["Network / TCP", NETCFG_KEYS],
];
const LABELS = {
  fw_ver: "Firmware version", rel_date: "Release date", wifi_mac: "WiFi MAC", ble_mac: "BLE MAC",
  wifi_rssi: "WiFi RSSI (dBm)", baud: "Baud rate", parity: "Parity", data_bits: "Data bits",
  stop_bits: "Stop bits", retry_cnt: "Retry count", retry_prd: "Retry period (ms)",
  ip: "IP address (live)", gw: "Gateway (live)", nm: "Netmask (live)", mb_port: "Modbus port",
  pt_port: "Pass-through port", http_port: "HTTP port", static_en: "Static IP enabled",
  srv_timeout: "TCP idle timeout (s)", pt_retry: "Pass-through retries",
};
const PARITY_NAMES = ["None","Even","Odd"];
const OTA_STATUS_NAMES = ["default","success","started","ongoing","retrying","failed",
                           "flashing","rebooting","idle"];
const OTA_FAILURE_NAMES = ["none","invalid_argument","bad_json_info","bad_json_negotiate",
                            "mem_alloc_failed","mem_ota_begin_failed","mem_ota_client_failed",
                            "download_packet_failed","write_packet_failed","sha_mismatch",
                            "ota_end_failed","set_boot_failed","open_client_failed",
                            "content_length_mismatch","no_crc_provided","crc_mismatch","http_error"];
const OTA_TRIGGER_NAMES = ["off","http","https"];
const LOG_STATUS_NAMES = ["idle","capturing","uploading","done","failed"];

function fmt(v) {
  if (v === null || v === undefined) return '--';
  if (typeof v === 'number' && !Number.isInteger(v)) return v.toFixed(3);
  return String(v);
}

function fmtRead(key, v) {
  if (v === null || v === undefined) return '--';
  if (key === 'parity') return `${v} (${PARITY_NAMES[v] || '?'})`;
  if (key === 'static_en') return v ? 'yes' : 'no';
  if (key === 'wifi_rssi' && v === 127) return 'not connected';
  return fmt(v);
}

function readVal(key) {
  const r = lastReads[key];
  return r && r.value != null ? r.value : null;
}

function showResult(el, ok, message) {
  el.textContent = ok ? ('OK -- ' + message) : ('Error: ' + message);
  el.style.color = ok ? 'var(--ok)' : 'var(--err)';
}

async function postAction(url, body, resultEl, busyText) {
  resultEl.textContent = busyText || 'working...';
  resultEl.style.color = 'var(--muted)';
  try {
    const res = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    const out = await res.json();
    showResult(resultEl, out.ok, out.message);
    return out.ok;
  } catch (e) {
    showResult(resultEl, false, String(e));
    return false;
  }
}

for (const btn of document.querySelectorAll('.tab-btn')) {
  btn.onclick = () => {
    for (const b of document.querySelectorAll('.tab-btn')) b.classList.remove('active');
    for (const c of document.querySelectorAll('.tab-content')) c.classList.remove('active');
    btn.classList.add('active');
    document.getElementById('tab-' + btn.dataset.tab).classList.add('active');
  };
}

async function poll() {
  document.getElementById('status').textContent = 'polling...';
  try {
    const res = await fetch('/api/poll');
    const snap = await res.json();
    render(snap);
    document.getElementById('status').textContent = 'updated ' + new Date().toLocaleTimeString();
  } catch (e) {
    document.getElementById('status').textContent = 'poll failed: ' + e;
  }
}

function render(snap) {
  const main = document.getElementById('main');
  main.innerHTML = '';
  for (const [deviceKey, dev] of Object.entries(snap.devices)) {
    currentDeviceKey = deviceKey;
    lastReads = dev.reads;
    renderFirmwareStatus();
    renderLogStatus();

    if (!targetInputsInitialized) {
      document.getElementById('targetIp').value = dev.pull_ip;
      document.getElementById('targetPort').value = dev.pull_port;
      targetInputsInitialized = true;
    }
    if (!otaInputsInitialized && dev.status === 'ok') {
      initOtaInputs();
      otaInputsInitialized = true;
    }

    const card = document.createElement('div');
    card.className = 'device';

    const okBad = dev.status === 'ok' ? 'ok' : 'bad';
    card.innerHTML = `
      <h2>${deviceKey}
        <span class="meta">${dev.pull_ip}:${dev.pull_port} &middot; slave ${dev.slave_id} &middot; ${dev.framer}
          &nbsp;<span class="pill ${okBad}">${dev.status}</span>
        </span>
      </h2>`;

    const table = document.createElement('table');
    table.innerHTML = '<thead><tr><th>Parameter</th><th>Value</th></tr></thead>';
    const tbody = document.createElement('tbody');
    const shown = new Set();
    const addRow = (key) => {
      const r = dev.reads[key];
      if (!r) return;
      shown.add(key);
      const tr = document.createElement('tr');
      const label = LABELS[key] || key;
      tr.innerHTML = r.error
        ? `<td>${label}</td><td class="err">${r.error}</td>`
        : `<td>${label}</td><td class="val">${fmtRead(key, r.value)}</td>`;
      tbody.appendChild(tr);
    };
    for (const [title, keys] of GROUPS) {
      const g = document.createElement('tr');
      g.className = 'group';
      g.innerHTML = `<td colspan="2">${title}</td>`;
      tbody.appendChild(g);
      keys.forEach(addRow);
    }
    // Anything added to readparameter.json that isn't in a group above.
    const rest = Object.keys(dev.reads).filter(k => !shown.has(k) && !OTA_KEYS.has(k) && !LOG_KEYS.has(k));
    if (rest.length) {
      const g = document.createElement('tr');
      g.className = 'group';
      g.innerHTML = '<td colspan="2">Other</td>';
      tbody.appendChild(g);
      rest.forEach(addRow);
    }
    table.appendChild(tbody);
    card.appendChild(table);

    const writes = Object.entries(dev.writes).filter(([key]) => !OTA_KEYS.has(key) && !LOG_KEYS.has(key));
    if (writes.length) {
      const bar = document.createElement('div');
      bar.className = 'writebar';
      const sel = document.createElement('select');
      for (const [key] of writes) {
        const opt = document.createElement('option');
        opt.value = key; opt.textContent = LABELS[key] || key;
        sel.appendChild(opt);
      }
      const input = document.createElement('input');
      input.placeholder = 'value';
      input.style.width = '10rem';
      const btn = document.createElement('button');
      btn.textContent = 'Write';
      const result = document.createElement('span');
      result.className = 'result';

      btn.onclick = () => postAction('/api/write',
        { device: deviceKey, param: sel.value, value: input.value }, result, 'writing...');

      bar.append(sel, input, btn, result);
      card.appendChild(bar);
    }

    main.appendChild(card);
  }
}

function renderFirmwareStatus() {
  const statusVal = readVal('fota_status');
  const pctVal = readVal('fota_percent');
  const failVal = readVal('fota_fail');
  const trigVal = readVal('ota_trigger');
  const name = (names, v) => v != null ? (names[v] || v) : '--';
  document.getElementById('fwStatus').innerHTML = `
    <div class="stat"><span class="k">Firmware</span><span class="v">${fmt(readVal('fw_ver'))}</span></div>
    <div class="stat"><span class="k">Trigger</span><span class="v">${name(OTA_TRIGGER_NAMES, trigVal)}</span></div>
    <div class="stat"><span class="k">Status</span><span class="v">${name(OTA_STATUS_NAMES, statusVal)}</span></div>
    <div class="stat"><span class="k">Progress</span><span class="v">${pctVal != null ? pctVal + '%' : '--'}</span></div>
    <div class="stat"><span class="k">Failure reason</span><span class="v">${name(OTA_FAILURE_NAMES, failVal)}</span></div>`;
}

function initOtaInputs() {
  const url = readVal('ota_url'), port = readVal('ota_port'), file = readVal('ota_file');
  if (url) document.getElementById('otaUrl').value = url;
  if (port) document.getElementById('otaPort').value = port;
  if (file) document.getElementById('otaFile').value = file;
  if (readVal('ota_trigger') === 2) document.getElementById('otaHttps').checked = true;
}

function renderLogStatus() {
  const st = readVal('log_status');
  document.getElementById('logStatus').innerHTML = `
    <div class="stat"><span class="k">Status</span><span class="v">${st != null ? (LOG_STATUS_NAMES[st] || st) : '--'}</span></div>
    <div class="stat"><span class="k">Duration</span><span class="v">${fmt(readVal('log_duration'))} s</span></div>
    <div class="stat"><span class="k">Path</span><span class="v">${fmt(readVal('log_path'))}</span></div>`;
}

document.getElementById('pollBtn').onclick = poll;
document.getElementById('autoPoll').onchange = resetTimer;
document.getElementById('interval').onchange = resetTimer;

async function applyTarget() {
  const resultEl = document.getElementById('targetResult');
  if (!currentDeviceKey) {
    showResult(resultEl, false, 'poll at least once first');
    return;
  }
  const ip = document.getElementById('targetIp').value.trim();
  const port = document.getElementById('targetPort').value;
  if (await postAction('/api/target', { device: currentDeviceKey, ip, port }, resultEl, 'applying...')) {
    otaInputsInitialized = false;
    poll();
  }
}
document.getElementById('applyTargetBtn').onclick = applyTarget;

function resetTimer() {
  if (timer) clearInterval(timer);
  timer = null;
  if (document.getElementById('autoPoll').checked) {
    const secs = Math.max(1, parseInt(document.getElementById('interval').value) || 5);
    timer = setInterval(poll, secs * 1000);
  }
}

function netcfgInputValue(key) {
  const el = document.getElementById('nc-' + key);
  if (el.type === 'checkbox') return el.checked ? 1 : 0;
  const v = el.value.trim();
  return el.type === 'number' && v !== '' ? +v : v;
}

async function loadNetcfg() {
  const resultEl = document.getElementById('netcfgResult');
  resultEl.textContent = 'loading...';
  resultEl.style.color = 'var(--muted)';
  await poll();
  const missing = NETCFG_KEYS.filter(k => readVal(k) == null);
  netcfgLoaded = {};
  for (const key of NETCFG_KEYS) {
    const v = readVal(key);
    const el = document.getElementById('nc-' + key);
    if (el.type === 'checkbox') el.checked = !!v;
    else el.value = v != null ? v : '';
    netcfgLoaded[key] = netcfgInputValue(key);
  }
  showResult(resultEl, missing.length === 0,
    missing.length ? ('could not read: ' + missing.join(', ')) : 'loaded');
}

async function saveNetcfg() {
  const resultEl = document.getElementById('netcfgResult');
  const fields = {};
  for (const key of NETCFG_KEYS) {
    const v = netcfgInputValue(key);
    if (v === '' || v === netcfgLoaded[key]) continue;
    fields[key] = v;
  }
  if (!Object.keys(fields).length) {
    showResult(resultEl, false, 'nothing changed since "Load current"');
    return;
  }
  if (await postAction('/api/netcfg', { fields }, resultEl, 'writing ' + Object.keys(fields).join(', ') + '...')) {
    Object.assign(netcfgLoaded, fields);
  }
}

document.getElementById('loadNetcfgBtn').onclick = loadNetcfg;
document.getElementById('saveNetcfgBtn').onclick = saveNetcfg;

async function saveAndTriggerOta() {
  const resultEl = document.getElementById('otaResult');
  const https = document.getElementById('otaHttps').checked;
  const url = document.getElementById('otaUrl').value.trim();
  const port = document.getElementById('otaPort').value;
  const file = document.getElementById('otaFile').value.trim();

  if (!url || !file) {
    showResult(resultEl, false, 'URL and file name are required');
    return;
  }
  await postAction('/api/ota', { https, url, port: +port, file }, resultEl, 'staging and triggering...');
  poll();
}
document.getElementById('otaSaveBtn').onclick = saveAndTriggerOta;

async function startLog() {
  const resultEl = document.getElementById('logResult');
  const duration = +document.getElementById('logDuration').value || 0;
  const path = document.getElementById('logPath').value.trim();
  await postAction('/api/log', { duration, path }, resultEl, 'starting...');
  poll();
}
document.getElementById('logStartBtn').onclick = startLog;

for (const btn of document.querySelectorAll('[data-cmd]')) {
  btn.onclick = () => {
    if (!confirm(btn.textContent + '?')) return;
    postAction('/api/command', { command: btn.dataset.cmd }, document.getElementById('cmdResult'), 'sending...');
  };
}

poll();
resetTimer();
</script>
</body>
</html>
"""


@app.get("/")
def index():
    return Response(INDEX_HTML, mimetype="text/html")


@app.get("/api/poll")
def api_poll():
    profile.poll()
    return jsonify(profile.snapshot())


@app.post("/api/target")
def api_target():
    body = request.get_json(force=True, silent=True) or {}
    device = body.get("device")
    ip = body.get("ip")
    port = body.get("port")
    if not device:
        return jsonify({"ok": False, "message": "device is required"}), 400
    ok, message = profile.set_target(device, ip=ip, port=port)
    return jsonify({"ok": ok, "message": message})


@app.post("/api/write")
def api_write():
    body = request.get_json(force=True, silent=True) or {}
    device = body.get("device")
    param = body.get("param")
    value = body.get("value")
    if device is None or param is None or value is None or value == "":
        return jsonify({"ok": False, "message": "device, param and value are required"}), 400
    ok, message = profile.write(device, param, value)
    return jsonify({"ok": ok, "message": message})


@app.post("/api/netcfg")
def api_netcfg():
    body = request.get_json(force=True, silent=True) or {}
    fields = body.get("fields")
    if not isinstance(fields, dict) or not fields:
        return jsonify({"ok": False, "message": "missing 'fields' object"}), 400
    ok, message = profile.write_netcfg(fields)
    return jsonify({"ok": ok, "message": message})


@app.post("/api/ota")
def api_ota():
    body = request.get_json(force=True, silent=True) or {}
    url = body.get("url")
    file = body.get("file")
    if not url or not file:
        return jsonify({"ok": False, "message": "url and file are required"}), 400
    ok, message = profile.write_ota_config(
        https=bool(body.get("https")), url=url, port=body.get("port") or 0, file=file)
    if ok:
        message = "upgrade triggered"
    return jsonify({"ok": ok, "message": message})


@app.post("/api/log")
def api_log():
    body = request.get_json(force=True, silent=True) or {}
    ok, message = profile.start_log_capture(
        duration=body.get("duration") or 0, path=body.get("path") or None)
    if ok:
        message = "capture started"
    return jsonify({"ok": ok, "message": message})


@app.post("/api/command")
def api_command():
    body = request.get_json(force=True, silent=True) or {}
    ok, message = profile.send_command(body.get("command"))
    return jsonify({"ok": ok, "message": message})


if __name__ == "__main__":
    dev = next(iter(profile.devices.values()))
    print(f"Target: {dev['pull_ip']}:{dev['pull_port']} "
          f"(slave {dev['slave_id']}, "
          f"{'RTU-over-TCP' if dev.get('framer_rtu') else 'Modbus/TCP'}; edit deviceconfig.json to change)")
    app.run(host="127.0.0.1", port=5000, debug=False)
