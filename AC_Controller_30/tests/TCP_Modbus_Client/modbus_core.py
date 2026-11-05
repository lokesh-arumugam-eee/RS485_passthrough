"""
Shared Modbus TCP polling/writing engine, factored out of the original
main.py so both the console tool and the web UI (web_server.py) use the
exact same read/write/encode logic against the exact same config files.

Targets the AC_Controller_30 ESP32 firmware's config server (port 502,
slave 249). Its address (IP/port/slave ID) comes from deviceconfig.json in
this folder -- edit that file to point at a different device, nothing here
is hardcoded. (configuration_conserv/, configuration_multispan/, and fw_up/
are separate devices' configs kept for reference but are no longer loaded
by this tool.)

Register map: main/modbus/modbus_rtu.h. The "add" field in
readparameter.json/writeparameter.json is 1-based (wire address + 1).

Two firmware behaviours shape every request made here:
  * populate_modbus_read_*/write() switch on the *starting* address only,
    so each request must cover exactly one field -- a multi-field block
    read/write just repeats the first field.
  * String fields (OTA URL/file, log path) must be written with exactly
    their full register count; fewer is rejected, more overflows the
    firmware's buffer.
"""

import json
import os
import struct
import threading

from pymodbus.client import ModbusTcpClient
from pymodbus.framer import FramerType

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

dictionary_template = {
    "reg": 3,
    "accs": 0,
    "add": 2700,
    "grp": 0,
    "bytes": 4,
    "dwn_typ": 0,
    "up_typ": 0,
    "Endian": 0,
    "Multiplier": 1.0,
    "st_bit": 0,
    "bit_cnt": 0,
    "data": 0.0,
    "is_data": 0,
}

device_template = {
    "slave_id": 0,
    "read_interval": 60,
    "push_inteval_m": 600,
    "push_interval_b": 600,
    "dummy": "tpa",
    "make": "config",
    "pull_ip": "192.168.192.1",
    "pull_port": 502,
    # Extra, optional 9th field. 0 = standard Modbus/TCP (MBAP) framing.
    # 1 = RTU-over-TCP -- required for the AC_Controller_30 config server,
    # which runs nanomodbus with NMBS_TRANSPORT_RTU on its TCP socket
    # (modbus_tcp.c), and for RS-485<->TCP gateways tunnelling raw RTU.
    "framer_rtu": 0,
}


def is_int(element) -> bool:
    if element is None:
        return False
    try:
        int(element)
        return True
    except ValueError:
        return False


def Endian(value, type_, nobytes):
    if nobytes > 4:
        byte_list = []
        for val in value:
            byte_list.extend(val.to_bytes(2, byteorder='big'))
        return byte_list

    bytes_array = value[0].to_bytes(2, byteorder='big')
    byte_A = bytes_array[0]
    byte_B = bytes_array[1]
    byte_C = byte_D = 0
    if nobytes > 2:
        bytes_array1 = value[1].to_bytes(2, byteorder='big')
        byte_C = bytes_array1[0]
        byte_D = bytes_array1[1]

    if type_ == 1:  # DCBA
        if nobytes > 2:
            byte_D = bytes_array[0]
            byte_C = bytes_array[1]
            byte_B = bytes_array1[0]
            byte_A = bytes_array1[1]
    if type_ == 2:  # BADC
        byte_B = bytes_array[0]
        byte_A = bytes_array[1]
        if nobytes > 2:
            byte_D = bytes_array1[0]
            byte_C = bytes_array1[1]
    if type_ == 3:  # CDAB
        if nobytes > 2:
            byte_C = bytes_array[0]
            byte_D = bytes_array[1]
            byte_A = bytes_array1[0]
            byte_B = bytes_array1[1]

    if nobytes > 2:
        return [byte_A, byte_B, byte_C, byte_D]
    return [byte_A, byte_B]


def convert(data_bytes, uplinktype, multiply):
    if uplinktype == 0:  # float
        packed_value = struct.pack('>BBBB', data_bytes[0], data_bytes[1], data_bytes[2], data_bytes[3])
        float_value = struct.unpack('>f', packed_value)[0]
        return float_value * multiply
    if uplinktype == 1 or uplinktype == 2:
        if len(data_bytes) > 2:
            packed_value = struct.pack('>BBBB', *data_bytes)
            int_value = struct.unpack('>I', packed_value)[0]
        else:
            packed_value = struct.pack('>BBBB', 0, 0, data_bytes[0], data_bytes[1])
            int_value = struct.unpack('>I', packed_value)[0]
        return int_value
    if uplinktype == 3:
        packed_value = struct.pack('>BB', data_bytes[0], data_bytes[1])
        return struct.unpack('>H', packed_value)[0]
    if uplinktype == 4:  # signed 16-bit (e.g. WiFi RSSI in dBm)
        packed_value = struct.pack('>BB', data_bytes[0], data_bytes[1])
        return struct.unpack('>h', packed_value)[0]
    if uplinktype == 5:
        packed_value = struct.pack('>B', data_bytes[0])
        return struct.unpack('>H', packed_value)[0]
    if uplinktype == 23:
        return (f"{format(data_bytes[0], '02x')}.{format(data_bytes[1], '02x')}."
                f"{format(data_bytes[2], '02x')}.{format(data_bytes[3], '02x')}."
                f"{format(data_bytes[4], '02x')}.{format(data_bytes[5], '02x')}")
    if uplinktype == 24:
        return f"{data_bytes[0]}.{data_bytes[1]}.{data_bytes[2]}.{data_bytes[3]}"
    if uplinktype == 25:
        return "".join(chr(b) for b in data_bytes if b != 0)
    if uplinktype == 26:  # firmware version, packed as HIGH*1e6 + LOW*1e3 + PATCH
        v = struct.unpack('>I', bytes(data_bytes[:4]))[0]
        return f"{v // 1000000}.{(v // 1000) % 1000}.{v % 1000}"
    if uplinktype == 27:  # release date, packed as DD*1e6 + MM*1e4 + YYYY
        v = struct.unpack('>I', bytes(data_bytes[:4]))[0]
        return f"{v // 1000000:02d}-{(v // 10000) % 100:02d}-{v % 10000:04d}"
    return 0


def convert_data(data, length, type_):
    my_ascii_list = []
    if type_ == 1:
        # Big-endian across `length` registers, high word first (baud rate
        # 115200 doesn't fit in one register).
        my_data = int(data)
        for _ in range(length):
            my_ascii_list.insert(0, my_data & 0xFFFF)
            my_data >>= 16
    if type_ == 24:
        ip_address = data.split(".")
        ip_address_int = [int(val) for val in ip_address]
        byte1 = ((ip_address_int[0] << 8) | (ip_address_int[1] & 0x00FF))
        my_ascii_list.append(byte1)
        byte1 = ((ip_address_int[2] << 8) | (ip_address_int[3] & 0x00FF))
        my_ascii_list.append(byte1)
    if type_ == 25:
        # Truncate so the result is exactly `length` registers and keeps a
        # NUL terminator -- the firmware copies every register it receives.
        my_data = [ord(ch) for ch in data][:length * 2 - 1]
        even = [my_data[i] for i in range(0, len(my_data), 2)]
        odd = [my_data[i] for i in range(1, len(my_data), 2)]
        if len(even) > len(odd):
            odd.append(0)
        for i in range(len(even)):
            even[i] = (even[i] << 8) | (odd[i] & 0x00FF)
        while len(even) < length:
            even.append(0)
        my_ascii_list = even
    return my_ascii_list


def _load_positional(path, template):
    with open(path, 'r') as f:
        config_data = json.load(f)
    out = {}
    for key, raw_string in config_data.items():
        values = raw_string.split(":")
        entry = template.copy()
        for i, field in enumerate(entry.keys()):
            if i < len(values):
                entry[field] = int(values[i]) if is_int(values[i]) else values[i]
        out[key] = entry
    return out


def read_device_config_file(path):
    return _load_positional(path, device_template)


def read_param_file(path, devices):
    with open(path, 'r') as f:
        config_data = json.load(f)
    full = {}
    for device_key in devices.keys():
        if device_key not in config_data:
            continue
        device_dict = config_data[device_key]
        params = {}
        for param_key, raw_string in device_dict.items():
            values = raw_string.split(":")
            entry = dictionary_template.copy()
            for i, field in enumerate(entry.keys()):
                if i < len(values):
                    entry[field] = int(values[i]) if is_int(values[i]) else values[i]
            params[param_key] = entry
        full[device_key] = params
    return full


class ModbusProfile:
    """One deviceconfig/readparameter/writeparameter triplet, with its own
    live-value state. Safe to poll/write independently of any other profile."""

    def __init__(self, folder):
        self.folder = folder
        self.name = "IR Blaster" if os.path.normpath(folder) == os.path.normpath(BASE_DIR) \
            else os.path.basename(folder.rstrip("/\\"))
        self.devices = read_device_config_file(os.path.join(folder, "deviceconfig.json"))
        self.read_params = read_param_file(os.path.join(folder, "readparameter.json"), self.devices)
        self.write_params = read_param_file(os.path.join(folder, "writeparameter.json"), self.devices)
        self.status = {dk: "never polled" for dk in self.devices}
        self.lock = threading.Lock()

    def _client_for(self, dev):
        framer = FramerType.RTU if dev.get("framer_rtu") else FramerType.SOCKET
        return ModbusTcpClient(dev["pull_ip"], port=dev["pull_port"], framer=framer)

    def set_target(self, device_key, ip=None, port=None):
        """Overrides a device's ip/port for the rest of this process's
        lifetime (in memory only -- does not rewrite deviceconfig.json).
        Lets the UI point at a device's new Modbus port (e.g. after a
        "netcfg"/"ipc" mb_port change) without editing the config file and
        restarting the server."""
        if device_key not in self.devices:
            return False, f"unknown device '{device_key}'"
        if ip:
            self.devices[device_key]["pull_ip"] = ip
        if port:
            port = int(port)
            if not (1 <= port <= 65535):
                return False, "port must be 1..65535"
            self.devices[device_key]["pull_port"] = port
        self.status[device_key] = "never polled"
        return True, "ok"

    def poll(self):
        """Reads every configured parameter for every device in this profile."""
        with self.lock:
            for device_key, dev in self.devices.items():
                params = self.read_params.get(device_key, {})
                if not params:
                    continue
                client = self._client_for(dev)
                if not client.connect():
                    self.status[device_key] = "connection failed"
                    continue
                try:
                    for key, p in params.items():
                        address = p["add"] - 1
                        count = max(1, int(p["bytes"] / 2))
                        try:
                            if p["reg"] == 3:
                                result = client.read_holding_registers(
                                    address=address, count=count, device_id=dev["slave_id"])
                            elif p["reg"] == 4:
                                result = client.read_input_registers(
                                    address=address, count=count, device_id=dev["slave_id"])
                            else:
                                continue
                            if result.isError():
                                p["error"] = str(result)
                                p["is_data"] = 0
                            else:
                                packed = Endian(result.registers, p["Endian"], p["bytes"])
                                value = convert(packed, p["up_typ"], float(p["Multiplier"]))
                                if value == 4294967295:
                                    value = -1
                                p["data"] = value
                                p["is_data"] = 1
                                p.pop("error", None)
                        except Exception as e:  # noqa: BLE001 -- surfaced to the UI, not swallowed silently
                            p["error"] = str(e)
                            p["is_data"] = 0
                    self.status[device_key] = "ok"
                finally:
                    client.close()

    def write(self, device_key, param_key, value):
        """Writes a single parameter's runtime value (from the UI/API caller,
        not writeparameter.json's own "data" field) using that parameter's
        stored address/byte-width/encoding metadata."""
        if device_key not in self.devices:
            return False, f"unknown device '{device_key}'"
        params = self.write_params.get(device_key, {})
        if param_key not in params:
            return False, f"'{param_key}' is not a configured write parameter for '{device_key}'"
        p = params[param_key]
        dev = self.devices[device_key]

        write_values = convert_data(value, int(p["bytes"] / 2), p["up_typ"])
        if not write_values:
            # Plain numeric field with no special up_typ conversion (e.g. a
            # flat holding-register value) -- write it directly.
            write_values = [int(value)]

        client = self._client_for(dev)
        if not client.connect():
            return False, "connection failed"
        try:
            address = p["add"] - 1
            result = client.write_registers(address=address, values=write_values, device_id=dev["slave_id"])
            if result.isError():
                return False, str(result)
            p["data"] = value
            p["is_data"] = 1
            return True, "ok"
        except Exception as e:  # noqa: BLE001
            return False, str(e)
        finally:
            client.close()

    # ---- Multi-step features ---------------------------------------------
    # Everything below is built on write() above: the firmware handles one
    # field per request (see the module docstring), so a "form" is just a
    # sequence of single-parameter writes, stopping at the first rejection.

    def _first_device_key(self):
        return next(iter(self.devices))

    def _write_sequence(self, steps):
        """steps: list of (param_key, value). Returns (ok, message)."""
        device_key = self._first_device_key()
        for key, value in steps:
            ok, message = self.write(device_key, key, value)
            if not ok:
                return False, f"{key}: {message}"
        return True, "ok"

    # Network / TCP settings, holding 18-28 + 130. The firmware only saves
    # these to its config file; the new IP/ports take effect after a reboot.
    # Reading 18/20/22 returns the *live* IP/gateway/netmask, not the stored
    # static config.
    NETCFG_KEYS = ["static_en", "ip", "gw", "nm", "mb_port", "pt_port",
                   "http_port", "srv_timeout", "pt_retry"]

    def write_netcfg(self, fields):
        steps = [(k, fields[k]) for k in self.NETCFG_KEYS
                 if fields.get(k) not in (None, "")]
        if not steps:
            return False, "no fields to write"
        ok, message = self._write_sequence(steps)
        if ok:
            return True, "saved -- reboot the device to apply"
        return ok, message

    # Firmware upgrade (HTTP OTA), holding 29/93/94/126; status on input
    # 127-129. The trigger value selects the scheme: 1 = http, 2 = https.
    def write_ota_config(self, https, url, port, file):
        return self._write_sequence([
            ("ota_url", url),
            ("ota_port", int(port)),
            ("ota_file", file),
            ("ota_trigger", 2 if https else 1),
        ])

    # Debug-log capture, holding 131-134 (log_capture.h). Status (133) is
    # read-only: 0 idle, 1 capturing, 2 uploading, 3 done, 4 failed.
    def start_log_capture(self, duration, path=None):
        steps = []
        if path:
            steps.append(("log_path", path))
        steps.append(("log_duration", int(duration)))
        steps.append(("log_trigger", 1))
        return self._write_sequence(steps)

    # Maintenance commands, outside the normal map (no boundary check) and
    # armed with the magic value 0x0A0A. NB: the firmware's names are the
    # wrong way round -- ADD_DELETE_FILE (566) actually schedules a reboot
    # (~9 s), and ADD_REBOOT_DEVICE (567) actually deletes the config file.
    # Keyed here by what they really do.
    COMMANDS = {"reboot": 566, "delete_config": 567}
    COMMAND_MAGIC = 0x0A0A

    def send_command(self, name):
        if name not in self.COMMANDS:
            return False, f"unknown command '{name}'"
        dev = self.devices[self._first_device_key()]
        client = self._client_for(dev)
        if not client.connect():
            return False, "connection failed"
        try:
            result = client.write_registers(
                address=self.COMMANDS[name], values=[self.COMMAND_MAGIC], device_id=dev["slave_id"])
            if result.isError():
                return False, str(result)
            if name == "reboot":
                return True, "reboot scheduled -- device restarts in ~9 s"
            return True, "config file deleted"
        except Exception as e:  # noqa: BLE001
            return False, str(e)
        finally:
            client.close()

    def snapshot(self):
        """JSON-serializable view of this profile's current state, for the API."""
        devices_out = {}
        for device_key, dev in self.devices.items():
            reads = {}
            for key, p in self.read_params.get(device_key, {}).items():
                reads[key] = {
                    "value": p.get("data") if p.get("is_data") else None,
                    "error": p.get("error"),
                }
            writes = {}
            for key, p in self.write_params.get(device_key, {}).items():
                writes[key] = {
                    "last_written": p.get("data") if p.get("is_data") else None,
                    "up_typ": p["up_typ"],
                }
            devices_out[device_key] = {
                "slave_id": dev["slave_id"],
                "pull_ip": dev["pull_ip"],
                "pull_port": dev["pull_port"],
                "framer": "RTU-over-TCP" if dev.get("framer_rtu") else "Modbus/TCP",
                "status": self.status.get(device_key, "never polled"),
                "reads": reads,
                "writes": writes,
            }
        return {"profile": self.name, "folder": os.path.basename(self.folder.rstrip("/\\")) or ".",
                "devices": devices_out}


def load_ir_blaster_profile():
    """Loads the device's deviceconfig.json/readparameter.json/
    writeparameter.json from this folder. Its address (IP/port/slave ID)
    is entirely config-file driven -- edit deviceconfig.json's "pull_ip"/
    "pull_port"/slave-id fields (the 7th/8th/1st colon-separated values) to
    point this at a different device; nothing here needs to change."""
    return ModbusProfile(BASE_DIR)
