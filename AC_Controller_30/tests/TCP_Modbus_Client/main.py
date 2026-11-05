import time
from datetime import datetime

from pymodbus.framer import FramerType
from prettytable import PrettyTable

from modbus_core import (
    Endian,
    convert,
    convert_data,
    read_device_config_file,
    read_param_file,
)

table = PrettyTable()
property_table = PrettyTable()
device_table = PrettyTable()
write_parameter_table = PrettyTable()

my_property = {}
my_write_property = {}
my_devices = {}


def run_client(devices, property_, read):
    from pymodbus.client import ModbusTcpClient

    for device in devices.keys():
        dev = devices[device]
        pull_ip = dev["pull_ip"]
        port = dev["pull_port"]
        slave_id = dev["slave_id"]
        current_device = property_[device]
        framer = FramerType.RTU if dev.get("framer_rtu") else FramerType.SOCKET
        client = ModbusTcpClient(pull_ip, port=port, framer=framer)

        print("Connecting to Nano modbus device...")
        if not client.connect():
            print("Connection Failed!")
            return

        try:
            for key in current_device.keys():
                print(f"key :{key}")
                print(current_device)
                regtyp = current_device[key]["reg"]
                address = current_device[key]["add"] - 1
                count = int(current_device[key]["bytes"] / 2)
                print(f"count : {count}\n")
                endian_type = current_device[key]["Endian"]
                no_of_bytes = current_device[key]["bytes"]
                upload_type = current_device[key]["up_typ"]
                multiplier = float(current_device[key]["Multiplier"])
                if current_device[key]["is_data"] == 1:
                    if read:
                        print(f"Data already read at {address}")
                    else:
                        print(f"Data already written at {address}")
                    continue

                if read:
                    if regtyp == 3:
                        result = client.read_holding_registers(address=address, count=count, device_id=slave_id)
                    elif regtyp == 4:
                        result = client.read_input_registers(address=address, count=count, device_id=slave_id)
                    else:
                        print("reg type read not implemented in this code")
                        continue
                else:
                    write_data = convert_data(current_device[key]["data"], int(no_of_bytes / 2), upload_type)
                    result = client.write_registers(address=address, values=write_data, device_id=slave_id)
                print(type(result))
                if result.isError():
                    print(f"Modbus Error: {result}")
                else:
                    if read:
                        print(f"Success! Register values: {result.registers}\n")
                        result_in_list = Endian(result.registers, endian_type, no_of_bytes)
                        print(f"result in list :{result_in_list}\n")
                        value = convert(result_in_list, upload_type, multiplier)
                        if value == 4294967295:
                            value = -1
                        my_property[device][key]["data"] = value
                        my_property[device][key]["is_data"] = 1
                    else:
                        print(f"Success! Register values: {result.registers}\n")
                        my_write_property[device][key]["is_data"] = 1

        except Exception as e:
            print(f"Error: {e}")
        finally:
            client.close()


def add_table():
    table.field_names = ["deviceid", "paramter", "value"]
    device_table.field_names = ["slave_id", "read_interval", "push_inteval_m", "push_interval_b", "dummy", "make", "pull_ip", "pull_port"]
    for devices in my_devices.keys():
        device_table.add_row([my_devices[devices]["slave_id"],
                             my_devices[devices]["read_interval"],
                             my_devices[devices]["push_inteval_m"],
                             my_devices[devices]["push_interval_b"],
                             my_devices[devices]["dummy"],
                             my_devices[devices]["make"],
                             my_devices[devices]["pull_ip"],
                             my_devices[devices]["pull_port"]])
    print(device_table)
    property_table.field_names = ["DeviceID", "parameter", "reg", "accs", "add", "grp", "bytes", "dwn_typ", "up_typ", "Endian", "Multiplier", "st_bit"]
    for device_key in my_property.keys():
        parameter = my_property[device_key]
        for params_key in parameter.keys():
            property_table.add_row([device_key, params_key,
                                    parameter[params_key]["reg"], parameter[params_key]["accs"],
                                    parameter[params_key]["add"], parameter[params_key]["grp"],
                                    parameter[params_key]["bytes"], parameter[params_key]["dwn_typ"],
                                    parameter[params_key]["up_typ"], parameter[params_key]["Endian"],
                                    parameter[params_key]["Multiplier"], parameter[params_key]["st_bit"]])
    print(property_table)

    write_parameter_table.field_names = ["DeviceID", "parameter", "reg", "accs", "add", "grp", "bytes", "dwn_typ", "up_typ",
                                  "Endian", "Multiplier", "st_bit", "write_date", "write_com"]
    for device_key in my_write_property.keys():
        parameter = my_write_property[device_key]
        for params_key in parameter.keys():
            write_parameter_table.add_row([device_key, params_key,
                                    parameter[params_key]["reg"], parameter[params_key]["accs"],
                                    parameter[params_key]["add"], parameter[params_key]["grp"],
                                    parameter[params_key]["bytes"], parameter[params_key]["dwn_typ"],
                                    parameter[params_key]["up_typ"], parameter[params_key]["Endian"],
                                    parameter[params_key]["Multiplier"], parameter[params_key]["st_bit"],
                                    parameter[params_key]["data"], parameter[params_key]["is_data"]])
    print(write_parameter_table)


def update_value():
    table.clear_rows()
    for device_keys in my_property.keys():
        device_parameter = my_property[device_keys]
        for parameter_key in device_parameter.keys():
            if device_parameter[parameter_key]["is_data"]:
                table.add_row([device_keys, parameter_key, device_parameter[parameter_key]["data"]])
            else:
                table.add_row([device_keys, parameter_key, "NO DATA"])
            my_property[device_keys][parameter_key]["is_data"] = 0
    print(table)


def save_to_file():
    with open("log.txt", "a") as f:
        current_time = datetime.now().strftime("%d/%m/%Y, %H:%M:%S")
        f.write(f"\ncurrent time:{current_time}\n")
        f.write(table.get_string())


if __name__ == "__main__":
    my_devices = read_device_config_file("deviceconfig.json")
    print(my_devices)
    my_property = read_param_file("readparameter.json", my_devices)
    my_write_property = read_param_file("writeparameter.json", my_devices)
    add_table()
    print(f"write property :{my_write_property}")
    for current_device in my_devices:
        print(f"current device {current_device}\n")
        current_device_property = my_write_property[current_device]
        for key in current_device_property:
            nobytes = current_device_property[key]["bytes"]
            uptype = current_device_property[key]["up_typ"]
            data = current_device_property[key]["data"]
            print(f"key :{key} no of bytes :{nobytes} type :{uptype} input data :{data}\n")
            write_data = convert_data(data, int(nobytes / 2), uptype)
    while 1:
        run_client(my_devices, my_write_property, read=False)
        run_client(my_devices, my_property, read=True)
        update_value()
        save_to_file()
        time.sleep(30)
