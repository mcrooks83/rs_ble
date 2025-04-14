import asyncio
from bleak import BleakScanner, BleakClient

import dbus
import time
import logging
#logging.basicConfig(level=logging.DEBUG)

# UUIDs for custom service and characteristic

'''
#define MEASUREMENT_SERVICE_UUID         "f8300001-67d2-4b32-9a68-5f3d93a8b6a5"
#define MEASUREMENT_NOTIFY_CHAR_UUID     "f8300002-67d2-4b32-9a68-5f3d93a8b6a5"
#define SENSOR_CONTROL_CHAR_UUID         "f8300003-67d2-4b32-9a68-5f3d93a8b6a5"
#define SENSOR_IDENTIFY_CHAR_UUID        "f8300004-67d2-4b32-9a68-5f3d93a8b6a5"

'''
BATTERY_SERVICE_UUIID     =  "0000180F-0000-1000-8000-00805F9B34FB"
BATTERY_SERVICE_CHAR_ID   =  "00002A19-0000-1000-8000-00805F9B34FB"



# Function to discover the device by name

def match_devices(names, devices):
    matching_devices = [
            (device, adv_data)
            for _, (device, adv_data) in devices.items()  # <- Unpack properly here
            if adv_data.local_name in names
        ]
    return matching_devices

def remove_all_devices():
    bus = dbus.SystemBus()
    obj_manager = bus.get_object("org.bluez", "/")
    manager = dbus.Interface(obj_manager, "org.freedesktop.DBus.ObjectManager")

    devices_removed = 0

    for path, interfaces in manager.GetManagedObjects().items():
        if "org.bluez.Device1" in interfaces:
            device = bus.get_object("org.bluez", path)
            adapter_path = "/".join(path.split("/")[:4])  # Extract adapter path (e.g., /org/bluez/hci0)
            adapter = bus.get_object("org.bluez", adapter_path)
            adapter_interface = dbus.Interface(adapter, "org.bluez.Adapter1")

            try:
                adapter_interface.RemoveDevice(path)
                print(f"Removed {path}, cache cleared.")
                devices_removed += 1
            except dbus.exceptions.DBusException as e:
                print(f"Failed to remove {path}: {e}")

    if devices_removed == 0:
        print("No devices found to remove.")

async def discover_device_by_name(target_name: str):
    devices = await BleakScanner.discover(return_adv=True)
    device = match_devices([target_name], devices)
    print(device)
    return device
    

# Callback to handle notifications from the notify characteristic
def on_batt(sender, data):
    battery_level = int(data[0])
    print(f"Battery Level: {battery_level}%")
# Connect and subscribe to notifications
async def connect_device(device):
    async with BleakClient(device) as client:
        print(f"Connected to {device}")
        _ = client.services  # triggers discovery
        services = await client.get_services()
        for s in services:
            print(s)
            for characteristic in s.characteristics:
                print(f"    Characteristic: {characteristic.uuid} - {characteristic.properties}")
                print(f"UUID: {characteristic.uuid} | Notify: {'notify' in characteristic.properties}")
        await asyncio.sleep(2)
        # Subscribe to a characteristic (you need to know UUID)
        # For example, subscribe to notifications (replace with correct UUID)
        #await client.start_notify(YOUR_CHARACTERISTIC_UUID, your_callback)
        
        # Subscribe to notifications
        await client.start_notify(BATTERY_SERVICE_CHAR_ID, on_batt)
        print("Subscribed to battery")

        print("try a batt read")
        level = await client.read_gatt_char(BATTERY_SERVICE_CHAR_ID)
        print("Battery Level (manual read):", int(level[0]))

        # try and indentify the device
        #response = await client.write_gatt_char(SENSOR_IDENTIFY_CHAR_UUID, bytearray([0x01]))
        # Simulate LED flashing logic
        #print("LED should be flashing now!")
        await asyncio.sleep(30)

        await client.disconnect()


# Callback for receiving notifications

# Main function to discover and connect
async def main():
    #devices = await discover_device_by_name("SekMo B")
    devices = await discover_device_by_name("RS_VAG")
    device_address = ""
    for device, advertisement_data in devices:
        device_address = device.address
        print(f"Device: {device}")
        print(f"Local Name: {advertisement_data.local_name}")
        print(f"Address: {device.address}")
        print("------")

    if device:
        await connect_device(device_address)
    else:
        print("Device not found")

# Run the main function
asyncio.run(main())
