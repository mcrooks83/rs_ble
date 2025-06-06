import asyncio
from bleak import BleakScanner, BleakClient

import dbus
import time
import logging
import numpy as np

from mmap import mmap, ACCESS_READ
import struct
from math import sqrt
#logging.basicConfig(level=logging.DEBUG)

# UUIDs for custom service and characteristic

'''
#define MEASUREMENT_SERVICE_UUID         "f8300001-67d2-4b32-9a68-5f3d93a8b6a5"
#define MEASUREMENT_NOTIFY_CHAR_UUID     "f8300002-67d2-4b32-9a68-5f3d93a8b6a5"
#define SENSOR_CONTROL_CHAR_UUID         "f8300003-67d2-4b32-9a68-5f3d93a8b6a5"
#define SENSOR_IDENTIFY_CHAR_UUID        "f8300004-67d2-4b32-9a68-5f3d93a8b6a5"

rs load sensor
#define MEASUREMENT_SERVICE_UUID         "f9300001-67d2-4b32-9a68-5f3d93a8b6a5"
#define MEASUREMENT_NOTIFY_CHAR_UUID     "f9300002-67d2-4b32-9a68-5f3d93a8b6a5"
#define SENSOR_IDENTIFY_CHAR_UUID        "f9300004-67d2-4b32-9a68-5f3d93a8b6a5"

vag sensor
#define MEASUREMENT_SERVICE_UUID         "f8300001-67d2-4b32-9a68-5f3d93a8b6a5"
#define MEASUREMENT_NOTIFY_CHAR_UUID     "f8300002-67d2-4b32-9a68-5f3d93a8b6a5"
#define SENSOR_IDENTIFY_CHAR_UUID        "f8300004-67d2-4b32-9a68-5f3d93a8b6a5"

'''
MEASUREMENT_SERVICE_UUID     =  "f8300001-67d2-4b32-9a68-5f3d93a8b6a5"
MEASUREMENT_CHAR_UUID        =  "f8300002-67d2-4b32-9a68-5f3d93a8b6a5"
SENSOR_IDENTIFY_CHAR_UUID    =  "f8300004-67d2-4b32-9a68-5f3d93a8b6a5"

#NORDIC_UART_SERVICE_UUID     =  "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

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

# acc = g  conversion 0.000488f
# gyro = degrees per second - conversion 0.061035 
def parse_packet(packet):
    # One sample: 6 signed 16-bit ints (gyro + accel)
    sample_format = "<hhhhhh"  # GyroX, GyroY, GyroZ, AccX, AccY, AccZ
    sample = struct.unpack(sample_format, packet)  # returns a tuple of 6 ints

    decoded_packet = np.array(sample)
    formatted_data = format_packet(decoded_packet)

    return formatted_data

def format_packet(decoded_packet):
        acc_conversion = 0.00048828125 # 4g conversion factor
        gyr_converstion = 0.06103515625
        formatted_data = []

        if(len(decoded_packet) == 6): # contains gryo and accel
            # Apply conversion factor to x, y, z
            sample = {
                "accel_x": decoded_packet[3]*acc_conversion,
                "accel_y": decoded_packet[4]*acc_conversion,
                "accel_z": decoded_packet[5]*acc_conversion,
                "gyr_x": decoded_packet[0]*gyr_converstion,
                "gyr_y": decoded_packet[1]*gyr_converstion,
                "gyr_z": decoded_packet[2]*gyr_converstion
            }
            # Add the converted sample to the list
            formatted_data.append(sample)
        return formatted_data # dont do any formatting yet

def on_data(sender: int, data: bytearray):
    #print(f"{data}", {len(data)})
    _packet = parse_packet(data)
    print(_packet)

def on_vag_data(sender: int, data: bytearray):
    print(data)

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
        await asyncio.sleep(2)
        # Subscribe to a characteristic (you need to know UUID)
        # For example, subscribe to notifications (replace with correct UUID)
        #await client.start_notify(YOUR_CHARACTERISTIC_UUID, your_callback)
        
        # Subscribe to notifications
        #MEASUREMENT_NOTIFY_CHAR_UUID NORDIC_UART_SERVICE_UUID
        await client.start_notify(MEASUREMENT_CHAR_UUID, on_vag_data)
        print("Subscribed to data")

        # try and indentify the device
        #response = await client.write_gatt_char(SENSOR_IDENTIFY_CHAR_UUID, bytearray([0x01]))
        # Simulate LED flashing logic
        #print("LED should be flashing now!")
        await asyncio.sleep(2)

        await client.write_gatt_char(MEASUREMENT_CHAR_UUID, bytearray([0x04]))  # asks to stream all axes and magnitude
        #await client.write_gatt_char(MEASUREMENT_CHAR_UUID, bytearray([0x02]))  # asks to stream  magnitude
        #await client.write_gatt_char(MEASUREMENT_CHAR_UUID, bytearray([0x03]))  # asks to stream  X
        #await client.write_gatt_char(MEASUREMENT_CHAR_UUID, bytearray([0x04]))  # asks to stream  Y
        #await client.write_gatt_char(MEASUREMENT_CHAR_UUID, bytearray([0x05]))  # asks to stream  Z
        print("Streaming started")
    
        # Wait while data streams — non-blocking sleep
        await asyncio.sleep(10)

        # Stop streaming
        print("stopping the stream")
        await client.write_gatt_char(MEASUREMENT_CHAR_UUID, bytearray([0x00]))
        #print("Streaming stopped")

        await client.disconnect()


# Callback for receiving notifications

# Main function to discover and connect
async def main():
    remove_all_devices()
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
