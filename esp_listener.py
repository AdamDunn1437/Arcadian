#!/usr/bin/env python3
import serial
import os

# Adjust this port as needed (often /dev/ttyACM0 or /dev/ttyUSB0)
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
except Exception as e:
    print("Failed to open serial port {}: {}".format(SERIAL_PORT, e))
    exit(1)

print("Listening for ESP32 trigger signals on {}...".format(SERIAL_PORT))

while True:
    try:
        # Read a line from the serial port
        line = ser.readline().decode('utf-8').strip()
        if line == "TRIGGER":
            print("ESP32 signal received")
            # Optionally, trigger the iglooh.py script
            os.system('python3 /home/pi/iglooh.py')
    except Exception as e:
        print("Error reading from serial: ", e)
