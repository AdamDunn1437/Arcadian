#!/usr/bin/env python3
import uRAD_USB_SDK11        # import uRAD library  
import serial
from time import time, sleep
from datetime import datetime

# True if USB, False if UART
usb_communication = True

# Input parameters
mode = 1                    # doppler mode
f0 = 125                    # output continuous frequency 24.125 GHz
BW = 240                    # not applied in doppler mode (mode = 1)
Ns = 200                    # 200 samples
Ntar = 3                    # 3 targets of interest
Vmax = 75                   # search full velocity range
MTI = 0                     # MTI disabled so static and moving targets are reported
Mth = 0                     # parameter not used since "movement" is not requested
Alpha = 20                  # signal must be 20 dB above the surrounding noise
distance_true = False       # mode 1 does not provide distance info
velocity_true = True        # request velocity information
SNR_true = True             # request Signal-to-Noise-Ratio info
I_true = False              # do not request In-Phase component (RAW data)
Q_true = False              # do not request Quadrature component (RAW data)
movement_true = False       # not interested in boolean movement detection

print('Connecting with uRAD...')

# Serial Port configuration
ser = serial.Serial()
if usb_communication:
    ser.port = '/dev/ttyACM0'
    ser.baudrate = 1e6
else:
    ser.port = '/dev/serial0'
    ser.baudrate = 115200

print('uRAD connected')

# Sleep time (seconds) between iterations
timeSleep = 5e-3

# Other serial parameters
ser.bytesize = serial.EIGHTBITS
ser.parity = serial.PARITY_NONE
ser.stopbits = serial.STOPBITS_ONE

# Method to correctly turn OFF and close uRAD
def closeProgram():
    return_code = uRAD_USB_SDK11.turnOFF(ser)
    if return_code != 0:
        exit()

# Open serial port
try:
    print('Opening serial port...')
    ser.open()
except Exception as e:
    print('Error opening serial port. Closing program.')
    closeProgram()

# Switch ON uRAD
print('Switching ON uRAD...')
return_code = uRAD_USB_SDK11.turnON(ser)
if return_code != 0:
    closeProgram()

if not usb_communication:
    sleep(timeSleep)

# Load configuration into uRAD
return_code = uRAD_USB_SDK11.loadConfiguration(ser, mode, f0, BW, Ns, Ntar, Vmax, MTI, Mth, Alpha,
                                                distance_true, velocity_true, SNR_true, I_true, Q_true, movement_true)
if return_code != 0:
    closeProgram()

if not usb_communication:
    sleep(timeSleep)

def run_radar(duration=30):
    """
    Runs the radar for the specified duration (seconds). During this period,
    detection data is collected and at the end the data is saved locally in a CSV file.
    """
    print("\nRadar triggered: Running for {} seconds.".format(duration))
    collected_data = []
    header = "Timestamp,Target,Velocity (m/s),SNR (dB)"
    collected_data.append(header)
    
    start_time = time()
    while time() - start_time < duration:
        # Request a detection measurement
        return_code, results, raw_results = uRAD_USB_SDK11.detection(ser)
        if return_code != 0:
            closeProgram()

        # Extract detection results
        NtarDetected = results[0]
        velocity = results[2]
        SNR = results[3]
        
        # Log current time for this measurement
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        # Record each detected target with SNR > 0
        for i in range(NtarDetected):
            if SNR[i] > 0:
                data_line = "{},{},{:.1f},{:.1f}".format(timestamp, i+1, velocity[i], SNR[i])
                collected_data.append(data_line)
                print("Target: {} | Timestamp: {} | Velocity: {:.1f} m/s | SNR: {:.1f} dB".format(i+1, timestamp, velocity[i], SNR[i]))
        
        if NtarDetected > 0:
            print("")
        
        sleep(timeSleep)
    
    # Save collected data to a CSV file with a timestamped filename
    epoch_time = int(time())
    filename = "radar_{}.csv".format(epoch_time)
    with open(filename, "w") as f:
        for line in collected_data:
            f.write(line + "\n")
    print("Radar data saved to {}\n".format(filename))

def main():
    run_radar(duration=30)
    closeProgram()

if __name__ == "__main__":
    main()
