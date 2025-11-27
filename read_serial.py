import serial
import csv


SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
FILE_NAME = 'adc_data.csv'

print(f"Waiting for data stream on {SERIAL_PORT}...")

try:
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2) as ser:
        with open(FILE_NAME, mode='w', newline='') as file_csv:
            writer = csv.writer(file_csv)

            writer.writerow(["Index", "ADC_raw"]) 

            while True:
                line = ser.readline().decode('utf-8').strip()
                
                # If the ending line has been read, stop
                if "*" in line:
                    break 
                
                # Check if the line exists and is consistent 
                if line and "," in line: 
                    vals = line.split(',')
                    if len(vals) == 2:
                        try:
                            idx = int(vals[0])
                            adc = int(vals[1])
                            
                            writer.writerow([idx, adc])
                        except ValueError:
                            pass

except serial.SerialException:
    print(f"ERROR: Failed to open port: {SERIAL_PORT}")
except KeyboardInterrupt:
    print("\nUser keyboard interrupted.")

print(f"Data saved in: {FILE_NAME}")