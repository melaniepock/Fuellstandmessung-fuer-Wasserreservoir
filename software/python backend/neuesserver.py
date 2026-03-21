import serial
import time
import csv
import re

SERIAL_PORT = 'COM4'
CSV_FILE = "newTestdata.csv"

def log_radar_data(data):
    ts_unix = time.time()
    with open(CSV_FILE, mode='a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f, delimiter=";")
        row = [ts_unix] + data
        writer.writerow(row)

if __name__ == "__main__":
    ser = serial.Serial(SERIAL_PORT, 115200, timeout=8)
    ser.flushInput()
    
    floats = []
    while len(floats) < 100:
        line = ser.readline().decode().strip()
        match = re.search(r'\d+:\s*([\d.]+)', line)
        if match:
            floats.append(float(match.group(1)))
    
    log_radar_data(floats)
    ser.close()





