'''
Docstring for Radar.V-LD1_modified
modified version of V-LD1-readout_example.py

only reads PDAT
'''

import serial
import struct
import time
from collections import namedtuple

import csv
from datetime import datetime

import pandas as pd

# Serial communication settings
SERIAL_PORT = 'COM3'  # Replace with your serial port
CSV_FILE = 'radar_data.csv'

# Define the bit field:
# Bit 0 (RADC)
# Bit 1 (RFFT)
# Bit 2 (PDAT)
# Bit 5 (DONE)
# Hier: nur PDAT + DONE aktiv
bit_field = 0b00100100

# Global serial object
ser = None

csv_header_written = False

# RadarParameters 
RadarParameters = namedtuple("RadarParameters", [
    "firmware_version",
    "unique_id",
    "distance_range",
    "threshold_offset",
    "minimum_range_filter",
    "maximum_range_filter",
    "distance_avg_count",
    "target_filter",
    "distance_precision",
    "tx_power",
    "chirp_integration_count",
    "short_range_distance_filter"
])

def write_csv_header():
    """Schreibt CSV-Header falls noch nicht vorhanden"""
    global csv_header_written
    with open(CSV_FILE, mode='a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp_unix', 'frame_number', 'distance_m', 'magnitude_dB'])
    csv_header_written = True

def log_radar_data(frame_number, pdat_data):
    """Loggt PDAT-Daten mit Timestamp in CSV"""
    # ts_iso = datetime.now().isoformat(timespec='milliseconds')
    ts_unix = time.time()
    
    with open(CSV_FILE, mode='a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        for target in pdat_data:
            writer.writerow([ts_unix, frame_number, target['Distance'], target['Magnitude']])


def open_serial_connection():
    """
    Opens a serial connection to the sensor.
    """
    global ser
    ser = serial.Serial(SERIAL_PORT, 115200, 8, 'E', 1, timeout=1)


def close_serial_connection():
    """
    Closes the serial connection to the sensor.
    """
    global ser
    if ser:
        ser.close()


def send_command(header, payload=b''):
    """
    Sends a command to the sensor with an optional payload.

    Args:
        header (str): The command header.
        payload (bytes): The payload data (default is an empty payload).

    Returns:
        int: The error code (0 for success, non-zero for error).
    """
    global ser
    if not ser:
        open_serial_connection()

    # Create the command packet
    command_packet = header.encode()
    payload_length = len(payload)
    payload_length_bytes = struct.pack('<I', payload_length)
    command_packet += payload_length_bytes
    command_packet += payload

    # Send the command to the sensor
    ser.write(command_packet)

    # Read the RESP message from the sensor
    response_header, response_payload = read_message()

    if response_header == 'RESP':
        error_code = struct.unpack('B', response_payload)[0]
        if error_code != 0:
            error_description = get_error_description(error_code)
            print(f"ERROR: Response error code {error_code} ({error_description})")
        return error_code
    else:
        return None


def read_message():
    """
    Reads a message from the sensor.

    Returns:
        tuple: A tuple containing the header (str) and payload (bytes) of the message.
    """
    global ser
    try:
        # Read the header (4 bytes)
        header_bytes = ser.read(4)

        if len(header_bytes) != 4:
            print("ERROR: Invalid header length received.")
            return None, None

        header = header_bytes.decode()

        # Read the payload length (4 bytes, little endian)
        payload_length_bytes = ser.read(4)

        if len(payload_length_bytes) != 4:
            print("ERROR: Invalid payload length received.")
            return None, None

        payload_length = struct.unpack('<I', payload_length_bytes)[0]

        # Read the payload based on the payload length
        payload = ser.read(payload_length)

        return header, payload

    except serial.SerialTimeoutException:
        print("ERROR: Timeout occurred while reading message.")
        return None, None


def get_error_description(error_code):
    """
    Gets the description of an error code.

    Args:
        error_code (int): The error code.

    Returns:
        str: The error description.
    """
    error_descriptions = {
        0: "OK, no error",
        1: "Unknown command",
        2: "Invalid parameter value",
        3: "Invalid RPST version",
        4: "UART error (parity, framing, noise)",
        5: "No calibration values",
        6: "Timeout",
        7: "Application corrupt or not programmed"
    }
    return error_descriptions.get(error_code, "Unknown Error")


def initialize_sensor(baud_rate=0):
    """
    Initializes the sensor with a specified baud rate.

    Args:
        baud_rate (int): The baud rate to use for communication.
            Possible values:
                0 = 115200
                1 = 460800
                2 = 921600
                3 = 2000000

    Returns:
        bool: True if initialization is successful, False otherwise.
    """
    valid_baud_rates = [115200, 460800, 921600, 2000000]

    if baud_rate not in {0, 1, 2, 3}:
        print("Invalid baud rate index specified.")
        return False

    selected_baud_rate = valid_baud_rates[baud_rate]  # Get the selected baud rate

    # Reset the baud rate of the serial interface to default
    ser.baudrate = 115200

    # Flush the input buffer
    ser.flushInput()

    header = 'INIT'
    payload = struct.pack('B', baud_rate)
    error_code = send_command(header, payload)

    if error_code == 0:
        print(f"OK: Initialization successful at {selected_baud_rate} baud")

        # Read and handle the VERS message
        response_header, response_payload = read_message()
        if response_header == 'VERS':
            firmware_version = response_payload.decode('utf-8').strip('\x00')
            print(f"OK: Firmware version: {firmware_version}")
        else:
            print("ERROR: Received unexpected message during firmware version retrieval:", response_header)

        # Set the baud rate of the serial interface
        ser.baudrate = selected_baud_rate

        return True
    else:
        print(f"ERROR: Initialization failed. Error Code {error_code}")
        return False


def disconnect_sensor():
    """
    Disconnects from the sensor.
    """
    header = 'GBYE'
    error_code = send_command(header)

    if error_code == 0:
        print("OK: Disconnected from sensor.")



def send_gnfd_command(bit_field, timeout_seconds=1):
    """
    Sends a GNFD (Get Next Frame Data) command to the sensor to enable or disable specific data packets and
    retrieves the requested data packets once available.

    Args:
        bit_field (int): Bit 2 (PDAT) und Bit 5 (DONE) werden hier genutzt.
        timeout_seconds (float): The maximum time (in seconds) to wait for all enabled packets.

    Returns:
        dict: A dictionary containing the requested data packets as keys and their respective data as values.
              Possible keys include 'PDAT' and 'DONE'.
    """
    # Mask out all don't care bits
    bit_field &= 0b00100111

    header = 'GNFD'
    payload = struct.pack('B', bit_field)
    send_command(header, payload)

    # Initialize the response dictionary
    response_data = {}

    # Track received packets
    received_packets = 0

    # Get the current time
    start_time = time.time()

    # Receive and parse all enabled packets
    while received_packets != bit_field:

        # Read the response header and payload
        response_header, response_payload = read_message()

        if response_header == 'PDAT' and (bit_field & 0x04):
            # Parse PDAT data (variable number of targets)
            pdat_data = []
            while len(response_payload) >= 6:
                distance = struct.unpack('<f', response_payload[:4])[0]
                magnitude = struct.unpack('<H', response_payload[4:6])[0] / 100.0
                pdat_data.append({'Distance': distance, 'Magnitude': magnitude})
                response_payload = response_payload[6:]
            response_data['PDAT'] = pdat_data
            received_packets |= 0x04

        elif response_header == 'DONE' and (bit_field & 0x20):
            # Parse DONE data (frame number)
            frame_number = struct.unpack('<I', response_payload)[0]
            response_data['DONE'] = frame_number
            received_packets |= 0x20

        # Check if the timeout has been reached
        elapsed_time = time.time() - start_time
        if elapsed_time > timeout_seconds:
            print("ERROR: Timeout occurred while waiting for data packets.")
            break

    return response_data


if __name__ == "__main__":
    open_serial_connection()  # Open serial connection to the sensor

    write_csv_header()

    # Example usage:
    if initialize_sensor(baud_rate=3):  # Initialize sensor with 2M baud rate

        print("Press CTRL+C to stop data acquisition...")
        print("-----------------------------------------------------------------------------")

        try:
            while True:
                # Send GNFD and read the responses
                response_data = send_gnfd_command(bit_field)

                if 'PDAT' in response_data:
                    pdat_data = response_data['PDAT']

                    if len(pdat_data) != 0:
                        for target in pdat_data:
                            distance = target['Distance']
                            magnitude = target['Magnitude']
                            print(f"OK: Received PDAT data - Distance: {distance:.3f} m, Magnitude: {magnitude} dB")
                    else:
                        print("No targets found during measurement")

                    # Process PDAT data (variable number of targets)

                if 'DONE' in response_data:
                    frame_number = response_data['DONE']
                    log_radar_data(frame_number, pdat_data)
                    print("OK: Received DONE data - Frame Number:", frame_number)
                    # Process DONE data (frame number)

                time.sleep(0.5)  # Adjust the interval as needed

        except KeyboardInterrupt:
            print("-----------------------------------------------------------------------------")
            print("Stopping data acquisition...")

        print("Konvertiere CSV zu Excel...")
        try:
            df = pd.read_csv(CSV_FILE)
            excel_file = CSV_FILE.replace('.csv', '.xlsx')
            df.to_excel(excel_file, index=False, sheet_name='Radar Data')
            print(f"Excel-Datei erstellt: {excel_file}")
        except Exception as e:
            print(f"Excel-Export fehlgeschlagen: {e}")

        # Disconnect from the sensor
        time.sleep(0.1)
        disconnect_sensor()
    else:
        print("ERROR: Failed to initialize sensor.")

    close_serial_connection()  # Close serial connection
