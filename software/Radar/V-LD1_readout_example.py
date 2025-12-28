"""
Company: RFbeam Microwave GmbH
Author: Ueli Giger
Description: This script demonstrates communication with the V-LD1 radar sensor and provides configurability options.
Version: 1.0
Date: September 20, 2023

Before running the script, please ensure the following requirements are met:
- Python distribution: This script is designed to work with Python 3. It has been tested with Python 3.7 and above.
- Required packages: Install the necessary Python packages by running the following commands:
  1. Install Matplotlib (for plotting data):
     pip install matplotlib
  2. Install pyserial (for serial communication):
     pip install pyserial

To run the script from the command line:
python V-LD1_readout_example.py

Ensure that you have connected the V-LD1 radar sensor to your computer and specified the correct serial port (SERIAL_PORT variable).

To configure which data packets to enable or disable, use the `bit_field` variable. Here's how to work with it:
- Bit 0 (RADC): Set to 1 to enable the RADC (Raw ADC values) data packet.
- Bit 1 (RFFT): Set to 1 to enable the RFFT (Raw FFT) data packet.
- Bit 2 (PDAT): Set to 1 to enable the PDAT (Detected target) data packet.
- Bit 5 (DONE): Set to 1 to enable the DONE (Frame Done) data packet.
- Bits 3-4 and Bits 6-7 are reserved (don't care).

Press CTRL+C to stop data acquisition.
"""

import serial
import struct
import time
from collections import namedtuple
import matplotlib.pyplot as plt

# Serial communication settings
SERIAL_PORT = 'COM6'  # Replace with your serial port

# Define the bit field to enable RADC, RFFT, PDAT, and DONE
bit_field = 0b00100111  # Change to your needs

# Global serial object
ser = None  

# Define the RadarParameters named tuple
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

# Create figures for RADC and RFFT plots (conditionally)
radc_fig = radc_ax = None
rfft_fig = rfft_ax = None

# Common functions
def open_figures():
   """
   Opens Matplotlib figures for RADC and RFFT plots based on the enabled data packets.

   The function checks the bit_field to determine which data packets are enabled (RADC and RFFT)
   and opens corresponding Matplotlib figures for plotting the data. These figures are created and
   made interactive for real-time updates.

   Global Variables:
       radc_fig (Figure): A Matplotlib figure for RADC data plotting.
       radc_ax (Axes): Matplotlib axes for RADC data plotting.
       rfft_fig (Figure): A Matplotlib figure for RFFT data plotting.
       rfft_ax (Axes): Matplotlib axes for RFFT data plotting.
   """
   global radc_fig, radc_ax, rfft_fig, rfft_ax  # Declare these as global variables

   if bit_field & 0b00000001:  # Check if RADC packet is enabled
       radc_fig, radc_ax = plt.subplots()
       plt.ion()
       plt.show()
   if bit_field & 0b00000010:  # Check if RFFT packet is enabled
       rfft_fig, rfft_ax = plt.subplots()
       plt.ion()
       plt.show()

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
    
    # Print the command being sent
    # print(f"Sending cmd: Header:{header}, Payload Length: {payload_length}, Payload (HEX):{payload.hex()}")    

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

# Functions to access sensor parameters

def set_distance_range(distance_range):
    """
    Sets the distance range for the sensor.

    Args:
        distance_range (int): The distance range to set.
            Possible values:
                0 = 20m
                1 = 50m
    """
    if distance_range not in {0, 1}:
        print("ERROR: Invalid distance range specified.")
        return

    header = 'RRAI'
    payload = struct.pack('B', distance_range)
    send_command(header, payload)

def set_threshold_offset(threshold_offset):
    """
    Sets the threshold offset for the sensor.

    Args:
        threshold_offset (int): The threshold offset to set (dB).
            Minimum: 20 dB
            Maximum: 90 dB
    """
    if not (20 <= threshold_offset <= 90):
        print("ERROR: Invalid threshold offset specified.")
        return

    header = 'THOF'
    payload = struct.pack('B', threshold_offset)
    send_command(header, payload)

def set_min_range_filter(min_range_filter):
    """
    Sets the minimum range filter for the sensor.

    Args:
        min_range_filter (int): The minimum range filter to set (number of bins).
            Minimum: 1
            Maximum: 510
    """
    if not (1 <= min_range_filter <= 510):
        print("ERROR: Invalid minimum range filter specified.")
        return

    header = 'MIRA'
    payload = struct.pack('<H', min_range_filter)
    send_command(header, payload)

def set_max_range_filter(max_range_filter):
    """
    Sets the maximum range filter for the sensor.

    Args:
        max_range_filter (int): The maximum range filter to set (number of bins).
            Minimum: 2
            Maximum: 511
    """
    if not (2 <= max_range_filter <= 511):
        print("ERROR: Invalid maximum range filter specified.")
        return

    header = 'MARA'
    payload = struct.pack('<H', max_range_filter)
    send_command(header, payload)

def set_distance_avg_count(avg_count):
    """
    Sets the distance average count for the sensor.

    Args:
        avg_count (int): The distance average count to set.
            Minimum: 1
            Maximum: 255
    """
    if not (1 <= avg_count <= 255):
        print("ERROR: Invalid distance average count specified.")
        return

    header = 'RAVG'
    payload = struct.pack('B', avg_count)
    send_command(header, payload)

def set_target_filter(target_filter):
    """
    Sets the target filter for the sensor.

    Args:
        target_filter (int): The target filter to set.
            Possible values:
                0 = Strongest first
                1 = Nearest first
                2 = Farthest first
    """
    if target_filter not in {0, 1, 2}:
        print("ERROR: Invalid target filter specified.")
        return

    header = 'TGFI'
    payload = struct.pack('B', target_filter)
    send_command(header, payload)

def set_distance_precision(precision_mode):
    """
    Sets the distance precision mode for the sensor.

    Args:
        precision_mode (int): The precision mode to set.
            Possible values:
                0 = Low precision
                1 = High precision
    """
    if precision_mode not in {0, 1}:
        print("ERROR: Invalid precision mode specified.")
        return

    header = 'PREC'
    payload = struct.pack('B', precision_mode)
    send_command(header, payload)

def set_tx_power(tx_power):
    """
    Sets the TX power for the sensor.

    Args:
        tx_power (int): The TX power to set.
            Minimum: 0 (Minimum output power)
            Maximum: 31 (Maximum output power)
    """
    if not (0 <= tx_power <= 31):
        print("ERROR: Invalid TX power specified.")
        return

    header = 'TXPW'
    payload = struct.pack('B', tx_power)
    send_command(header, payload)

def set_chirp_integration_count(integration_count):
    """
    Sets the chirp integration count for the sensor.

    Args:
        integration_count (int): The chirp integration count to set.
            Minimum: 1
            Maximum: 100
    """
    if not (1 <= integration_count <= 100):
        print("ERROR: Invalid chirp integration count specified.")
        return

    header = 'INTN'
    payload = struct.pack('B', integration_count)
    send_command(header, payload)

def set_short_range_distance_filter(enable_filter):
    """
    Sets the short range distance filter for the sensor.

    Args:
        enable_filter (bool): True to enable the short range distance filter, False to disable it.
    """
    header = 'SRDF'
    payload = struct.pack('B', enable_filter)
    send_command(header, payload)

def read_radar_parameters():
    """
    Reads and parses the radar parameter structure into a named tuple.

    Returns:
        RadarParameters: A named tuple containing radar parameters.
    """
    header = 'GRPS'
    send_command(header)

    # Read the RPST message for radar parameter structure
    response_header, response_payload = read_message()

    if response_header == 'RPST':
        radar_params = parse_radar_parameters(response_payload)
        print("-----------------------------------------------------------------------------")        
        print("Radar Parameters:")
        print(f"Firmware Version: {radar_params.firmware_version}")
        print(f"Unique ID: {radar_params.unique_id}")
        print(f"Distance Range: {radar_params.distance_range} (0 = 20m, 1 = 50m)")
        print(f"Threshold Offset: {radar_params.threshold_offset} dB")
        print(f"Minimum Range Filter: {radar_params.minimum_range_filter} bins")
        print(f"Maximum Range Filter: {radar_params.maximum_range_filter} bins")
        print(f"Distance Average Count: {radar_params.distance_avg_count}")
        print(f"Target Filter: {radar_params.target_filter} (0 = Strongest first, 1 = Nearest first, 2 = Farthest first)")
        print(f"Distance Precision Mode: {radar_params.distance_precision} (0 = Low precision, 1 = High precision)")
        print(f"TX Power: {radar_params.tx_power}")
        print(f"Chirp Integration Count: {radar_params.chirp_integration_count}")
        print(f"Short Range Distance Filter: {radar_params.short_range_distance_filter}")
        print("-----------------------------------------------------------------------------")        
        return radar_params
    else:
        print("ERROR: Failed to retrieve radar parameters.")
        return None

def parse_radar_parameters(payload):
    """
    Parses the radar parameter structure payload into a named tuple.

    Args:
        payload (bytes): The payload containing radar parameters.

    Returns:
        RadarParameters: A named tuple containing radar parameters.
    """
    radar_parameters = RadarParameters(
        firmware_version=payload[:19].decode('utf-8').strip('\x00'),
        unique_id=payload[19:31].decode('utf-8').strip('\x00'),
        distance_range=struct.unpack('B', payload[31:32])[0],
        threshold_offset=struct.unpack('B', payload[32:33])[0],
        minimum_range_filter=struct.unpack('<H', payload[33:35])[0],
        maximum_range_filter=struct.unpack('<H', payload[35:37])[0],
        distance_avg_count=struct.unpack('B', payload[37:38])[0],
        target_filter=struct.unpack('B', payload[38:39])[0],
        distance_precision=struct.unpack('B', payload[39:40])[0],
        tx_power=struct.unpack('B', payload[40:41])[0],
        chirp_integration_count=struct.unpack('B', payload[41:42])[0],
        short_range_distance_filter=bool(struct.unpack('B', payload[42:43])[0])
    )
    return radar_parameters

def send_gnfd_command(bit_field, timeout_seconds=1):
    """
    Sends a GNFD (Get Next Frame Data) command to the sensor to enable or disable specific data packets and
    retrieves the requested data packets once available.

    Args:
        bit_field (int): An 8-bit binary-coded bit-field for enabling or disabling data packets.
            Bit 0 (RADC): Set to 1 to enable the RADC (Raw ADC values) data packet.
            Bit 1 (RFFT): Set to 1 to enable the RFFT (Raw FFT) data packet.
            Bit 2 (PDAT): Set to 1 to enable the PDAT (Detected target) data packet.
            Bit 3-4: Reserved (don't care).            
            Bit 5 (DONE): Set to 1 to enable the DONE (Frame Done) data packet.
            Bit 6-7: Reserved (don't care).            
        timeout_seconds (float): The maximum time (in seconds) to wait for all enabled packets.

    Returns:
        dict: A dictionary containing the requested data packets as keys and their respective data as values.
            Possible keys include 'RADC', 'RFFT', 'PDAT', and 'DONE'.
            The structure of the returned data varies depending on the enabled data packets.
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

        if response_header == 'RADC':
            # Parse RADC data
            adc_data = struct.unpack('<1024h', response_payload)
            response_data['RADC'] = adc_data
            received_packets |= 0x01
        elif response_header == 'RFFT' and (bit_field & 0x02):
            # Parse RFFT data
            spectrum_data = struct.unpack('<512H', response_payload[:1024])
            threshold_data = struct.unpack('<512H', response_payload[1024:])
            response_data['RFFT'] = {
                'Spectrum': [x / 100.0 for x in spectrum_data],
                'Threshold': [x / 100.0 for x in threshold_data]
            }
            received_packets |= 0x02
        elif response_header == 'PDAT':
            # Parse PDAT data (variable number of targets)
            pdat_data = []
            while len(response_payload) >= 6:
                distance = struct.unpack('<f', response_payload[:4])[0]
                magnitude = struct.unpack('<H', response_payload[4:6])[0] / 100.0
                pdat_data.append({'Distance': distance, 'Magnitude': magnitude})
                response_payload = response_payload[6:]
            response_data['PDAT'] = pdat_data
            received_packets |= 0x04
        elif response_header == 'DONE':
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

# Main function example
if __name__ == "__main__":
    open_serial_connection()  # Open serial connection to the sensor

    # Example usage:
    if initialize_sensor(baud_rate=3):  # Initialize sensor with 2M baud rate
        set_distance_range(0)  # Set distance range to 20m
        set_threshold_offset(50)  # Set threshold offset to 50 dB
        set_min_range_filter(6)  # Set minimum range filter to 6 bins
        set_max_range_filter(400)  # Set maximum range filter to 400 bins
        set_distance_avg_count(2)  # Set distance average count to 2
        set_target_filter(0)  # Set target filter to Strongest first
        set_distance_precision(1)  # Set distance precision to High precision
        set_tx_power(31)  # Set TX power to 31
        set_chirp_integration_count(1)  # Set chirp integration count to 11
        set_short_range_distance_filter(False)  # Enable short range distance filter

        # Read and print radar parameters
        radar_parameters = read_radar_parameters()
        
        # Open figures conditionally based on bit_field
        open_figures()

        print("Press CTRL+C to stop data acquisition...")
        print("-----------------------------------------------------------------------------")          
        
        try:
            while True:    
                # Send GNFD and read the responses
                response_data = send_gnfd_command(bit_field)
    
                if 'RADC' in response_data:
                    radc_data = response_data['RADC']
                    print("OK: Received RADC data - see plot")

                    # Plot RADC data
                    radc_ax.clear()
                    radc_ax.plot(range(len(radc_data)), radc_data, label='IF RX1')
                    radc_ax.set_ylim(-2048, 2048)                    
                    radc_ax.set_xlabel('ADC sample number')
                    radc_ax.set_ylabel('ADC value [bit]')
                    radc_ax.set_title('RAW ADC data')
                    radc_ax.legend()
                    radc_ax.set_yticks([-2048, -1536, -1024, -512, 0, 512, 1024, 1536, 2048])
                    radc_ax.set_xticks([0, 128, 256, 384, 512, 640, 768, 896, 1024])
                    radc_ax.grid(True)
                    radc_fig.canvas.draw()                    
                    radc_fig.canvas.flush_events()

                    # Process RADC data as needed
    
                if 'RFFT' in response_data:
                    rfft_data = response_data['RFFT']
                    print("OK: Received RFFT data - see plot")
                    
                    # Calculate the scaling factor based on the distance range
                    if radar_parameters.distance_range == 0:  # 20 meters
                        scaling_factor = 0.03934  # 3.934 cm converted to meters
                    elif radar_parameters.distance_range == 1:  # 50 meters
                        scaling_factor = 0.09943  # 9.943 cm converted to meters
                    else:
                        print("ERROR: Invalid Distance Range value:", radar_parameters.distance_range)
                        continue

                    # Create distances in meters using the calculated scaling factor
                    distances_meters = [i * scaling_factor for i in range(len(rfft_data['Spectrum']))]
                    
                    # Plot RFFT data
                    spectrum_data = rfft_data['Spectrum']
                    threshold_data = rfft_data['Threshold']
                    
                    rfft_ax.clear()
                    rfft_ax.plot(distances_meters, spectrum_data, label='Spectrum')
                    rfft_ax.plot(distances_meters, threshold_data, label='Threshold')
                    rfft_ax.set_xlabel('Distance [m]')
                    rfft_ax.set_ylabel('Magnitude [dB]')
                    rfft_ax.set_title('RAW RFFT data')
                    rfft_ax.legend()
                    rfft_ax.grid(True)
                    rfft_fig.canvas.draw()
                    rfft_fig.canvas.flush_events()              

                    # Process RFFT data as needed (spectrum and threshold)
    
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
                    print("OK: Received DONE data - Frame Number:", frame_number)
                    # Process DONE data (frame number)
                    
                time.sleep(0.5)  # Adjust the interval as needed  
                
        except KeyboardInterrupt:
            print("-----------------------------------------------------------------------------")              
            print("Stopping data acquisition...")                

        # Close the RADC and RFFT figure if created
        if radc_fig is not None:
            plt.close(radc_fig)
        if rfft_fig is not None:
            plt.close(rfft_fig)
        
        # Disconnect from the sensor
        time.sleep(0.1)
        disconnect_sensor()  
    else:
        print("ERROR: Failed to initialize sensor.")

    close_serial_connection()  # Close serial connection
