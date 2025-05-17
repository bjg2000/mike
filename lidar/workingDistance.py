import serial
import time
import sys

def calculate_checksum(data):
    """
    Calculates the checksum for the given data according to the DFRobot SEN0366 protocol.
    The checksum is the sum of all previous bytes, reversed (bitwise NOT), and added 1.
    """
    checksum = 0
    for byte in data:
        checksum += byte
    # Apply the two's complement (reverse and add 1) and ensure it's an 8-bit value
    checksum = (~checksum + 1) & 0xFF
    return checksum

def read_single_distance(ser):
    """
    Sends the single measurement command and reads the distance data from the sensor.

    Args:
        ser: The serial port object.

    Returns:
        The distance in meters (float) or None on error or if no valid response is received.
    """
    # Command for Single Measurement: 80 06 02 78 (ADDR 06 02 CS)
    # Note: Checksum 78 is based on the documented example.
    single_measurement_command = b'\x80\x06\x02\x78'

    print(f"Sending single measurement command: {single_measurement_command.hex()}")
    ser.write(single_measurement_command)

    # Wait for the sensor's response. A single measurement reply is 11 bytes.
    # Use a timeout to prevent the script from hanging indefinitely if no response is received.
    ser.timeout = 1.0 # Set a timeout (e.g., 1 second)

    response = ser.read(11) # Try to read 11 bytes (the expected message length)

    # Restore the original timeout if needed later, though not strictly necessary in this single-mode script
    # ser.timeout = None

    if len(response) == 11:
        # A full message was received, now parse and validate it.
        # Expected reply start sequence for distance data: 80 06 82
        expected_start_sequence = b'\x80\x06\x82'

        if response[0:3] != expected_start_sequence:
            print(f"Received unexpected start sequence: {response[0:3].hex()}. Full message: {response.hex()}")
            return None

        # Validate the message checksum
        received_data = response[:-1]
        received_checksum = response[-1]
        calculated_checksum = calculate_checksum(received_data)

        if received_checksum != calculated_checksum:
            print(
                f"Checksum error! Expected: {calculated_checksum:02X}, Received: {received_checksum:02X}. Message: {response.hex()}"
            )
            return None

        # Check for error message ('ERR' in ASCII)
        # The error message format is ADDR 06 82 E R R ... CS
        # 'E' is 0x45, 'R' is 0x52
        if response[3:6] == b'ERR': # Bytes 3, 4, 5 should be 'E', 'R', 'R'
            print("Sensor returned 'Out of range'")
            return None

        # Extract distance (ASCII bytes from index 3 to 9)
        # The format is typically XXX.YYY or XXX.YYYY for 0.1mm resolution
        try:
            # Find the '.' character (0x2E) to determine the decimal position
            decimal_index_in_payload = -1
            for i in range(3, 10): # Search within the data payload part
                if response[i] == 0x2E:
                    decimal_index_in_payload = i
                    break

            if decimal_index_in_payload != -1:
                 # Extract the ASCII bytes for the number
                distance_ascii_bytes = response[3:10] # Assuming XXX.YYY or similar format fits in these bytes
                distance_str = distance_ascii_bytes.decode('ascii')
                distance = float(distance_str)
                return distance # Return the successfully parsed distance
            else:
                print(f"Invalid data format (no decimal point) in message payload: {response[3:10].hex()}")
                return None

        except ValueError:
            print(f"Error converting ASCII data to float from message: {response.hex()}")
            return None
        except Exception as e:
            print(f"An unexpected error during distance extraction: {e}. Message: {response.hex()}")
            return None

    elif len(response) > 0:
        # Received some bytes, but not a full message
        print(f"Received incomplete response ({len(response)} bytes): {response.hex()}")
        return None
    else:
        # No response received within the timeout
        print("No response received from sensor.")
        return None


def main():
    """
    Main function to initialize serial communication and read single distances repeatedly.
    """
    # --- Configuration ---
    com_port = 'COM9'  # Set the COM port here
    baud_rate = 9600   # According to the documentation
    read_interval = 1.0 # Time to wait between single measurements (in seconds)

    # --- Serial Port Setup ---
    ser = None
    try:
        # Configure the serial port
        # Timeout is handled within the read_single_distance function
        ser = serial.Serial(com_port, baud_rate)
        print(f"Serial port {ser.name} opened successfully.")

        # Clear any potential old data in the input buffer
        ser.flushInput()
        ser.flushOutput()
        time.sleep(0.1) # Give the port a moment to settle

        print("Taking single measurements...")

        while True:
            distance = read_single_distance(ser)
            if distance is not None:
                print(f"Distance: {distance:.3f} meters")

            # Wait before taking the next measurement
            time.sleep(read_interval)

    except serial.SerialException as e:
        print(f"Serial port error: {e}")
    except FileNotFoundError:
        print(f"Error: Serial port '{com_port}' not found.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if ser is not None and ser.is_open:
            # Before closing, send the Shut Down command (80 04 02 7A) to turn off the laser
            # Command: 80 04 02 7A (ADDR 04 02 CS) - Based on documented example
            full_shutdown_command = b'\x80\x04\x02\x7A'
            try:
                print(f"Sending Shut Down command: {full_shutdown_command.hex()}")
                ser.write(full_shutdown_command)
                time.sleep(0.1) # Give sensor time to process
            except Exception as e:
                print(f"Error sending shutdown command: {e}")

            ser.close()
            print(f"Serial port {ser.name} closed.")


if __name__ == "__main__":
    main()
