import serial
import time
import sys
import math # Import math for sqrt and radians conversion

# --- Lidar Functions (from your speedCalc.py) ---

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
        ser: The serial port object for the lidar.

    Returns:
        The distance in meters (float) or None on error or if no valid response is received.
    """
    # Command for Single Measurement: 80 06 02 78 (ADDR 06 02 CS)
    # Note: Checksum 78 is based on the documented example.
    single_measurement_command = b'\x80\x06\x02\x78'

    try:
        ser.write(single_measurement_command)

        # Wait for the sensor's response. A single measurement reply is 11 bytes.
        # Use a timeout to prevent the script from hanging indefinitely if no response is received.
        ser.timeout = 0.5 # Set a timeout (e.g., 0.5 second) for lidar response

        response = ser.read(11) # Try to read 11 bytes (the expected message length)

        if len(response) == 11:
            # A full message was received, now parse and validate it.
            # Expected reply start sequence for distance data: 80 06 82
            expected_start_sequence = b'\x80\x06\x82'

            if response[0:3] != expected_start_sequence:
                # print(f"Lidar: Received unexpected start sequence: {response[0:3].hex()}. Full message: {response.hex()}")
                return None

            # Validate the message checksum
            received_data = response[:-1]
            received_checksum = response[-1]
            calculated_checksum = calculate_checksum(received_data)

            if received_checksum != calculated_checksum:
                # print(
                #     f"Lidar: Checksum error! Expected: {calculated_checksum:02X}, Received: {received_checksum:02X}. Message: {response.hex()}"
                # )
                return None

            # Check for error message ('ERR' in ASCII)
            # The error message format is ADDR 06 82 E R R ... CS
            # 'E' is 0x45, 'R' is 0x52
            if response[3:6] == b'ERR': # Bytes 3, 4, 5 should be 'E', 'R', 'R'
                # print("Lidar: Sensor returned 'Out of range'")
                return None

            # Extract distance (ASCII bytes from index 3 to 9)
            # The format is typically XXX.YYY or XXX.YYYY for 0.1mm resolution
            try:
                # Find the '.' character (0x2E) to determine the decimal position
                decimal_index_in_payload = -1
                # Search within the data payload part (bytes 3 to 9)
                for i in range(3, 10):
                    if response[i] == 0x2E:
                        decimal_index_in_payload = i
                        break

                if decimal_index_in_payload != -1:
                     # Extract the ASCII bytes for the number
                    # Assuming XXX.YYY or similar format fits in these bytes
                    distance_ascii_bytes = response[3:10]
                    distance_str = distance_ascii_bytes.decode('ascii')
                    distance = float(distance_str)
                    return distance # Return the successfully parsed distance
                else:
                    # print(f"Lidar: Invalid data format (no decimal point) in message payload: {response[3:10].hex()}")
                    return None

            except ValueError:
                # print(f"Lidar: Error converting ASCII data to float from message: {response.hex()}")
                return None
            except Exception as e:
                # print(f"Lidar: An unexpected error during distance extraction: {e}. Message: {response.hex()}")
                return None

        elif len(response) > 0:
            # Received some bytes, but not a full message
            # print(f"Lidar: Received incomplete response ({len(response)} bytes): {response.hex()}")
            return None
        else:
            # No response received within the timeout
            # print("Lidar: No response received from sensor.")
            return None
    except serial.SerialException as e:
        print(f"Lidar Serial Error: {e}")
        return None
    except Exception as e:
        print(f"Lidar Read Error: {e}")
        return None


# --- IMU Function ---

def read_imu_data(ser):
    """
    Reads a line of data from the IMU serial port and extracts the gyroscope data.
    Assumes data format: AccelX AccelY AccelZ GyroX GyroY GyroZ (tab-separated).

    Args:
        ser: The serial port object for the IMU.

    Returns:
        A tuple containing (gyroX, gyroY, gyroZ) in degrees per second,
        or (None, None, None) on error or if no valid data is received.
    """
    try:
        # Read a line from the serial buffer. Adjust timeout as needed.
        ser.timeout = 0.1 # Set a short timeout for reading a line
        line = ser.readline().decode('ascii').strip()

        if line:
            # Split the line by tabs
            data_points = line.split('\t')

            # Expected format has 6 values (AccelX,Y,Z, GyroX,Y,Z)
            if len(data_points) == 6:
                try:
                    # Extract gyroscope values (assuming they are the last three)
                    gyroX = float(data_points[3])
                    gyroY = float(data_points[4])
                    gyroZ = float(data_points[5])
                    return (gyroX, gyroY, gyroZ)
                except ValueError:
                    # print(f"IMU: Error converting IMU data to float: {line}")
                    return (None, None, None)
            else:
                # print(f"IMU: Received unexpected number of data points ({len(data_points)}): {line}")
                return (None, None, None)
        else:
            # print("IMU: No data received from IMU.")
            return (None, None, None)
    except serial.SerialException as e:
        print(f"IMU Serial Error: {e}")
        return (None, None, None)
    except Exception as e:
        print(f"IMU Read Error: {e}")
        return (None, None, None)


# --- Main Calculation Logic ---

def main():
    """
    Main function to initialize serial communication, read sensor data,
    and calculate target speed.
    """
    # --- Configuration ---
    lidar_port = 'COM11'  # Set the COM port for the lidar
    imu_port = 'COM10'   # Set the COM port for the IMU
    lidar_baud_rate = 9600   # Baud rate for the lidar
    imu_baud_rate = 115200  # Baud rate for the IMU (from your main.cpp)
    read_interval = 0.05 # Time to wait between calculation cycles (in seconds)

    # --- Serial Port Setup ---
    ser_lidar = None
    ser_imu = None

    try:
        # Configure and open serial ports
        ser_lidar = serial.Serial(lidar_port, lidar_baud_rate)
        print(f"Serial port {ser_lidar.name} opened successfully for Lidar.")
        ser_imu = serial.Serial(imu_port, imu_baud_rate)
        print(f"Serial port {ser_imu.name} opened successfully for IMU.")

        # Clear any potential old data in the input buffers
        ser_lidar.flushInput()
        ser_imu.flushInput()
        time.sleep(0.2) # Give ports a moment to settle

        print("Starting speed calculation...")

        # Variables for speed calculation
        previous_distance = None
        previous_time = time.time()

        while True:
            current_time = time.time()
            time_diff = current_time - previous_time

            # Read data from sensors
            current_distance = read_single_distance(ser_lidar)
            imu_data = read_imu_data(ser_imu) # (gyroX, gyroY, gyroZ)

            # --- Perform Calculations ---
            speed = None # Initialize speed for this cycle

            if current_distance is not None and imu_data is not (None, None, None):
                gyroZ_degs = imu_data[2] # Get the Z-axis angular rate (assuming horizontal tracking)

                # Calculate Radial Velocity (v_r)
                v_r = 0.0 # Default to 0 if no previous distance
                if previous_distance is not None and time_diff > 0:
                    delta_r = current_distance - previous_distance
                    v_r = delta_r / time_diff

                # Calculate Tangential Velocity (v_t)
                # Convert angular rate from degrees/s to radians/s
                omega_rads = math.radians(gyroZ_degs)
                v_t = current_distance * omega_rads

                # Calculate Total Speed (v)
                speed = math.sqrt(v_r**2 + v_t**2)

                # Print results
                print(f"Time: {current_time:.2f}s, Distance (r): {current_distance:.3f} m, "
                      f"Radial Vel (v_r): {v_r:.3f} m/s, Tangential Vel (v_t): {v_t:.3f} m/s, "
                      f"IMU GyroZ: {gyroZ_degs:.2f} deg/s, Total Speed (v): {speed:.3f} m/s")

                # Update previous values for the next cycle
                previous_distance = current_distance
                previous_time = current_time

            elif current_distance is None:
                # print("Skipping calculation: No valid lidar distance.")
                pass # Don't print every time, can be noisy
            elif imu_data is (None, None, None):
                # print("Skipping calculation: No valid IMU data.")
                 pass # Don't print every time, can be noisy


            # Wait before the next reading cycle
            time.sleep(read_interval)

    except serial.SerialException as e:
        print(f"Serial port error: {e}")
    except FileNotFoundError:
        print(f"Error: One or both serial ports not found. Check COM port assignments.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        # Ensure serial ports are closed properly
        if ser_lidar is not None and ser_lidar.is_open:
             # Before closing, send the Shut Down command (80 04 02 7A) to turn off the laser
            full_shutdown_command = b'\x80\x04\x02\x7A'
            try:
                print(f"Sending Lidar Shut Down command: {full_shutdown_command.hex()}")
                ser_lidar.write(full_shutdown_command)
                time.sleep(0.1) # Give sensor time to process
            except Exception as e:
                print(f"Error sending lidar shutdown command: {e}")
            ser_lidar.close()
            print(f"Serial port {ser_lidar.name} closed.")

        if ser_imu is not None and ser_imu.is_open:
            ser_imu.close()
            print(f"Serial port {ser_imu.name} closed.")


if __name__ == "__main__":
    main()
