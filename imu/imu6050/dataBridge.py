import serial
import asyncio
import websockets
import json
import sys
import concurrent.futures # Import for ThreadPoolExecutor

# --- Configuration ---
SERIAL_PORT = 'COM10'  # Change this to your Arduino's serial port
BAUD_RATE = 115200     # Must match the baud rate in your Arduino code
WEB_SOCKET_PORT = 8080 # Port for the WebSocket server

# --- Global variable to hold connected WebSocket clients ---
connected_clients = set()

# --- Function to read data from the serial port using run_in_executor ---
async def read_serial(serial_connection, loop):
    """Reads data from the serial port line by line in a separate thread."""
    # Use a ThreadPoolExecutor for running blocking serial reads
    # max_workers=1 is sufficient as we are reading from a single serial port
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)

    print("Starting serial read task...")
    while True:
        try:
            # Run the blocking readline() in a separate thread using run_in_executor
            # This prevents the main asyncio event loop from being blocked
            line = await loop.run_in_executor(executor, serial_connection.readline)
            line = line.decode('utf-8').strip() # Decode bytes to string and remove leading/trailing whitespace

            if line:
                # print(f"Received from Arduino: {line}") # Uncomment for debugging received lines

                # --- Added check to skip header lines ---
                # Check if the line looks like a header or separator
                if line.startswith("MPU6050 Data") or line.startswith("----") or line.startswith("AccelX"):
                    # print(f"Skipping header line: {line}") # Uncomment to see skipped lines
                    continue # Skip the rest of the loop for this line

                try:
                    # Split the line by tabs, as used in the Arduino code
                    data_values = line.split('\t')
                    # Expecting 6 values (accelX, Y, Z, gyroX, Y, Z)
                    if len(data_values) == 6:
                        # Convert string values to floats
                        sensor_data = {
                            'accelX': float(data_values[0]),
                            'accelY': float(data_values[1]),
                            'accelZ': float(data_values[2]),
                            'gyroX': float(data_values[3]),
                            'gyroY': float(data_values[4]),
                            'gyroZ': float(data_values[5])
                            # Add pitch, roll, yaw here if your Arduino calculates them
                            # 'pitch': pitch_value,
                            # 'roll': roll_value,
                            # 'yaw': yaw_value
                        }

                        # Send data to all connected WebSocket clients
                        message = json.dumps(sensor_data)
                        await broadcast(message)

                    # Add a check for the header lines if they are still being sent after setup
                    # This check is now less critical due to the explicit header check above,
                    # but can still catch other unexpected formats.
                    elif not line.startswith("MPU6050 Data") and not line.startswith("----"):
                         print(f"Warning: Received line with unexpected number of values ({len(data_values)}): {line}")


                except ValueError as e:
                    print(f"Error parsing serial data: {e} - Line: {line}")
                except Exception as e:
                    print(f"An unexpected error occurred during data processing: {e}")

        except serial.SerialException as e:
            print(f"Serial port error: {e}")
            print("Please check if the Arduino is connected and the correct port is selected.")
            # Wait before retrying serial connection
            await asyncio.sleep(5)
        except Exception as e:
            print(f"An unexpected error occurred during serial read: {e}")
            # Wait before retrying serial read
            await asyncio.sleep(1)


# --- Function to handle WebSocket connections ---
async def handler(websocket):
    """Handles new WebSocket connections."""
    print(f"Client connected from {websocket.remote_address}")
    connected_clients.add(websocket)
    try:
        # Keep the connection open until the client disconnects
        await websocket.wait_closed()
    finally:
        # Remove the client when the connection is closed
        print(f"Client disconnected from {websocket.remote_address}")
        connected_clients.remove(websocket)

# --- Function to broadcast message to all connected clients ---
async def broadcast(message):
    """Sends a message to all currently connected WebSocket clients."""
    # Send message to all connected clients concurrently
    if connected_clients:
        # Use asyncio.gather to send messages concurrently and handle potential exceptions
        await asyncio.gather(*(client.send(message) for client in connected_clients), return_exceptions=True)

# --- Main function to start the server and serial reading ---
async def main():
    """Starts the WebSocket server and the serial reading task."""
    print(f"Starting WebSocket server on port {WEB_SOCKET_PORT}")
    print(f"Attempting to open serial port {SERIAL_PORT} at {BAUD_RATE} baud...")

    # Get the current running event loop
    loop = asyncio.get_running_loop()

    try:
        # Open the serial port with a small timeout
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print("Serial port opened successfully.")

        # Start the WebSocket server
        async with websockets.serve(handler, "localhost", WEB_SOCKET_PORT):
            print("WebSocket server started.")
            # Run the serial reading task indefinitely using run_in_executor
            # This task will run in a separate thread, not blocking the main event loop
            await read_serial(ser, loop)

    except serial.SerialException as e:
        print(f"Failed to open serial port {SERIAL_PORT}: {e}")
        print("Please ensure the Arduino is connected and the correct port is selected.")
        sys.exit(1) # Exit the script if serial port cannot be opened
    except Exception as e:
        print(f"An error occurred: {e}")
        sys.exit(1)

# --- Entry point of the script ---
if __name__ == "__main__":
    # Essential for asyncio on Windows
    if sys.platform == 'win32':
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    # Run the main asynchronous function
    asyncio.run(main())
