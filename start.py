from pymavlink import mavutil
from pymavlink import fgFDM

# Connect to the ArduPilot system
connection = 'COM3'  # Replace with your telemetry connection
baudrate = 57600  # For serial connections
master = mavutil.mavlink_connection(connection, baud=baudrate)

# Wait for the heartbeat message to confirm connection
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Heartbeat received from system (System ID: {master.target_system}, Component ID: {master.target_component})")

# Function to parse and print telemetry data
def get_telemetry():
    while True:
        try:
            # Wait for the next telemetry message
            message = master.recv_match(blocking=True)
            # Print the message name and content
            if message.get_type():
                print(f"{message.get_type()} -> {message.to_dict()}")
        except KeyboardInterrupt:
            print("\nExiting telemetry stream...")
            break

# Start telemetry streaming
get_telemetry()
