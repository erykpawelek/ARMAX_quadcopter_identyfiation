
import time
from pymavlink import mavutil
from scipy import signal

# Connect to the simulator
connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# Wait for the first heartbeat
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

try:
    # Get mode ID for GUIDED mode instead of ALT_HOLD
    mode_id = connection.mode_mapping()['GUIDED']
except Exception as e:
    print("Cannot download mode id from flight controller")
    exit()

# Arming loop
while not connection.motors_armed():
    # Set to GUIDED mode
    connection.set_mode(mode_id)
    
    # Send arm command
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, # 1 means Arm
        0, 0, 0, 0, 0, 0
    )
    print("System disarmed, waiting for system setup...")
    time.sleep(2)
    # Update internal state by waiting for heartbeat
    connection.wait_heartbeat()

print("System armed")

# Target altitude in meters
target_altitude = 2.0

# Send autonomous takeoff command
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, # Confirmation
    0, 0, 0, 0, 0, 0, # Params 1-6 are ignored for vertical takeoff
    target_altitude # Param 7: Target altitude
)
print("Takeoff command sent. Ascending to 2 meters.")
time.sleep(4)