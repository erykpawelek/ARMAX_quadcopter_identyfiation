import time
import subprocess
import argparse
from pymavlink import mavutil

# Helper function to delay script execution while keeping MAVLink buffer empty
def safe_delay(delay_seconds, mav_connection):
    start_t = time.time()
    while (time.time() - start_t < delay_seconds):
        mav_connection.recv_match(blocking=False)

parser = argparse.ArgumentParser(description="Drone Hover Vibration Test")
parser.add_argument("--hover_dur", type=float, default=30.0, help="Duration of hover recording phases in seconds")
parser.add_argument("--stab_dur", type=float, default=10.0, help="Duration of stabilization phases in seconds")
parser.add_argument("--alt", type=float, default=2.0, help="Altitude at which test will be executed")

args = parser.parse_args()

# Connect to the simulator
connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# Wait for the first heartbeat
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

try:
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
    safe_delay(2.0, connection)

print("System armed")

# Send autonomous takeoff command
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, # Confirmation
    0, 0, 0, 0, 0, 0, 
    args.alt
)
print(f"Takeoff command sent. Ascending to {args.alt} meters.")

# Wait safely to reach altitude and stabilize
print("Phase: Initial stabilization")
safe_delay(args.stab_dur, connection)

# Phase: Hovering with payload
print("Phase: Hovering with payload")
# Send log marker to ArduPilot .bin log
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: HOVER_PAYLOAD_START")
safe_delay(args.hover_dur, connection)
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: HOVER_PAYLOAD_STOP")

# Phase: Payload drop
print("Phase: Payload drop")
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: PAYLOAD_DROP")
try:
    # Execute Gazebo topic command to detach payload
    subprocess.run(
        ["gz", "topic", "-t", "/payload/detach", "-m", "gz.msgs.Empty", "-p", " "],
        check=True
    )
    print("Payload detached successfully.")
except Exception as e:
    print("Error during payload detachement:", e)

# Phase: Stabilization after drop
print("Phase: Stabilization after payload drop")
safe_delay(args.stab_dur, connection)

# Phase: Hovering without payload
print("Phase: Hovering without payload")
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: HOVER_NO_PAYLOAD_START")
safe_delay(args.hover_dur, connection)
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: HOVER_NO_PAYLOAD_STOP")

print("Experiment finished.")