import time
import argparse
from pymavlink import mavutil

# Helper function to delay script execution while keeping MAVLink buffer empty
def safe_delay(delay_seconds, mav_connection):
    start_t = time.time()
    while (time.time() - start_t < delay_seconds):
        mav_connection.recv_match(blocking=False)

parser = argparse.ArgumentParser(description="Drone Drone System Identification Validation")
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

# Wait for 4 seconds safely to reach altitude
safe_delay(args.stab_dur, connection)

print("Phase: Step stymuli with payload")
# Send log marker to ArduPilot .bin log
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: START_STEP_PAYLOAD")
connection.mav.set_attitude_target_send(
            0,
            connection.target_system,
            connection.target_component,
            7,
            [1.0, 0.0, 0.0, 0.0], # Quaternion for level flight
            0, 0, 0, 
            noise[i]  
        )
safe_delay(4.0, connection)
connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, b"MARKER: STOP_STEP_PAYLOAD")
        