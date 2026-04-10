#!/usr/bin/env python3
"""
=============================================================
ARMAX IMU Data Logger - Gazebo Harmonic
=============================================================
Records roll, pitch, yaw and angular rates from the Iris
drone IMU during payload drop experiment.

Output CSV is formatted for direct import into MATLAB for
ARMAX (AutoRegressive Moving Average with eXogenous input)
system identification analysis.

Columns:
  time_s            - Elapsed time in seconds
  phase             - PRE_DROP / DROP / POST_DROP
  roll_deg          - Roll angle (degrees)
  pitch_deg         - Pitch angle (degrees)
  yaw_deg           - Yaw angle (degrees)
  rollspeed_deg_s   - Roll angular rate (deg/s)
  pitchspeed_deg_s  - Pitch angular rate (deg/s)
  yawspeed_deg_s    - Yaw angular rate (deg/s)
  drop_event        - 1 at drop moment, 0 otherwise (ARMAX input u)

Usage:
  1. Start Gazebo + SITL, arm and takeoff to 5m
  2. Run: python3 armax_logger.py
  3. Press ENTER when drone is stable at 5m
  4. Drop payload: gz topic -t /payload_drop -m gz.msgs.Empty -p " "
  5. Type 'd' + ENTER in this terminal to mark the drop
  6. Ctrl+C after ~30s post-drop to save CSV

MATLAB ARMAX Usage:
  data = readtable('armax_data_YYYYMMDD_HHMMSS.csv');
  y = data.roll_deg;           % output signal
  u = data.drop_event;         % input (impulse at drop)
  Ts = 0.5;                    % sample time
  ze = iddata(y, u, Ts);
  model = armax(ze, [2 2 2]);  % adjust orders as needed
=============================================================
"""

from gz.transport13 import Node
from gz.msgs10.imu_pb2 import IMU
import csv
import time
import datetime
import math
import os
import threading
import sys

# ── Configuration ─────────────────────────────────────────
SAMPLE_INTERVAL   = 0.5        # seconds between samples
POST_DROP_DURATION = 30.0      # seconds to log after drop
IMU_TOPIC = (
    '/world/armax_hover/model/iris_with_ardupilot'
    '/model/iris_with_standoffs/link/imu_link'
    '/sensor/imu_sensor/imu'
)
# ──────────────────────────────────────────────────────────

# Shared state
imu = {
    'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
    'wx':   0.0, 'wy':   0.0,  'wz':  0.0
}
payload_dropped  = False
drop_time        = None
drop_event_flag  = False   # True for one sample at drop moment


def imu_callback(msg):
    """Convert IMU quaternion to Euler angles and store."""
    q = msg.orientation

    # Roll (X-axis)
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x ** 2 + q.y ** 2)
    imu['roll'] = math.degrees(math.atan2(sinr, cosr))

    # Pitch (Y-axis)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    imu['pitch'] = math.degrees(math.asin(sinp))

    # Yaw (Z-axis)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
    imu['yaw'] = math.degrees(math.atan2(siny, cosy))

    # Angular rates
    imu['wx'] = math.degrees(msg.angular_velocity.x)
    imu['wy'] = math.degrees(msg.angular_velocity.y)
    imu['wz'] = math.degrees(msg.angular_velocity.z)


def wait_for_drop_command():
    """Background thread - type 'd' + ENTER to mark drop."""
    global payload_dropped, drop_time, drop_event_flag
    while True:
        try:
            cmd = input()
        except EOFError:
            break
        if cmd.strip().lower() == 'd' and not payload_dropped:
            payload_dropped  = True
            drop_event_flag  = True
            drop_time        = time.time()
            elapsed = drop_time - start_time
            print(f"\n{'='*65}")
            print(f"  *** PAYLOAD DROP MARKED at t = {elapsed:.1f}s ***")
            print(f"{'='*65}\n")


def print_header():
    print()
    print("=" * 65)
    print("  ARMAX IMU LOGGER  —  Gazebo Harmonic")
    print("=" * 65)
    print(f"  IMU topic     : {IMU_TOPIC}")
    print(f"  Sample rate   : {1/SAMPLE_INTERVAL:.1f} Hz  "
          f"(every {SAMPLE_INTERVAL}s)")
    print(f"  Post-drop log : {POST_DROP_DURATION}s")
    print("=" * 65)
    print()


def print_table_header():
    print(f"{'Time':>8}  {'Phase':>10}  {'Roll':>8}  "
          f"{'Pitch':>8}  {'Yaw':>8}  {'Drop':>5}")
    print("-" * 65)


def main():
    global start_time

    print_header()

    # Connect to Gazebo IMU
    node = Node()
    node.subscribe(IMU, IMU_TOPIC, imu_callback)
    print("  Waiting for IMU data from Gazebo...")
    time.sleep(2.0)
    print("  IMU connected.\n")

    # Setup CSV output
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filepath  = os.path.expanduser(f"~/armax_data_{timestamp}.csv")

    input("  Press ENTER when drone is stable at 5m to begin logging...\n")

    start_time = time.time()

    # Start drop-command listener
    drop_thread = threading.Thread(target=wait_for_drop_command, daemon=True)
    drop_thread.start()

    print(f"  Logging to: {filepath}")
    print("  Type  'd' + ENTER  to mark payload drop moment.")
    print("  Press  Ctrl+C  to stop.\n")
    print_table_header()

    # CSV columns
    fieldnames = [
        'time_s',
        'phase',
        'roll_deg',
        'pitch_deg',
        'yaw_deg',
        'rollspeed_deg_s',
        'pitchspeed_deg_s',
        'yawspeed_deg_s',
        'drop_event'
    ]

    sample_count = 0
    last_sample  = 0.0

    with open(filepath, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        # Write metadata comments
        csvfile.write("# ARMAX IMU Data Log\n")
        csvfile.write(f"# Recorded: {datetime.datetime.now()}\n")
        csvfile.write(f"# Sample interval: {SAMPLE_INTERVAL}s\n")
        csvfile.write(f"# MATLAB: y=roll_deg, u=drop_event, Ts={SAMPLE_INTERVAL}\n")
        csvfile.write("#\n")

        writer.writeheader()

        try:
            while True:
                now     = time.time()
                elapsed = now - start_time

                # ── Determine phase ────────────────────────
                if not payload_dropped:
                    phase = "PRE_DROP"
                elif elapsed - (drop_time - start_time) < 1.0:
                    phase = "DROP"
                else:
                    phase = "POST_DROP"

                # ── Sample at correct interval ─────────────
                if elapsed - last_sample >= SAMPLE_INTERVAL:
                    global drop_event_flag

                    drop_event = 1 if drop_event_flag else 0
                    drop_event_flag = False   # clear after one sample

                    row = {
                        'time_s':           round(elapsed, 2),
                        'phase':            phase,
                        'roll_deg':         round(imu['roll'],  6),
                        'pitch_deg':        round(imu['pitch'], 6),
                        'yaw_deg':          round(imu['yaw'],   6),
                        'rollspeed_deg_s':  round(imu['wx'],    6),
                        'pitchspeed_deg_s': round(imu['wy'],    6),
                        'yawspeed_deg_s':   round(imu['wz'],    6),
                        'drop_event':       drop_event
                    }

                    writer.writerow(row)
                    csvfile.flush()
                    sample_count += 1
                    last_sample   = elapsed

                    # Console output
                    print(f"  {elapsed:7.1f}s  "
                          f"[{phase:>10s}]  "
                          f"Roll={imu['roll']:8.3f}°  "
                          f"Pitch={imu['pitch']:7.3f}°  "
                          f"Yaw={imu['yaw']:8.3f}°  "
                          f"{'DROP' if drop_event else '    '}")

                # ── Auto-stop after post-drop duration ─────
                if payload_dropped and drop_time:
                    if now - drop_time >= POST_DROP_DURATION:
                        print(f"\n  Post-drop duration complete ({POST_DROP_DURATION}s).")
                        raise KeyboardInterrupt

                time.sleep(0.01)

        except KeyboardInterrupt:
            pass

    # ── Summary ────────────────────────────────────────────
    print()
    print("=" * 65)
    print(f"  Logging complete.")
    print(f"  Samples recorded : {sample_count}")
    print(f"  Duration         : {elapsed:.1f}s")
    print(f"  CSV saved to     : {filepath}")
    print()
    print("  MATLAB commands:")
    print(f"  >> data = readtable('{filepath}');")
    print(f"  >> y  = data.roll_deg;")
    print(f"  >> u  = data.drop_event;")
    print(f"  >> Ts = {SAMPLE_INTERVAL};")
    print(f"  >> ze = iddata(y, u, Ts);")
    print(f"  >> model = armax(ze, [2 2 2]);")
    print("=" * 65)


if __name__ == '__main__':
    main()
