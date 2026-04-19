import sys
import argparse
import numpy as np
import scipy.io as sio
from pymavlink import mavutil

parser = argparse.ArgumentParser(description="Multi-Axis Drone Log Converter for MATLAB")
parser.add_argument("--bin_path", type=str, default="flight.BIN", help="Path to .bin file")
parser.add_argument("--mat_path", type=str, default="armax_complete.mat", help="Output .mat file")
args = parser.parse_args()

print(f"Opening log file: {args.bin_path}")
try:
    mlog = mavutil.mavlink_connection(args.bin_path)
except Exception as e:
    print(f"Error: {e}")
    sys.exit(1)

# Main data structure
data = {
    'ctun': {'time': [], 'thro': [], 'alt': [], 'dalt': []},
    'rcou': {'time': [], 'c1': [], 'c2': [], 'c3': [], 'c4': []},
    'imu':  {'time': [], 'accX': [], 'accY': [], 'accZ': []},
    'att':  {'time': [], 'roll': [], 'pitch': [], 'yaw': [], 'des_roll': [], 'des_pitch': [], 'des_yaw': []},
    'markers': {'time': [], 'text': []}
}

print("Parsing all flight dynamics (CTUN, RCOU, IMU, ATT, MSG)...")

while True:
    msg = mlog.recv_match(type=['CTUN', 'RCOU', 'MSG', 'IMU', 'ATT'])
    
    if msg is None:
        break
    
    m_type = msg.get_type()
    t_sec = msg.TimeUS / 1e6
    
    if m_type == 'CTUN':
        data['ctun']['time'].append(t_sec)
        data['ctun']['thro'].append(msg.ThO)
        data['ctun']['alt'].append(msg.Alt)
        data['ctun']['dalt'].append(msg.DAlt)
        
    elif m_type == 'RCOU':
        data['rcou']['time'].append(t_sec)
        data['rcou']['c1'].append(msg.C1)
        data['rcou']['c2'].append(msg.C2)
        data['rcou']['c3'].append(msg.C3)
        data['rcou']['c4'].append(msg.C4)
        
    elif m_type == 'IMU':
        data['imu']['time'].append(t_sec)
        data['imu']['accX'].append(msg.AccX)
        data['imu']['accY'].append(msg.AccY)
        data['imu']['accZ'].append(msg.AccZ)

    elif m_type == 'ATT':
        data['att']['time'].append(t_sec)
        data['att']['roll'].append(msg.Roll)
        data['att']['pitch'].append(msg.Pitch)
        data['att']['yaw'].append(msg.Yaw)
        data['att']['des_roll'].append(msg.DesRoll)
        data['att']['des_pitch'].append(msg.DesPitch)
        data['att']['des_yaw'].append(msg.DesYaw)
        
    elif m_type == 'MSG':
        msg_text = msg.Message
        if "MARKER" in msg_text:
            data['markers']['time'].append(t_sec)
            data['markers']['text'].append(msg_text)

# Conversion and normalization
for category in data:
    for key in data[category]:
        data[category][key] = np.array(data[category][key])

sio.savemat(args.mat_path, data)
print(f"Finished! {args.mat_path} contains flight log in .mat format.")