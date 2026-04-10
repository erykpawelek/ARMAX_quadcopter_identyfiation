# Iris Drone Simulation - ARMAX Payload Drop

## Overview
Gazebo Harmonic simulation of iris drone with detachable payload
for ARMAX system identification analysis.

## Requirements
- Gazebo Harmonic (gz-sim 8.11.0)
- ArduPilot SITL (ArduCopter 4.6)
- ardupilot_gazebo plugin (main branch)
- Python 3 with gz-transport13 bindings

## Files
- `armax_hover.sdf`          - Gazebo world file
- `iris_with_ardupilot_model.sdf` - Iris drone with gripper and payload
- `armax_logger.py`          - IMU data logger (MATLAB-ready CSV output)

## How to Run

### 1. Start SITL
```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py \
  -v ArduCopter -f gazebo-iris \
  --model JSON --console --no-rebuild -I 0
```

### 2. Start Gazebo (after seeing "Waiting for connection")
```bash
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
gz sim -r ~/ardupilot_gazebo/worlds/armax_hover.sdf
```

### 3. Arm and Takeoff (in MAVProxy Xterm)
mode GUIDED
arm throttle
takeoff 5
### 4. Start Logger (new terminal)
```bash
python3 iris_sim/armax_logger.py
```

### 5. Drop Payload (new terminal)
```bash
gz topic -t /payload_drop -m gz.msgs.Empty -p " "
```

### 6. MATLAB ARMAX Analysis
```matlab
data  = readtable('armax_data_YYYYMMDD_HHMMSS.csv');
y     = data.roll_deg;
u     = data.drop_event;
Ts    = 0.5;
ze    = iddata(y, u, Ts);
model = armax(ze, [2 2 2]);
```
