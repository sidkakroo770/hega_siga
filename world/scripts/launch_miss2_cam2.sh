#!/bin/bash

echo "=========================================================="
echo "       DUAL CAMERA SIMULATION (CAM1 + CAM2)             "
echo "=========================================================="
killall -9 gz gz-sim-server gz-sim-gui sim_vehicle.py mavproxy.py arducopter 2>/dev/null
fuser -k -9 9002/udp 9003/udp 5760/tcp 14550/udp 2>/dev/null
sleep 2

echo "[1/5] Spawning Window 1: Gazebo Sim..."
gnome-terminal --title="[1] Gazebo Sim World (Dual Cam)" -- bash -c '
source ~/miniforge3/etc/profile.d/conda.sh
conda activate gz-dev
export GZ_SIM_RESOURCE_PATH=/home/prem/ardupilot_gazebo/models:/home/prem/Vegh/models:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/prem/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
gz sim -r -v 4 /home/prem/Vegh/simulation/miss2_cam2_world.sdf
exec bash
'

sleep 5

echo "[2/5] Spawning Window 2: ArduPilot SITL / MAVLink..."
gnome-terminal --title="[2] ArduPilot SITL / MAVLink" -- bash -c '
source ~/miniforge3/etc/profile.d/conda.sh
conda activate gz-dev
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console
exec bash
'

sleep 5

echo "[3/5] Spawning Window 3: Downward Camera Stream (CAM1)..."
gnome-terminal --title="[3] Downward Camera Viewer (CAM1)" -- bash -c '
source ~/miniforge3/etc/profile.d/conda.sh
conda activate gz-dev
python3 /home/prem/Vegh/rpi/cam1_viewer.py
exec bash
'

echo "[4/5] Spawning Window 4: Front Camera Stream (CAM2)..."
gnome-terminal --title="[4] Front Camera Viewer (CAM2)" -- bash -c '
source ~/miniforge3/etc/profile.d/conda.sh
conda activate gz-dev
python3 /home/prem/Vegh/rpi/cam2_viewer.py
exec bash
'

echo "[5/5] Spawning Window 5: Raspberry Pi Shell..."
gnome-terminal --title="[5] Raspberry Pi Virtual Terminal" -- bash --rcfile /home/prem/.rpi_bashrc

echo "[SUCCESS] Dual Camera Simulation launched successfully!"
