#!/bin/bash

# ============================================================
# AJAO-LELO-MERA
# ONE COMMAND LAUNCHER
# ============================================================

set -e

# ============================================================
# PATHS — YOUR PC
# ============================================================

REPO="$HOME/Prem/ajao-lelo-mera"
VENV="$REPO/.venv"

ARDUPILOT="$HOME/ardupilot"
ARDUPILOT_GAZEBO="$HOME/ardupilot_gazebo"

WORLD="$REPO/worlds/miss2_cam2_world.sdf"

# Your repository stores models here:
MODEL_PATH="$REPO/models/models"

# ArduPilot Gazebo plugin build directory
PLUGIN_PATH="$ARDUPILOT_GAZEBO/build"

# MAVLink output used by your Python code
MAVLINK_PORT=14550

# Gazebo Python bindings are installed system-wide
GZ_PYTHON_PATH="/usr/lib/python3/dist-packages"


echo ""
echo "=========================================================="
echo "        AJAO-LELO-MERA SIMULATION LAUNCHER"
echo "=========================================================="
echo ""


# ============================================================
# CHECK VENV
# ============================================================

if [ ! -f "$VENV/bin/activate" ]; then
    echo "[ERROR] Python virtual environment not found:"
    echo "$VENV"
    echo ""
    echo "Create it using:"
    echo "python3 -m venv $VENV"
    exit 1
fi


# ============================================================
# CHECK WORLD
# ============================================================

if [ ! -f "$WORLD" ]; then
    echo "[ERROR] World not found:"
    echo "$WORLD"
    exit 1
fi


# ============================================================
# CHECK MODELS
# ============================================================

if [ ! -d "$MODEL_PATH" ]; then
    echo "[ERROR] Model directory not found:"
    echo "$MODEL_PATH"
    exit 1
fi


# ============================================================
# CHECK ARDUPILOT GAZEBO PLUGIN
# ============================================================

if [ ! -f "$PLUGIN_PATH/libArduPilotPlugin.so" ]; then
    echo "[ERROR] ArduPilotPlugin not found:"
    echo "$PLUGIN_PATH/libArduPilotPlugin.so"
    exit 1
fi


# ============================================================
# ACTIVATE PROJECT VENV
# ============================================================

source "$VENV/bin/activate"

echo "[OK] Python environment:"
echo "     $VIRTUAL_ENV"

echo "[OK] Python:"
which python3

echo "[OK] Gazebo:"
which gz


# ============================================================
# PYTHONPATH
# ============================================================
# Needed because gz.transport13 is installed by the system
# under /usr/lib/python3/dist-packages.

export PYTHONPATH="$GZ_PYTHON_PATH:${PYTHONPATH:-}"

echo "[OK] Python path configured."


# ============================================================
# GAZEBO ENVIRONMENT
# ============================================================

export GZ_SIM_RESOURCE_PATH="$MODEL_PATH:$ARDUPILOT_GAZEBO/models:${GZ_SIM_RESOURCE_PATH:-}"

export GZ_SIM_SYSTEM_PLUGIN_PATH="$PLUGIN_PATH:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

echo "[OK] Gazebo resource path:"
echo "     $GZ_SIM_RESOURCE_PATH"

echo "[OK] Gazebo plugin path:"
echo "     $GZ_SIM_SYSTEM_PLUGIN_PATH"


# ============================================================
# CHECK PYTHON DEPENDENCIES
# ============================================================

echo ""
echo "[CHECK] Python dependencies..."

python3 - <<'PY'
import cv2
import numpy
import pymavlink

print("[OK] OpenCV:", cv2.__version__)
print("[OK] NumPy:", numpy.__version__)
print("[OK] pymavlink:", pymavlink.__file__)

from gz.transport13 import Node
print("[OK] Gazebo Python transport")
PY


# ============================================================
# CLEAN OLD PROCESSES
# ============================================================

echo ""
echo "[1/5] Cleaning old processes..."

 pkill -9 -f "gz-sim-server" 2>/dev/null || true
pkill -9 -f "gz-sim-gui" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true

pkill -9 -f "sim_vehicle.py" 2>/dev/null || true
pkill -9 -f "mavproxy.py" 2>/dev/null || true
pkill -9 -f "arducopter" 2>/dev/null || true


pkill -9 -f "mission_runner.py" 2>/dev/null || true
sleep 2

echo "[OK] Old processes cleaned."


# ============================================================
# TERMINAL 1 — GAZEBO
# ============================================================

echo ""
echo "[2/5] Starting Gazebo..."

gnome-terminal \
    --title="1 - Gazebo MISS2" \
    -- bash -c '
        cd ~/Prem/ajao-lelo-mera

        source .venv/bin/activate

        export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"

        export GZ_SIM_RESOURCE_PATH="$HOME/Prem/ajao-lelo-mera/models/models:$HOME/ardupilot_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"

        export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

        echo "WORLD:"
        echo "$HOME/Prem/ajao-lelo-mera/worlds/miss2_cam2_world.sdf"

        gz sim -r -v4 "$HOME/Prem/ajao-lelo-mera/worlds/miss2_cam2_world.sdf"

        exec bash
    '
echo "[OK] Gazebo started."

# Give Gazebo time to load world + cameras
sleep 8


# ============================================================
# TERMINAL 2 — ARDUPILOT + MAVPROXY
# ============================================================

echo ""
echo "[3/5] Starting ArduPilot + MAVProxy..."

gnome-terminal \
    --title="2 - ArduPilot SITL / MAVLink" \
    -- bash -c "
        source '$VENV/bin/activate'

        export PYTHONPATH='$GZ_PYTHON_PATH:\${PYTHONPATH:-}'

        cd '$ARDUPILOT/ArduCopter'

        echo ''
        echo '=============================================='
        echo '       ARDUPILOT + MAVPROXY'
        echo '=============================================='
        echo ''

        sim_vehicle.py \
            -v ArduCopter \
            -f gazebo-iris \
            --model JSON \
            --console

        echo ''
        echo 'ArduPilot closed.'
        exec bash
    "

echo "[OK] ArduPilot + MAVProxy started."

# Give SITL + MAVProxy time to start
sleep 8
# ============================================================
# TERMINAL 5 — MISSION RUNNER (AUTONOMY)
# ============================================================

echo ""
echo "[6/6] Starting Mission Runner..."

gnome-terminal \
    --title="5 - Mission Runner" \
    -- bash -c "
        source '$VENV/bin/activate'

        export PYTHONPATH='$GZ_PYTHON_PATH:\${PYTHONPATH:-}'

        cd '$REPO'

        echo ''
        echo '=============================================='
        echo '       AUTONOMOUS MISSION RUNNER'
        echo '=============================================='
        echo ''

        python3 autonomy/behaviors/mission_runner.py

        echo ''
        echo 'Mission Runner closed.'
        exec bash
    "

echo "[OK] Mission Runner started."

sleep 2


# ============================================================
# COMPLETE
# ============================================================

echo ""
echo "=========================================================="
echo "               ALL 5 TERMINALS STARTED"
echo "=========================================================="
echo ""
echo "  [1] Gazebo MISS2"
echo "  [2] ArduPilot + MAVProxy"
echo "  [3] CAM1 Downward"
echo "  [4] CAM2 Forward"
echo "  [5] Mission Runner"
echo ""
echo "  Repo:"
echo "  $REPO"
echo ""
echo "  Venv:"
echo "  $VENV"
echo ""
echo "  MAVLink:"
echo "  127.0.0.1:$MAVLINK_PORT"
echo ""
echo "=========================================================="
echo ""
