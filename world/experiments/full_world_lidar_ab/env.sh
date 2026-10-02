#!/usr/bin/env bash
# Environment for stationary full-environment LiDAR A/B tests.
export FULL_LIDAR_AB_ROOT="$HOME/sae_mission2/world/experiments/full_world_lidar_ab"
export GZ_VERSION=harmonic
export GZ_IP=127.0.0.1
export GZ_PARTITION=sae_full_lidar_ab
export GZ_DISCOVERY_MULTICAST_IP=239.255.0.7
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$FULL_LIDAR_AB_ROOT/models:$HOME/sae_mission2/world/models/models:$HOME/ardupilot_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"
export SDF_PATH="$GZ_SIM_RESOURCE_PATH"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
export PYTHONPATH="$HOME/sae_mission2/corridor:/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
