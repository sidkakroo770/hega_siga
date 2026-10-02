#!/usr/bin/env bash
# Source this file in every test terminal. Dedicated Gazebo partition/topics.
export CORRIDOR_TEST_ROOT="$HOME/sae_mission2/world/experiments/corridor_only"
export GZ_VERSION=harmonic
export GZ_IP=127.0.0.1
export GZ_PARTITION=sae_corridor_only
export GZ_DISCOVERY_MULTICAST_IP=239.255.0.7
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$CORRIDOR_TEST_ROOT/models:$HOME/ardupilot_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"
export SDF_PATH="$GZ_SIM_RESOURCE_PATH"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
export PYTHONPATH="$HOME/sae_mission2/corridor:/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
