#!/usr/bin/env bash
set -euo pipefail
source "$HOME/sae_mission2/world/experiments/corridor_only/env.sh"
# Keep this experiment's parameters and flight logs separate from the mission.
mkdir -p "$CORRIDOR_TEST_ROOT/artifacts/sitl"
cd "$HOME/ardupilot/ArduCopter"
exec ../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris \
  --model JSON --no-mavproxy --use-dir "$CORRIDOR_TEST_ROOT/artifacts/sitl"
