#!/usr/bin/env bash
# Bounded headless sensor test. This validation model has no ArduPilot plugin.
set -euo pipefail
source "$HOME/sae_mission2/world/experiments/corridor_only/env.sh"
export GZ_PARTITION=sae_corridor_validation
export GZ_SIM_RESOURCE_PATH="$CORRIDOR_TEST_ROOT/artifacts/validation_models:$GZ_SIM_RESOURCE_PATH"
timeout 35s gz sim -s -r --headless-rendering "$CORRIDOR_TEST_ROOT/corridor_only.sdf" \
  > "$CORRIDOR_TEST_ROOT/artifacts/gazebo_sensor_validation.log" 2>&1 &
corridor_validation_pid=$!
trap 'kill "$corridor_validation_pid" 2>/dev/null || true; wait "$corridor_validation_pid" 2>/dev/null || true' EXIT
python3 -u "$CORRIDOR_TEST_ROOT/run.py" --output "$CORRIDOR_TEST_ROOT/artifacts/sensor_validation"
