#!/usr/bin/env bash
# One bounded, read-only scan capture. No SITL, MAVLink or motion commands.
set -euo pipefail
source "$HOME/sae_mission2/world/experiments/full_world_lidar_ab/env.sh"
sudo ip link set dev lo multicast on
sudo ip route replace 239.255.0.7/32 dev lo src 127.0.0.1

case "${1:-}" in
  top|bottom) variant="$1" ;;
  *) echo 'Usage: bash probe.sh {top|bottom}' >&2; exit 2 ;;
esac

export GZ_PARTITION="sae_full_lidar_ab_${variant}"
output="$FULL_LIDAR_AB_ROOT/artifacts/$variant"
mkdir -p "$output"
timeout 40s gz sim -s -r --headless-rendering "$FULL_LIDAR_AB_ROOT/${variant}_mount.sdf" \
  > "$output/gazebo.log" 2>&1 &
server_pid=$!
trap 'kill "$server_pid" 2>/dev/null || true; wait "$server_pid" 2>/dev/null || true' EXIT

python3 -u "$HOME/sae_mission2/world/experiments/corridor_only/run.py" \
  --topic /iris/lidar/scan \
  --output "$output" \
  2>&1 | tee "$output/probe.log"
