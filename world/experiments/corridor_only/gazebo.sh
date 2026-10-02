#!/usr/bin/env bash
set -euo pipefail
source "$HOME/sae_mission2/world/experiments/corridor_only/env.sh"
sudo ip link set dev lo multicast on
sudo ip route replace 239.255.0.7/32 dev lo src 127.0.0.1
exec gz sim -r -v3 "$CORRIDOR_TEST_ROOT/corridor_only.sdf"
