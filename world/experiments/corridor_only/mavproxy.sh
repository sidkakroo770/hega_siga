#!/usr/bin/env bash
set -euo pipefail
source "$HOME/sae_mission2/world/experiments/corridor_only/env.sh"
mkdir -p "$CORRIDOR_TEST_ROOT/artifacts/mavproxy"
cd "$CORRIDOR_TEST_ROOT/artifacts/mavproxy"
exec mavproxy.py --master=tcp:127.0.0.1:5760 \
  --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14552 \
  --streamrate=20 --console
