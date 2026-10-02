#!/usr/bin/env bash
set -euo pipefail
source "$HOME/sae_mission2/world/experiments/corridor_only/env.sh"
mkdir -p "$CORRIDOR_TEST_ROOT/artifacts"
if [[ "${1:-}" == "--inspect" ]]; then
  python3 -u "$CORRIDOR_TEST_ROOT/run.py" \
    --output "$CORRIDOR_TEST_ROOT/artifacts/inspection" \
    2>&1 | tee "$CORRIDOR_TEST_ROOT/artifacts/inspection.log"
elif [[ $# == 0 ]]; then
  python3 -u "$CORRIDOR_TEST_ROOT/run.py" --fly \
    --output "$CORRIDOR_TEST_ROOT/artifacts/flight" \
    2>&1 | tee "$CORRIDOR_TEST_ROOT/artifacts/flight.log"
else
  echo 'Usage: bash mission.sh [--inspect]' >&2
  exit 2
fi
