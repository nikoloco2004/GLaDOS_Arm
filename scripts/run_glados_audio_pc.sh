#!/usr/bin/env bash
# Full GLaDOS on PC: mic + speaker using configs/pi_potato.yaml
# Usage (from repo root):  bash scripts/run_glados_audio_pc.sh
# With typing + mic:        bash scripts/run_glados_audio_pc.sh --both

set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
VENV_PY="${ROOT}/personality_core/.venv/bin/python"
CONFIG="${ROOT}/configs/pi_potato.yaml"

if [[ ! -x "$VENV_PY" ]]; then
  echo "Missing venv: $VENV_PY — create personality_core/.venv and pip install -e personality_core first." >&2
  exit 1
fi
if [[ ! -f "$CONFIG" ]]; then
  echo "Missing config: $CONFIG" >&2
  exit 1
fi

cd "${ROOT}/personality_core"
if [[ "${1:-}" == "--both" ]]; then
  exec "$VENV_PY" -m glados.cli start --config "$CONFIG" --input-mode both
fi
exec "$VENV_PY" -m glados.cli start --config "$CONFIG"
