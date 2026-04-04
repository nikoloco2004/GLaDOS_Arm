#!/usr/bin/env bash
# One-time on the Pi: install editable personality_core + download Silero/ASR assets so always-on mic works.
# Run from repo root with your venv activated:
#   source .venv/bin/activate   # or personality_core/.venv
#   bash scripts/pi_setup_mic_stream.sh

set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

if ! command -v python >/dev/null 2>&1; then
  echo "Activate a venv first (e.g. source .venv/bin/activate)" >&2
  exit 1
fi

echo "==> pip install -e personality_core (CPU ONNX)"
python -m pip install -U pip
python -m pip install -e "./personality_core[cpu]"

echo "==> glados.cli download (VAD + small assets under personality_core/models/)"
cd "$ROOT/personality_core"
python -m glados.cli download
cd "$ROOT"

echo "Done. Start pi_runtime as usual; default is always-on VAD mic."
echo "Push-to-talk only: export PI_MIC_MODE=push"
