#!/usr/bin/env bash
# List PortAudio devices (same backend as GLaDOS sounddevice). Run on the Pi from repo root after
# personality_core venv exists. Use the printed index with GLADOS_SD_INPUT_DEVICE / GLADOS_SD_OUTPUT_DEVICE.
#
# Usage:
#   chmod +x scripts/pi_list_audio_devices.sh
#   ./scripts/pi_list_audio_devices.sh

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}/personality_core"

if ! command -v uv >/dev/null 2>&1; then
  export PATH="${HOME}/.local/bin:${HOME}/.cargo/bin:${PATH}"
fi

if command -v uv >/dev/null 2>&1; then
  uv run python -c "import sounddevice as sd; print(sd.query_devices()); print('Default input:', sd.default.device[0], 'Default output:', sd.default.device[1])"
else
  echo "error: uv not found. Install from install_personality_pi.sh or activate personality_core/.venv" >&2
  exit 1
fi
