#!/usr/bin/env bash
# One-time on the Pi: install editable personality_core + download Silero/ASR assets so always-on mic works.
#
# NEVER uses system Python/pip (PEP 668 on Raspberry Pi OS). Only these interpreters:
#   $PI_SETUP_PYTHON, or ./.venv/bin/python, or ./personality_core/.venv/bin/python
#
# Create the venv first:
#   cd ~/Documents/Cursor/GLaDOS_Arm && python3 -m venv .venv
#   .venv/bin/python -m pip install -U pip
#   .venv/bin/python -m pip install -e ./robot_link -e ./pi_runtime

set -euo pipefail
_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
ROOT="$(cd "$_SCRIPT_DIR/.." && pwd)"
cd "$ROOT"

resolve_py() {
  if [ -n "${PI_SETUP_PYTHON:-}" ]; then
    if [ -x "${PI_SETUP_PYTHON}" ]; then
      echo "${PI_SETUP_PYTHON}"
      return 0
    fi
    echo "PI_SETUP_PYTHON is not executable: ${PI_SETUP_PYTHON}" >&2
    exit 1
  fi
  for candidate in "$ROOT/.venv/bin/python" "$ROOT/.venv/bin/python3" \
                   "$ROOT/personality_core/.venv/bin/python" "$ROOT/personality_core/.venv/bin/python3"; do
    if [ -x "$candidate" ]; then
      echo "$candidate"
      return 0
    fi
  done
  return 1
}

if ! PY="$(resolve_py)"; then
  echo "" >&2
  echo "No venv Python found under this repo (will not use system pip — PEP 668)." >&2
  echo "  Repo root: $ROOT" >&2
  echo "" >&2
  echo "Create .venv here, install robot_link + pi_runtime, then re-run:" >&2
  echo "  cd \"$ROOT\"" >&2
  echo "  python3 -m venv .venv" >&2
  echo "  .venv/bin/python -m pip install -U pip" >&2
  echo "  .venv/bin/python -m pip install -e ./robot_link -e ./pi_runtime" >&2
  echo "  bash scripts/pi_setup_mic_stream.sh" >&2
  echo "" >&2
  echo "Or:  PI_SETUP_PYTHON=/path/to/venv/bin/python bash scripts/pi_setup_mic_stream.sh" >&2
  exit 1
fi

echo "Repo root: $ROOT"
echo "Using Python: $PY ($("$PY" --version 2>&1))"

echo "==> pip install -e personality_core[cpu]"
"$PY" -m pip install -U pip
"$PY" -m pip install -e "./personality_core[cpu]"

echo "==> glados.cli download (VAD + assets under personality_core/models/)"
echo "    Using --sequential to avoid parallel GitHub overload on Raspberry Pi."
cd "$ROOT/personality_core"
"$PY" -m glados.cli download --sequential
cd "$ROOT"

echo "Done. Start pi_runtime with the SAME interpreter as above (not /usr/bin/python):"
echo "  export PI_RUNTIME_HOST=0.0.0.0 PI_RUNTIME_PORT=8765"
echo "  \"$PY\" -m pi_runtime"
echo "Or: source .venv/bin/activate  # then  python -m pi_runtime"
echo "Push-to-talk only: export PI_MIC_MODE=push"
