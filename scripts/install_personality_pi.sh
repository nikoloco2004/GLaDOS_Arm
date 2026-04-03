#!/usr/bin/env bash
# Install system deps + run upstream GLaDOS installer on Raspberry Pi OS (Bookworm / Pi 5).
# Usage (from repo clone on the Pi):
#   chmod +x scripts/install_personality_pi.sh
#   ./scripts/install_personality_pi.sh
#
# Requires network. Models downloaded by "glados download" can be large (~hundreds of MB).

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PC="${REPO_ROOT}/personality_core"

if [[ ! -f "${PC}/scripts/install.py" ]]; then
  echo "error: ${PC}/scripts/install.py not found. Clone this repo with personality_core included." >&2
  exit 1
fi

echo "==> APT: PortAudio (mic/speaker), build tools, Python"
sudo apt-get update
sudo apt-get install -y \
  libportaudio2 \
  libportaudio2-dev \
  portaudio19-dev \
  ffmpeg \
  git \
  curl \
  build-essential \
  pkg-config \
  python3 \
  python3-venv \
  python3-dev

echo "==> uv (Python toolchain; upstream install.py expects it)"
if ! command -v uv >/dev/null 2>&1; then
  curl -LsSf https://astral.sh/uv/install.sh | sh
fi
# Typical install locations for the curl script
export PATH="${HOME}/.local/bin:${HOME}/.cargo/bin:${PATH}"
if ! command -v uv >/dev/null 2>&1; then
  echo "error: uv not on PATH after install. Open a new shell or add ~/.local/bin to PATH." >&2
  exit 1
fi

echo "==> Upstream install (CPU ONNX + model download; may take several minutes)"
cd "${PC}"
python3 scripts/install.py

echo ""
echo "==> Done with Python deps and models under ${PC}/.venv"
echo "Next steps:"
echo "  1) Install Ollama for Linux ARM64: https://ollama.com/download/linux"
echo "  2) ollama pull llama3.2:1b"
echo "  3) ollama serve   # if not already a systemd service"
echo "  4) cd ${PC} && uv run glados start --config ../configs/pi_potato.yaml --input-mode audio"
echo ""
echo "If the wrong mic/speaker is used: arecord -l && aplay -l, then set default device (ALSA/PipeWire)."
