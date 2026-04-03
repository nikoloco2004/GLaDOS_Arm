#!/usr/bin/env bash
# Install Ollama on Raspberry Pi OS / Linux ARM64 and pull the small chat model used by configs/pi_potato.yaml.
# Run after scripts/install_personality_pi.sh (needs curl; uses official installer).
#
# Usage:
#   chmod +x scripts/setup_ollama_pi.sh
#   ./scripts/setup_ollama_pi.sh

set -euo pipefail

MODEL="${GLADOS_OLLAMA_MODEL:-llama3.2:1b}"

echo "==> Installing Ollama (official script from ollama.com)"
curl -fsSL https://ollama.com/install.sh | sh

echo "==> Enabling Ollama service (if systemd is available)"
if command -v systemctl >/dev/null 2>&1; then
  sudo systemctl enable --now ollama || true
  sleep 2
fi

echo "==> Pulling model: ${MODEL}  (override with GLADOS_OLLAMA_MODEL=...)"
ollama pull "${MODEL}"

echo "==> Smoke test"
ollama run "${MODEL}" "Reply with exactly: ok"

echo ""
echo "Done. configs/pi_potato.yaml expects:"
echo "  llm_model: \"${MODEL}\""
echo "  completion_url: \"http://localhost:11434/api/chat\""
echo "If you change the model name, edit ../configs/pi_potato.yaml to match."
