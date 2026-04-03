#!/usr/bin/env bash
# Quick checks after install on the Pi (or Linux): uv, venv, config parse, Ollama reachability.
# Does not run full voice/TTS (needs models + audio). Run from repo root.
#
# Usage:
#   chmod +x scripts/smoke_test_personality.sh
#   ./scripts/smoke_test_personality.sh

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PC="${REPO_ROOT}/personality_core"
CFG="${REPO_ROOT}/configs/pi_potato.yaml"

echo "==> Repo root: ${REPO_ROOT}"
cd "${PC}"

export PATH="${HOME}/.local/bin:${HOME}/.cargo/bin:${PATH}"

if ! command -v uv >/dev/null 2>&1; then
  echo "error: uv not found. Run scripts/install_personality_pi.sh first." >&2
  exit 1
fi

if [[ ! -d .venv ]]; then
  echo "error: personality_core/.venv missing. Run scripts/install_personality_pi.sh first." >&2
  exit 1
fi

echo "==> Parse pi_potato.yaml via GladosConfig"
export GLADOS_TEST_CFG="${CFG}"
uv run python -c "
import os
from pathlib import Path
from glados.core.engine import GladosConfig
p = Path(os.environ['GLADOS_TEST_CFG'])
c = GladosConfig.from_yaml(p)
print('  llm_model:', c.llm_model)
print('  completion_url:', c.completion_url)
print('  autonomy:', c.autonomy.enabled if c.autonomy else None)
print('ok: GladosConfig loaded')
"

echo "==> glados CLI"
uv run glados --help >/dev/null
echo "ok: glados --help"

echo "==> Ollama (optional)"
if command -v curl >/dev/null 2>&1; then
  if curl -fsS --connect-timeout 2 http://127.0.0.1:11434/api/tags >/dev/null 2>&1; then
    echo "ok: Ollama responding on http://127.0.0.1:11434"
  else
    echo "warn: Ollama not reachable on :11434 — run scripts/setup_ollama_pi.sh"
  fi
else
  echo "skip: curl not installed"
fi

echo ""
echo "Smoke test finished. Next: uv run glados say \"test\"  OR  full start with ../configs/pi_potato.yaml"
