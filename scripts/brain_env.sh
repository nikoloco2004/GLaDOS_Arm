#!/usr/bin/env bash
# Load configs/brain.env from repo root (parent of scripts/). Use before brain_runtime or glados on laptop/main PC.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ENV_FILE="${ROOT}/configs/brain.env"
if [[ -f "${ENV_FILE}" ]]; then
  set -a
  # shellcheck source=/dev/null
  source "${ENV_FILE}"
  set +a
  echo "brain_env: loaded ${ENV_FILE}"
else
  echo "brain_env: ${ENV_FILE} not found — set PI_WS_URL yourself or copy configs/brain.env.example"
fi
