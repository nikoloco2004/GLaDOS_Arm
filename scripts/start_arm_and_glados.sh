#!/usr/bin/env bash
set -euo pipefail

# Start both:
#   1) arm face-tracking loop
#   2) local GLaDOS chatbot (personality_core)
#
# Override commands without editing this file:
#   ARM_CMD='...' CHATBOT_CMD='...' bash scripts/start_arm_and_glados.sh

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOG_DIR="${ROOT}/logs"
mkdir -p "${LOG_DIR}"

TS="$(date +%Y%m%d_%H%M%S)"
ARM_LOG="${LOG_DIR}/arm_${TS}.log"
BOT_LOG="${LOG_DIR}/glados_${TS}.log"

ARM_PY="${ROOT}/.venv/bin/python"
if [[ ! -x "${ARM_PY}" ]]; then
  ARM_PY="python3"
fi

BOT_PY="${ROOT}/personality_core/.venv/bin/python"
if [[ ! -x "${BOT_PY}" ]]; then
  BOT_PY="python3"
fi

ensure_python_module() {
  local py_bin="$1"
  local module_name="$2"
  local pip_pkg="$3"
  if "${py_bin}" - <<PY >/dev/null 2>&1
import importlib.util
raise SystemExit(0 if importlib.util.find_spec("${module_name}") else 1)
PY
  then
    return 0
  fi
  echo "Missing Python module '${module_name}' for ${py_bin}; installing '${pip_pkg}'..."
  "${py_bin}" -m pip install "${pip_pkg}"
}

# Arm tracking needs pyserial (`import serial`).
ensure_python_module "${ARM_PY}" serial pyserial

ARM_PORT="${ARM_PORT:-/dev/ttyACM0}"
ARM_CMD_DEFAULT="${ARM_PY} -m glados_arm.main track --preview --control-mode ik --color-mode bgr --port ${ARM_PORT}"
CHATBOT_CMD_DEFAULT="${BOT_PY} -m glados.cli start --config ${ROOT}/configs/pi_potato.yaml --input-mode both"

ARM_CMD="${ARM_CMD:-${ARM_CMD_DEFAULT}}"
CHATBOT_CMD="${CHATBOT_CMD:-${CHATBOT_CMD_DEFAULT}}"

USE_PTY_FOR_CHATBOT="${USE_PTY_FOR_CHATBOT:-1}"

echo "Starting arm + chatbot..."
echo "ARM_CMD: ${ARM_CMD}"
echo "CHATBOT_CMD: ${CHATBOT_CMD}"
echo "Logs:"
echo "  Arm: ${ARM_LOG}"
echo "  Bot: ${BOT_LOG}"

cleanup() {
  local code=$?
  echo
  echo "Stopping processes..."
  [[ -n "${ARM_PID:-}" ]] && kill "${ARM_PID}" 2>/dev/null || true
  [[ -n "${BOT_PID:-}" ]] && kill "${BOT_PID}" 2>/dev/null || true
  wait "${ARM_PID:-}" 2>/dev/null || true
  wait "${BOT_PID:-}" 2>/dev/null || true
  exit "${code}"
}
trap cleanup INT TERM EXIT

# Run arm from repo root so relative imports/config stay stable.
(cd "${ROOT}" && bash -lc "${ARM_CMD}") >"${ARM_LOG}" 2>&1 &
ARM_PID=$!

# Small stagger so camera/serial init settles before chatbot starts.
sleep 1

# Run chatbot from personality_core. Use a PTY when available because some CLI
# input modes exit immediately without a TTY.
if [[ "${USE_PTY_FOR_CHATBOT}" == "1" ]] && command -v script >/dev/null 2>&1; then
  (cd "${ROOT}/personality_core" && script -q -e -c "${CHATBOT_CMD}" /dev/null) >"${BOT_LOG}" 2>&1 &
else
  (cd "${ROOT}/personality_core" && bash -lc "${CHATBOT_CMD}") >"${BOT_LOG}" 2>&1 &
fi
BOT_PID=$!

echo "PIDs: arm=${ARM_PID}, chatbot=${BOT_PID}"
echo "Press Ctrl+C to stop both."

# If either exits, stop the other and exit.
wait -n "${ARM_PID}" "${BOT_PID}" || true
ARM_ALIVE=0
BOT_ALIVE=0
if kill -0 "${ARM_PID}" 2>/dev/null; then ARM_ALIVE=1; fi
if kill -0 "${BOT_PID}" 2>/dev/null; then BOT_ALIVE=1; fi

if [[ "${ARM_ALIVE}" -eq 0 ]]; then
  wait "${ARM_PID}" || ARM_EXIT=$?
  ARM_EXIT="${ARM_EXIT:-0}"
  echo "Arm process exited (code=${ARM_EXIT})."
  echo "--- tail ${ARM_LOG} ---"
  tail -n 60 "${ARM_LOG}" || true
fi

if [[ "${BOT_ALIVE}" -eq 0 ]]; then
  wait "${BOT_PID}" || BOT_EXIT=$?
  BOT_EXIT="${BOT_EXIT:-0}"
  echo "Chatbot process exited (code=${BOT_EXIT})."
  echo "--- tail ${BOT_LOG} ---"
  tail -n 60 "${BOT_LOG}" || true
fi

echo "One process exited. Shutting down the other."

