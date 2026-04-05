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

ARM_PORT="${ARM_PORT:-/dev/ttyACM0}"
ARM_CMD_DEFAULT="${ARM_PY} -m glados_arm.main track --preview --control-mode ik --color-mode bgr --port ${ARM_PORT}"
CHATBOT_CMD_DEFAULT="${BOT_PY} -m glados.cli start --config ${ROOT}/configs/pi_potato.yaml --input-mode both"

ARM_CMD="${ARM_CMD:-${ARM_CMD_DEFAULT}}"
CHATBOT_CMD="${CHATBOT_CMD:-${CHATBOT_CMD_DEFAULT}}"

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

bash -lc "${ARM_CMD}" >"${ARM_LOG}" 2>&1 &
ARM_PID=$!

# Small stagger so camera/serial init settles before chatbot starts.
sleep 1

bash -lc "${CHATBOT_CMD}" >"${BOT_LOG}" 2>&1 &
BOT_PID=$!

echo "PIDs: arm=${ARM_PID}, chatbot=${BOT_PID}"
echo "Press Ctrl+C to stop both."

# If either exits, stop the other and exit.
wait -n "${ARM_PID}" "${BOT_PID}" || true
echo "One process exited. Shutting down the other."

