#!/usr/bin/env bash
set -euo pipefail

# Start both:
#   1) arm face-tracking loop
#   2) Pi brain bridge (pi_runtime websocket server for laptop brain_runtime)
#
# Override commands without editing this file:
#   ARM_CMD='...' BRIDGE_CMD='...' bash scripts/start_arm_and_glados.sh

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

BRIDGE_PY="${ROOT}/.venv/bin/python"
if [[ ! -x "${BRIDGE_PY}" ]]; then
  BRIDGE_PY="python3"
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

has_python_module() {
  local py_bin="$1"
  local module_name="$2"
  "${py_bin}" - <<PY >/dev/null 2>&1
import importlib.util
raise SystemExit(0 if importlib.util.find_spec("${module_name}") else 1)
PY
}

# Picamera2 is commonly installed via apt for system python, not venv.
if ! has_python_module "${ARM_PY}" picamera2; then
  if command -v python3 >/dev/null 2>&1 && has_python_module python3 picamera2; then
    echo "Arm venv missing picamera2; falling back arm runtime to system python3."
    ARM_PY="python3"
  else
    echo "Missing picamera2 for arm runtime."
    echo "Install on Pi: sudo apt install -y python3-picamera2"
    exit 1
  fi
fi

ARM_PORT="${ARM_PORT:-/dev/ttyACM0}"
ARM_CMD_DEFAULT="${ARM_PY} -m glados_arm.main track --preview --control-mode ik --color-mode bgr --port ${ARM_PORT}"
PI_RUNTIME_HOST="${PI_RUNTIME_HOST:-0.0.0.0}"
PI_RUNTIME_PORT="${PI_RUNTIME_PORT:-8765}"
BRIDGE_CMD_DEFAULT="PI_RUNTIME_HOST=${PI_RUNTIME_HOST} PI_RUNTIME_PORT=${PI_RUNTIME_PORT} ${BRIDGE_PY} -m pi_runtime"

ARM_CMD="${ARM_CMD:-${ARM_CMD_DEFAULT}}"
BRIDGE_CMD="${BRIDGE_CMD:-${BRIDGE_CMD_DEFAULT}}"

USE_PTY_FOR_CHATBOT="${USE_PTY_FOR_CHATBOT:-0}"
STOP_GRACE_S="${STOP_GRACE_S:-4}"
ARM_NEUTRAL_ON_EXIT="${ARM_NEUTRAL_ON_EXIT:-1}"

echo "Starting arm + pi_runtime..."
echo "ARM_CMD: ${ARM_CMD}"
echo "BRIDGE_CMD: ${BRIDGE_CMD}"
echo "Logs:"
echo "  Arm: ${ARM_LOG}"
echo "  Bridge: ${BOT_LOG}"

cleanup() {
  local code=$?
  echo
  echo "Stopping processes..."

  graceful_stop_pid() {
    local pid="$1"
    local name="$2"
    [[ -z "${pid}" ]] && return 0
    if ! kill -0 "${pid}" 2>/dev/null; then
      return 0
    fi
    # First try SIGINT so Python apps can run cleanup/finally handlers.
    kill -INT "${pid}" 2>/dev/null || true
    local deadline=$((SECONDS + STOP_GRACE_S))
    while kill -0 "${pid}" 2>/dev/null; do
      if (( SECONDS >= deadline )); then
        break
      fi
      sleep 0.2
    done
    if kill -0 "${pid}" 2>/dev/null; then
      echo "${name} still running after ${STOP_GRACE_S}s, sending SIGTERM..."
      kill -TERM "${pid}" 2>/dev/null || true
      sleep 0.5
    fi
    if kill -0 "${pid}" 2>/dev/null; then
      echo "${name} still running, sending SIGKILL..."
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  }

  graceful_stop_pid "${ARM_PID:-}" "Arm"
  graceful_stop_pid "${BOT_PID:-}" "Bridge"

  # Fallback: if arm process didn't neutralize itself, send one direct command.
  if [[ "${ARM_NEUTRAL_ON_EXIT}" == "1" ]] && [[ -n "${ARM_PORT:-}" ]] && [[ -e "${ARM_PORT}" ]]; then
    {
      stty -F "${ARM_PORT}" 115200 raw -echo 2>/dev/null || true
      printf "NEUTRAL\n" > "${ARM_PORT}" 2>/dev/null || true
    } || true
  fi

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

# Run bridge from repo root. PTY is optional but usually unnecessary for pi_runtime.
if [[ "${USE_PTY_FOR_CHATBOT}" == "1" ]] && command -v script >/dev/null 2>&1; then
  (cd "${ROOT}" && script -q -e -c "${BRIDGE_CMD}" /dev/null) >"${BOT_LOG}" 2>&1 &
else
  (cd "${ROOT}" && bash -lc "${BRIDGE_CMD}") >"${BOT_LOG}" 2>&1 &
fi
BOT_PID=$!

echo "PIDs: arm=${ARM_PID}, chatbot=${BOT_PID}"
echo "Press Ctrl+C to stop both."

# Early health probe so startup failures are obvious.
sleep 2
if ! kill -0 "${ARM_PID}" 2>/dev/null; then
  echo "Arm process failed during startup."
  echo "--- tail ${ARM_LOG} ---"
  tail -n 80 "${ARM_LOG}" || true
  exit 1
fi
if ! kill -0 "${BOT_PID}" 2>/dev/null; then
  echo "Bridge process failed during startup."
  echo "--- tail ${BOT_LOG} ---"
  tail -n 80 "${BOT_LOG}" || true
  exit 1
fi

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
  echo "Bridge process exited (code=${BOT_EXIT})."
  echo "--- tail ${BOT_LOG} ---"
  tail -n 60 "${BOT_LOG}" || true
fi

echo "One process exited. Shutting down the other."

