# pi_runtime

Runs on the **Raspberry Pi**: WebSocket server, heartbeat, failsafe, command execution.

## Install (from repo root)

```bash
pip install -e ./robot_link
pip install -e ./pi_runtime
```

## Run

```bash
export PI_RUNTIME_HOST=0.0.0.0
export PI_RUNTIME_PORT=8765
python -m pi_runtime
```

## Env

| Variable | Default | Meaning |
|----------|---------|---------|
| `PI_RUNTIME_HOST` | `0.0.0.0` | Bind address |
| `PI_RUNTIME_PORT` | `8765` | TCP port |
| `PI_FAILSAFE_S` | `8.0` | No valid brain ping → failsafe |

Wire `executor.py` to `glados_arm` when ready.
