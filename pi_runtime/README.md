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
| `PI_VOICE_INTERRUPT` | `1` | `0` to disable mic barge-in during TTS playback |
| `PI_SD_INPUT_DEVICE` / `GLADOS_SD_INPUT_DEVICE` | default | PortAudio input index for interrupt detection |
| `PI_INTERRUPT_DELAY_MS` | `280` | Wait after TTS starts before listening (reduces speaker→mic feedback) |
| `PI_INTERRUPT_RMS` | `0.028` | RMS threshold; raise if false triggers, lower if too hard to interrupt |
| `PI_INTERRUPT_HITS` | `4` | Consecutive loud blocks required before stop |
| `PI_INTERRUPT_BLOCKSIZE` | `512` | Input block size for RMS |

Wire `executor.py` to `glados_arm` when ready.
