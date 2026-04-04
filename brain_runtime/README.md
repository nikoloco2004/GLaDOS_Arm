# brain_runtime

Runs on the **gaming laptop**: connects to `pi_runtime`, sends commands, receives telemetry.

Phase 2: run `personality_core` here (mic, ASR, Ollama, TTS) and translate LLM tool calls → `command` messages.

## Install

```bash
pip install -e ./robot_link
pip install -e ./brain_runtime
```

## Run (smoke test)

```bash
export PI_WS_URL=ws://raspberrypi.local:8765
python -m brain_runtime
```

Sends `ping` then `neutral` (stub on Pi).
