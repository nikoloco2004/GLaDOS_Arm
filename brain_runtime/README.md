# brain_runtime

Runs on the **gaming laptop**: connects to `pi_runtime`, sends commands, receives telemetry.

Phase 2: run `personality_core` here (mic, ASR, Ollama, TTS) and translate LLM tool calls → `command` messages.

## Install

```bash
pip install -e ./robot_link
pip install -e ./brain_runtime
```

## Run (smoke test)

From repo root, after copying `configs/brain.env.example` → `configs/brain.env` and setting `PI_WS_URL`:

```bash
source scripts/brain_env.sh
cd personality_core   # avoid shadowing: repo folder `brain_runtime/` breaks `python -m brain_runtime` from root
python -m brain_runtime
```

Windows PowerShell: `. .\scripts\brain_env.ps1` then either `. .\scripts\run_brain_runtime.ps1` or `cd personality_core` and `.\.venv\Scripts\python.exe -m brain_runtime`.

Full laptop/desktop setup and hotswap between main PC and laptop: [`docs/LAPTOP_BRAIN_SETUP.md`](../docs/LAPTOP_BRAIN_SETUP.md).

Sends `ping` then `neutral` (stub on Pi).
