# Voice loop: type on Pi → PC (Ollama + TTS) → audio on Pi speaker

This path uses the existing WebSocket link:

1. **Pi** (`pi_runtime`): reads lines from the **same SSH terminal** where the server runs and sends `user_text` to the PC.
2. **PC** (`brain_runtime`): receives `user_text`, calls **Ollama**, runs **GLaDOS ONNX TTS** (`personality_core`), sends `tts_pcm` (float32 mono, base64) back.
3. **Pi** plays the PCM on the **default output device** (3.5 mm / USB / HDMI as configured in ALSA).

## Prerequisites

- **PC:** Ollama running; same model as `OLLAMA_MODEL` (default `llama3.2:1b`).
- **PC:** `personality_core` installed in the **same** venv as `brain_runtime`, and `python -m glados.cli download` already run (ONNX models).
- **Pi:** `pip install` / `pip install -e` updated `pi_runtime` (pull latest) so `sounddevice` + `numpy` are installed.
- **Pi:** Audio output works (e.g. `speaker-test` or `aplay`).

## Run

**Terminal 1 — Pi**

```bash
cd ~/Documents/Cursor/GLaDOS_Arm   # your path
source .venv/bin/activate
export PI_RUNTIME_HOST=0.0.0.0
export PI_RUNTIME_PORT=8765
# optional: disable typing on this terminal
# export PI_VOICE_LOOP=0
python -m pi_runtime
```

When you see `voice loop: type lines on this terminal`, type a message and press **Enter**. (SSH session must be a TTY.)

**Terminal 2 — PC** (same venv as GLaDOS; `personality_core` + `brain_runtime` installed editable)

```powershell
cd C:\Users\pc\Documents\GLaDOS\GLaDOS_Arm\personality_core
$env:PI_WS_URL = "ws://nicopi.local:8765"
# optional overrides:
# $env:OLLAMA_URL = "http://127.0.0.1:11434"
# $env:OLLAMA_MODEL = "llama3.2:1b"
# $env:GLADOS_VOICE = "glados"
# $env:GLADOS_CHAT_SYSTEM_PROMPT = "You are GLaDOS ..."
.\.venv\Scripts\python.exe -m brain_runtime
```

Audio should play on the **Pi** speaker after a short delay.

## Environment variables

| Variable | Where | Meaning |
|----------|--------|---------|
| `PI_VOICE_LOOP` | Pi | `0` to disable stdin → `user_text` (default `1`). |
| `PI_WS_URL` | PC | WebSocket URL of the Pi. |
| `OLLAMA_URL` | PC | Ollama base, default `http://127.0.0.1:11434`. |
| `OLLAMA_MODEL` | PC | Default `llama3.2:1b`. |
| `GLADOS_VOICE` | PC | TTS voice, default `glados`. |
| `GLADOS_CHAT_SYSTEM_PROMPT` | PC | Short system prompt for chat (not full `pi_potato.yaml` unless you paste it). |

## Pi audio: “Invalid sample rate” (ALSA)

GLaDOS TTS is often **22050 Hz**; many Pi ALSA devices only accept **48000 / 44100 / 16000 Hz**. The runtime **resamples** automatically and probes a working rate on first playback.

If playback still fails, set the rate explicitly on the **Pi** before `pi_runtime`:

```bash
export PI_AUDIO_OUTPUT_SR=48000
# optional: PortAudio device index if the wrong card is used
# export GLADOS_SD_OUTPUT_DEVICE=1
```

## Notes

- **GLaDOS `start` on the PC** is separate: use it for mic + typing on the **PC**. The voice loop is **Pi keyboard → Pi speaker** via the PC brain.
- To use the **full** `pi_potato.yaml` persona for this path, set `GLADOS_CHAT_SYSTEM_PROMPT` to the same system block (or extend `pipeline.py` to load YAML later).
- **Large frames:** WebSocket max message size is raised to 12 MiB for PCM payloads.
