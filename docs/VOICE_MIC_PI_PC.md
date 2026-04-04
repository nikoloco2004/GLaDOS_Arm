# Pi microphone → PC ASR → GLaDOS (split brain)

Same WebSocket as [`VOICE_LOOP_PI_PC.md`](VOICE_LOOP_PI_PC.md), with an extra message type:

1. **Pi** captures audio from the **Pi mic** (default input, or `GLADOS_SD_INPUT_DEVICE` / `PI_SD_INPUT_DEVICE`).
2. **PC** (`brain_runtime`) runs **Parakeet ASR** (same ONNX stack as full `glados`), then **Ollama + TTS** as for typed `user_text`.
3. **Pi** plays **`tts_pcm`** on the Pi speaker as before.

Typing and mic can both be used in one session.

## Pi: what to run

Same as the text voice loop: `python -m pi_runtime` with venv, `PI_RUNTIME_HOST`, and **`brain_runtime` connected** from the PC.

## Pi: how to talk

On the SSH terminal where `pi_runtime` runs, **type the mic command and press Enter** (not the message itself):

- Default command: **`/mic`**
- The Pi records for **`PI_MIC_SECONDS`** seconds (default **5**), then uploads PCM to the PC.

Example:

```text
/mic
```

Speak during the recording window. You should see a log like `pi → brain user_audio_pcm: … samples @ … Hz`.

## PC: requirements

- Same venv as today: **`personality_core` installed editable** next to `brain_runtime` (for ASR + TTS ONNX models).
- **`python -m glados.cli download`** already run on the PC (ASR weights).
- **`python -m brain_runtime`** with `PI_WS_URL` pointing at the Pi.

## Environment variables

| Variable | Where | Meaning |
|----------|--------|---------|
| `PI_MIC_COMMAND` | Pi | Trigger string (default `/mic`). |
| `PI_MIC_SECONDS` | Pi | Record length in seconds (default `5`, clamped ~0.5–60). |
| `PI_MIC_UPLINK` | Pi | Set `0` to disable mic uplink (typing only). |
| `GLADOS_SD_INPUT_DEVICE` / `PI_SD_INPUT_DEVICE` | Pi | PortAudio **input** index for the mic. |
| `GLADOS_ASR_ENGINE` | PC | `tdt` (default) or `ctc` — same as full Glados ASR. |

## Notes

- **Push-to-talk UX:** one line (`/mic`) starts a **fixed-length** clip. This keeps the protocol simple; streaming/VAD can be added later.
- **Sample rate:** capture uses the device’s default rate; the brain **resamples to 16 kHz** before ASR.
- **Failsafe:** long ASR + LLM can exceed `PI_FAILSAFE_S`; increase it on the Pi if you see spurious failsafes during recognition.
