# Pi microphone → PC ASR → GLaDOS (split brain)

Same WebSocket as [`VOICE_LOOP_PI_PC.md`](VOICE_LOOP_PI_PC.md), with an extra message type:

1. **Pi** captures audio from the **Pi mic** (default input, or `GLADOS_SD_INPUT_DEVICE` / `PI_SD_INPUT_DEVICE`).
2. **PC** (`brain_runtime`) runs **Parakeet ASR** (same ONNX stack as full `glados`), then **Ollama + TTS** as for typed `user_text`.
3. **Pi** plays **`tts_pcm`** on the Pi speaker as before.

Typing and mic can both be used in one session.

## Pi: what to run

Same as the text voice loop: `python -m pi_runtime` with venv, `PI_RUNTIME_HOST`, and **`brain_runtime` connected** from the PC.

## Pi: how to talk

### Push-to-talk (default)

On the SSH terminal where `pi_runtime` runs, **type the mic command and press Enter** (not the message itself):

- Default command: **`/mic`**
- The Pi records for **`PI_MIC_SECONDS`** seconds (default **5**), then uploads PCM to the PC.

Example:

```text
/mic
```

Speak during the recording window. You should see a log like `pi → brain user_audio_pcm: … samples @ … Hz`.

### Continuous listen (Silero VAD)

Set **`PI_MIC_MODE=stream`** on the Pi so the mic stays open: **Silero VAD** (same ONNX as full Glados) runs on 32 ms frames at 16 kHz. Random noise does not trigger uploads; **speech** does. After **~640 ms** of silence, the completed utterance is sent as **`user_audio_pcm`** (16 kHz float32), same as `/mic`.

**Pi requirements for stream mode:**

- Install **`personality_core`** editable on the Pi (or ensure `glados` is importable) and run **`python -m glados.cli download`** so `silero_vad_16k_op15.onnx` is present.
- **`onnxruntime`** (pulled in by `personality_core`).

You can still use **`/mic`** for fixed-length clips when stream mode is enabled.

## PC: requirements

- Same venv as today: **`personality_core` installed editable** next to `brain_runtime` (for ASR + TTS ONNX models).
- **`python -m glados.cli download`** already run on the PC (ASR weights).
- **`python -m brain_runtime`** with `PI_WS_URL` pointing at the Pi.

## Environment variables

| Variable | Where | Meaning |
|----------|--------|---------|
| `PI_MIC_COMMAND` | Pi | Trigger string (default `/mic`). |
| `PI_MIC_SECONDS` | Pi | Record length in seconds (default `5`, clamped ~0.5–60). |
| `PI_MIC_MODE` | Pi | `stream` = continuous Silero VAD → utterances; unset = push-to-talk only. |
| `PI_MIC_STREAM_MIN_MS` | Pi | Min utterance length (default `200`) to drop noise blips. |
| `PI_MIC_STREAM_MAX_MS` | Pi | Max utterance length (default `30000`) before force-send. |
| `PI_MIC_UPLINK` | Pi | Set `0` to disable mic uplink (typing only). |
| `GLADOS_SD_INPUT_DEVICE` / `PI_SD_INPUT_DEVICE` | Pi | PortAudio **input** index for the mic. |
| `GLADOS_ASR_ENGINE` | PC | `tdt` (default) or `ctc` — same as full Glados ASR. |

## Notes

- **Push-to-talk** (`/mic`) sends a **fixed-length** clip; **stream** mode sends **VAD-segmented** utterances continuously.
- **Sample rate:** capture may use the device default; VAD always sees **16 kHz** after resampling. The brain **resamples to 16 kHz** for ASR if needed.
- **Failsafe:** long ASR + LLM can exceed `PI_FAILSAFE_S`; increase it on the Pi if you see spurious failsafes during recognition.
