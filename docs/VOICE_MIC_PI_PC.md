# Pi microphone → PC ASR → GLaDOS (split brain)

Same WebSocket as [`VOICE_LOOP_PI_PC.md`](VOICE_LOOP_PI_PC.md), with an extra message type:

1. **Pi** captures audio from the **Pi mic** (default input, or `GLADOS_SD_INPUT_DEVICE` / `PI_SD_INPUT_DEVICE`).
2. **PC** (`brain_runtime`) runs **Parakeet ASR** (same ONNX stack as full `glados`), then **Ollama + TTS** as for typed `user_text`.
3. **Pi** plays **`tts_pcm`** on the Pi speaker as before.

Typing and mic can both be used in one session.

## Pi: what to run

Same as the text voice loop: `python -m pi_runtime` with venv, `PI_RUNTIME_HOST`, and **`brain_runtime` connected** from the PC.

## Pi: how to talk

### Continuous listen (Silero VAD) — **default**

The mic stays open: **Silero VAD** (same ONNX as full Glados) runs on 32 ms frames at 16 kHz. After **~640 ms** of silence, each utterance is sent as **`user_audio_pcm`** (16 kHz float32) to the PC for ASR.

**Pi requirements (one-time):**

- From repo root: **`bash scripts/pi_setup_mic_stream.sh`** (or manually: `pip install -e ./personality_core[cpu]` and `cd personality_core && python -m glados.cli download --sequential` so `silero_vad_16k_op15.onnx` exists). If only VAD failed after a full download: `python -m glados.cli download --only-vad --sequential`.

If the VAD model is missing, `pi_runtime` logs a warning and you can still use push-to-talk below.

**Opt out (push-to-talk only):** `export PI_MIC_MODE=push` before `python -m pi_runtime`.

### Push-to-talk (`/mic`)

With **`PI_MIC_MODE=push`** (or if continuous mode could not start), on the SSH terminal **type the mic command and press Enter**:

- Default command: **`/mic`**
- The Pi records for **`PI_MIC_SECONDS`** seconds (default **5**), then uploads PCM to the PC.

When continuous mode is on, **`/mic`** still works for a fixed-length clip.

**ALSA / one mic handle:** Many Pi USB mics allow only **one** open capture stream. Stream mode keeps that handle for VAD, so a **second** mic open (e.g. old “voice interrupt” during TTS) fails with `Device unavailable`. In stream mode, **barge-in** reuses the same VAD stream (Silero speech frames), not a second `InputStream`.

**Speaker → mic (echo):** While GLaDOS speaks, the built-in mic may hear the speaker. **Default:** **`PI_STREAM_VOICE_DURING_TTS` is on** — Silero can **barge-in** (stop TTS when you talk) and segment speech, same as a headset setup. If the speaker is loud and you get **false** interrupts or bogus **`user_audio_pcm`** clips, set **`PI_STREAM_VOICE_DURING_TTS=0`** on the Pi — then use **Enter** (Pi SSH or PC `brain_runtime` terminal) to stop playback instead.

## PC: requirements

- Same venv as today: **`personality_core` installed editable** next to `brain_runtime` (for ASR + TTS ONNX models).
- **`python -m glados.cli download`** already run on the PC (ASR weights).
- **`python -m brain_runtime`** with `PI_WS_URL` pointing at the Pi.

## Environment variables

| Variable | Where | Meaning |
|----------|--------|---------|
| `PI_MIC_COMMAND` | Pi | Trigger string (default `/mic`). |
| `PI_MIC_SECONDS` | Pi | Record length in seconds (default `5`, clamped ~0.5–60). |
| `PI_MIC_MODE` | Pi | **Default:** continuous VAD. Set **`push`** / **`ptt`** / **`0`** / **`off`** for **`/mic`** only. |
| `PI_MIC_STREAM_MIN_MS` | Pi | Min utterance length (default `200`) to drop noise blips. |
| `PI_MIC_STREAM_MAX_MS` | Pi | Max utterance length (default `30000`) before force-send. |
| `PI_STREAM_VOICE_DURING_TTS` | Pi | Default **on**: VAD segments + mic barge-in **during** TTS. Set **`0`** if speaker→mic echo causes false interrupts. |
| `PI_MIC_UPLINK` | Pi | Set `0` to disable mic uplink (typing only). |
| `GLADOS_SD_INPUT_DEVICE` / `PI_SD_INPUT_DEVICE` | Pi | PortAudio **input** index for the mic. |
| `GLADOS_ASR_ENGINE` | PC | `tdt` (default) or `ctc` — same as full Glados ASR. |
| `GLADOS_SD_OUTPUT_DEVICE` / `PI_SD_OUTPUT_DEVICE` | Pi | PortAudio **output** index for TTS (set when switching USB speaker ↔ Bluetooth). |
| `PI_AUDIO_OUTPUT_SR` | Pi | Force sample rate (e.g. `48000`) if probing fails. |
| `PI_AUDIO_OUTPUT_CHANNELS` | Pi | `1` or `2` to force; default probes mono then **stereo** (many **Bluetooth** sinks reject mono). |

### Bluetooth headphones / AirPods on the Pi

After pairing, the **default** ALSA/PulseAudio output may still point at HDMI or an old USB card. List devices:

`python -c "import sounddevice as sd; print(sd.query_devices())"`

Set **`GLADOS_SD_OUTPUT_DEVICE=<index>`** for the A2DP sink. **`pi_runtime`** probes **mono, then stereo** output; Bluetooth often only accepts **stereo** (the runtime duplicates mono to L/R). If playback still fails, try **`export PI_AUDIO_OUTPUT_SR=48000`**.

## Notes

- **Default** sends **VAD-segmented** utterances continuously; **`/mic`** sends a **fixed-length** clip (always available when mic uplink is on).
- **Sample rate:** capture may use the device default; VAD always sees **16 kHz** after resampling. The brain **resamples to 16 kHz** for ASR if needed.
- **Failsafe:** long ASR + LLM can exceed `PI_FAILSAFE_S`; increase it on the Pi if you see spurious failsafes during recognition.
