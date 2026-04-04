# Audio GLaDOS on the PC (mic + speaker)

This is the **full** `personality_core` stack: **ASR → Ollama → TTS** on **your machine**, with **`configs/pi_potato.yaml`** (potato persona, tools off, interruptible). It is **not** the split-brain path (Pi typing → `brain_runtime` → Pi speaker); see [`VOICE_LOOP_PI_PC.md`](VOICE_LOOP_PI_PC.md) for that.

## Prerequisites

- Same venv as usual: `personality_core` installed with `pip install -e ".[cpu]"` (or CUDA extra if you use it), and `python -m glados.cli download` done once.
- **Ollama** on this PC: `ollama pull llama3.2` (matches `pi_potato.yaml`).
- Working **microphone** and **speakers/headphones** (USB headset is ideal for echo).

## Run (Windows PowerShell)

From repo root:

```powershell
cd GLaDOS_Arm\personality_core
.\.venv\Scripts\Activate.ps1
python -m glados.cli start --config ..\configs\pi_potato.yaml
```

`pi_potato.yaml` already sets `input_mode: "audio"`. To also type in the same terminal:

```powershell
python -m glados.cli start --config ..\configs\pi_potato.yaml --input-mode both
```

Or use the helper (same as above, audio-only):

```powershell
cd GLaDOS_Arm
.\scripts\run_glados_audio_pc.ps1
```

## ASR on PC vs Pi

`pi_potato.yaml` uses **`asr_engine: "ctc"`** — good on a weak **Raspberry Pi**. On a **PC with a 3050-class GPU**, you usually get **better recognition** with the TDT engine (same default as `glados_config.yaml`):

1. Copy `configs/pi_potato.yaml` to a **local** file (e.g. `configs/local_pi_potato_audio.yaml`, gitignored if you prefer), **or**
2. Change **one line** in your copy: `asr_engine: "tdt"` instead of `ctc`.

Then:

```powershell
python -m glados.cli start --config ..\configs\local_pi_potato_audio.yaml
```

## Audio devices (wrong mic / no sound)

**Windows**

- Set default recording/playback devices in **Settings → System → Sound**, or  
- Set PortAudio indices (see `personality_core` README / `GLADOS_SD_INPUT_DEVICE`, `GLADOS_SD_OUTPUT_DEVICE`).

**Linux (including Pi running full Glados, not common)**

- `bash scripts/pi_list_audio_devices.sh` lists indices; export the `GLADOS_SD_*` variables before `glados start`.

## What you get

- **Interrupt:** speak over GLaDOS — same `SoundDeviceAudioIO` + `SpeechListener` path as upstream.
- **Numbers / digits:** full app uses `SpokenTextConverter` in the TTS thread (already).
- **Personality:** unchanged — still whatever is in `pi_potato.yaml` / `personality_preprompt` (do not duplicate here).

## Troubleshooting

| Issue | What to try |
|--------|-------------|
| First utterance very slow | Normal: ONNX ASR “warm-up” on first use. |
| LLM speaks JSON or tool noise | `pi_potato.yaml` has `llm_tools_enabled: false`; keep it. |
| Garbled digits | Already handled in main TTS path; `brain_runtime`-only quirks do not apply here. |
