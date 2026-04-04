# GLaDOS + Pi stack — context for future merge (arm + voice)

This note is for future work when **robot arm / movement** and **GLaDOS speaking (split brain)** are merged into one coherent codebase. It records **paths**, **layout**, and **what was wired** on this setup.

---

## Frozen baseline (do not casually refactor)

**User intent:** The **split-brain voice path is confirmed working** (Pi `pi_runtime` + PC `brain_runtime`, WebSocket, mic → ASR on PC, TTS on Pi). **Leave AI, ASR engines, recognition pipelines, and related personality_core/brain_runtime inference code untouched** unless there is an explicit request or a hard bug. Prefer **environment variables** and **config** for small tuning, not rewrites.

**Future work (robot arm):** Run **in tandem** with this stack — keep **one long-lived `pi_runtime` process** on the Pi (WebSocket + audio + stdin loop). Extend **`pi_runtime` command / `executor` handling** and **`robot_link`** messages for arm motion; avoid a second process grabbing the same ALSA mic. Firmware: `firmware/GLaDOS_Arm/GLaDOS_Arm.ino` + serial from Pi as already planned in `docs/SPLIT_BRAIN_ARCHITECTURE.md`.

**Bluetooth / AirPods Pi work** was **reverted** from `pi_runtime` mic logic; do not reintroduce “prefer USB over BT default” without a dedicated task.

---

## Paths (this machine / Pi)

| Role | Path |
|------|------|
| **Windows repo (Cursor workspace)** | `C:\Users\Nikol\OneDrive\Documents\GLaDOS\GLaDOS Cursor` |
| **Typical venv (brain / GLaDOS)** | `...\personality_core\.venv\` — run `.\run_brain_runtime.ps1` or `..\scripts\run_brain_runtime.ps1` from `personality_core` |
| **Raspberry Pi repo** | `~/Documents/Cursor/GLaDOS_Arm` (user `nicopi` → full e.g. `/home/nicopi/Documents/Cursor/GLaDOS_Arm`) |
| **Pi venv** | `$REPO/.venv/bin/python` — **not** system `/usr/bin/python` for `pi_runtime` / `glados.cli` |

Git remote used in-thread: **`nikoloco2004/GLaDOS_Arm`** (confirm on `git remote -v`).

---

## Repo layout (relevant to merge)

- **`personality_core/`** — GLaDOS ONNX (ASR, TTS, VAD models under `personality_core/models/` after `glados.cli download`), Ollama chat on PC.
- **`brain_runtime/`** — PC WebSocket **client**: Pi `user_text` / `user_audio_pcm` → Ollama → TTS → `tts_pcm` to Pi.
- **`pi_runtime/`** — Pi WebSocket **server**: mic (VAD), speaker playback, stdin voice loop, `robot_link` envelopes.
- **`robot_link/`** — Shared message types (`Envelope`, `tts_pcm`, `user_audio_pcm`, `command`, etc.).
- **`scripts/`** — `run_brain_runtime.ps1`, `run_glados_audio_pc.ps1`, `pi_setup_mic_stream.sh`.
- **`configs/`** — `pi_potato.yaml`, `pi_potato_system_prompt.txt`, `brain.env` / examples.

Arm work (when present) should hook **`pi_runtime` `executor.py`** / `command` handling and stay compatible with **`robot_link`** envelopes.

---

## Behaviors implemented (split-brain session)

Use this as a checklist when merging with arm control:

- **Single brain WebSocket** — Second PC client gets close code **1008**; `brain_runtime` exits on 1008 instead of reconnect spam.
- **Model downloads on Pi** — `glados.cli download --sequential`; **`--only-vad --sequential`** for fixing only Silero VAD; avoids parallel GitHub overload.
- **Interrupts** — Pi **stdin** (blank Enter stops TTS); PC **`brain_runtime` terminal** sends `command: interrupt_playback`; playback runs in **`asyncio.create_task`** + **inbound queue** so interrupts are processed **during** long `tts_pcm`, not after.
- **Voice over TTS** — **`PI_STREAM_VOICE_DURING_TTS`** defaults **on** (opt out with `0` if speaker→mic echo); barge-in via Silero + `PI_INTERRUPT_*`.
- **Failsafe** — Default **`PI_FAILSAFE_S=60`**; watchdog fed during Pi TTS playback, and after Pi **uplink** (`user_text` / `user_audio_pcm` / `user_interrupt`) so ASR+LLM gaps don’t false-trigger.
- **Persona / refusals** — `configs/pi_potato_system_prompt.txt` (+ yaml sync) strengthened against generic LLM “I cannot…” refusals in **in-fiction** Portal roleplay.
- **Interrupt flavor** — `brain_runtime` `append_interrupt_context` + optional **`GLADOS_INTERRUPT_HINT`** for a short GLaDOS reaction after cut-off.

---

## Merge considerations (arm + voice)

1. **One process on Pi** — Keep **`pi_runtime`** as the single long-lived service: WebSocket + audio + future **`execute_command`** for arm; avoid a second process opening the same ALSA mic.
2. **`robot_link`** — Extend **`command` / payloads** for arm telemetry and motion; keep **heartbeat** semantics so the watchdog story stays one place.
3. **PC** — **`brain_runtime`** remains the “brain” unless you fold it into **`personality_core`**; either way, **one** WebSocket to the Pi matches current **`pi_runtime`** design.
4. **Paths in docs/scripts** — Replace hardcoded `Documents/Cursor/GLaDOS_Arm` with **`$REPO`** / env vars in new scripts so Windows vs Pi stays clear.

---

## Quick reference commands

**Pi (after `cd` to repo) — working mic + VAD + brain:**

```bash
cd ~/Documents/Cursor/GLaDOS_Arm
export XDG_RUNTIME_DIR=/run/user/$(id -u)
export PI_RUNTIME_HOST=0.0.0.0 PI_RUNTIME_PORT=8765
export PI_STREAM_VOICE_DURING_TTS=1
export PI_INTERRUPT_HITS=2
export PI_INTERRUPT_DELAY_MS=400
export PI_VAD_THRESHOLD=0.45
export PI_MIC_DEBUG=1
./.venv/bin/python -m pi_runtime
```

Pin the mic only if needed: `export GLADOS_SD_INPUT_DEVICE=<index>` (from `python -c "import sounddevice as sd; print(sd.query_devices())"`). Optional: `PI_MIC_INPUT_DEVICE=pulse` for PipeWire/Pulse logical device.

**Minimal Pi (no extra tuning):**

```bash
export PI_RUNTIME_HOST=0.0.0.0 PI_RUNTIME_PORT=8765
./.venv/bin/python -m pi_runtime
```

**PC (from `personality_core`):**

```powershell
.\run_brain_runtime.ps1
# $env:PI_WS_URL = "ws://<pi-host>:8765"
```

---

*Last updated: 2026-04-04 — frozen “working” split-brain baseline; Pi launch block; explicit no-touch on AI/recognition; arm tandem notes.*
