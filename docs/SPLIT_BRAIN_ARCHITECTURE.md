# Split architecture: Pi (body) + laptop (brain)

This document describes the migration from **everything on the Raspberry Pi** to a **distributed** layout: the Pi owns **hardware, safety, and telemetry**; the **gaming laptop** owns **ASR, LLM, TTS, persona, and orchestration**.

---

## 1. Current codebase map

| Area | Location | Role today | After split |
|------|-----------|------------|-------------|
| **Personality / voice / AI** | `personality_core/` (upstream GLaDOS) | Engine, ASR, TTS, ONNX, Ollama, TUI, MCP, optional vision models | **Laptop only** (`brain_runtime` + vendored `personality_core` or pip install). Pi **does not** load large models. |
| **Robot arm / kinematics** | `glados_arm/` | IK, mapping, serial to Arduino | **Pi** (`pi_runtime` calls into `glados_arm`). |
| **Firmware** | `firmware/` | Arduino servo control | **Pi** (unchanged; serial from Pi). |
| **Face / camera tracking** | `glados_arm` + Picamera2 (per README) | Local vision → joint angles | **Phase 2**: Pi streams **frames or landmarks** to laptop **or** keeps lightweight tracking on Pi and sends **high-level pose** only. |
| **Configs** | `configs/pi_potato.yaml` etc. | Single-machine GLaDOS | **Laptop** brain config; **Pi** small `pi_runtime.yaml` (host, timeouts, pins). |
| **Scripts** | `scripts/` | Pi install, Ollama | **Laptop** scripts for brain; Pi scripts slimmed (no Ollama requirement on Pi). |

**Move to laptop (rewrite “where it runs”, not necessarily delete repo code):**

- `personality_core/src/glados/core/*` (engine, LLM, listeners, TTS pipeline)
- `personality_core` ASR/TTS/Vision ONNX stacks
- Ollama / remote OpenAI — all **inference** on laptop

**Keep on Pi:**

- `glados_arm/**` (Python control stack)
- `firmware/**`
- GPIO, sensors, serial, camera **capture** (optional streaming)
- **Failsafe**: watchdog if laptop link is lost

**Shared:**

- **Message schemas** in `robot_link/` (JSON types both sides import)

---

## 2. Recommended architecture (opinionated)

**Single long-lived bidirectional connection: WebSocket + JSON.**

Why not HTTP-only? Commands and events need **both directions** without polling; HTTP + SSE is awkward for actuator acks and speech chunks.

Why not gRPC? Heavier toolchain on Pi; overkill for v1.

Why not ZeroMQ? Excellent for robotics, but adds broker patterns and ops overhead; revisit if you need multi-subscriber fan-out later.

**WebSocket** gives:

- One TCP port (e.g. `8765`), easy firewall rule
- Full-duplex, low overhead
- Browser / tools can attach for debug (optional)
- JSON matches your existing Python style and logs well

Upgrade path: same JSON payloads over **WSS** + TLS, or swap transport later; **schemas stay in `robot_link`**.

---

## 3. Text diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                     GAMING LAPTOP (brain_runtime)                 │
│  Mic + speakers (optional) OR receive audio from Pi later       │
│  ┌──────────┐   ┌─────────┐   ┌──────────┐   ┌──────────────┐  │
│  │ ASR/VAD  │──▶│  LLM    │──▶│   TTS    │──▶│ personality  │  │
│  │ (ONNX)   │   │ Ollama/ │   │ (ONNX)   │   │   config     │  │
│  └──────────┘   │ OpenAI  │   └──────────┘   └──────────────┘  │
│                 └─────────┘                                      │
│       │                    ▲                                     │
│       │  JSON over WebSocket (commands, status, audio meta)     │
└───────┼────────────────────┼─────────────────────────────────────┘
        │                    │
        ▼                    │
┌───────┴────────────────────┴─────────────────────────────────────┐
│                  RASPBERRY PI 5 (pi_runtime)                        │
│  ┌─────────────┐    ┌──────────────┐    ┌────────────────────────┐ │
│  │ WS server   │    │ Safety /     │    │ glados_arm + serial    │ │
│  │ + reconnect │    │ watchdog     │    │ → Arduino / servos    │ │
│  └─────────────┘    └──────────────┘    └────────────────────────┘ │
│  Optional: mic PCM stream → laptop (phase 2)                        │
│  Optional: camera frames → laptop (phase 2)                         │
└────────────────────────────────────────────────────────────────────┘
```

---

## 4. Target folder layout

```
GLaDOS_Arm/
  docs/
    SPLIT_BRAIN_ARCHITECTURE.md    # this file
  robot_link/                      # shared: schemas + envelope only
    pyproject.toml
    src/robot_link/
      __init__.py
      envelope.py                  # v, id, type, ts, payload
      messages.py                  # typed payloads (dataclasses)
  pi_runtime/                      # install on Pi only
    pyproject.toml
    src/pi_runtime/
      __init__.py
      config.py
      server.py                    # asyncio WebSocket server
      safety.py                    # comm loss → neutral / estop
      executor.py                  # map Command → glados_arm / GPIO
  brain_runtime/                   # install on laptop
    pyproject.toml
    src/brain_runtime/
      __init__.py
      config.py
      client.py                    # WebSocket client + reconnect
      session.py                   # optional: glue to personality_core later
  personality_core/                # unchanged upstream tree; **runs on laptop**
  glados_arm/                      # stays on Pi; imported by pi_runtime
  firmware/
  configs/
```

Vendoring: `robot_link` can be a path dependency in both `pi_runtime` and `brain_runtime` `pyproject.toml`.

---

## 5. Message envelope (all frames)

```json
{
  "v": 1,
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "type": "heartbeat",
  "ts": 1712345678.901,
  "payload": { }
}
```

---

## 6. Message types (v1)

| `type` | Direction | Purpose |
|--------|-----------|---------|
| `hello` | Pi → brain | Version, hostname, caps |
| `hello_ack` | brain → Pi | Session id, accepted protocol v |
| `heartbeat` | Pi → brain | uptime_s, rssi_or_none, arm_ok |
| `heartbeat_ack` | brain → Pi | optional echo (RTT) |
| `sensor` | Pi → brain | structured readings (GPIO, temp, etc.) |
| `command` | brain → Pi | high-level robot intent |
| `command_ack` | Pi → brain | accepted / rejected + correlation id |
| `actuator_result` | Pi → brain | outcome after motion |
| `error` | either | code, message, fatal? |
| `failsafe` | Pi → brain | reason: comm_loss, estop, watchdog |
| `user_text` | Pi → brain | typed line → LLM on brain |
| `user_audio_pcm` | Pi → brain | float32 mono mic clip → ASR on brain → LLM |
| `user_interrupt` | Pi → brain | barge-in metadata (split-brain voice loop) |
| `tts_pcm` | brain → Pi | float32 mono TTS playback |
| `speech_audio_chunk` | phase 2 | base64 pcm or ref id |
| `tts_downlink` | phase 2 | play this wav on Pi speaker |

Stub implementations use the same `type` strings in `robot_link.messages`.

---

## 7. Migration plan (phased)

### Phase 0 — Prep (no behavior change)

- Add `robot_link`, `pi_runtime`, `brain_runtime` stubs; document ports and env vars.
- Pi keeps running current `glados` + arm as today.

### Phase 1 — **Minimal remote control** (your deliverable)

- Pi runs `pi_runtime` WebSocket server only (+ safety).
- Laptop runs `brain_runtime` client; sends `command` (e.g. `{"name":"neutral"}`).
- Pi executes via existing `glados_arm` / serial; replies `command_ack` + `actuator_result`.
- Heartbeats both ways; Pi triggers **failsafe** (e.g. neutral pose) if no `heartbeat_ack` / brain ping for **N seconds**.

### Phase 2 — **Voice + LLM on laptop**

- Laptop: run `personality_core` `glados` with config pointing to local Ollama/OpenAI.
- Audio: **simplest** = mic/speaker on laptop first (no Pi audio).  
- **Next**: stream PCM from Pi mic → laptop over WebSocket binary or separate UDP; brain returns TTS audio → Pi playback.

### Phase 3 — **Vision optional**

- Pi captures JPEG/NDArray thumbnails → `vision_frame` messages; laptop runs FastVLM / GLaDOS vision stack.

### What to delete later?

- Do **not** delete `personality_core` from repo; **stop installing/running** full stack on Pi in production.
- Pi `scripts/install_personality_pi.sh` becomes optional or split into `install_pi_runtime.sh` (minimal deps).

---

## 8. Code: move / rewrite / delete (summary)

| Action | What |
|--------|------|
| **Keep** | `glados_arm`, `firmware`, `personality_core` (as laptop brain library) |
| **New** | `robot_link`, `pi_runtime`, `brain_runtime` |
| **Rewrite** | Nothing in `glados_arm` core math — only **add** a thin `pi_runtime.executor` that calls existing APIs |
| **Delete (later)** | Pi systemd user services that launch full `uv run glados` if you replace with `pi_runtime` |

---

## 9. OpenAI / local LLM compatibility

Brain side keeps using existing `GladosConfig.completion_url` — **Ollama on laptop**, or **OpenAI-compatible** remote API. No change to protocol: only the **brain** process reads YAML.

---

## 10. Logging & reconnection

- **Brain client**: exponential backoff reconnect; log each connect/disconnect.
- **Pi server**: log each peer; on disconnect run **failsafe** path.
- Correlation: every `command.id` echoed in `command_ack` / `actuator_result`.

---

## 11. Minimal v1 acceptance test

1. Start `pi_runtime` on Pi (`python -m pi_runtime` or uv).
2. Start `brain_runtime` on laptop with `PI_WS_URL=ws://pi.local:8765`.
3. Send one `command` → Pi moves arm or prints serial.
4. Observe `actuator_result` + heartbeat in logs.

---

## 12. Phase 2 acceptance (voice)

1. Laptop runs GLaDOS; user speaks into laptop mic.
2. TTS audio plays on laptop speaker.
3. (Optional) Same pipeline with audio routed Pi↔laptop as designed in phase 2.

This file is the **source of truth** for the split; implementation stubs live under `robot_link/`, `pi_runtime/`, `brain_runtime/`.

**Setup the gaming laptop or main PC as the brain** (Ollama, `PI_WS_URL`, hotswap between machines): see [`LAPTOP_BRAIN_SETUP.md`](LAPTOP_BRAIN_SETUP.md).
