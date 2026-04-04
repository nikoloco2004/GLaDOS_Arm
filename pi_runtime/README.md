# pi_runtime

Runs on the **Raspberry Pi**: WebSocket server, heartbeat, failsafe, command execution.

## Install (from repo root)

**Debian / Raspberry Pi OS** blocks system-wide `pip` (PEP 668: “externally managed environment”). Use a **venv** — do not use `--break-system-packages` for this.

**Option A — venv at repo root** (recommended if you only run `pi_runtime` here):

```bash
cd ~/Documents/Cursor/GLaDOS_Arm   # your clone path
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ./robot_link -e ./pi_runtime
```

**Option B — reuse `personality_core`’s venv** (if you already use it for GLaDOS on the Pi):

```bash
cd ~/Documents/Cursor/GLaDOS_Arm/personality_core
source .venv/bin/activate
python -m pip install -e ../robot_link -e ../pi_runtime
```

Use **`python -m pip`**, not bare **`pip`**. On some setups `pip` resolves to the **system** installer and triggers PEP 668 even after `activate`; `python -m pip` always uses the venv’s Python.

After either option, check:

```bash
which python    # …/GLaDOS_Arm/.venv/bin/python or …/personality_core/.venv/bin/python
python -m pip --version   # should mention that same venv path
```

### `No module named pip` inside the venv

That means the environment was created **without pip** (broken/minimal venv) or `python3-venv` was incomplete.

**Fix (pick one):**

1. **Install pip into the existing venv** (safest first try):

   ```bash
   source .venv/bin/activate   # or personality_core/.venv
   python -m ensurepip --upgrade
   python -m pip install --upgrade pip
   ```

2. **Recreate the venv** after OS packages are correct:

   ```bash
   sudo apt update
   sudo apt install -y python3-venv python3-full python3-pip
   cd ~/Documents/Cursor/GLaDOS_Arm/personality_core   # or repo root for .venv
   rm -rf .venv
   python3 -m venv .venv
   source .venv/bin/activate
   python -m pip install --upgrade pip
   python -m pip install -e ../robot_link -e ../pi_runtime
   ```

3. **Bootstrap pip manually** if `ensurepip` fails:

   ```bash
   source .venv/bin/activate
   curl -sS https://bootstrap.pypa.io/get-pip.py | python
   ```

## Run

Use the **venv’s Python** (where `pi_runtime` is installed). If you see  
`No module named pi_runtime.__main__` and the path is **`/usr/bin/python`**, you ran **system** Python instead of `.venv/bin/python`.

```bash
cd ~/Documents/Cursor/GLaDOS_Arm   # repo root
export PI_RUNTIME_HOST=0.0.0.0
export PI_RUNTIME_PORT=8765
./.venv/bin/python -m pi_runtime
```

Or activate first, then `python -m pi_runtime`:

```bash
source .venv/bin/activate
export PI_RUNTIME_HOST=0.0.0.0 PI_RUNTIME_PORT=8765
python -m pi_runtime
```

Console script (same venv): **`pi-runtime`** (hyphen) after `activate`, or **`.venv/bin/pi-runtime`** from repo root.

### Always-on microphone (default)

By default the Pi tries **continuous Silero VAD** on the default input: speech is segmented and sent as `user_audio_pcm` to the PC brain (no need to type `/mic`). Requires **`personality_core`** + **`python -m glados.cli download`** on the Pi (see `scripts/pi_setup_mic_stream.sh`).

- **Push-to-talk only:** `export PI_MIC_MODE=push` before `python -m pi_runtime` (then use `/mic` as before).
- **Disable mic uplink entirely:** `export PI_MIC_UPLINK=0`.

IPv6 dual-stack: use `export PI_RUNTIME_HOST=::` if you prefer (see env table below).

**No venv** (not recommended): you can run from source without installing, but you must set `PYTHONPATH` to both `src` trees:

```bash
cd ~/Documents/Cursor/GLaDOS_Arm
export PYTHONPATH="$(pwd)/robot_link/src:$(pwd)/pi_runtime/src"
python3 -m pi_runtime
```

## Env

| Variable | Default | Meaning |
|----------|---------|---------|
| `PI_RUNTIME_HOST` | `::` | Bind address. **`::`** listens on **IPv6 and IPv4** (dual-stack on typical Linux). Use `0.0.0.0` for IPv4-only. If `::` fails to bind, the server falls back to `0.0.0.0`. |
| `PI_RUNTIME_PORT` | `8765` | TCP port |
| `PI_FAILSAFE_S` | `60.0` | Seconds without **inbound** brain traffic (or after your last Pi uplink) before failsafe; voice/ASR/LLM often needs ≥60s |
| `PI_VOICE_INTERRUPT` | `1` | `0` to disable mic barge-in during TTS playback |
| `PI_STDIN_INTERRUPT` | `1` | `0` to disable stopping speech when you type a new line while she talks |
| `PI_SD_INPUT_DEVICE` / `GLADOS_SD_INPUT_DEVICE` | default | PortAudio input index for interrupt detection |
| `PI_INTERRUPT_DELAY_MS` | `280` | Wait after TTS starts before listening (reduces speaker→mic feedback) |
| `PI_INTERRUPT_RMS` | `0.028` | RMS threshold; raise if false triggers, lower if too hard to interrupt |
| `PI_INTERRUPT_HITS` | `4` | Consecutive loud blocks required before stop |
| `PI_INTERRUPT_BLOCKSIZE` | `512` | Input block size for RMS |
| `PI_MIC_MODE` | *(on)* | **Default: continuous VAD** (`user_audio_pcm` segments). Set **`push`** (or `ptt`, `off`, `0`) for **`/mic` only**. |
| `PI_MIC_COMMAND` | `/mic` | Push-to-talk: type this + Enter → fixed-length capture |
| `PI_MIC_SECONDS` | `5` | Seconds of mic capture per `/mic` |
| `PI_MIC_UPLINK` | `1` | `0` to disable mic uplink (typing only) |
| `PI_MIC_STREAM_MIN_MS` / `PI_MIC_STREAM_MAX_MS` | `200` / `30000` | VAD utterance length limits (continuous mode) |

Wire `executor.py` to `glados_arm` when ready.

**Pi mic → PC brain:** [`docs/VOICE_MIC_PI_PC.md`](../docs/VOICE_MIC_PI_PC.md).
