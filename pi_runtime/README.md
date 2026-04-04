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

## Run

Always **activate the venv** first, then:

```bash
export PI_RUNTIME_HOST=0.0.0.0
export PI_RUNTIME_PORT=8765
python -m pi_runtime
```

**No venv** (not recommended): you can run from source without installing, but you must set `PYTHONPATH` to both `src` trees:

```bash
cd ~/Documents/Cursor/GLaDOS_Arm
export PYTHONPATH="$(pwd)/robot_link/src:$(pwd)/pi_runtime/src"
python3 -m pi_runtime
```

## Env

| Variable | Default | Meaning |
|----------|---------|---------|
| `PI_RUNTIME_HOST` | `0.0.0.0` | Bind address |
| `PI_RUNTIME_PORT` | `8765` | TCP port |
| `PI_FAILSAFE_S` | `8.0` | No valid brain ping → failsafe |
| `PI_VOICE_INTERRUPT` | `1` | `0` to disable mic barge-in during TTS playback |
| `PI_SD_INPUT_DEVICE` / `GLADOS_SD_INPUT_DEVICE` | default | PortAudio input index for interrupt detection |
| `PI_INTERRUPT_DELAY_MS` | `280` | Wait after TTS starts before listening (reduces speaker→mic feedback) |
| `PI_INTERRUPT_RMS` | `0.028` | RMS threshold; raise if false triggers, lower if too hard to interrupt |
| `PI_INTERRUPT_HITS` | `4` | Consecutive loud blocks required before stop |
| `PI_INTERRUPT_BLOCKSIZE` | `512` | Input block size for RMS |

Wire `executor.py` to `glados_arm` when ready.
