# GLaDOS potato arm — Pi + Arduino Uno R4 WiFi

Modular control stack: **Python on Raspberry Pi** (kinematics, IK, mapping) and **Arduino firmware** (servo execution, clamps, optional slew).

---

## 1. Architecture (short)

| Layer | Role |
|--------|------|
| **Vision / tracking** | `face_tracking` + **Picamera2** (libcamera): face bbox → proportional **base** (horizontal) + **shoulder/elbow** (vertical); tune `vision_config.py`. |
| **`glados_arm` (Python)** | **Base yaw** solves azimuth; **2R planar IK** (shoulder + elbow) solves vertical-plane tip position; **wrist** is trim only in v1. **Mapping** converts model angles ↔ servo degrees with inversion/offsets. |
| **Arduino** | Parses line commands, **clamps** per joint, optional **slew**, drives four servos. No IK. |

Arduino servo order: **1=wrist, 2=elbow, 3=base, 4=shoulder**.

---

## 2. Coordinate system

- **World**: `+Y` up, `+X` to the robot’s right (viewed from above), `+Z` forward at base yaw ψ = 0 (align with your install).
- **Base yaw ψ**: rotation about **+Y**. Only ψ performs **horizontal** aiming (image X / azimuth).
- **Vertical chain plane**: 2D FK with `+x` forward in the median plane, `+z` up. Tip `(x, z)` in **mm** is what shoulder/elbow IK solves.

Full 3D tip = rotate plane forward by ψ (see `robot_model.plane_to_world`).

---

## 3. Horizontal vs vertical decomposition

- **Horizontal**: **base servo only** — maps to model `base_yaw_rad`. Shoulder/elbow are **not** used for image X in this model.
- **Vertical**: **shoulder + elbow** (primary IK); **wrist** optional trim / future pitch compensation — **not** used for horizontal positioning.

Do **not** treat this as symmetric XY IK on both joints for image (u, v).

---

## 4. Model-space vs servo-space

| Space | Meaning |
|--------|---------|
| **Model** | `base_yaw_rad`, `q_shoulder_rad`, `q_elbow_rad`, `q_wrist_rad` — **kinematics** variables; zeros at **validated neutral pose**. |
| **Servo** | Integer **degrees** after clamp on the Pi; firmware maps clamped angle with **`map(deg, 0, 270, tick_min, tick_max)`** to PCA9685 PWM ticks — **neutral offsets**, **elbow inversion**, and scaling live in `mapping.py`. |

Elbow “inverted” behavior: implemented in mapping (`ELBOW_INVERT`): increasing upward model motion → **decreasing** servo counts from `NEUTRAL_ELBOW` when configured that way.

---

## 5. Arduino firmware

- Path: `firmware/GLaDOS_Arm/GLaDOS_Arm.ino`
- **Driver**: **Adafruit PCA9685** over **I2C** (not direct GPIO `Servo`). Install **Adafruit PWM Servo Driver Library** + **Adafruit BusIO** in Arduino Library Manager.
- **Wiring**: Uno R4 **SDA/SCL** → PCA9685 **SDA/SCL**, **common GND**. Servo power from the PCA9685 **V+** rail (appropriate supply for MG996R/DS3225 — **not** from USB). Default I2C address **0x40** (change `PCA9685_I2C_ADDR` if A0–A5 bridged).
- **Channels**: **0=wrist, 1=elbow, 2=base, 3=shoulder** (matches logical servo 1–4 order in `SET_SERVO`).
- **Degree limits and neutrals** in the sketch must match `glados_arm/config.py` (wrist/elbow/base/shoulder min, max, neutral).
- **PWM mapping:** each joint’s clamped angle is mapped with **`map(deg, 0, 270, PWM_TICK_MIN, PWM_TICK_MAX)`** into PCA9685 tick counts (same as the validated prototype using 102–512). Tune **`PWM_TICK_MIN` / `PWM_TICK_MAX`** in the sketch if overall throw needs scaling.
- **Bench:** **`SET_PWM <ch 0-3> <ticks 0-4095>`** for raw channel tests (does not update cached degrees — use **`SET_SERVO`** to resync).
- Commands: `PING`, `HELP`, `NEUTRAL`, `SET_SERVO w e b s`, **`SET_PWM`**, `SET_SLEW deg_per_sec`, `DEBUG 0|1`, `STATUS`, **`I2C_SCAN`** (debug wiring/address).
- **Slew**: `SET_SLEW 0` → instant moves; `SET_SLEW 360` → default smooth cap.

---

## 6. Python usage

```bash
cd "path/to/GLaDOS Cursor"
pip install -r requirements.txt

# FK / IK / mapping (no serial)
python -m glados_arm.main fk --shoulder-deg 0 --elbow-deg 0
python -m glados_arm.main ik 250 0
python -m glados_arm.main solve 250 0 --base-yaw 0
python -m glados_arm.main assumptions

# Serial (set port: Linux `/dev/ttyACM0`, Windows `COM3`)
python -m glados_arm.main serial --port COM3 ping
python -m glados_arm.main serial --port COM3 neutral
python -m glados_arm.main serial --port COM3 set_servo --wrist 60 --elbow 270 --base 90 --shoulder 0
```

### Raspberry Pi Camera Module (Picamera2)

Use **libcamera / Picamera2**, not `cv2.VideoCapture` on a V4L device index.

1. **Hardware / driver check** (on the Pi): `rpicam-hello -t 0` — confirm the sensor is detected.
2. **Python stack**: `sudo apt install -y python3-picamera2` (and `python3-opencv` or `pip install opencv-python-headless` per your preference).
3. **Serial to Arduino**: default port is **`/dev/ttyACM0`** ([`glados_arm/config.py`](glados_arm/config.py)); override with `--port` if needed.
4. **Run face tracking** (on the Pi, with display optional):

```bash
# Dry run: camera + detection only (no serial)
python -m glados_arm.face_tracking --no-serial

# Full: send SET_SERVO to Arduino on /dev/ttyACM0
python -m glados_arm.main track --port /dev/ttyACM0

# Narrower FOV / faster: lower resolution (defaults in vision_config.py are 1280x720 for wide view)
python -m glados_arm.main track --port /dev/ttyACM0 --width 640 --height 480

# Preview: ON automatically when DISPLAY is set (Pi desktop terminal). Force: --preview ; headless: --no-preview
python -m glados_arm.main track --preview
# If you use pip's opencv-python-headless, imshow will not work — use apt python3-opencv for a GUI build.
```

Frames are captured with **Picamera2**, converted to OpenCV arrays, Haar face detection runs, then normalized bbox error drives **base** (image X) and **shoulder/elbow** (image Y) with clamps. Adjust gains and signs in [`glados_arm/vision_config.py`](glados_arm/vision_config.py).

If preview colors look wrong on your Pi build, force camera color order:

```bash
python -m glados_arm.main track --color-mode bgr
# or
python -m glados_arm.main track --color-mode rgb
```

For FPS: lower capture size and/or detection cost (see `CAMERA_WIDTH/HEIGHT`, `DETECT_MAX_WIDTH`, `DETECT_EVERY_N_FRAMES` in `vision_config.py`).

---

## 7. Personality core (voice / LLM)

This repo includes **[dnhkng/GLaDOS](https://github.com/dnhkng/GLaDOS)** as **`personality_core/`** (mic, ASR, TTS, Ollama). It does **not** share code with `glados_arm/`; arm motion is unchanged.

**Where this repo lives (examples):**

| Machine | Path |
|--------|------|
| Raspberry Pi (this project) | `/home/nicopi/Documents/Cursor/GLaDOS_Arm/` |
| Windows (development) | `C:\Users\pc\Documents\GLaDOS\GLaDOS_Arm\` |

Use your own username if different; on the Pi, `cd` to the repo root before running the scripts below.

### Raspberry Pi 5 (recommended path)

0. **Update the clone and fix script permissions** (fresh Git checkouts often have scripts non-executable):

```bash
cd /home/nicopi/Documents/Cursor/GLaDOS_Arm
git pull origin main
chmod +x scripts/*.sh
```

If you still get **Permission denied**, run scripts with **`bash`** (no execute bit needed):

```bash
bash scripts/install_personality_pi.sh
bash scripts/setup_ollama_pi.sh
```

1. **One-shot installer** (APT + `uv` + upstream `scripts/install.py` + model download):

```bash
cd /home/nicopi/Documents/Cursor/GLaDOS_Arm   # repo root containing personality_core/
bash scripts/install_personality_pi.sh
```

2. **Put `uv` on your PATH** (the installer adds it under `~/.local/bin`). In **every new terminal** before `uv run`:

```bash
source scripts/pi_env.sh
```

To make this permanent, add one line to `~/.bashrc`:

```bash
echo 'export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

Or call `uv` by full path: `~/.local/bin/uv run glados ...`

3. **Install Ollama** (official installer + pull the small model):

```bash
bash scripts/setup_ollama_pi.sh
```

Or install manually from [ollama.com/download/linux](https://ollama.com/download/linux), then `ollama pull llama3.2:1b`. The model name must match [`configs/pi_potato.yaml`](configs/pi_potato.yaml) (`llm_model`).

4. **Run GLaDOS** with the potato config:

```bash
cd /home/nicopi/Documents/Cursor/GLaDOS_Arm
source scripts/pi_env.sh
cd personality_core
uv run glados start --config ../configs/pi_potato.yaml --input-mode audio
```

5. **Audio devices** (USB mic + amp/speaker): GLaDOS uses PortAudio via `sounddevice`. List devices and defaults:

```bash
cd /home/nicopi/Documents/Cursor/GLaDOS_Arm
source scripts/pi_env.sh
bash scripts/pi_list_audio_devices.sh
```

Pick the **device index** for your mic and speaker, then before starting GLaDOS:

```bash
source scripts/pi_env.sh
export GLADOS_SD_INPUT_DEVICE=2    # example: your USB mic index
export GLADOS_SD_OUTPUT_DEVICE=1    # example: headphone jack / USB DAC index
cd personality_core && uv run glados start --config ../configs/pi_potato.yaml --input-mode audio
```

You can also set the system default with `arecord -l` / `aplay -l` and PipeWire/ALSA (`wpctl`, `pactl set-default-source`, or `~/.asoundrc`) so the default PortAudio device is correct without env vars.

**Pi troubleshooting**

| Problem | What to do |
|--------|------------|
| `Permission denied` on `./scripts/...` | Run `chmod +x scripts/*.sh` **or** use `bash scripts/NAME.sh` |
| `uv: command not found` | Run `source scripts/pi_env.sh` or add `~/.local/bin` to `PATH` (see step 2). Install scripts must finish successfully first. |
| `smoke_test_personality.sh: No such file` | Run `git pull origin main` — that file was added in a later commit. |
| Scripts ran but new shell lost `uv` | `source ~/.bashrc` or `source scripts/pi_env.sh` |
| `git pull` refuses (local changes / untracked files) | See **Pi: fixing a messy `git pull`** below. |
| `Unable to locate package libportaudio2-dev` | Fixed in current `install_personality_pi.sh` (use `portaudio19-dev` only). `git pull` and re-run `bash scripts/install_personality_pi.sh`. |

**Notes**

- Pi-focused config: [`configs/pi_potato.yaml`](configs/pi_potato.yaml) (potato persona, `llama3.2:1b`, autonomy off, lean MCP). Change `llm_model` if you use another Ollama tag.
- The upstream installer uses **CPU ONNX** on Pi (no NVIDIA). First run downloads ASR/VAD/TTS assets; allow time and disk space.
- Do **not** enable FastVLM / heavy vision in GLaDOS on Pi until you profile CPU/RAM; use Picamera2 face tracking under **§6** for the arm.

**Manual install** (if you skip the script): `sudo apt install -y libportaudio2 portaudio19-dev build-essential python3 python3-venv`, install `uv`, then `cd personality_core && python3 scripts/install.py`.

**Smoke test** (after install + Ollama optional): from repo root, `git pull` then `source scripts/pi_env.sh && bash scripts/smoke_test_personality.sh` — loads [`configs/pi_potato.yaml`](configs/pi_potato.yaml), checks `glados --help`, probes Ollama on `:11434`.

**Pi: fixing a messy `git pull`**

If `git pull` says it would overwrite local changes or untracked files, you usually want the **GitHub version** and can reset (this discards uncommitted edits in the repo):

```bash
cd /home/nicopi/Documents/Cursor/GLaDOS_Arm
git fetch origin
git reset --hard origin/main
```

If untracked files still block a merge, remove only the conflicting paths (example: old hand-copied `scripts/pi_env.sh`), then `git pull` again. When in doubt, back up the folder, then `git clean -fd` **only if** you accept deleting untracked files.

---

## 8. Tests

```bash
python -m unittest discover -s tests -p "test_*.py"
```

---

## 9. Validation workflow (staged)

1. **Servos**: `serial neutral`, then sweep **one joint at a time** with `SET_SERVO` (keep others at neutral).
2. **Mapping**: `model-to-servo` / `servo-to-model` CLI; confirm directions (base L/R, shoulder up/down, elbow inversion, wrist).
3. **FK**: `fk` with known offsets; compare tip mm to rough ruler measurements.
4. **IK**: `ik` / `solve`; check unreachable messages for far targets.
5. **Integrated**: `solve` then `serial set_servo` with clamped values; confirm horizontal motion is **mostly base** and vertical **mostly shoulder/elbow**.

---

## 10. What still needs physical calibration

See **`CALIBRATION.md`** for the checklist (link lengths are already filled; **theta references**, **signs**, and **vision scaling** are not).

---

## 11. Serial protocol note

Python is the source of truth for **`AIM_*`** style math. The Arduino sketch intentionally focuses on **`SET_SERVO`** execution. Add a thin Python wrapper if you want `AIM_AZ_EL` as a **CLI string** — it should compute IK and send `SET_SERVO` only.
