# Laptop / desktop “brain” setup + hotswap between machines

This guide assumes the **Raspberry Pi** runs `pi_runtime` (WebSocket on port **8765**) and your **gaming laptop** or **main PC** runs the AI stack (`personality_core`, Ollama, `brain_runtime`).

---

## 1. What “hotswap” means here

You keep **one Git repo** (or two clones at the same commit). You run the brain on **either** your main PC **or** your laptop by:

1. Using the **same env vars** (`PI_WS_URL`, paths).
2. Keeping a **local, gitignored** `configs/brain.env` on each machine (optional but recommended).
3. Giving the Pi a **stable address** (static DHCP reservation or `raspberrypi.local` / `glados-pi.local`).

Nothing in the repo hard-codes “laptop only” — only **which machine you sit at** changes.

---

## 2. Pi prerequisites (unchanged)

- `pi_runtime` listening: `ws://<pi-host>:8765`
- Firewall: allow TCP **8765** from your LAN (or VPN) to the Pi.

---

## 3. Install on **each** brain machine (laptop + main PC)

Use the **same steps** on both so you can hop between them.

### 3.1 Clone / pull the repo

```bash
git clone https://github.com/<you>/GLaDOS_Arm.git
cd GLaDOS_Arm
git pull
```

### 3.2 Python 3.11+

- **Windows:** install from [python.org](https://www.python.org/downloads/) or use `winget install Python.Python.3.12`.
- **Linux:** `sudo apt install python3.11 python3.11-venv` (or your distro’s package).

### 3.3 `uv` (optional; faster installs)

If `uv` is not installed or not on `PATH`, **skip to §3.3a** and use `pip` + `venv` only.

```powershell
# Windows (PowerShell) — then add the install directory to PATH if the installer says so
irm https://astral.sh/uv/install.ps1 | iex
```

```bash
# Linux / macOS
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### 3.3a `personality_core` **without** `uv` (Windows / any OS)

From repo root, after `robot_link` and `brain_runtime` are installed:

```powershell
cd personality_core
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -U pip
pip install -e ".[cpu]"
python -m glados.cli download
```

If **`python -m pip install -U pip`** still says pip lives under **`C:\Python313\`** (or “Defaulting to user installation”), activation did not apply—often **execution policy** blocks `Activate.ps1`. Either run `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` once, or **avoid activation** and call the venv interpreter explicitly:

```powershell
.\.venv\Scripts\python.exe -m pip install -U pip
.\.venv\Scripts\python.exe -m pip install -e ".[cpu]"
.\.venv\Scripts\python.exe -m glados.cli download
```

With the venv activated (or using `.\.venv\Scripts\python.exe`), `glados.exe` is under `.venv\Scripts`; add that folder to `PATH` or keep using `python -m glados.cli`.

Linux/macOS:

```bash
cd personality_core
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -e ".[cpu]"
python -m glados.cli download
```

The `[cpu]` extra pulls **onnxruntime** (CPU) for ASR/TTS models.

### 3.4 Shared packages: `robot_link`, `brain_runtime`, `personality_core`

From **repo root** `GLaDOS_Arm`:

```bash
pip install -e ./robot_link
pip install -e ./brain_runtime
```

**With `uv`** (if installed and on `PATH`):

```bash
cd personality_core
uv sync
uv run glados download
```

**Without `uv`:** use §3.3a above.

### 3.4a Windows: `pip` user install and `PATH`

If you see **“Defaulting to user installation”** right after `Activate.ps1`, see the **execution policy / explicit `.\.venv\Scripts\python.exe`** note in §3.3a—you are still using the **system** `python`, not the venv.

If installs went to the **user** site-packages and warnings say `Scripts` is not on `PATH`:

- Add this folder to your user **PATH** (version may differ):

  `C:\Users\<you>\AppData\Roaming\Python\Python313\Scripts`

- Or always invoke tools via Python (no PATH change needed):

  ```powershell
  python -m brain_runtime
  ```

  After a working venv activation, use `glados ...` or `python -m glados.cli ...`; with only user installs, the same commands still work if `python` resolves to the interpreter that has `glados` installed.

### 3.5 Ollama on the **brain** machine (laptop or main PC)

Install from [ollama.com](https://ollama.com/download), then:

```bash
ollama pull llama3.2:1b
```

Match `llm_model` in `configs/pi_potato.yaml` (or your brain-only YAML copy on the PC).

### 3.6 GLaDOS models (first run on that machine)

Already covered by **`python -m glados.cli download`** in §3.3a, or with `uv`:

```bash
cd personality_core
uv run glados download
```

---

## 4. Point the brain at the Pi (one variable)

All clients use:

| Variable | Example | Meaning |
|----------|---------|---------|
| `PI_WS_URL` | `ws://192.168.1.50:8765` | Pi `pi_runtime` WebSocket URL |

**Stable Pi address (pick one):**

- Router **DHCP reservation** for the Pi’s MAC → fixed IP.
- **mDNS**: `ws://raspberrypi.local:8765` (works on many home LANs).
- **/etc/hosts** on each PC: `192.168.1.50 glados-pi` → `ws://glados-pi:8765`.

---

## 5. Easy hotswap: `configs/brain.env`

1. Copy the example file:

   ```bash
   cp configs/brain.env.example configs/brain.env
   ```

2. Edit **`configs/brain.env`** on **each** machine (file is gitignored — can differ per PC):

   ```env
   PI_WS_URL=ws://192.168.1.50:8765
   ```

3. Load before running commands.

**Git Bash / WSL / Linux / macOS:**

```bash
cd GLaDOS_Arm
source scripts/brain_env.sh
python -m brain_runtime
```

**Windows PowerShell** (from repo root):

```powershell
cd GLaDOS_Arm
. .\scripts\brain_env.ps1
```

Do **not** run `python -m brain_runtime` from repo root: the project folder `brain_runtime/` has the same name as the Python package and **shadows** the installed module. Either:

```powershell
cd personality_core
.\.venv\Scripts\python.exe -m brain_runtime
```

(adjust venv path if needed), or use the helper:

```powershell
. .\scripts\run_brain_runtime.ps1
```

**One-liner without helper:**

```bash
export PI_WS_URL=ws://192.168.1.50:8765   # Linux/macOS/Git Bash
```

```powershell
$env:PI_WS_URL = "ws://192.168.1.50:8765"  # PowerShell
```

---

## 6. Run GLaDOS voice stack on the brain machine

After `source scripts/brain_env.sh` (or `.ps1`):

```bash
cd personality_core
uv run glados start --config ../configs/pi_potato.yaml --input-mode both
```

Ollama must be **running on this same machine** (the brain), not on the Pi.

---

## 7. Run only the Pi bridge (smoke test)

```bash
source scripts/brain_env.sh   # or brain_env.ps1
python -m brain_runtime
```

You should see `hello`, `heartbeat`, and stub `ping` / `neutral` traffic in the logs.

---

## 8. Switching from main PC to laptop (workflow)

| Step | Action |
|------|--------|
| 1 | `git pull` on **both** machines (same branch). |
| 2 | Copy or recreate `configs/brain.env` on the laptop if you use it (same `PI_WS_URL` if same home network). |
| 3 | Stop GLaDOS / `brain_runtime` on the **first** machine (Ctrl+C). |
| 4 | On the **second** machine: `source scripts/brain_env.sh` → start `glados` or `brain_runtime` again. |

**Do not** run two full GLaDOS brains against the **same** mic/speaker unless you intend to — only one process should own audio. The Pi can stay up; only the **brain** process moves.

---

## 9. Optional: VPN or remote LAN

If the laptop is not on the same Wi‑Fi as the Pi:

- Tailscale / ZeroTier / WireGuard: use the Pi’s **virtual IP** in `PI_WS_URL`.
- Ensure `pi_runtime` binds `0.0.0.0` and the VPN allows port **8765**.

---

## 10. Troubleshooting

| Issue | Check |
|--------|--------|
| `Connection refused` | Pi: `pi_runtime` running? Firewall? Correct IP? |
| Works on PC, not laptop | Same `PI_WS_URL`? Laptop on same subnet / VPN? |
| Ollama errors | Ollama running **on the brain machine** (`curl http://127.0.0.1:11434/api/tags`). |

---

## 11. Files added for this workflow

- `configs/brain.env.example` — template.
- `configs/brain.env` — **you create**; gitignored.
- `scripts/brain_env.sh` / `scripts/brain_env.ps1` — load `brain.env` into the shell.
