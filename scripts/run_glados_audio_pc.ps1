# Full GLaDOS on PC: mic + speaker using configs/pi_potato.yaml (audio input).
# Run from repo root:  .\scripts\run_glados_audio_pc.ps1
# Optional: -Both  also enables typing (same as --input-mode both)
# Optional: -Laptop uses configs/pi_potato_laptop.yaml (canonical laptop profile: 3B LLM, text barge-in, canon-tuned prompt)

param(
    [switch]$Both,
    [switch]$Laptop
)

$Root = Split-Path -Parent $PSScriptRoot
$VenvPython = Join-Path $Root "personality_core\.venv\Scripts\python.exe"
if (-not (Test-Path $VenvPython)) {
    Write-Error "Missing venv Python: $VenvPython - create personality_core\.venv and pip install -e personality_core first."
    exit 1
}

$ConfigName = if ($Laptop) { "configs\pi_potato_laptop.yaml" } else { "configs\pi_potato.yaml" }
$Config = Join-Path $Root $ConfigName
if (-not (Test-Path $Config)) {
    Write-Error "Missing config: $Config"
    exit 1
}

Set-Location (Join-Path $Root "personality_core")
$Args = @("-m", "glados.cli", "start", "--config", $Config)
if ($Both) {
    $Args += @("--input-mode", "both")
}

& $VenvPython @Args
