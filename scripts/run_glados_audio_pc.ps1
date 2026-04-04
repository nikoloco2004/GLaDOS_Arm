# Full GLaDOS on PC: mic + speaker using configs/pi_potato.yaml (audio input).
# Run from repo root:  .\scripts\run_glados_audio_pc.ps1
# Optional: -Both  also enables typing (same as --input-mode both)

param(
    [switch]$Both
)

$Root = Split-Path -Parent $PSScriptRoot
$VenvPython = Join-Path $Root "personality_core\.venv\Scripts\python.exe"
if (-not (Test-Path $VenvPython)) {
    Write-Error "Missing venv Python: $VenvPython — create personality_core\.venv and pip install -e personality_core first."
    exit 1
}

$Config = Join-Path $Root "configs\pi_potato.yaml"
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
