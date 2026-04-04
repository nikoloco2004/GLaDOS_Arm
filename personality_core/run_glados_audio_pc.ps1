# Run from personality_core/ — forwards to ../scripts/run_glados_audio_pc.ps1
# Example:  .\run_glados_audio_pc.ps1 -Laptop
# Example:  .\run_glados_audio_pc.ps1 -Laptop -Both

$RepoRoot = Split-Path -Parent $PSScriptRoot
$Script = Join-Path $RepoRoot "scripts\run_glados_audio_pc.ps1"
if (-not (Test-Path $Script)) {
    Write-Error "Missing $Script"
    exit 1
}
& $Script @args
