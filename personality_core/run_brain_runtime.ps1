# Run from personality_core/ — forwards to ../scripts/run_brain_runtime.ps1
# Example:  .\run_brain_runtime.ps1

$RepoRoot = Split-Path -Parent $PSScriptRoot
$Script = Join-Path $RepoRoot "scripts\run_brain_runtime.ps1"
if (-not (Test-Path $Script)) {
    Write-Error "Missing $Script"
    exit 1
}
& $Script @args
