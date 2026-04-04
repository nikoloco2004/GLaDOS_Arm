# Run brain_runtime from a cwd that does not shadow the `brain_runtime` package name.
# (From repo root, the folder `brain_runtime/` shadows `python -m brain_runtime`.)
$Root = Split-Path -Parent $PSScriptRoot
$VenvPython = Join-Path $Root "personality_core\.venv\Scripts\python.exe"
if (-not (Test-Path $VenvPython)) {
    Write-Error "Missing venv Python: $VenvPython - create personality_core\.venv first."
    exit 1
}
. (Join-Path $Root "scripts\brain_env.ps1")
Set-Location (Join-Path $Root "personality_core")
& $VenvPython -m brain_runtime
