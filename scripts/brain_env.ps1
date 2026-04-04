# Load configs/brain.env into the current PowerShell session (repo root = parent of scripts/).
$Root = Split-Path -Parent $PSScriptRoot
$EnvFile = Join-Path $Root "configs\brain.env"
if (Test-Path $EnvFile) {
    foreach ($line in Get-Content $EnvFile) {
        $line = $line.Trim()
        if ($line -eq "" -or $line.StartsWith("#")) { continue }
        $i = $line.IndexOf("=")
        if ($i -lt 1) { continue }
        $name = $line.Substring(0, $i).Trim()
        $value = $line.Substring($i + 1).Trim()
        Set-Item -Path "Env:$name" -Value $value
    }
    Write-Host "brain_env: loaded $EnvFile"
} else {
    Write-Host "brain_env: $EnvFile not found - set `$env:PI_WS_URL yourself or copy configs/brain.env.example"
}
