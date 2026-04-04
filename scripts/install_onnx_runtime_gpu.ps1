# Swap ONNX Runtime wheel so ASR/VAD/TTS can use GPU (huge speedup vs CPU-only).
# Use ONE backend only. Run from repo root after venv exists.
#
# NVIDIA (CUDA):  .\scripts\install_onnx_runtime_gpu.ps1 -Backend Cuda
# Windows iGPU/Arc (DirectML):  .\scripts\install_onnx_runtime_gpu.ps1 -Backend DirectML

param(
    [ValidateSet("Cuda", "DirectML")]
    [string]$Backend = "Cuda"
)

$Root = Split-Path -Parent $PSScriptRoot
$Py = Join-Path $Root "personality_core\.venv\Scripts\python.exe"
if (-not (Test-Path $Py)) {
    Write-Error "Missing venv: $Py"
    exit 1
}

Write-Host "Removing any existing onnxruntime wheels..."
& $Py -m pip uninstall -y onnxruntime onnxruntime-gpu onnxruntime-directml 2>$null

if ($Backend -eq "Cuda") {
    Write-Host "Installing onnxruntime-gpu (NVIDIA CUDA)..."
    & $Py -m pip install "onnxruntime-gpu>=1.16.0"
} else {
    Write-Host "Installing onnxruntime-directml (Windows GPU)..."
    & $Py -m pip install "onnxruntime-directml>=1.16.0"
}

Write-Host "Done. Restart GLaDOS; first run may take a moment while models warm up."
& $Py -c "import onnxruntime as ort; print('Providers:', ort.get_available_providers())"
