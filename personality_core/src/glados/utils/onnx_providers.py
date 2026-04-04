"""ONNX Runtime execution provider order (GPU when the installed wheel supports it)."""

from __future__ import annotations

import onnxruntime as ort  # type: ignore


def onnx_execution_providers() -> list[str]:
    """
    Preference order: CUDA (NVIDIA), DirectML (Windows GPU), then CPU / any remainder.

    Use exactly one package: ``onnxruntime`` (CPU), ``onnxruntime-gpu`` (CUDA),
    or ``onnxruntime-directml`` (Windows). See ``scripts/install_onnx_runtime_gpu.ps1``.
    """
    raw = set(ort.get_available_providers())
    for drop in ("TensorrtExecutionProvider", "CoreMLExecutionProvider"):
        raw.discard(drop)

    ordered: list[str] = []
    if "CUDAExecutionProvider" in raw:
        ordered.append("CUDAExecutionProvider")
    if "DmlExecutionProvider" in raw:
        ordered.append("DmlExecutionProvider")
    if "CPUExecutionProvider" in raw:
        ordered.append("CPUExecutionProvider")

    return ordered if ordered else ["CPUExecutionProvider"]
