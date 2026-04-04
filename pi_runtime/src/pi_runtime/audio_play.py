"""Play float32 mono PCM on the default output device (Pi speaker)."""

from __future__ import annotations

import base64

import numpy as np
from numpy.typing import NDArray

try:
    import sounddevice as sd  # type: ignore
except ImportError as e:  # pragma: no cover
    raise ImportError("pi_runtime voice loop requires sounddevice on the Pi: pip install sounddevice") from e


def pcm_b64_to_numpy(pcm_b64: str) -> NDArray[np.float32]:
    raw = base64.b64decode(pcm_b64.encode("ascii"))
    return np.frombuffer(raw, dtype=np.float32).copy()


def play_float32_mono(samples: NDArray[np.float32], sample_rate: float) -> None:
    if samples.size == 0:
        return
    sd.play(samples, float(sample_rate))
    sd.wait()
