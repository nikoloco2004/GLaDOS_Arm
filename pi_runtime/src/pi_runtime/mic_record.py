"""Record mono float32 PCM from the Pi default (or configured) input device."""

from __future__ import annotations

import logging

import numpy as np
from numpy.typing import NDArray

try:
    import sounddevice as sd  # type: ignore
except ImportError as e:  # pragma: no cover
    raise ImportError("pi_runtime mic uplink requires sounddevice on the Pi: pip install sounddevice") from e

log = logging.getLogger(__name__)


def record_mic_float32_mono(seconds: float) -> tuple[NDArray[np.float32], float]:
    """Block until ``seconds`` of audio are captured; return (samples, sample_rate)."""
    if seconds <= 0:
        return np.array([], dtype=np.float32), 16000.0
    from .mic_stream_vad import mic_input_device_spec

    dev = mic_input_device_spec()
    try:
        info = sd.query_devices(dev, "input")
        fs = float(info.get("default_samplerate") or 16000.0)
    except Exception:
        fs = 16000.0
    n_frames = max(1, int(round(seconds * fs)))
    log.info("Pi mic: recording %.2f s @ %.0f Hz (device %s)", seconds, fs, dev)
    audio = sd.rec(n_frames, samplerate=fs, channels=1, dtype="float32", device=dev)
    sd.wait()
    samples = np.asarray(audio, dtype=np.float32).reshape(-1).copy()
    return samples, fs
