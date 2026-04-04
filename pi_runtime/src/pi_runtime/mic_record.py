"""Record mono float32 PCM from the Pi default (or configured) input device."""

from __future__ import annotations

import logging
import os

import numpy as np
from numpy.typing import NDArray

try:
    import sounddevice as sd  # type: ignore
except ImportError as e:  # pragma: no cover
    raise ImportError("pi_runtime mic uplink requires sounddevice on the Pi: pip install sounddevice") from e

log = logging.getLogger(__name__)


def _input_dev() -> int:
    raw = os.environ.get("GLADOS_SD_INPUT_DEVICE", "").strip() or os.environ.get(
        "PI_SD_INPUT_DEVICE", ""
    ).strip()
    if raw:
        try:
            return int(raw)
        except ValueError:
            pass
    return int(sd.default.device[0])


def record_mic_float32_mono(seconds: float) -> tuple[NDArray[np.float32], float]:
    """Block until ``seconds`` of audio are captured; return (samples, sample_rate)."""
    if seconds <= 0:
        return np.array([], dtype=np.float32), 16000.0
    dev = _input_dev()
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
