"""Play float32 mono PCM on the default output device (Pi speaker)."""

from __future__ import annotations

import base64
import logging
import os
from typing import Any

import numpy as np
from numpy.typing import NDArray

try:
    import sounddevice as sd  # type: ignore
except ImportError as e:  # pragma: no cover
    raise ImportError("pi_runtime voice loop requires sounddevice on the Pi: pip install sounddevice") from e

log = logging.getLogger(__name__)

_cached_out_sr: float | None = None


def _env_output_device() -> int | None:
    raw = os.environ.get("GLADOS_SD_OUTPUT_DEVICE", "").strip() or os.environ.get("PI_SD_OUTPUT_DEVICE", "").strip()
    if not raw:
        return None
    try:
        return int(raw)
    except ValueError:
        return None


def _output_dev() -> int:
    d = _env_output_device()
    if d is not None:
        return d
    return int(sd.default.device[1])


def _default_sr_for_output() -> float:
    try:
        info = sd.query_devices(_output_dev(), "output")
        sr = float(info.get("default_samplerate") or 0.0)
        if sr > 0:
            return sr
    except Exception:
        pass
    return 48000.0


def _resample_linear_mono(
    data: NDArray[np.float32],
    orig_sr: float,
    target_sr: float,
) -> NDArray[np.float32]:
    if orig_sr == target_sr or data.size == 0:
        return np.asarray(data, dtype=np.float32).reshape(-1)
    x = np.asarray(data, dtype=np.float64).reshape(-1)
    n = x.shape[0]
    duration = n / orig_sr
    target_n = max(1, int(round(duration * target_sr)))
    t_old = np.linspace(0.0, duration, n, endpoint=False)
    t_new = np.linspace(0.0, duration, target_n, endpoint=False)
    return np.interp(t_new, t_old, x).astype(np.float32)


def _sr_supported(sr: float) -> bool:
    dev = _output_dev()

    def _out_cb(outdata: NDArray[np.float32], frames: int, t: Any, st: Any) -> None:
        outdata.fill(0)

    try:
        stream = sd.OutputStream(
            device=dev,
            samplerate=sr,
            channels=1,
            callback=_out_cb,
            blocksize=1024,
        )
        stream.start()
        stream.stop()
        stream.close()
        return True
    except Exception:
        return False


def _ordered_sr_candidates() -> list[float]:
    env = os.environ.get("PI_AUDIO_OUTPUT_SR", "").strip() or os.environ.get("GLADOS_AUDIO_OUTPUT_SR", "").strip()
    if env:
        try:
            return [float(env)]
        except ValueError:
            log.warning("PI_AUDIO_OUTPUT_SR / GLADOS_AUDIO_OUTPUT_SR invalid, probing rates")
    d = _default_sr_for_output()
    preferred: list[float] = [d]
    for r in (48000.0, 44100.0, 32000.0, 24000.0, 22050.0, 16000.0):
        if not any(abs(r - x) < 0.5 for x in preferred):
            preferred.append(r)
    return preferred


def resolve_output_samplerate() -> float:
    """Pick a rate ALSA/PortAudio accepts; cache result."""
    global _cached_out_sr
    if _cached_out_sr is not None:
        return _cached_out_sr
    for sr in _ordered_sr_candidates():
        if _sr_supported(sr):
            _cached_out_sr = float(sr)
            log.info("Pi TTS playback using output sample rate %.0f Hz (device %s)", _cached_out_sr, _output_dev())
            return _cached_out_sr
    raise RuntimeError(
        "No working output sample rate. Try: export PI_AUDIO_OUTPUT_SR=48000 "
        "(or 44100), and set GLADOS_SD_OUTPUT_DEVICE to the correct PortAudio index."
    )


def pcm_b64_to_numpy(pcm_b64: str) -> NDArray[np.float32]:
    raw = base64.b64decode(pcm_b64.encode("ascii"))
    return np.frombuffer(raw, dtype=np.float32).copy()


def play_float32_mono(samples: NDArray[np.float32], sample_rate: float) -> None:
    """Resample to a device-supported rate if needed (e.g. 22050 TTS → 48000 ALSA)."""
    if samples.size == 0:
        return
    out_sr = resolve_output_samplerate()
    if abs(float(sample_rate) - out_sr) < 0.5:
        play_data = np.asarray(samples, dtype=np.float32).reshape(-1)
    else:
        play_data = _resample_linear_mono(samples, float(sample_rate), out_sr)
        log.debug("Resampled playback %.0f Hz -> %.0f Hz for Pi ALSA", sample_rate, out_sr)
    sd.play(play_data, out_sr)
    sd.wait()
