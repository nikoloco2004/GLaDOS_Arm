"""Continuous mic → utterance segments using Silero VAD (same idea as personality_core SpeechListener)."""

from __future__ import annotations

import logging
import os
import queue
import threading
import time
from collections import deque

import numpy as np
from numpy.typing import NDArray

try:
    import sounddevice as sd  # type: ignore
except ImportError as e:  # pragma: no cover
    raise ImportError("mic stream requires sounddevice: pip install sounddevice") from e

log = logging.getLogger(__name__)


def mic_mode_wants_continuous_stream() -> bool:
    """Whether to run always-on mic + Silero VAD (default: yes).

    Opt out with **PI_MIC_MODE=push** (or ``ptt``, ``0``, ``false``, ``off``) to use only ``/mic``
    push-to-talk. Any other value (including unset, ``stream``, ``1``) tries continuous capture when
    the VAD model is installed.
    """
    raw = os.environ.get("PI_MIC_MODE", "").strip().lower()
    if raw in ("push", "ptt", "ptt_only", "0", "false", "no", "off"):
        return False
    return True


# Match glados.core.speech_listener + sounddevice_io defaults
_VAD_SIZE_MS = 32
_BUFFER_MS = 800
_PAUSE_MS = 640
_VAD_THRESHOLD = 0.8
_ASR_SR = 16000.0
_CHUNK = 512  # 32 ms @ 16 kHz (Silero ONNX)

# TTS barge-in uses the same mic stream (ALSA often allows only one capture open).
_barge_lock = threading.Lock()
_barge_stop: threading.Event | None = None
_barge_hits = 0
_barge_ignore_until = 0.0

# While TTS plays, speaker → mic bleed looks like speech to Silero (echo). Default: gate off
# segmentation + barge-in until playback ends. Set PI_STREAM_VOICE_DURING_TTS=1 for headset/duplex.
_playback_active = threading.Event()
_det_ref_lock = threading.Lock()
_detector_ref: "_UtteranceDetector | None" = None


def duplex_voice_during_tts() -> bool:
    return os.environ.get("PI_STREAM_VOICE_DURING_TTS", "0").strip().lower() in (
        "1",
        "true",
        "yes",
        "on",
    )


def _playback_gate_enabled() -> bool:
    """When True, do not segment or barge-in while Pi is playing TTS (avoids acoustic echo)."""
    return not duplex_voice_during_tts()


def set_detector_ref(det: "_UtteranceDetector | None") -> None:
    global _detector_ref
    with _det_ref_lock:
        _detector_ref = det


def set_playback_playing(active: bool) -> None:
    """Called around tts_pcm playback when VAD stream is active (unless duplex mode)."""
    with _det_ref_lock:
        det = _detector_ref
        if active:
            _playback_active.set()
        else:
            _playback_active.clear()
    if det is not None:
        det._reset()
        if not active:
            try:
                det._vad.reset_states()  # type: ignore[attr-defined]
            except Exception:
                pass


def set_barge_in_target(stop: threading.Event | None) -> None:
    """While TTS plays, Silero speech frames can set ``stop`` (same env as PI_INTERRUPT_*)."""
    global _barge_stop, _barge_hits, _barge_ignore_until
    with _barge_lock:
        _barge_stop = stop
        _barge_hits = 0
        delay_ms = float(os.environ.get("PI_INTERRUPT_DELAY_MS", "280"))
        _barge_ignore_until = time.monotonic() + max(0.0, delay_ms / 1000.0)


def _check_barge_in(speech: bool) -> None:
    global _barge_hits
    with _barge_lock:
        st = _barge_stop
        if st is None:
            return
        if time.monotonic() < _barge_ignore_until:
            return
        need = max(1, int(os.environ.get("PI_INTERRUPT_HITS", "4")))
        if not speech:
            _barge_hits = 0
            return
        _barge_hits += 1
        if _barge_hits >= need:
            st.set()
            _barge_hits = 0


def _resample_mono(x: NDArray[np.float32], orig_sr: float, target_sr: float) -> NDArray[np.float32]:
    if orig_sr == target_sr or x.size == 0:
        return np.asarray(x, dtype=np.float32).reshape(-1)
    a = np.asarray(x, dtype=np.float64).reshape(-1)
    n = a.shape[0]
    duration = n / orig_sr
    target_n = max(1, int(round(duration * target_sr)))
    t_old = np.linspace(0.0, duration, n, endpoint=False)
    t_new = np.linspace(0.0, duration, target_n, endpoint=False)
    return np.interp(t_new, t_old, a).astype(np.float32)


def _try_import_vad() -> object | None:
    try:
        from glados.audio_io.vad import VAD  # type: ignore

        return VAD()
    except Exception as e:
        log.warning("Silero VAD unavailable (install personality_core + glados download): %s", e)
        return None


def vad_stream_available() -> bool:
    try:
        from glados.audio_io.vad import VAD  # type: ignore

        return VAD.VAD_MODEL.exists()
    except Exception:
        return False


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


def _default_samplerate_for_device(dev: int) -> float:
    try:
        info = sd.query_devices(dev, "input")
        sr = float(info.get("default_samplerate") or 0.0)
        if sr > 0:
            return sr
    except Exception:
        pass
    return 48000.0


def _ordered_input_samplerates(device: int) -> list[float]:
    """Try 16 kHz first (VAD/ASR), then PortAudio default, then common ALSA rates."""
    d = _default_samplerate_for_device(device)
    preferred = [16000.0]
    if abs(d - 16000.0) > 0.5:
        preferred.append(d)
    rest = [48000.0, 44100.0, 32000.0, 24000.0, 22050.0, 12000.0, 8000.0]
    out: list[float] = []
    for x in preferred + rest:
        if not any(abs(x - y) < 0.5 for y in out):
            out.append(x)
    return out


class _UtteranceDetector:
    """Pre-roll buffer → record until pause (silence) → emit one ndarray."""

    def __init__(self, vad: object, threshold: float) -> None:
        self._vad = vad
        self._threshold = threshold
        self._pre: deque[NDArray[np.float32]] = deque(maxlen=_BUFFER_MS // _VAD_SIZE_MS)
        self._recording = False
        self._samples: list[NDArray[np.float32]] = []
        self._gap = 0
        self._pause_chunks = _PAUSE_MS // _VAD_SIZE_MS
        min_ms = float(os.environ.get("PI_MIC_STREAM_MIN_MS", "200"))
        self._min_samples = max(_CHUNK, int(_ASR_SR * min_ms / 1000.0))
        max_ms = float(os.environ.get("PI_MIC_STREAM_MAX_MS", "30000"))
        self._max_samples = int(_ASR_SR * max_ms / 1000.0)

    def _reset(self) -> None:
        self._recording = False
        self._samples = []
        self._gap = 0
        self._pre.clear()

    def feed_frame_16k_512(self, chunk: NDArray[np.float32]) -> NDArray[np.float32] | None:
        if chunk.size != _CHUNK:
            return None
        if _playback_gate_enabled() and _playback_active.is_set():
            # Keep Silero state advancing only; skip echo → ASR / self-interrupt.
            self._vad(np.expand_dims(chunk.astype(np.float32), 0))  # type: ignore[operator]
            return None
        vad_out = self._vad(np.expand_dims(chunk.astype(np.float32), 0))  # type: ignore[operator]
        score = float(np.asarray(vad_out).reshape(-1)[0])
        speech = score > self._threshold
        _check_barge_in(speech)

        if not self._recording:
            self._pre.append(chunk.copy())
            if speech:
                self._recording = True
                self._samples = [c.copy() for c in self._pre]
                self._gap = 0
            return None

        self._samples.append(chunk.copy())
        total = sum(a.size for a in self._samples)
        if total >= self._max_samples:
            utt = np.concatenate(self._samples)
            self._reset()
            return utt if utt.size >= self._min_samples else None

        if speech:
            self._gap = 0
        else:
            self._gap += 1
            if self._gap >= self._pause_chunks:
                utt = np.concatenate(self._samples)
                self._reset()
                if utt.size < self._min_samples:
                    return None
                return utt
        return None


def run_vad_stream_thread(
    out_queue: "queue.Queue[NDArray[np.float32]]",
    stop_event: threading.Event,
    vad_threshold: float | None = None,
) -> None:
    """Blocking thread: capture mic, VAD segment, push float32 mono 16 kHz utterances to queue."""
    vad = _try_import_vad()
    if vad is None:
        return

    thr = _VAD_THRESHOLD if vad_threshold is None else vad_threshold
    det = _UtteranceDetector(vad, thr)
    dev = _input_dev()
    pending = np.array([], dtype=np.float32)

    def make_callback(hw_sr: float):
        def audio_callback(
            indata: NDArray[np.float32],
            frames: int,
            time_info: object,
            status: object,
        ) -> None:
            nonlocal pending
            if stop_event.is_set():
                return
            if status:
                log.debug("audio callback status: %s", status)
            data = np.asarray(indata, dtype=np.float32).copy().squeeze()
            if float(hw_sr) != _ASR_SR:
                data = _resample_mono(data, float(hw_sr), _ASR_SR)
            pending = np.concatenate([pending, data])
            while pending.size >= _CHUNK and not stop_event.is_set():
                frame = pending[:_CHUNK].astype(np.float32)
                pending = pending[_CHUNK:]
                utt = det.feed_frame_16k_512(frame)
                if utt is not None and utt.size > 0:
                    try:
                        out_queue.put_nowait(utt)
                    except queue.Full:
                        log.warning("utterance queue full; dropping segment")

        return audio_callback

    last_err: BaseException | None = None
    for sr in _ordered_input_samplerates(dev):
        try:
            blocksize = max(1, int(round(sr * _VAD_SIZE_MS / 1000.0)))
            stream = sd.InputStream(
                device=dev,
                samplerate=sr,
                channels=1,
                dtype="float32",
                callback=make_callback(sr),
                blocksize=blocksize,
            )
            stream.start()
            set_detector_ref(det)
            log.info(
                "Pi VAD stream: capture @ %.0f Hz block=%d → resampled 16 kHz / 512 for Silero",
                sr,
                blocksize,
            )
            try:
                while not stop_event.wait(0.2):
                    pass
            finally:
                try:
                    stream.stop()
                    stream.close()
                except Exception:
                    pass
                set_playback_playing(False)
                set_detector_ref(None)
            log.info("Pi VAD stream thread stopped")
            return
        except Exception as e:
            last_err = e
            log.warning("Open mic sr=%.0f failed: %s", sr, e)
            continue

    log.error("Could not open Pi microphone for VAD stream: %s", last_err)
