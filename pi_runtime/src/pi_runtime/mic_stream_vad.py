"""Continuous mic → utterance segments using Silero VAD (same idea as personality_core SpeechListener)."""

from __future__ import annotations

import logging
import os
import queue
import threading
import time
from collections import deque
from typing import TypeAlias

import numpy as np
from numpy.typing import NDArray

try:
    import sounddevice as sd  # type: ignore
except ImportError as e:  # pragma: no cover
    raise ImportError("mic stream requires sounddevice: pip install sounddevice") from e

log = logging.getLogger(__name__)

# int index; ALSA logical name if PortAudio exposes it; None = host default input (device=None).
InputDeviceSpec: TypeAlias = int | str | None


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
# Default tuned for typical USB mics on Pi; 0.8 (common desktop default) often misses quiet speech.
_VAD_THRESHOLD = 0.5
_ASR_SR = 16000.0
_CHUNK = 512  # 32 ms @ 16 kHz (Silero ONNX)


def _mic_debug_enabled() -> bool:
    return os.environ.get("PI_MIC_DEBUG", "").strip().lower() in ("1", "true", "yes")


def _vad_threshold_from_env() -> float:
    """Silero speech probability threshold; lower = more sensitive (default 0.5; use 0.65–0.8 if noisy)."""
    raw = os.environ.get("PI_VAD_THRESHOLD", "").strip() or os.environ.get("GLADOS_VAD_THRESHOLD", "").strip()
    if not raw:
        return _VAD_THRESHOLD
    try:
        t = float(raw)
        return max(0.05, min(0.99, t))
    except ValueError:
        return _VAD_THRESHOLD

# TTS barge-in uses the same mic stream (ALSA often allows only one capture open).
_barge_lock = threading.Lock()
_barge_stop: threading.Event | None = None
_barge_hits = 0
_barge_ignore_until = 0.0

# While TTS plays, speaker → mic bleed can look like speech (echo). Default: **allow** Silero barge-in
# so you can talk over her; set PI_STREAM_VOICE_DURING_TTS=0 to gate echo (use Enter / PC interrupt only).
_playback_active = threading.Event()
_det_ref_lock = threading.Lock()
_detector_ref: "_UtteranceDetector | None" = None


def duplex_voice_during_tts() -> bool:
    """If True: VAD barge-in + utterance segmentation while Pi plays TTS (default). If False: echo-safe gate."""
    raw = os.environ.get("PI_STREAM_VOICE_DURING_TTS", "1").strip().lower()
    if raw in ("0", "false", "no", "off"):
        return False
    return True


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


def _sd_num_devices() -> int:
    try:
        d = sd.query_devices()
        if isinstance(d, np.ndarray):
            return int(d.shape[0])
        return len(d)
    except Exception:
        return 0


def _name_looks_like_bluetooth_input(name: str) -> bool:
    """PipeWire often makes AirPods HFP the default *input*; capture can be silent or wrong."""
    n = name.lower()
    if "bluez" in n:
        return True
    if "bluetooth" in n and "usb" not in n:
        return True
    return False


def _first_wired_usbish_input_device() -> int | None:
    """Prefer USB Composite / UAC dongle capture when BT is not the intended mic."""
    n = _sd_num_devices()
    usbish: list[int] = []
    other: list[int] = []
    for i in range(n):
        try:
            info = sd.query_devices(i)
        except Exception:
            continue
        if int(info.get("max_input_channels") or 0) <= 0:
            continue
        name = str(info.get("name", ""))
        if _name_looks_like_bluetooth_input(name):
            continue
        low = name.lower()
        if any(x in low for x in ("usb", "composite", "uac", "codec")):
            usbish.append(i)
        else:
            other.append(i)
    if usbish:
        return usbish[0]
    if other:
        return other[0]
    return None


def _env_raw_input_device() -> str | None:
    for key in ("PI_MIC_INPUT_DEVICE", "GLADOS_SD_INPUT_DEVICE", "PI_SD_INPUT_DEVICE"):
        v = os.environ.get(key, "").strip()
        if v:
            return v
    return None


def _parse_input_spec(raw: str) -> InputDeviceSpec:
    low = raw.lower()
    if low in ("pulse", "default", "pipewire", "sysdefault", "dmix"):
        return low
    try:
        return int(raw)
    except ValueError:
        return raw


def _default_int_input_device() -> int:
    """PortAudio index when no env override; avoid BT default when PI_MIC_PREFER_USB=1."""
    try:
        def_idx = int(sd.default.device[0])
    except Exception:
        return 0

    prefer_usb = os.environ.get("PI_MIC_PREFER_USB", "1").strip().lower() not in (
        "0",
        "false",
        "no",
        "off",
    )
    if not prefer_usb:
        return def_idx

    try:
        def_name = str(sd.query_devices(def_idx, "input").get("name", ""))
    except Exception:
        return def_idx

    if not _name_looks_like_bluetooth_input(def_name):
        return def_idx

    alt = _first_wired_usbish_input_device()
    if alt is not None and alt != def_idx:
        try:
            alt_name = str(sd.query_devices(alt, "input").get("name", ""))
        except Exception:
            alt_name = "?"
        log.info(
            "Pi VAD: default input looks like Bluetooth (%s); using wired/USB capture device %s — %s "
            "(pairing AirPods can steal the default; GLADOS_SD_INPUT_DEVICE=N or PI_MIC_PREFER_USB=0 to override)",
            def_name,
            alt,
            alt_name,
        )
        return alt
    return def_idx


def _resolve_logical_input(spec: str) -> InputDeviceSpec:
    """``pulse``/``default``/etc.: many ALSA builds do not expose that string to PortAudio — find pulse/pipewire or default."""
    try:
        sd.query_devices(spec, "input")
        return spec
    except Exception:
        pass
    needles = ("pulse", "pipewire", "pulseaudio")
    n = _sd_num_devices()
    for i in range(n):
        try:
            info = sd.query_devices(i)
        except Exception:
            continue
        if int(info.get("max_input_channels") or 0) <= 0:
            continue
        name = str(info.get("name", "")).lower()
        if any(nd in name for nd in needles):
            log.info(
                "Pi VAD: %r is not a PortAudio device name here; using index %s — %s",
                spec,
                i,
                info.get("name", "?"),
            )
            return i
    alt = _first_wired_usbish_input_device()
    if alt is not None:
        log.warning(
            "Pi VAD: no pulse/pipewire in PortAudio names; using wired USB index %s (same as PI_MIC_PREFER_USB). "
            "If capture is still silent, set GLADOS_SD_INPUT_DEVICE to the mic index from query_devices().",
            alt,
        )
        return alt
    log.warning(
        "Pi VAD: no pulse/pipewire or USB capture found; using host default (device=None). "
        "Set GLADOS_SD_INPUT_DEVICE=<index> from: python -c \"import sounddevice as sd; print(sd.query_devices())\"",
    )
    return None


def mic_input_device_spec() -> InputDeviceSpec:
    """Input for VAD, /mic, interrupt: int index, ALSA name, or None for host default."""
    raw = _env_raw_input_device()
    if raw:
        spec = _parse_input_spec(raw)
        if isinstance(spec, str) and spec.lower() in ("pulse", "default", "pipewire", "sysdefault", "dmix"):
            return _resolve_logical_input(spec)
        return spec
    return _default_int_input_device()


def mic_input_device_index() -> int:
    """Backward compat — only works when the resolved device is an integer."""
    d = mic_input_device_spec()
    if isinstance(d, int):
        return d
    raise TypeError(
        "Input device is %r (use mic_input_device_spec() with sounddevice); set PI_MIC_INPUT_DEVICE to an int index."
        % (d,)
    )


def _default_samplerate_for_device(dev: InputDeviceSpec) -> float:
    try:
        if dev is None:
            info = sd.query_devices(None, "input")
        else:
            info = sd.query_devices(dev, "input")
        sr = float(info.get("default_samplerate") or 0.0)
        if sr > 0:
            return sr
    except Exception:
        pass
    return 48000.0


def _ordered_input_samplerates(device: InputDeviceSpec) -> list[float]:
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
        pause_ms = float(os.environ.get("PI_VAD_PAUSE_MS", str(_PAUSE_MS)))
        pause_ms = max(float(_VAD_SIZE_MS), min(pause_ms, 5000.0))
        self._pause_chunks = max(1, int(round(pause_ms / float(_VAD_SIZE_MS))))
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

    thr = float(vad_threshold) if vad_threshold is not None else _vad_threshold_from_env()
    det = _UtteranceDetector(vad, thr)
    dev = mic_input_device_spec()
    pending = np.array([], dtype=np.float32)

    def make_callback(hw_sr: float):
        # [last log time, already warned about all-zero capture]
        last_debug_log = [0.0, False]

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
            raw = np.asarray(indata, dtype=np.float32).copy().squeeze()
            if _mic_debug_enabled():
                t = time.monotonic()
                if t - last_debug_log[0] >= 1.5:
                    last_debug_log[0] = t
                    rms = float(np.sqrt(np.mean(np.square(np.atleast_1d(raw)))))
                    if rms < 1e-7:
                        if not last_debug_log[1]:
                            last_debug_log[1] = True
                            log.warning(
                                "Mic capture is digital silence (RMS≈0). Not a VAD threshold issue — "
                                "the OS/driver is not delivering audio. Fix: (1) Hardware mute off. "
                                "(2) alsamixer → F6 → pick USB card → unmute Mic/Capture (M key) and raise level. "
                                "(3) pavucontrol → Input → your USB mic → unmute + volume. "
                                "(4) If still zero: arecord -l and try GLADOS_SD_INPUT_DEVICE=<PortAudio index>."
                            )
                    else:
                        log.info(
                            "Pi MIC_DEBUG: capture RMS=%.6f (speech often 0.001–0.05+; VAD can use this)",
                            rms,
                        )
            data = raw
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
            try:
                if dev is None:
                    info = sd.query_devices(None, "input")
                    in_name = str(info.get("name", "(default input)"))
                else:
                    info = sd.query_devices(dev, "input")
                    in_name = str(info.get("name", "?"))
            except Exception:
                in_name = "?"
            log.info(
                "Pi VAD: input device %s — %s; Silero threshold=%.2f (set PI_VAD_THRESHOLD=0.35–0.45 if still no speech; "
                "PI_MIC_DEBUG=1 logs RMS)",
                dev,
                in_name,
                thr,
            )
            if isinstance(dev, str):
                log.info(
                    "Pi VAD: ALSA logical input %r (PipeWire/Pulse). If hw:* was RMS=0, keep PI_MIC_INPUT_DEVICE=%s.",
                    dev,
                    dev,
                )
            elif dev is None:
                log.info(
                    "Pi VAD: using PortAudio default input (device=None). Set GLADOS_SD_INPUT_DEVICE=<index> to pin a mic."
                )
            log.info(
                "Pi VAD stream: capture @ %.0f Hz block=%d → resampled 16 kHz / 512 for Silero",
                sr,
                blocksize,
            )
            log.info(
                "Pi VAD: segment uplinks after ~%.0f ms of silence at phrase end (PI_VAD_PAUSE_MS; default 640 — speak, then pause)",
                float(det._pause_chunks) * _VAD_SIZE_MS,
            )
            log.info(
                "Silero talk-over-TTS (barge-in): %s — set PI_STREAM_VOICE_DURING_TTS=0 if speaker echo causes false stops",
                "on" if duplex_voice_during_tts() else "off",
            )
            if not duplex_voice_during_tts():
                log.info(
                    "Mic utterances to the brain are suppressed while TTS is playing (echo-safe). "
                    "Speak after she finishes, or export PI_STREAM_VOICE_DURING_TTS=1 for talk-over (echo risk)."
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
