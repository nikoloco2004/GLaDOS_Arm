"""Play float32 mono PCM on the default output device (Pi speaker)."""

from __future__ import annotations

import base64
import logging
import os
import shutil
import subprocess
import tempfile
import threading
import time
import wave
from typing import Any, TypeAlias

import numpy as np
from numpy.typing import NDArray

try:
    import sounddevice as sd  # type: ignore
except ImportError as e:  # pragma: no cover
    raise ImportError("pi_runtime voice loop requires sounddevice on the Pi: pip install sounddevice") from e

log = logging.getLogger(__name__)

# PortAudio accepts int index, ALSA logical name (e.g. default/pulse), or None for host default.
OutputDeviceSpec: TypeAlias = int | str | None

_cached_out_sr: float | None = None
_cached_out_channels: int | None = None  # 1 or 2 (Bluetooth A2DP often requires stereo)
_cached_output_device: OutputDeviceSpec = None  # set after successful probe (may differ from sd.default)
_cached_playback_backend: str | None = None  # "portaudio" | "pulse_cli" (paplay or pw-play)
_cached_pulse_cli: list[str] | None = None  # argv[0] when using pulse_cli


def _env_output_device() -> OutputDeviceSpec:
    raw = os.environ.get("GLADOS_SD_OUTPUT_DEVICE", "").strip() or os.environ.get("PI_SD_OUTPUT_DEVICE", "").strip()
    if not raw:
        return None
    low = raw.lower()
    if low in ("default", "pulse", "sysdefault"):
        return low
    try:
        return int(raw)
    except ValueError:
        return raw


def _default_output_index_invalid() -> bool:
    try:
        return int(sd.default.device[1]) < 0
    except Exception:
        return True


def _named_output_fallbacks() -> list[OutputDeviceSpec]:
    """When ALSA only lists hardware (e.g. USB mic) but PipeWire/Pulse routes BT/music via a virtual sink."""
    out: list[OutputDeviceSpec] = []
    for name in ("default", "pulse"):
        try:
            sd.query_devices(name)
            out.append(name)
        except Exception:
            pass
    out.append(None)
    return out


def _merge_output_candidates(found: list[int]) -> list[OutputDeviceSpec]:
    """Prefer enumerated devices first; add default/pulse/None when the host default output is missing (-1)."""
    merged: list[OutputDeviceSpec] = list(found)
    if _default_output_index_invalid():
        for x in _named_output_fallbacks():
            if x not in merged:
                merged.append(x)
    elif not found:
        merged.extend(_named_output_fallbacks())
    return merged


def _output_dev() -> OutputDeviceSpec:
    """Output device for TTS. After the first successful probe, uses the cached working device."""
    global _cached_output_device
    if _cached_output_device is not None:
        return _cached_output_device
    d = _env_output_device()
    if d is not None:
        return d
    try:
        i = int(sd.default.device[1])
        if i >= 0:
            return i
    except Exception:
        pass
    return None


def _portaudio_num_devices() -> int:
    try:
        d = sd.query_devices()
        if isinstance(d, np.ndarray):
            return int(d.shape[0])
        if isinstance(d, (list, tuple)):
            return len(d)
        return len(d)
    except Exception:
        return 0


def _enumerate_output_devices() -> list[int]:
    """Scan PortAudio for output device indices (no env override). Prefer kind=output; fall back if hosts mis-report."""
    found: list[int] = []

    # 1) sounddevice can list output-only indices (most reliable on PipeWire/Pulse/Bluetooth).
    try:
        out = sd.query_devices(kind="output")
        if out is not None:
            if isinstance(out, np.ndarray):
                out = out.tolist()
            for x in out:
                if isinstance(x, (int, np.integer)):
                    found.append(int(x))
                elif isinstance(x, dict):
                    # Some versions return device dicts; skip (handled by scan below).
                    pass
    except Exception as e:
        log.debug("query_devices(kind=output): %s", e)

    # 2) Scan by max_output_channels (ALSA/USB).
    if not found:
        try:
            n = _portaudio_num_devices()
            for i in range(n):
                info = sd.query_devices(i)
                if int(info.get("max_output_channels") or 0) > 0:
                    found.append(i)
        except Exception as e:
            log.warning("enumerate output devices (scan): %s", e)

    # 3) Some Pulse/Bluetooth nodes report 0 output channels; try remaining indices and let probe decide.
    #    Skip input-only devices (e.g. USB mic: max_out=0, max_in>0) — they are never valid TTS outputs.
    if not found:
        n = _portaudio_num_devices()
        if n > 0:
            for i in range(n):
                try:
                    info = sd.query_devices(i)
                    mo = int(info.get("max_output_channels") or 0)
                    mi = int(info.get("max_input_channels") or 0)
                except Exception:
                    found.append(i)
                    continue
                if mo > 0:
                    found.append(i)
                elif mo == 0 and mi > 0:
                    pass  # input-only; skip
                else:
                    # mo==0 and mi==0: odd/phantom; still try (some hosts mis-report Bluetooth)
                    found.append(i)
            if found:
                log.warning(
                    "No devices with max_output_channels>0; trying %d PortAudio index(es) (Bluetooth/Pulse quirk)",
                    len(found),
                )

    try:
        def_pair = sd.default.device
        if isinstance(def_pair, (tuple, list, np.ndarray)) and len(def_pair) >= 2:
            def_out = int(def_pair[1])
            if def_out >= 0 and def_out in found:
                found.remove(def_out)
                found.insert(0, def_out)
    except Exception:
        pass
    return found


def _preferred_sr_for_device(device: OutputDeviceSpec) -> float:
    try:
        if device is None:
            info = sd.query_devices(None)
        elif isinstance(device, str):
            info = sd.query_devices(device)
        else:
            info = sd.query_devices(device, "output")
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


def _audio_debug() -> bool:
    return os.environ.get("PI_AUDIO_DEBUG", "").strip().lower() in ("1", "true", "yes")


def _channel_order_for_device(device: OutputDeviceSpec) -> list[int]:
    """USB speakers often work mono-first; PipeWire/Pulse names + Bluetooth often need stereo."""
    forced = os.environ.get("PI_AUDIO_OUTPUT_CHANNELS", "").strip()
    if forced in ("1", "2"):
        return [int(forced)]
    if isinstance(device, str) or device is None:
        if os.environ.get("PI_AUDIO_NAMED_STEREO_FIRST", "1").strip().lower() not in (
            "0",
            "false",
            "no",
            "off",
        ):
            return [2, 1]
    return [1, 2]


def _probe_output_stream(sr: float, channels: int, device: OutputDeviceSpec) -> bool:
    """Return True if PortAudio can open this output device at sr/channels (Bluetooth often needs stereo)."""

    def _out_cb(outdata: NDArray[np.float32], frames: int, t: Any, st: Any) -> None:
        outdata.fill(0)

    for blocksize in (1024, None, 0, 512):
        try:
            kw: dict[str, Any] = dict(
                device=device,
                samplerate=sr,
                channels=channels,
                callback=_out_cb,
            )
            if blocksize is not None:
                kw["blocksize"] = blocksize
            stream = sd.OutputStream(**kw)
            stream.start()
            stream.stop()
            stream.close()
            return True
        except Exception as e:
            if _audio_debug():
                log.debug(
                    "PortAudio probe fail sr=%s ch=%s dev=%s block=%s: %s",
                    sr,
                    channels,
                    device,
                    blocksize,
                    e,
                )
            continue
    return False


def _find_pulse_cli() -> list[str] | None:
    """First match on PATH for cache/logging: prefer paplay (Pulse), then pw-play (PipeWire)."""
    for name in ("paplay", "pw-play"):
        p = shutil.which(name)
        if p:
            return [p]
    return None


def _list_pulse_cli_exes() -> list[str]:
    """Try paplay first (raw PCM / Pulse), then pw-play; both accept WAV via libsndfile."""
    out: list[str] = []
    for name in ("paplay", "pw-play"):
        p = shutil.which(name)
        if p:
            out.append(p)
    return out


def _write_wav_s16(path: str, samples_f32_mono: NDArray[np.float32], sample_rate: float, channels: int) -> None:
    """Standard PCM WAV (S16 LE). pw-play/paplay open this reliably; raw float/``.raw`` is not."""
    mono = np.asarray(samples_f32_mono, dtype=np.float32).reshape(-1)
    s16 = np.clip(mono.astype(np.float64) * 32767.0, -32768, 32767).astype(np.int16)
    with wave.open(path, "wb") as w:
        w.setnchannels(channels)
        w.setsampwidth(2)
        w.setframerate(int(round(sample_rate)))
        if channels == 2:
            inter = np.empty((s16.shape[0] * 2,), dtype=np.int16)
            inter[0::2] = s16
            inter[1::2] = s16
            w.writeframes(inter.tobytes())
        else:
            w.writeframes(s16.tobytes())


def _pulse_cli_env() -> dict[str, str]:
    """SSH sessions often omit XDG_RUNTIME_DIR; Pulse/PipeWire sockets live under /run/user/<uid>."""
    env = os.environ.copy()
    uid = os.getuid()
    if not env.get("XDG_RUNTIME_DIR"):
        candidate = f"/run/user/{uid}"
        if os.path.isdir(candidate):
            env["XDG_RUNTIME_DIR"] = candidate
    rdir = env.get("XDG_RUNTIME_DIR", "")
    if rdir:
        pulse_native = os.path.join(rdir, "pulse", "native")
        try:
            if os.path.exists(pulse_native) and not env.get("PULSE_SERVER"):
                env["PULSE_SERVER"] = f"unix:{pulse_native}"
        except OSError:
            pass
    return env


def _try_pulse_cli_fallback_cache() -> bool:
    """When PortAudio cannot open PipeWire/Pulse virtual devices, use paplay or pw-play (same path as GUI players)."""
    global _cached_out_sr, _cached_out_channels, _cached_output_device, _cached_playback_backend, _cached_pulse_cli
    if os.environ.get("PI_PLAYBACK_PAPLAY", "1").strip().lower() in ("0", "false", "no", "off"):
        return False
    cli = _find_pulse_cli()
    if not cli:
        return False
    sr = 48000.0
    env_sr = os.environ.get("PI_AUDIO_OUTPUT_SR", "").strip() or os.environ.get("GLADOS_AUDIO_OUTPUT_SR", "").strip()
    if env_sr:
        try:
            sr = float(env_sr)
        except ValueError:
            pass
    ch_env = os.environ.get("PI_AUDIO_OUTPUT_CHANNELS", "").strip()
    if ch_env == "2":
        ch = 2
    elif ch_env == "1":
        ch = 1
    else:
        ch = 2
    _cached_playback_backend = "pulse_cli"
    _cached_pulse_cli = cli
    _cached_out_sr = sr
    _cached_out_channels = ch
    _cached_output_device = None
    log.info(
        "Pi TTS playback: using %s fallback — S16 WAV via CLI (PortAudio opened no device).",
        os.path.basename(cli[0]),
    )
    return True


def _play_pulse_cli_interruptible(
    samples: NDArray[np.float32],
    sample_rate: float,
    stop_event: threading.Event,
    out_sr: float,
) -> bool:
    """Play via ``paplay`` / ``pw-play`` on a temp **WAV** file (libsndfile cannot play raw float from ``-``)."""
    exes = _list_pulse_cli_exes()
    if not exes:
        log.warning("pulse_cli: no paplay or pw-play on PATH")
        return False

    out_ch = int(_cached_out_channels or 1)
    if abs(float(sample_rate) - out_sr) < 0.5:
        play_data = np.asarray(samples, dtype=np.float32).reshape(-1)
    else:
        play_data = _resample_linear_mono(samples, float(sample_rate), out_sr)
        log.debug("Resampled playback %.0f Hz -> %.0f Hz for pulse_cli", sample_rate, out_sr)

    fd, wav_path = tempfile.mkstemp(prefix="glados_tts_", suffix=".wav")
    try:
        os.close(fd)
        _write_wav_s16(wav_path, play_data, out_sr, out_ch)
        env = _pulse_cli_env()
        for exe in exes:
            try:
                proc = subprocess.Popen(
                    [exe, wav_path],
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    env=env,
                )
            except OSError as e:
                log.warning("pulse_cli %s failed to start: %s", os.path.basename(exe), e)
                continue
            while proc.poll() is None:
                if stop_event.is_set():
                    proc.terminate()
                    try:
                        proc.wait(timeout=1.5)
                    except Exception:
                        pass
                    return True
                time.sleep(0.02)
            err = b""
            try:
                if proc.stderr:
                    err = proc.stderr.read() or b""
            except Exception:
                pass
            code = proc.returncode
            if code in (0, None) or code in (-15, -9):
                log.debug("pulse_cli %s finished ok", os.path.basename(exe))
                return stop_event.is_set()
            log.warning(
                "pulse_cli %s exited %s: %s",
                os.path.basename(exe),
                code,
                err[:400].decode(errors="replace") if err else "",
            )
        return stop_event.is_set()
    finally:
        try:
            os.unlink(wav_path)
        except OSError:
            pass


def _ordered_sr_candidates(device: OutputDeviceSpec) -> list[float]:
    env = os.environ.get("PI_AUDIO_OUTPUT_SR", "").strip() or os.environ.get("GLADOS_AUDIO_OUTPUT_SR", "").strip()
    if env:
        try:
            return [float(env)]
        except ValueError:
            log.warning("PI_AUDIO_OUTPUT_SR / GLADOS_AUDIO_OUTPUT_SR invalid, probing rates")
    d = _preferred_sr_for_device(device)
    preferred: list[float] = [d]
    for r in (48000.0, 44100.0, 32000.0, 24000.0, 22050.0, 16000.0):
        if not any(abs(r - x) < 0.5 for x in preferred):
            preferred.append(r)
    return preferred


def resolve_output_samplerate() -> float:
    """Pick device + rate + channel count PortAudio accepts; cache result."""
    global _cached_out_sr, _cached_out_channels, _cached_output_device, _cached_playback_backend, _cached_pulse_cli
    if (
        _cached_out_sr is not None
        and _cached_out_channels is not None
        and _cached_playback_backend is not None
    ):
        return _cached_out_sr

    fixed = _env_output_device()
    if fixed is not None:
        devices: list[OutputDeviceSpec] = [fixed]
    else:
        devices = _merge_output_candidates(_enumerate_output_devices())

    if not devices:
        if _portaudio_num_devices() == 0:
            raise RuntimeError(
                "PortAudio reports zero audio devices (check PipeWire/ALSA; pi needs sounddevice + working audio stack)."
            )
        hint = ""
        try:
            if int(sd.default.device[1]) < 0:
                hint = (
                    " Default output is unset (default device shows -1 for output). "
                    "Connect USB speakers, HDMI audio, or pair Bluetooth headphones/A2DP "
                    "and set that sink as default in the OS, then restart pi_runtime."
                )
        except Exception:
            pass
        raise RuntimeError(
            "No PortAudio output device found (only input/capture devices, or no playable sink)."
            + hint
            + " Check: ./.venv/bin/python -c \"import sounddevice as sd; print(sd.query_devices()); print(sd.default.device)\""
            " — you need at least one device with output channels (max_output_channels>0) or a working default sink."
        )

    for device in devices:
        ch_order = _channel_order_for_device(device)
        for sr in _ordered_sr_candidates(device):
            for ch in ch_order:
                if _probe_output_stream(sr, ch, device):
                    _cached_out_sr = float(sr)
                    _cached_out_channels = ch
                    _cached_output_device = device
                    _cached_playback_backend = "portaudio"
                    _cached_pulse_cli = None
                    try:
                        if device is None:
                            name = "(default)"
                        elif isinstance(device, str):
                            name = device
                        else:
                            name = str(sd.query_devices(device).get("name", ""))
                    except Exception:
                        name = ""
                    log.info(
                        "Pi TTS playback: %.0f Hz, %d ch, device %s — %s",
                        _cached_out_sr,
                        ch,
                        device,
                        name,
                    )
                    return _cached_out_sr

    if _try_pulse_cli_fallback_cache():
        if _cached_out_sr is None:  # pragma: no cover
            raise RuntimeError("pulse_cli fallback did not set sample rate")
        return _cached_out_sr

    raise RuntimeError(
        "No working output: PortAudio failed and paplay/pw-play not found. "
        "Install pulseaudio-utils (paplay) or ensure pw-play is on PATH; "
        "or set PI_AUDIO_OUTPUT_SR=48000 and GLADOS_SD_OUTPUT_DEVICE=<index|default|pulse>. "
        "PI_AUDIO_DEBUG=1 logs PortAudio probe errors. "
        "Disable CLI fallback with PI_PLAYBACK_PAPLAY=0 only if you must."
    )


def _output_channel_count() -> int:
    resolve_output_samplerate()
    return _cached_out_channels if _cached_out_channels is not None else 1


def pcm_b64_to_numpy(pcm_b64: str) -> NDArray[np.float32]:
    raw = base64.b64decode(pcm_b64.encode("ascii"))
    return np.frombuffer(raw, dtype=np.float32).copy()


def play_float32_mono(samples: NDArray[np.float32], sample_rate: float) -> None:
    """Resample to a device-supported rate if needed (e.g. 22050 TTS → 48000 ALSA)."""
    ev = threading.Event()
    play_float32_mono_interruptible(samples, sample_rate, ev)


def _voice_interrupt_enabled() -> bool:
    return os.environ.get("PI_VOICE_INTERRUPT", "1").strip().lower() not in (
        "0",
        "false",
        "no",
        "off",
    )


def _input_dev() -> int:
    from .mic_stream_vad import mic_input_device_index

    return mic_input_device_index()


def _default_input_sr() -> float:
    try:
        info = sd.query_devices(_input_dev(), "input")
        sr = float(info.get("default_samplerate") or 0.0)
        if sr > 0:
            return sr
    except Exception:
        pass
    return 16000.0


def _mic_stream_capture_active() -> bool:
    """ALSA usually allows one InputStream per device; VAD stream already holds the mic."""
    from .mic_stream_vad import mic_mode_wants_continuous_stream

    return mic_mode_wants_continuous_stream()


def mic_interrupt_monitor(stop_event: threading.Event, playback_done: threading.Event) -> None:
    """Background: open mic and set stop_event when sustained voice energy is detected."""
    if not _voice_interrupt_enabled():
        return
    if _mic_stream_capture_active():
        # Barge-in is driven by Silero in mic_stream_vad (set_barge_in_target); second capture fails with -9985.
        return
    delay_ms = float(os.environ.get("PI_INTERRUPT_DELAY_MS", "280"))
    time.sleep(max(0.0, delay_ms / 1000.0))
    if playback_done.is_set():
        return
    threshold = float(os.environ.get("PI_INTERRUPT_RMS", "0.028"))
    hits_needed = max(1, int(os.environ.get("PI_INTERRUPT_HITS", "4")))
    blocksize = max(128, int(os.environ.get("PI_INTERRUPT_BLOCKSIZE", "512")))
    in_sr = _default_input_sr()
    hits = 0

    def callback(indata: NDArray[np.float32], frames: int, _t: Any, status: Any) -> None:
        nonlocal hits
        if status:
            return
        if playback_done.is_set():
            return
        rms = float(np.sqrt(np.mean(np.square(indata))))
        if rms >= threshold:
            hits += 1
            if hits >= hits_needed:
                stop_event.set()
        else:
            hits = 0

    try:
        with sd.InputStream(
            device=_input_dev(),
            channels=1,
            samplerate=in_sr,
            blocksize=blocksize,
            dtype="float32",
            callback=callback,
        ):
            while not playback_done.is_set() and not stop_event.is_set():
                time.sleep(0.04)
    except Exception as e:
        log.warning("voice interrupt mic unavailable (disable with PI_VOICE_INTERRUPT=0): %s", e)


def play_float32_mono_interruptible(
    samples: NDArray[np.float32],
    sample_rate: float,
    stop_event: threading.Event,
) -> bool:
    """Play resampled mono PCM; return True if ``stop_event`` was set (interrupt).

    Uses a dedicated ``OutputStream`` and stops **only that stream** on interrupt.
    **Never** call ``sd.stop()`` here: that stops *all* PortAudio streams and tears down
    the mic ``InputStream`` used for barge-in, which can ``double free`` on Linux/ALSA.
    """
    if samples.size == 0:
        return False
    out_sr = resolve_output_samplerate()
    if _cached_playback_backend == "pulse_cli":
        return _play_pulse_cli_interruptible(samples, sample_rate, stop_event, out_sr)
    out_ch = _output_channel_count()
    if abs(float(sample_rate) - out_sr) < 0.5:
        play_data = np.asarray(samples, dtype=np.float32).reshape(-1)
    else:
        play_data = _resample_linear_mono(samples, float(sample_rate), out_sr)
        log.debug("Resampled playback %.0f Hz -> %.0f Hz for Pi ALSA", sample_rate, out_sr)

    dev = _output_dev()
    blocksize = max(256, int(os.environ.get("PI_PLAYBACK_BLOCKSIZE", "1024")))
    n_total = int(play_data.shape[0])
    pos = 0

    def callback(outdata: NDArray[np.float32], frames: int, _time_info: Any, status: Any) -> None:
        nonlocal pos
        if status:
            log.debug("OutputStream status: %s", status)
        if stop_event.is_set():
            raise sd.CallbackStop
        remaining = n_total - pos
        if remaining <= 0:
            raise sd.CallbackStop
        n = min(frames, remaining)
        chunk = play_data[pos : pos + n]
        if out_ch == 1:
            outdata[:n, 0] = chunk
            if n < frames:
                outdata[n:, 0] = 0
        else:
            outdata[:n, 0] = chunk
            outdata[:n, 1] = chunk
            if n < frames:
                outdata[n:, 0] = 0
                outdata[n:, 1] = 0
        pos += n

    try:
        with sd.OutputStream(
            device=dev,
            samplerate=out_sr,
            channels=out_ch,
            dtype="float32",
            blocksize=blocksize,
            callback=callback,
        ) as stream:
            stream.start()
            while stream.active:
                if stop_event.is_set():
                    stream.stop()
                    break
                time.sleep(0.02)
    except Exception as e:
        log.warning("playback OutputStream error: %s", e)
        return stop_event.is_set()

    return stop_event.is_set()
