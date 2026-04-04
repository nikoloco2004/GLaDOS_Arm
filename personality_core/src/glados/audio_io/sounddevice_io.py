import os
import queue
import threading
from typing import Any

from loguru import logger
import numpy as np
from numpy.typing import NDArray
import sounddevice as sd  # type: ignore

from . import VAD


def _device_index_from_env(name: str) -> int | None:
    """Parse GLADOS_SD_INPUT_DEVICE / GLADOS_SD_OUTPUT_DEVICE (PortAudio device index)."""
    raw = os.environ.get(name, "").strip()
    if not raw:
        return None
    try:
        return int(raw)
    except ValueError as e:
        raise ValueError(f"{name} must be an integer PortAudio device index, got {raw!r}") from e


def _resample_linear_mono(
    data: NDArray[np.float32],
    orig_sr: float,
    target_sr: float,
) -> NDArray[np.float32]:
    """Linear resample mono float32 audio (no scipy; works on Raspberry Pi ALSA quirks)."""
    if orig_sr == target_sr or data.size == 0:
        return np.asarray(data, dtype=np.float32).reshape(-1)
    x = np.asarray(data, dtype=np.float64).reshape(-1)
    n = x.shape[0]
    duration = n / orig_sr
    target_n = max(1, int(round(duration * target_sr)))
    t_old = np.linspace(0.0, duration, n, endpoint=False)
    t_new = np.linspace(0.0, duration, target_n, endpoint=False)
    return np.interp(t_new, t_old, x).astype(np.float32)


def _device_index(device: int | None, *, is_input: bool) -> int:
    if device is None:
        return sd.default.device[0] if is_input else sd.default.device[1]
    return device


def _default_samplerate_for_device(device: int | None, *, is_input: bool) -> float:
    """PortAudio-reported default sample rate (may not match what ALSA actually accepts)."""
    idx = _device_index(device, is_input=is_input)
    info = sd.query_devices(idx)
    sr = float(info.get("default_samplerate") or 0.0)
    if sr <= 0:
        return 48000.0
    return sr


def _env_float(name: str) -> float | None:
    raw = os.environ.get(name, "").strip()
    if not raw:
        return None
    try:
        return float(raw)
    except ValueError:
        logger.warning("{} must be a number, ignoring", name)
        return None


def _tts_trailing_silence_ms() -> float:
    """Silence appended after each TTS clip (natural pause between sentences). 0 to disable."""
    raw = os.environ.get("GLADOS_TTS_TRAILING_SILENCE_MS", "200").strip()
    try:
        return max(0.0, float(raw))
    except ValueError:
        logger.warning("GLADOS_TTS_TRAILING_SILENCE_MS must be a number, using 200")
        return 200.0


def _samplerate_is_supported(device: int | None, sr: float, *, is_input: bool) -> bool:
    """True if PortAudio can open this device at sr.

    On ALSA, ``sd.check`` / Pa_IsFormatSupported can lie; a short real open is authoritative.
    """
    dev = _device_index(device, is_input=is_input)
    try:
        if is_input:

            def _in_cb(indata: NDArray[np.float32], frames: int, t: Any, st: Any) -> None:
                pass

            stream = sd.InputStream(
                device=dev,
                samplerate=sr,
                channels=1,
                callback=_in_cb,
                blocksize=1024,
            )
        else:

            def _out_cb(outdata: NDArray[np.float32], frames: int, t: Any, st: Any) -> None:
                outdata.fill(0)

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
        pass
    check = getattr(sd, "check", None)
    if callable(check):
        try:
            check(device=dev, samplerate=sr, channels=1)
            return True
        except Exception:
            return False
    return False


def _ordered_input_samplerates(device: int | None) -> list[float]:
    """Try 16 kHz first (ASR), then PortAudio default, then common ALSA rates."""
    d = _default_samplerate_for_device(device, is_input=True)
    preferred = [16000.0]
    if abs(d - 16000.0) > 0.5:
        preferred.append(d)
    rest = [48000.0, 44100.0, 32000.0, 24000.0, 22050.0, 12000.0, 8000.0]
    out: list[float] = []
    for x in preferred + rest:
        if not any(abs(x - y) < 0.5 for y in out):
            out.append(x)
    return out


def _ordered_output_samplerates(device: int | None) -> list[float]:
    """Prefer PortAudio default, then rates USB / I2S hats usually support."""
    d = _default_samplerate_for_device(device, is_input=False)
    preferred: list[float] = []
    if d > 0:
        preferred.append(d)
    rest = [48000.0, 44100.0, 32000.0, 24000.0, 22050.0, 16000.0, 12000.0, 8000.0]
    out: list[float] = []
    for x in preferred + rest:
        if not any(abs(x - y) < 0.5 for y in out):
            out.append(x)
    return out


class SoundDeviceAudioIO:
    """Audio I/O implementation using sounddevice for both input and output.

    This class provides an implementation of the AudioIO interface using the
    sounddevice library to interact with system audio devices. It handles
    real-time audio capture with voice activity detection and audio playback.
    """

    SAMPLE_RATE: int = 16000  # Sample rate for input stream
    VAD_SIZE: int = 32  # Milliseconds of sample for Voice Activity Detection (VAD)
    VAD_THRESHOLD: float = 0.8  # Threshold for VAD detection

    def __init__(self, vad_threshold: float | None = None) -> None:
        """Initialize the sounddevice audio I/O.

        Args:
            vad_threshold: Threshold for VAD detection (default: 0.8)

        Raises:
            ImportError: If the sounddevice module is not available
            ValueError: If invalid parameters are provided
        """
        if vad_threshold is None:
            self.vad_threshold = self.VAD_THRESHOLD
        else:
            self.vad_threshold = vad_threshold

        if not 0 <= self.vad_threshold <= 1:
            raise ValueError("VAD threshold must be between 0 and 1")

        self._vad_model = VAD()

        self._input_device: int | None = _device_index_from_env("GLADOS_SD_INPUT_DEVICE")
        self._output_device: int | None = _device_index_from_env("GLADOS_SD_OUTPUT_DEVICE")
        if self._input_device is not None or self._output_device is not None:
            logger.info(
                "PortAudio devices from env: input={} output={}",
                self._input_device,
                self._output_device,
            )

        self._sample_queue: queue.Queue[tuple[NDArray[np.float32], bool]] = queue.Queue()
        self.input_stream: sd.InputStream | None = None
        self._capture_sr: float = float(self.SAMPLE_RATE)  # actual InputStream rate (may != 16 kHz)
        self._cached_output_sr: float | None = None  # first working playback rate (ALSA often != default_samplerate)
        self._is_playing = False
        self._playback_thread = None
        self._stop_event = threading.Event()

    def start_listening(self) -> None:
        """Start capturing audio from the system microphone.

        Creates and starts a sounddevice InputStream that continuously captures
        audio from the default input device. Each audio chunk is processed with
        the VAD model and placed in the sample queue.

        Raises:
            RuntimeError: If the audio input stream cannot be started
            sd.PortAudioError: If there's an issue with the audio hardware
        """
        if self.input_stream is not None:
            self.stop_listening()

        def make_callback(hw_sr: float):
            def audio_callback(
                indata: NDArray[np.float32],
                frames: int,
                time: sd.CallbackStop,
                status: sd.CallbackFlags,
            ) -> None:
                if status:
                    logger.debug(f"Audio callback status: {status}")

                data = np.array(indata).copy().squeeze()
                if hw_sr != float(self.SAMPLE_RATE):
                    data = _resample_linear_mono(data, hw_sr, float(self.SAMPLE_RATE))
                vad_value = self._vad_model(np.expand_dims(data, 0))
                vad_confidence = vad_value > self.vad_threshold
                self._sample_queue.put((data, bool(vad_confidence)))

            return audio_callback

        def try_open(hw_sr: float) -> None:
            self._capture_sr = hw_sr
            stream_kw: dict[str, Any] = {
                "samplerate": hw_sr,
                "channels": 1,
                "callback": make_callback(hw_sr),
                "blocksize": max(1, int(hw_sr * self.VAD_SIZE / 1000)),
            }
            if self._input_device is not None:
                stream_kw["device"] = self._input_device
            stream = sd.InputStream(**stream_kw)
            try:
                stream.start()
            except Exception:
                try:
                    stream.close()
                except Exception:
                    pass
                raise
            self.input_stream = stream

        env_in = _env_float("GLADOS_AUDIO_INPUT_SR")
        candidates = _ordered_input_samplerates(self._input_device)
        if env_in is not None and env_in > 0:
            candidates = [env_in] + [x for x in candidates if abs(x - env_in) > 0.5]
            logger.info("Input sample rate candidates start with GLADOS_AUDIO_INPUT_SR={} Hz", env_in)

        last_err: BaseException | None = None
        for sr in candidates:
            try:
                try_open(float(sr))
                if abs(float(sr) - float(self.SAMPLE_RATE)) > 0.5:
                    logger.warning(
                        "Microphone opened at {} Hz; resampling to {} Hz for ASR/VAD",
                        sr,
                        self.SAMPLE_RATE,
                    )
                return
            except sd.PortAudioError as e:
                last_err = e
                continue
        raise RuntimeError(
            f"Failed to start audio input stream after trying rates {candidates!r}: {last_err}"
        ) from last_err

    def stop_listening(self) -> None:
        """Stop capturing audio and clean up resources.

        Stops the input stream if it's active and releases associated resources.
        This method should be called when audio input is no longer needed or
        before application shutdown.
        """
        if self.input_stream is not None:
            try:
                self.input_stream.stop()
                self.input_stream.close()
            except Exception as e:
                logger.error(f"Error stopping input stream: {e}")
            finally:
                self.input_stream = None

    def start_speaking(self, audio_data: NDArray[np.float32], sample_rate: int | None = None, text: str = "") -> tuple[bool, int]:
        """Play audio through the system speakers and block until it finishes or is stopped.

        ``sounddevice.play`` is non-blocking; we ``wait()`` so the next queued sentence
        cannot call ``stop_speaking()`` while this clip is still playing (which was cutting off audio).

        Returns:
            (interrupted, percentage_played): ``interrupted`` is True if ``stop_speaking()`` was used
            mid-play; percentage is a rough estimate for clipping (100 if completed).
        """
        if not isinstance(audio_data, np.ndarray) or audio_data.size == 0:
            raise ValueError("Invalid audio data")

        if sample_rate is None:
            sample_rate = self.SAMPLE_RATE

        # Stop any existing playback
        self.stop_speaking()

        # Reset the stop event
        self._stop_event.clear()

        out_sr = self._resolve_output_samplerate()
        play_data = np.asarray(audio_data, dtype=np.float32)
        if float(sample_rate) != out_sr:
            play_data = _resample_linear_mono(play_data, float(sample_rate), out_sr)
            logger.debug("Resampled playback {} Hz -> {} Hz for PortAudio device", sample_rate, out_sr)
        silence_ms = _tts_trailing_silence_ms()
        if silence_ms > 0:
            n_pad = int(round(float(out_sr) * silence_ms / 1000.0))
            if n_pad > 0:
                play_data = np.concatenate(
                    [play_data, np.zeros(n_pad, dtype=np.float32)]
                )
                logger.debug(
                    "Appended {:.0f} ms trailing silence ({} samples) for sentence spacing",
                    silence_ms,
                    n_pad,
                )
        logger.debug(
            "Playing audio with sample rate: {} Hz (device), length: {} samples",
            out_sr,
            len(play_data),
        )
        self._is_playing = True
        play_kw: dict[str, Any] = {}
        if self._output_device is not None:
            play_kw["device"] = self._output_device
        sd.play(play_data, out_sr, **play_kw)
        sd.wait()
        interrupted = self._stop_event.is_set()
        if not interrupted:
            self._is_playing = False
        percentage_played = 50 if interrupted else 100
        return interrupted, percentage_played

    def _resolve_output_samplerate(self) -> float:
        """Pick a rate ALSA accepts; cache result. Override: GLADOS_AUDIO_OUTPUT_SR=48000."""
        if self._cached_output_sr is not None:
            return self._cached_output_sr
        env = _env_float("GLADOS_AUDIO_OUTPUT_SR")
        if env is not None and env > 0:
            self._cached_output_sr = env
            logger.info("Playback sample rate from GLADOS_AUDIO_OUTPUT_SR: {} Hz", env)
            return env
        for sr in _ordered_output_samplerates(self._output_device):
            if _samplerate_is_supported(self._output_device, sr, is_input=False):
                self._cached_output_sr = sr
                default = _default_samplerate_for_device(self._output_device, is_input=False)
                if abs(sr - default) > 0.5:
                    logger.info(
                        "Playback using {} Hz (PortAudio default_samplerate {} Hz was not usable)",
                        sr,
                        default,
                    )
                return sr
        raise RuntimeError(
            "No working output sample rate. Try: export GLADOS_AUDIO_OUTPUT_SR=48000 "
            "(or 44100), and set GLADOS_SD_OUTPUT_DEVICE to the correct PortAudio index."
        )

    def measure_percentage_spoken(self, total_samples: int, sample_rate: int | None = None) -> tuple[bool, int]:
        """
        Monitor audio playback progress and return completion status with interrupt detection.

        Streams audio samples through PortAudio and actively tracks the number of samples
        that have been played. The playback can be interrupted by setting self.processing
        to False or self.shutdown_event. Uses a non-blocking callback system with a completion event for
        synchronization.

        Args:
            total_samples (int): Total number of samples in the audio data being played.
        Returns:
            tuple[bool, int]: A tuple containing:
                - bool: True if playback was interrupted, False if completed normally
                - int: Percentage of audio played (0-100)
        """
        if sample_rate is None:
            sample_rate = self.SAMPLE_RATE

        out_sr = self._resolve_output_samplerate()
        if float(sample_rate) != out_sr:
            total_samples = int(round(total_samples * (out_sr / float(sample_rate))))
            sample_rate = int(out_sr)

        interrupted = False
        progress = 0
        completion_event = threading.Event()

        def stream_callback(
            outdata: NDArray[np.float32], frames: int, time: dict[str, Any], status: sd.CallbackFlags
        ) -> None:
            nonlocal progress, interrupted
            progress += frames
            if self._is_playing is False:
                interrupted = True
                completion_event.set()
            if progress >= total_samples:
                completion_event.set()
            outdata.fill(0)

        try:
            logger.debug(f"Using sample rate: {sample_rate} Hz, total samples: {total_samples}")
            out_kw: dict[str, Any] = {
                "callback": stream_callback,
                "samplerate": float(sample_rate),
                "channels": 1,
                "finished_callback": completion_event.set,
            }
            if self._output_device is not None:
                out_kw["device"] = self._output_device
            stream = sd.OutputStream(**out_kw)
            with stream:
                # Add a reasonable maximum timeout to prevent indefinite blocking
                max_timeout = total_samples / sample_rate
                completed = completion_event.wait(max_timeout + 1)  # Add a small buffer to ensure completion
                if not completed:
                    # If the event timed out, force interruption
                    self._is_playing = False
                    interrupted = True
                    logger.debug("Audio playback timed out, forcing interruption")

        except (sd.PortAudioError, RuntimeError):
            logger.debug("Audio stream already closed or invalid")

        percentage_played = min(int(progress / total_samples * 100), 100) if total_samples else 0
        return interrupted, percentage_played

    def check_if_speaking(self) -> bool:
        """Check if audio is currently being played.

        Returns:
            bool: True if audio is currently playing, False otherwise
        """
        return self._is_playing

    def stop_speaking(self) -> None:
        """Stop audio playback and clean up resources.

        Interrupts any ongoing audio playback and waits for the playback thread
        to terminate. This ensures clean resource management and prevents
        multiple overlapping playbacks.
        """
        if self._is_playing:
            self._stop_event.set()
            sd.stop()

            self._is_playing = False

    def get_sample_queue(self) -> queue.Queue[tuple[NDArray[np.float32], bool]]:
        """Get the queue containing audio samples and VAD confidence.

        Returns:
            queue.Queue: A thread-safe queue containing tuples of
                        (audio_sample, vad_confidence)
        """
        return self._sample_queue
