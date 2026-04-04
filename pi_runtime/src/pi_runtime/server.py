"""Async WebSocket server (Pi side)."""

from __future__ import annotations

import asyncio
import base64
import logging
import os
import queue
import socket
import sys
import threading
import time
from typing import Any

import numpy as np
import websockets
from websockets.server import WebSocketServerProtocol

from robot_link import Envelope
from robot_link.messages import (
    ActuatorResultPayload,
    CommandAckPayload,
    CommandPayload,
    ErrorPayload,
    FailsafePayload,
    HeartbeatPayload,
    HelloPayload,
    TtsPcmPayload,
    UserAudioPcmPayload,
    UserInterruptPayload,
    UserTextPayload,
)

from .audio_play import mic_interrupt_monitor, pcm_b64_to_numpy, play_float32_mono_interruptible
from .mic_record import record_mic_float32_mono
from .executor import execute_command
from .safety import LinkWatchdog

log = logging.getLogger(__name__)

# One capture stream per mic (typical USB audio): a second brain client would start a second VAD
# thread and get ALSA "Device unavailable". Reject extras so the first session keeps the mic.
_brain_sessions = 0
_brain_sessions_lock: asyncio.Lock | None = None


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, str(default)))
    except ValueError:
        return default


def _stdin_interrupt_enabled() -> bool:
    return os.environ.get("PI_STDIN_INTERRUPT", "1").strip().lower() not in (
        "0",
        "false",
        "no",
        "off",
    )


def _mic_uplink_enabled() -> bool:
    return os.environ.get("PI_MIC_UPLINK", "1").strip().lower() not in (
        "0",
        "false",
        "no",
        "off",
    )


def _mic_stream_enabled() -> bool:
    """Continuous Silero VAD → utterance clips → user_audio_pcm (requires personality_core on Pi)."""
    from .mic_stream_vad import mic_mode_wants_continuous_stream

    return mic_mode_wants_continuous_stream()


async def _handler(ws: WebSocketServerProtocol) -> None:
    global _brain_sessions
    lock = _brain_sessions_lock
    if lock is None:
        raise RuntimeError("pi_runtime server lock not initialized (internal error)")
    async with lock:
        if _brain_sessions > 0:
            log.warning(
                "rejecting extra brain connection from %s (only one client; second would steal/break the mic)",
                ws.remote_address,
            )
            await ws.close(
                code=1008,
                reason="pi_runtime: only one brain WebSocket at a time",
            )
            return
        _brain_sessions += 1
    try:
        await _handler_session(ws)
    finally:
        async with lock:
            _brain_sessions -= 1


async def _handler_session(ws: WebSocketServerProtocol) -> None:
    peer = ws.remote_address
    log.info("brain connected: %s", peer)
    watchdog = LinkWatchdog(failsafe_s=_env_float("PI_FAILSAFE_S", 8.0))
    watchdog.on_brain_message()

    host = socket.gethostname()
    caps = ["stub_commands", "heartbeat", "voice_loop", "voice_interrupt", "mic_uplink"]
    if _mic_stream_enabled() and _mic_uplink_enabled():
        try:
            from .mic_stream_vad import vad_stream_available

            if vad_stream_available():
                caps.append("mic_stream_vad")
        except Exception:
            pass
    hello = Envelope(
        type="hello",
        payload=HelloPayload(hostname=host, capabilities=caps).to_dict(),
    )
    await ws.send(hello.to_json())

    # While TTS plays, this handler blocks in asyncio.to_thread(...) and cannot read the WebSocket.
    # Without feeding the watchdog, long clips exceed PI_FAILSAFE_S and we log false comm_loss.
    playback_active = threading.Event()

    async def heartbeat_loop() -> None:
        while True:
            await asyncio.sleep(2.0)
            if playback_active.is_set():
                watchdog.on_brain_message()
            if watchdog.check():
                fs = Envelope(
                    type="failsafe",
                    payload=FailsafePayload(reason="comm_loss", action_taken="stub_log_only").to_dict(),
                )
                await ws.send(fs.to_json())
                log.error("failsafe: no brain traffic within window")
                watchdog.reset_after_failsafe()
            hb = Envelope(
                type="heartbeat",
                payload=HeartbeatPayload(
                    uptime_s=time.monotonic(),
                    arm_serial_ok=False,
                    notes="stub",
                ).to_dict(),
            )
            await ws.send(hb.to_json())

    hb_task = asyncio.create_task(heartbeat_loop())

    # TTS playback state: stop event + metadata for user_interrupt ordering (matches GLaDOS speech_player).
    playback: dict[str, Any] = {
        "stop": None,
        "cid": "",
        "text": "",
        "stdin_skip": False,
    }

    def request_playback_stop() -> None:
        ev = playback["stop"]
        if ev is not None:
            ev.set()

    async def stdin_interrupt_and_stop_playback() -> None:
        if playback["stop"] is not None and _stdin_interrupt_enabled():
            try:
                ui = Envelope(
                    type="user_interrupt",
                    payload=UserInterruptPayload(
                        correlation_id=str(playback.get("cid", "")),
                        full_intended_output=str(playback.get("text", "")),
                    ).to_dict(),
                )
                await ws.send(ui.to_json())
                playback["stdin_skip"] = True
                log.info("pi → brain user_interrupt (stdin) before new input")
            except Exception as e:
                log.warning("send user_interrupt (stdin) failed: %s", e)
        if _stdin_interrupt_enabled():
            request_playback_stop()

    async def stdin_to_brain() -> None:
        """Forward lines typed on the Pi (SSH/console) to the brain as user_text or /mic PCM."""
        loop = asyncio.get_event_loop()
        mic_cmd = os.environ.get("PI_MIC_COMMAND", "/mic").strip().lower()
        while True:
            line = await loop.run_in_executor(None, sys.stdin.readline)
            if not line:
                break
            text = line.strip()
            # Interrupt on Enter even with an empty line (user expects blank line to cut TTS).
            await stdin_interrupt_and_stop_playback()
            if not text:
                continue

            if _mic_uplink_enabled() and text.lower() == mic_cmd:
                sec = _env_float("PI_MIC_SECONDS", 5.0)
                sec = max(0.5, min(60.0, sec))
                try:
                    samples, sr = await asyncio.to_thread(record_mic_float32_mono, sec)
                except Exception as e:
                    log.exception("mic record failed: %s", e)
                    continue
                raw = base64.b64encode(np.asarray(samples, dtype=np.float32).tobytes()).decode("ascii")
                cid = str(time.time())
                env = Envelope(
                    type="user_audio_pcm",
                    payload=UserAudioPcmPayload(
                        pcm_b64=raw,
                        sample_rate=int(round(sr)),
                        correlation_id=cid,
                    ).to_dict(),
                )
                try:
                    await ws.send(env.to_json())
                    log.info(
                        "pi → brain user_audio_pcm: %d samples @ %d Hz",
                        samples.size,
                        int(round(sr)),
                    )
                except Exception as e:
                    log.warning("send user_audio_pcm failed: %s", e)
                    break
                continue

            cid = str(time.time())
            env = Envelope(
                type="user_text",
                payload=UserTextPayload(text=text, correlation_id=cid).to_dict(),
            )
            try:
                await ws.send(env.to_json())
                log.info("pi → brain user_text: %s", text[:120])
            except Exception as e:
                log.warning("send user_text failed: %s", e)
                break

    stdin_task: asyncio.Task | None = None
    if sys.stdin.isatty() and os.environ.get("PI_VOICE_LOOP", "1") not in ("0", "false", "False"):
        stdin_task = asyncio.create_task(stdin_to_brain())
        if _mic_uplink_enabled():
            log.info(
                "voice loop: lines → user_text; %s + Enter → record Pi mic → ASR on PC (Ctrl+D to stop stdin)",
                os.environ.get("PI_MIC_COMMAND", "/mic"),
            )
        else:
            log.info("voice loop: type lines on this terminal → user_text (Ctrl+D to stop stdin)")

    stream_stop = threading.Event()
    utterance_q: queue.Queue[np.ndarray] = queue.Queue(maxsize=32)
    vad_thread: threading.Thread | None = None
    vad_task: asyncio.Task[None] | None = None

    if _mic_stream_enabled() and _mic_uplink_enabled():
        try:
            from .mic_stream_vad import run_vad_stream_thread, vad_stream_available

            if vad_stream_available():
                vad_thread = threading.Thread(
                    target=run_vad_stream_thread,
                    args=(utterance_q, stream_stop),
                    name="pi-vad-mic",
                    daemon=True,
                )
                vad_thread.start()

                async def vad_utterance_uplink() -> None:
                    sr_16k = 16000

                    def _get_utt() -> np.ndarray:
                        return utterance_q.get(timeout=0.5)

                    while not stream_stop.is_set():
                        try:
                            samples = await asyncio.to_thread(_get_utt)
                        except queue.Empty:
                            continue
                        await stdin_interrupt_and_stop_playback()
                        raw = base64.b64encode(
                            np.asarray(samples, dtype=np.float32).tobytes()
                        ).decode("ascii")
                        cid = str(time.time())
                        env = Envelope(
                            type="user_audio_pcm",
                            payload=UserAudioPcmPayload(
                                pcm_b64=raw,
                                sample_rate=sr_16k,
                                correlation_id=cid,
                            ).to_dict(),
                        )
                        try:
                            await ws.send(env.to_json())
                            log.info(
                                "pi → brain user_audio_pcm (VAD): %d samples @ %d Hz",
                                samples.size,
                                sr_16k,
                            )
                        except Exception as e:
                            log.warning("send user_audio_pcm (VAD) failed: %s", e)
                            break

                vad_task = asyncio.create_task(vad_utterance_uplink())
                log.info(
                    "Pi mic stream: always-on Silero VAD → utterances → user_audio_pcm "
                    "(PI_MIC_MODE=push for %s-only)",
                    os.environ.get("PI_MIC_COMMAND", "/mic"),
                )
            else:
                log.warning(
                    "Continuous mic requested (default) but VAD model missing; "
                    "install personality_core on Pi and run: cd personality_core && python -m glados.cli download "
                    "(or set PI_MIC_MODE=push to use only %s)",
                    os.environ.get("PI_MIC_COMMAND", "/mic"),
                )
        except Exception as e:
            log.warning("VAD mic stream not started: %s", e)

    try:
        async for raw in ws:
            watchdog.on_brain_message()
            if isinstance(raw, bytes):
                raw = raw.decode("utf-8")
            try:
                env = Envelope.from_json(raw)
            except Exception as e:
                err = Envelope(
                    type="error",
                    payload=ErrorPayload(code="parse", message=str(e), fatal=False).to_dict(),
                )
                await ws.send(err.to_json())
                continue

            if env.type in ("heartbeat_ack", "hello_ack"):
                continue

            if env.type == "tts_pcm":
                p = env.payload
                try:
                    payload = TtsPcmPayload(
                        pcm_b64=str(p.get("pcm_b64", "")),
                        sample_rate=int(p.get("sample_rate", 22050)),
                        text=str(p.get("text", "")),
                        correlation_id=str(p.get("correlation_id", "")),
                    )
                    samples = pcm_b64_to_numpy(payload.pcm_b64)
                    # Stop any clip already playing (e.g. user typed again before this frame arrived).
                    request_playback_stop()
                    stop_ev = threading.Event()
                    playback["stop"] = stop_ev
                    playback["cid"] = payload.correlation_id
                    playback["text"] = payload.text
                    done_ev = threading.Event()
                    vad_barge = vad_task is not None
                    if vad_barge:
                        from .mic_stream_vad import (
                            duplex_voice_during_tts,
                            set_barge_in_target,
                            set_playback_playing,
                        )

                        if duplex_voice_during_tts():
                            set_barge_in_target(stop_ev)
                        else:
                            set_playback_playing(True)
                    mic_thread = threading.Thread(
                        target=mic_interrupt_monitor,
                        args=(stop_ev, done_ev),
                        name="pi-mic-interrupt",
                        daemon=True,
                    )
                    mic_thread.start()
                    interrupted = False
                    stdin_already_sent = False
                    playback_active.set()
                    try:
                        interrupted = await asyncio.to_thread(
                            play_float32_mono_interruptible,
                            samples,
                            float(payload.sample_rate),
                            stop_ev,
                        )
                    finally:
                        playback_active.clear()
                        if vad_barge:
                            from .mic_stream_vad import (
                                duplex_voice_during_tts,
                                set_barge_in_target,
                                set_playback_playing,
                            )

                            if duplex_voice_during_tts():
                                set_barge_in_target(None)
                            else:
                                set_playback_playing(False)
                        done_ev.set()
                        mic_thread.join(timeout=2.0)
                        if playback["stop"] is stop_ev:
                            stdin_already_sent = bool(playback.get("stdin_skip"))
                            playback["stop"] = None
                            playback["cid"] = ""
                            playback["text"] = ""
                            playback["stdin_skip"] = False

                    if interrupted:
                        log.info("tts_pcm interrupted (stdin, mic, or overlapping clip)")
                        if not stdin_already_sent:
                            try:
                                ui = Envelope(
                                    type="user_interrupt",
                                    payload=UserInterruptPayload(
                                        correlation_id=payload.correlation_id,
                                        full_intended_output=payload.text,
                                    ).to_dict(),
                                )
                                await ws.send(ui.to_json())
                            except Exception as send_e:
                                log.warning("send user_interrupt failed: %s", send_e)

                    log.info(
                        "played tts_pcm: %d samples @ %d Hz (%s)%s",
                        samples.size,
                        payload.sample_rate,
                        payload.text[:80],
                        " [interrupted]" if interrupted else "",
                    )
                except Exception as e:
                    log.exception("tts_pcm playback failed: %s", e)
                continue

            if env.type == "command":
                p = env.payload
                cmd = CommandPayload(
                    name=str(p.get("name", "")),
                    args=dict(p.get("args") or {}),
                    correlation_id=str(p.get("correlation_id") or env.id),
                )
                ok, detail = execute_command(cmd.name, cmd.args)
                ack = Envelope(
                    type="command_ack",
                    payload=CommandAckPayload(
                        correlation_id=cmd.correlation_id,
                        accepted=ok,
                        reason=detail if not ok else "ok",
                    ).to_dict(),
                )
                await ws.send(ack.to_json())
                res = Envelope(
                    type="actuator_result",
                    payload=ActuatorResultPayload(
                        correlation_id=cmd.correlation_id,
                        ok=ok,
                        detail=detail,
                    ).to_dict(),
                )
                await ws.send(res.to_json())
            else:
                log.debug("ignored type=%s", env.type)

    finally:
        stream_stop.set()
        if vad_task:
            vad_task.cancel()
            try:
                await vad_task
            except asyncio.CancelledError:
                pass
        if vad_thread and vad_thread.is_alive():
            vad_thread.join(timeout=3.0)
        hb_task.cancel()
        try:
            await hb_task
        except asyncio.CancelledError:
            pass
        if stdin_task:
            stdin_task.cancel()
            try:
                await stdin_task
            except asyncio.CancelledError:
                pass
        log.info("brain disconnected: %s", peer)


def _ws_create_server_kwargs(bind_host: str) -> dict[str, Any]:
    """IPv6 any-address needs AF_INET6 so Linux dual-stack accepts IPv4 + IPv6."""
    if bind_host in ("::", "::0"):
        return {"family": socket.AF_INET6}
    return {}


def _format_listen_url(bind_host: str, port: int) -> str:
    """Log line / URL form (bracket IPv6)."""
    if bind_host in ("::", "::0"):
        return f"[::]:{port}"
    if ":" in bind_host and not bind_host.startswith("["):
        return f"[{bind_host}]:{port}"
    return f"{bind_host}:{port}"


async def run_server(host: str, port: int) -> None:
    global _brain_sessions_lock
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    _brain_sessions_lock = asyncio.Lock()

    async def _serve_bound(bind_host: str) -> None:
        async with websockets.serve(
            _handler,
            bind_host,
            port,
            ping_interval=20,
            ping_timeout=40,
            max_size=12 * 1024 * 1024,
            **_ws_create_server_kwargs(bind_host),
        ):
            log.info("pi_runtime listening on ws://%s", _format_listen_url(bind_host, port))
            await asyncio.Future()

    try:
        await _serve_bound(host)
    except OSError as e:
        if host in ("::", "::0"):
            log.warning(
                "bind on %s failed (%s); falling back to 0.0.0.0 (IPv4 only)",
                _format_listen_url(host, port),
                e,
            )
            await _serve_bound("0.0.0.0")
        else:
            raise
