"""Async WebSocket server (Pi side)."""

from __future__ import annotations

import asyncio
import logging
import os
import socket
import sys
import time
from typing import Any

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
    UserTextPayload,
)

from .audio_play import pcm_b64_to_numpy, play_float32_mono
from .executor import execute_command
from .safety import LinkWatchdog

log = logging.getLogger(__name__)


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, str(default)))
    except ValueError:
        return default


async def _handler(ws: WebSocketServerProtocol) -> None:
    peer = ws.remote_address
    log.info("brain connected: %s", peer)
    watchdog = LinkWatchdog(failsafe_s=_env_float("PI_FAILSAFE_S", 8.0))
    watchdog.on_brain_message()

    host = socket.gethostname()
    caps = ["stub_commands", "heartbeat", "voice_loop"]
    hello = Envelope(
        type="hello",
        payload=HelloPayload(hostname=host, capabilities=caps).to_dict(),
    )
    await ws.send(hello.to_json())

    async def heartbeat_loop() -> None:
        while True:
            await asyncio.sleep(2.0)
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

    async def stdin_to_brain() -> None:
        """Forward lines typed on the Pi (SSH/console) to the brain as user_text."""
        loop = asyncio.get_event_loop()
        while True:
            line = await loop.run_in_executor(None, sys.stdin.readline)
            if not line:
                break
            text = line.strip()
            if not text:
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
        log.info("voice loop: type lines on this terminal; they go to the PC brain (Ctrl+D to stop stdin)")

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
                    await asyncio.to_thread(play_float32_mono, samples, float(payload.sample_rate))
                    log.info("played tts_pcm: %d samples @ %d Hz (%s)", samples.size, payload.sample_rate, payload.text[:80])
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


async def run_server(host: str, port: int) -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    async with websockets.serve(
        _handler,
        host,
        port,
        ping_interval=20,
        ping_timeout=40,
        max_size=12 * 1024 * 1024,
    ):
        log.info("pi_runtime listening on ws://%s:%s", host, port)
        await asyncio.Future()
