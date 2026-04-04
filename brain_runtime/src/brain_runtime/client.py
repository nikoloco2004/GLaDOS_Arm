"""WebSocket client with reconnect + keepalive for Pi watchdog."""

from __future__ import annotations

import asyncio
import logging
import time
import uuid

import websockets
from websockets.client import WebSocketClientProtocol

from robot_link import Envelope
from robot_link.messages import CommandPayload, HeartbeatAckPayload

from . import pipeline

log = logging.getLogger(__name__)


class BrainClient:
    def __init__(self, url: str, reconnect_base_s: float = 1.0, reconnect_max_s: float = 30.0) -> None:
        self.url = url
        self.reconnect_base_s = reconnect_base_s
        self.reconnect_max_s = reconnect_max_s

    async def run_forever(self) -> None:
        delay = self.reconnect_base_s
        while True:
            try:
                async with websockets.connect(
                    self.url,
                    ping_interval=20,
                    ping_timeout=40,
                    max_size=12 * 1024 * 1024,
                ) as ws:
                    log.info("connected to %s", self.url)
                    delay = self.reconnect_base_s
                    pipeline.reset_conversation()
                    await self._session(ws)
            except Exception as e:
                log.warning("disconnected: %s — retry in %.1fs", e, delay)
                await asyncio.sleep(delay)
                delay = min(delay * 2, self.reconnect_max_s)

    async def _session(self, ws: WebSocketClientProtocol) -> None:
        keepalive = asyncio.create_task(self._keepalive(ws))
        try:
            async for raw in ws:
                if isinstance(raw, bytes):
                    raw = raw.decode("utf-8")
                env = Envelope.from_json(raw)
                log.info("pi → %s %s", env.type, env.payload)

                if env.type == "user_text":
                    p = env.payload
                    text = str(p.get("text", "")).strip()
                    cid = str(p.get("correlation_id", ""))
                    if text:
                        await pipeline.handle_user_text(ws, text, cid)
                    continue

                if env.type == "user_interrupt":
                    p = env.payload
                    full_out = str(p.get("full_intended_output", "") or "")
                    log.info("pi → user_interrupt (barge-in) cid=%s", p.get("correlation_id"))
                    pipeline.append_interrupt_context(full_out)
                    continue

                if env.type == "user_audio_pcm":
                    p = env.payload
                    pcm = str(p.get("pcm_b64", ""))
                    sr = int(p.get("sample_rate", 16000))
                    cid = str(p.get("correlation_id", ""))
                    if pcm:
                        await pipeline.handle_user_audio_pcm(ws, pcm, sr, cid)
                    continue
        finally:
            keepalive.cancel()
            try:
                await keepalive
            except asyncio.CancelledError:
                pass

    async def _keepalive(self, ws: WebSocketClientProtocol) -> None:
        """Periodic traffic so Pi watchdog sees brain activity."""
        await asyncio.sleep(0.5)
        await self._send_command(ws, "ping", {})
        await asyncio.sleep(0.3)
        await self._send_command(ws, "neutral", {})
        while True:
            await asyncio.sleep(3.0)
            ack = Envelope(
                type="heartbeat_ack",
                payload=HeartbeatAckPayload(rtt_ms=None).to_dict(),
                ts=time.time(),
            )
            await ws.send(ack.to_json())

    async def _send_command(self, ws: WebSocketClientProtocol, name: str, args: dict) -> None:
        cid = str(uuid.uuid4())
        env = Envelope(
            type="command",
            payload=CommandPayload(name=name, args=args, correlation_id=cid).to_dict(),
        )
        log.info("brain → command %s", name)
        await ws.send(env.to_json())
