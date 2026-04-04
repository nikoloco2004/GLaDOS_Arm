"""PC-side: Pi user_text → Ollama → GLaDOS TTS → tts_pcm back to Pi."""

from __future__ import annotations

import asyncio
import base64
import logging
import os
import uuid
from typing import Any

import httpx
import numpy as np
from numpy.typing import NDArray
from websockets.client import WebSocketClientProtocol

from robot_link import Envelope
from robot_link.messages import TtsPcmPayload

log = logging.getLogger(__name__)

_ollama_chat_url: str | None = None
_tts_model: Any = None


def _chat_url() -> str:
    global _ollama_chat_url
    if _ollama_chat_url is None:
        base = os.environ.get("OLLAMA_URL", "http://127.0.0.1:11434").rstrip("/")
        _ollama_chat_url = f"{base}/api/chat"
    return _ollama_chat_url


def _ollama_model() -> str:
    return os.environ.get("OLLAMA_MODEL", "llama3.2:1b")


def _system_prompt() -> str:
    return os.environ.get(
        "GLADOS_CHAT_SYSTEM_PROMPT",
        "You are GLaDOS from Portal and Portal 2. Short replies (one to three sentences). "
        "Never use ALL CAPS (speech is read aloud). Dry, sarcastic, in character.",
    )


def _voice() -> str:
    return os.environ.get("GLADOS_VOICE", "glados")


def _ollama_reply_sync(user_text: str) -> str:
    payload: dict[str, Any] = {
        "model": _ollama_model(),
        "stream": False,
        "messages": [
            {"role": "system", "content": _system_prompt()},
            {"role": "user", "content": user_text},
        ],
    }
    r = httpx.post(_chat_url(), json=payload, timeout=120.0)
    r.raise_for_status()
    j = r.json()
    msg = j.get("message") or {}
    content = msg.get("content", "")
    return str(content).strip() if content else ""


def _get_tts() -> Any:
    global _tts_model
    if _tts_model is None:
        try:
            from glados.TTS import get_speech_synthesizer
        except ImportError as e:
            raise ImportError(
                "GLaDOS TTS requires personality_core in the same venv: "
                "pip install -e ../personality_core (from GLaDOS_Arm)"
            ) from e
        _tts_model = get_speech_synthesizer(_voice())
    return _tts_model


def _synthesize_sync(reply_text: str) -> tuple[NDArray[np.float32], int]:
    tts = _get_tts()
    audio = tts.generate_speech_audio(reply_text)
    return audio, int(tts.sample_rate)


async def handle_user_text(
    ws: WebSocketClientProtocol,
    user_text: str,
    correlation_id: str,
) -> None:
    """LLM reply + TTS; send tts_pcm to Pi for speaker playback."""
    if not user_text.strip():
        return

    loop = asyncio.get_event_loop()
    reply: str
    try:
        reply = await loop.run_in_executor(None, _ollama_reply_sync, user_text)
    except Exception as e:
        log.exception("Ollama chat failed")
        reply = f"I'm having trouble reaching my brain. {e}"

    if not reply:
        reply = "…"

    try:
        audio, sr = await loop.run_in_executor(None, _synthesize_sync, reply)
    except Exception:
        log.exception("TTS failed")
        return

    raw = np.asarray(audio, dtype=np.float32).tobytes()
    pcm_b64 = base64.b64encode(raw).decode("ascii")
    cid = correlation_id or str(uuid.uuid4())
    env = Envelope(
        type="tts_pcm",
        payload=TtsPcmPayload(
            pcm_b64=pcm_b64,
            sample_rate=sr,
            text=reply,
            correlation_id=cid,
        ).to_dict(),
    )
    await ws.send(env.to_json())
    log.info("brain → tts_pcm (%d samples @ %d Hz)", len(audio), sr)
