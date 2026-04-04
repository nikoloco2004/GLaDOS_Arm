"""PC-side: Pi user_text → Ollama → GLaDOS TTS → tts_pcm back to Pi."""

from __future__ import annotations

import asyncio
import base64
import logging
import os
import uuid
from collections import deque
from pathlib import Path
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
_system_prompt_cache: str | None = None

# Sliding window of {user, assistant} pairs so GLaDOS stays in character across turns (matches full glados stack).
def _history_maxlen() -> int:
    raw = int(os.environ.get("OLLAMA_CHAT_HISTORY_MAX", "24"))
    return max(2, min(128, raw))


_chat_history: deque[dict[str, str]] = deque(maxlen=_history_maxlen())


def reset_conversation() -> None:
    """Clear chat history (call when a new WebSocket session starts)."""
    _chat_history.clear()


def _append_turn(user_text: str, assistant_text: str) -> None:
    _chat_history.append({"role": "user", "content": user_text})
    _chat_history.append({"role": "assistant", "content": assistant_text})


def _chat_url() -> str:
    global _ollama_chat_url
    if _ollama_chat_url is None:
        base = os.environ.get("OLLAMA_URL", "http://127.0.0.1:11434").rstrip("/")
        _ollama_chat_url = f"{base}/api/chat"
    return _ollama_chat_url


def _ollama_model() -> str:
    # Default matches personality_core/configs/glados_config.yaml (original project); use llama3.2:1b on Pi-only.
    return os.environ.get("OLLAMA_MODEL", "llama3.2")


def _ollama_extra_options() -> dict[str, Any]:
    ctx = int(os.environ.get("OLLAMA_NUM_CTX", "8192"))
    return {"num_ctx": max(2048, min(131072, ctx))}


def _system_prompt() -> str:
    """Same persona as configs/pi_potato.yaml: env override, then pi_potato_system_prompt.txt."""
    global _system_prompt_cache
    if _system_prompt_cache is not None:
        return _system_prompt_cache
    if os.environ.get("GLADOS_CHAT_SYSTEM_PROMPT", "").strip():
        _system_prompt_cache = os.environ["GLADOS_CHAT_SYSTEM_PROMPT"].strip()
        return _system_prompt_cache
    path_env = os.environ.get("GLADOS_SYSTEM_PROMPT_FILE", "").strip()
    candidates: list[Path] = []
    if path_env:
        candidates.append(Path(path_env))
    here = Path(__file__).resolve()
    # .../GLaDOS_Arm/brain_runtime/src/brain_runtime/pipeline.py -> parents[3] == GLaDOS_Arm
    candidates.append(here.parents[3] / "configs" / "pi_potato_system_prompt.txt")
    candidates.append(Path.cwd() / "configs" / "pi_potato_system_prompt.txt")
    for p in candidates:
        try:
            if p.is_file():
                _system_prompt_cache = p.read_text(encoding="utf-8").strip()
                log.info("brain pipeline: system prompt from %s", p)
                return _system_prompt_cache
        except OSError:
            continue
    _system_prompt_cache = (
        "You are GLaDOS from Portal and Portal 2. Short replies (one to three sentences). "
        "Never use ALL CAPS (speech is read aloud). Dry, sarcastic, in character."
    )
    log.warning("brain pipeline: pi_potato_system_prompt.txt not found; using short default")
    return _system_prompt_cache


def _voice() -> str:
    return os.environ.get("GLADOS_VOICE", "glados")


def _ollama_reply_sync(user_text: str) -> str:
    payload: dict[str, Any] = {
        "model": _ollama_model(),
        "stream": False,
        "messages": [
            {"role": "system", "content": _system_prompt()},
            *list(_chat_history),
            {"role": "user", "content": user_text},
        ],
        "options": _ollama_extra_options(),
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
    llm_ok = False
    try:
        reply = await loop.run_in_executor(None, _ollama_reply_sync, user_text)
        llm_ok = True
    except Exception as e:
        log.exception("Ollama chat failed")
        reply = f"I'm having trouble reaching my brain. {e}"

    if not reply:
        reply = "…"

    if llm_ok:
        _append_turn(user_text, reply)

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
