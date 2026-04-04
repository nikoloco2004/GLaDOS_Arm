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

_tts_model: Any = None
_spoken_text_converter: Any = None
_asr_model: Any = None
_system_prompt_cache: str | None = None

# Parakeet ASR expects 16 kHz mono float32 (see personality_core models/ASR/*.yaml).
_ASR_TARGET_SR = 16000.0

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


def append_interrupt_context(full_intended_output: str = "") -> None:
    """Same idea as personality_core SpeechPlayer: SYSTEM user line after interrupt."""
    if full_intended_output.strip():
        _chat_history.append(
            {
                "role": "user",
                "content": (
                    "[SYSTEM: User interrupted mid-response! Full intended output: "
                    f"'{full_intended_output}']"
                ),
            }
        )
    else:
        _chat_history.append(
            {
                "role": "user",
                "content": "[SYSTEM: User interrupted playback.]",
            }
        )


def _ollama_base_url() -> str:
    """Ollama listen URL (no path). Accepts OLLAMA_URL or OLLAMA_HOST (some env files used the latter)."""
    raw = (os.environ.get("OLLAMA_URL") or os.environ.get("OLLAMA_HOST") or "").strip()
    if not raw:
        return "http://127.0.0.1:11434"
    if not (raw.startswith("http://") or raw.startswith("https://")):
        log.warning(
            "OLLAMA_URL/OLLAMA_HOST must start with http:// or https:// (got %r); using http://127.0.0.1:11434",
            raw[:80],
        )
        return "http://127.0.0.1:11434"
    return raw.rstrip("/")


def _chat_url() -> str:
    return f"{_ollama_base_url()}/api/chat"


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
    url = _chat_url()
    r = httpx.post(url, json=payload, timeout=120.0)
    try:
        r.raise_for_status()
    except httpx.HTTPStatusError as e:
        if e.response.status_code == 404:
            hint = (
                f"Ollama returned 404 at {url!r}. "
                f"If the model is missing, run: ollama pull {_ollama_model()}"
            )
            body = (e.response.text or "")[:400]
            if body:
                hint = f"{hint} — server said: {body}"
            raise RuntimeError(hint) from e
        raise
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


def _get_spoken_text_converter() -> Any:
    """Same as full Glados stack: digits → words so TTS says 'nineteen' not silence/garble on '19'."""
    global _spoken_text_converter
    if _spoken_text_converter is None:
        try:
            from glados.utils.spoken_text_converter import SpokenTextConverter
        except ImportError as e:
            raise ImportError(
                "GLaDOS spoken text requires personality_core in the same venv: "
                "pip install -e ../personality_core (from GLaDOS_Arm)"
            ) from e
        _spoken_text_converter = SpokenTextConverter()
    return _spoken_text_converter


def _text_for_tts(reply_text: str) -> str:
    if os.environ.get("GLADOS_TTS_SPOKEN_TEXT", "1").strip().lower() in ("0", "false", "no"):
        return reply_text
    spoken = _get_spoken_text_converter().text_to_spoken(reply_text)
    if spoken != reply_text:
        log.debug("brain TTS: spoken text differs from raw (digits/symbols normalized)")
    return spoken


def _synthesize_sync(reply_text: str) -> tuple[NDArray[np.float32], int]:
    tts = _get_tts()
    to_speak = _text_for_tts(reply_text)
    audio = tts.generate_speech_audio(to_speak)
    return audio, int(tts.sample_rate)


def _resample_mono_linear(x: NDArray[np.float32], orig_sr: float, target_sr: float) -> NDArray[np.float32]:
    if orig_sr == target_sr or x.size == 0:
        return np.asarray(x, dtype=np.float32).reshape(-1)
    a = np.asarray(x, dtype=np.float64).reshape(-1)
    n = a.shape[0]
    duration = n / orig_sr
    target_n = max(1, int(round(duration * target_sr)))
    t_old = np.linspace(0.0, duration, n, endpoint=False)
    t_new = np.linspace(0.0, duration, target_n, endpoint=False)
    return np.interp(t_new, t_old, a).astype(np.float32)


def _get_asr() -> Any:
    global _asr_model
    if _asr_model is None:
        try:
            from glados.ASR import get_audio_transcriber
        except ImportError as e:
            raise ImportError(
                "Pi mic → ASR requires personality_core in the same venv: "
                "pip install -e ../personality_core (from GLaDOS_Arm)"
            ) from e
        engine = os.environ.get("GLADOS_ASR_ENGINE", "tdt").strip().lower()
        _asr_model = get_audio_transcriber(engine)
        log.info("brain pipeline: ASR engine %s", engine)
    return _asr_model


def transcribe_pi_pcm_sync(pcm_b64: str, sample_rate: int) -> str:
    raw = base64.b64decode(pcm_b64.encode("ascii"))
    samples = np.frombuffer(raw, dtype=np.float32).copy()
    if samples.size == 0:
        return ""
    if abs(float(sample_rate) - _ASR_TARGET_SR) > 0.5:
        samples = _resample_mono_linear(samples, float(sample_rate), _ASR_TARGET_SR)
    return str(_get_asr().transcribe(samples)).strip()


async def handle_user_audio_pcm(
    ws: WebSocketClientProtocol,
    pcm_b64: str,
    sample_rate: int,
    correlation_id: str,
) -> None:
    """Decode Pi mic PCM, run ASR on PC, then same LLM/TTS path as typed user_text."""
    if not pcm_b64.strip():
        return
    loop = asyncio.get_event_loop()
    try:
        text = await loop.run_in_executor(None, transcribe_pi_pcm_sync, pcm_b64, sample_rate)
    except Exception as e:
        log.exception("ASR failed: %s", e)
        text = ""

    if not text.strip():
        log.warning("ASR returned empty transcript; skipping LLM")
        return

    log.info("brain ASR: %s", text[:200])
    await handle_user_text(ws, text, correlation_id)


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
