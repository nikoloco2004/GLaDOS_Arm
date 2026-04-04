from __future__ import annotations

import json
import uuid
import time
from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass
class Envelope:
    """Wire format for every WebSocket text frame (JSON)."""

    v: int = 1
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    type: str = ""
    ts: float = field(default_factory=time.time)
    payload: dict[str, Any] = field(default_factory=dict)

    def to_json(self) -> str:
        return json.dumps(asdict(self), separators=(",", ":"))

    @classmethod
    def from_json(cls, raw: str) -> Envelope:
        d = json.loads(raw)
        return cls(
            v=int(d.get("v", 1)),
            id=str(d.get("id", str(uuid.uuid4()))),
            type=str(d["type"]),
            ts=float(d.get("ts", time.time())),
            payload=dict(d.get("payload") or {}),
        )
