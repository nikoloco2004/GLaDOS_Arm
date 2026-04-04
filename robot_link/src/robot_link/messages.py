"""Typed payload bodies (stored under envelope.payload)."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass
class HelloPayload:
    hostname: str = ""
    robot_link_version: str = "0.1.0"
    capabilities: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class HelloAckPayload:
    session_id: str = ""
    protocol_v: int = 1

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class HeartbeatPayload:
    uptime_s: float = 0.0
    arm_serial_ok: bool = False
    notes: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class HeartbeatAckPayload:
    rtt_ms: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class SensorPayload:
    name: str = "telemetry"
    readings: dict[str, float] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class CommandPayload:
    """Brain → Pi high-level command."""

    name: str = ""  # e.g. neutral, pose, raw_set_servo
    args: dict[str, Any] = field(default_factory=dict)
    correlation_id: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class CommandAckPayload:
    correlation_id: str = ""
    accepted: bool = False
    reason: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class ActuatorResultPayload:
    correlation_id: str = ""
    ok: bool = False
    detail: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class UserTextPayload:
    """Pi → brain: user typed on the Pi (keyboard / local UI)."""

    text: str = ""
    correlation_id: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class TtsPcmPayload:
    """Brain → Pi: float32 mono PCM for local speaker playback."""

    pcm_b64: str = ""
    sample_rate: int = 22050
    text: str = ""
    correlation_id: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class ErrorPayload:
    code: str = ""
    message: str = ""
    fatal: bool = False

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class FailsafePayload:
    reason: str = ""  # comm_loss, estop, watchdog
    action_taken: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)
