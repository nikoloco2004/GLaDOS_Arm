"""
Model-space ↔ servo-space mapping (offsets, inversion, clamps).

Model conventions
-----------------
* **base_yaw_rad** ψ: 0 at neutral; positive direction set by BASE_YAW_SIGN.
* **q_shoulder**, **q_elbow**, **q_wrist**: offsets from neutral (radians), 0 at neutral pose.

Elbow inversion is implemented here: increasing "up" model motion maps to **decreasing**
servo command from NEUTRAL_ELBOW when ELBOW_INVERT is True.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from . import config
from .config import DEFAULT_SERVO_LIMITS, ServoLimits


@dataclass(frozen=True)
class ServoCommand:
    """Integer degrees in Arduino order: wrist, elbow, base, shoulder."""

    wrist: int
    elbow: int
    base: int
    shoulder: int


@dataclass(frozen=True)
class ModelJointState:
    """Continuous model-space state for IK / control."""

    base_yaw_rad: float
    q_shoulder_rad: float
    q_elbow_rad: float
    q_wrist_rad: float


def clamp_servo(cmd: ServoCommand, limits: ServoLimits | None = None) -> tuple[ServoCommand, list[str]]:
    """Clamp each joint; return warnings for any clip."""
    lim = limits or DEFAULT_SERVO_LIMITS
    notes: list[str] = []

    def c(v: int, lo: int, hi: int, name: str) -> int:
        if v < lo:
            notes.append(f"clipped_{name}_min")
            return lo
        if v > hi:
            notes.append(f"clipped_{name}_max")
            return hi
        return v

    out = ServoCommand(
        wrist=c(cmd.wrist, *lim.wrist, "wrist"),
        elbow=c(cmd.elbow, *lim.elbow, "elbow"),
        base=c(cmd.base, *lim.base, "base"),
        shoulder=c(cmd.shoulder, *lim.shoulder, "shoulder"),
    )
    return out, notes


def model_to_servo(m: ModelJointState) -> ServoCommand:
    """Convert model offsets + yaw to raw servo degrees (before clamp)."""
    base_deg = config.NEUTRAL_BASE + config.BASE_YAW_SIGN * m.base_yaw_rad * config.BASE_RAD_TO_SERVO_DEG

    sh_deg = config.NEUTRAL_SHOULDER + config.SHOULDER_SIGN * m.q_shoulder_rad * config.SHOULDER_RAD_TO_SERVO_DEG

    if config.ELBOW_INVERT:
        el_deg = config.NEUTRAL_ELBOW - config.ELBOW_SIGN * m.q_elbow_rad * config.ELBOW_RAD_TO_SERVO_DEG
    else:
        el_deg = config.NEUTRAL_ELBOW + config.ELBOW_SIGN * m.q_elbow_rad * config.ELBOW_RAD_TO_SERVO_DEG

    wr_deg = config.NEUTRAL_WRIST + config.WRIST_SIGN * m.q_wrist_rad * config.WRIST_RAD_TO_SERVO_DEG

    return ServoCommand(
        wrist=int(round(wr_deg)),
        elbow=int(round(el_deg)),
        base=int(round(base_deg)),
        shoulder=int(round(sh_deg)),
    )


def servo_to_model(s: ServoCommand) -> ModelJointState:
    """Inverse mapping (approximate inverse of linearized model_to_servo)."""
    base_yaw = (s.base - config.NEUTRAL_BASE) / (config.BASE_YAW_SIGN * config.BASE_RAD_TO_SERVO_DEG)
    q_shoulder = (s.shoulder - config.NEUTRAL_SHOULDER) / (config.SHOULDER_SIGN * config.SHOULDER_RAD_TO_SERVO_DEG)
    if config.ELBOW_INVERT:
        q_elbow = (config.NEUTRAL_ELBOW - s.elbow) / (config.ELBOW_SIGN * config.ELBOW_RAD_TO_SERVO_DEG)
    else:
        q_elbow = (s.elbow - config.NEUTRAL_ELBOW) / (config.ELBOW_SIGN * config.ELBOW_RAD_TO_SERVO_DEG)
    q_wrist = (s.wrist - config.NEUTRAL_WRIST) / (config.WRIST_SIGN * config.WRIST_RAD_TO_SERVO_DEG)
    return ModelJointState(
        base_yaw_rad=base_yaw,
        q_shoulder_rad=q_shoulder,
        q_elbow_rad=q_elbow,
        q_wrist_rad=q_wrist,
    )


def solve_base_yaw_from_azimuth_error_rad(error_rad: float) -> float:
    """
    Map a desired horizontal correction (e.g. from vision) to base yaw delta.
    Vision layer should feed **angular** error, not raw pixels, after calibration.
    """
    return config.BASE_YAW_SIGN * error_rad
