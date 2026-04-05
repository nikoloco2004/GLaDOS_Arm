"""
High-level robot interface: horizontal (base) vs vertical (shoulder/elbow) decomposition.

Errors are explicit: unreachable IK, servo clipping, invalid model — never silent.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal, Sequence

from . import config, kinematics
from .kinematics import IKResult
from .mapping import ModelJointState, ServoCommand, clamp_servo, model_to_servo
from .serial_comm import ArmSerial


@dataclass(frozen=True)
class VerticalSolveResult:
    ok: bool
    ik: IKResult
    model: ModelJointState
    servo_raw: ServoCommand
    servo_clamped: ServoCommand
    clip_notes: list[str]
    message: str


def solve_vertical_plane(
    x_mm: float,
    z_mm: float,
    base_yaw_rad: float,
    *,
    q_wrist_rad: float = 0.0,
    prefer: Literal["elbow_up", "elbow_down"] = "elbow_up",
) -> VerticalSolveResult:
    """
    Vertical chain IK in plane + base yaw + optional wrist trim.

    Horizontal aiming is **only** `base_yaw_rad`. The plane target (x,z) is for the 2R chain.
    """
    ik = kinematics.inverse_kinematics_plane(x_mm, z_mm, prefer=prefer)
    if not ik.ok:
        m = ModelJointState(
            base_yaw_rad=base_yaw_rad,
            q_shoulder_rad=0.0,
            q_elbow_rad=0.0,
            q_wrist_rad=q_wrist_rad,
        )
        raw = model_to_servo(m)
        cl, notes = clamp_servo(raw)
        return VerticalSolveResult(
            ok=False,
            ik=ik,
            model=m,
            servo_raw=raw,
            servo_clamped=cl,
            clip_notes=notes,
            message=f"IK failed: {ik.reason}",
        )

    m = ModelJointState(
        base_yaw_rad=base_yaw_rad,
        q_shoulder_rad=ik.q_shoulder,
        q_elbow_rad=ik.q_elbow,
        q_wrist_rad=q_wrist_rad,
    )
    raw = model_to_servo(m)
    cl, notes = clamp_servo(raw)
    ok = len(notes) == 0
    msg = "ok" if ok else "servo_limits_clipped:" + ",".join(notes)
    return VerticalSolveResult(
        ok=ok and ik.ok,
        ik=ik,
        model=m,
        servo_raw=raw,
        servo_clamped=cl,
        clip_notes=notes,
        message=msg,
    )


def solve_azimuth_elevation(
    azimuth_rad: float,
    elevation_rad: float,
    range_mm: float,
    *,
    q_wrist_rad: float = 0.0,
    prefer: Literal["elbow_up", "elbow_down"] = "elbow_up",
) -> VerticalSolveResult:
    """
    Aim at a direction in world: azimuth about vertical axis, elevation above horizontal.

    Converts to plane target: x = range*cos(elevation), z = range*sin(elevation)
    in the vertical plane at that azimuth (handled only by base_yaw = azimuth_rad).
    """
    x = range_mm * math.cos(elevation_rad)
    z = range_mm * math.sin(elevation_rad)
    return solve_vertical_plane(x, z, base_yaw_rad=azimuth_rad, q_wrist_rad=q_wrist_rad, prefer=prefer)


@dataclass
class RobotController:
    """Optional serial bridge; can be used math-only without hardware."""

    serial: ArmSerial | None = None

    def connect(self) -> None:
        if self.serial:
            self.serial.connect()

    def close(self) -> None:
        if self.serial:
            self.serial.close()

    def send_servo(self, cmd: ServoCommand) -> list[str]:
        if not self.serial:
            raise RuntimeError("no serial configured")
        line = f"SET_SERVO {cmd.wrist} {cmd.elbow} {cmd.base} {cmd.shoulder}"
        self.serial.write_line(line)
        return [self.serial.read_line()]

    def neutral(self) -> list[str]:
        if not self.serial:
            raise RuntimeError("no serial configured")
        self.serial.write_line("NEUTRAL")
        return [self.serial.read_line()]

    def startup(self) -> list[str]:
        if not self.serial:
            raise RuntimeError("no serial configured")
        line = (
            f"SET_SERVO {config.STARTUP_WRIST} {config.STARTUP_ELBOW} "
            f"{config.STARTUP_BASE} {config.STARTUP_SHOULDER}"
        )
        self.serial.write_line(line)
        return [self.serial.read_line()]

    def ping(self) -> bool:
        if not self.serial:
            return False
        return self.serial.ping()


def format_servo_line(cmd: ServoCommand) -> str:
    return f"SET_SERVO {cmd.wrist} {cmd.elbow} {cmd.base} {cmd.shoulder}"


def explain_assumptions() -> Sequence[str]:
    return (
        "Horizontal: only base_yaw_rad changes azimuth; shoulder/elbow do not solve image X.",
        "Vertical: IK is planar 2R (shoulder+elbow); wrist is trim only in v1.",
        f"Link lengths: L1={config.LINK_SHOULDER_ELBOW_MM}mm, L2={config.LINK_ELBOW_WRIST_MM}mm.",
        f"FK reference at neutral: theta1_ref={config.THETA1_REF_NEUTRAL_RAD} rad, "
        f"theta2_ref={config.THETA2_REF_NEUTRAL_RAD} rad (calibrate).",
    )
