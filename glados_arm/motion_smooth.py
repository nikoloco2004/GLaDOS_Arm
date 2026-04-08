"""Shared smoothing, clamping, and rate limiting for arm motion V1."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .mapping import ServoCommand


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def apply_deadband(v: float, db: float) -> float:
    if abs(v) < db:
        return 0.0
    return v


def step_toward(cur: float, target: float, max_step: float) -> float:
    d = target - cur
    if abs(d) <= max_step:
        return target
    return cur + max_step if d > 0 else cur - max_step


def lowpass_scalar(prev: float, target: float, alpha: float) -> float:
    a = clamp(alpha, 0.0, 1.0)
    return (1.0 - a) * prev + a * target


@dataclass
class JointRateState:
    """Per-joint velocity for acceleration limiting."""

    wrist: float = 0.0
    elbow: float = 0.0
    base: float = 0.0
    shoulder: float = 0.0


def rate_limit_servo_deg(
    prev: ServoCommand,
    target: ServoCommand,
    *,
    max_step_deg: tuple[float, float, float, float],
) -> ServoCommand:
    """Limit per-joint absolute delta (degrees per frame / per update)."""
    return ServoCommand(
        wrist=int(round(step_toward(float(prev.wrist), float(target.wrist), max_step_deg[0]))),
        elbow=int(round(step_toward(float(prev.elbow), float(target.elbow), max_step_deg[1]))),
        base=int(round(step_toward(float(prev.base), float(target.base), max_step_deg[2]))),
        shoulder=int(round(step_toward(float(prev.shoulder), float(target.shoulder), max_step_deg[3]))),
    )


def rate_limit_servo_deg_per_sec(
    prev: ServoCommand,
    target: ServoCommand,
    dt: float,
    max_dps: tuple[float, float, float, float],
) -> ServoCommand:
    """Limit per-joint velocity in deg/s."""
    if dt <= 1e-9:
        return target
    max_step = tuple(min(m * dt, 1e6) for m in max_dps)
    return rate_limit_servo_deg(prev, target, max_step_deg=max_step)


def sync_step_servo_toward(
    prev: ServoCommand,
    target: ServoCommand,
    dt: float,
    max_dps: tuple[float, float, float, float],
) -> ServoCommand:
    """
    Move all joints toward target by the same fraction of each joint's remaining error (0..1],
    capped so no joint exceeds its deg/s limit. Avoids independent LPF per joint (which makes
    one joint appear to lead when errors differ in servo space).
    """
    if dt <= 1e-9:
        return target
    pairs = (
        (float(prev.wrist), float(target.wrist), max_dps[0]),
        (float(prev.elbow), float(target.elbow), max_dps[1]),
        (float(prev.base), float(target.base), max_dps[2]),
        (float(prev.shoulder), float(target.shoulder), max_dps[3]),
    )
    max_frac = 1.0
    for p, t, dps in pairs:
        d = t - p
        ad = abs(d)
        if ad < 1e-9:
            continue
        max_frac = min(max_frac, (dps * dt) / ad)
    if max_frac < 1e-12:
        return prev
    p0, t0, _ = pairs[0]
    p1, t1, _ = pairs[1]
    p2, t2, _ = pairs[2]
    p3, t3, _ = pairs[3]
    nw = p0 + (t0 - p0) * max_frac
    ne = p1 + (t1 - p1) * max_frac
    nb = p2 + (t2 - p2) * max_frac
    ns = p3 + (t3 - p3) * max_frac
    return ServoCommand(
        wrist=int(round(nw)),
        elbow=int(round(ne)),
        base=int(round(nb)),
        shoulder=int(round(ns)),
    )


def servo_command_to_float_tuple(cmd: ServoCommand) -> tuple[float, float, float, float]:
    return (float(cmd.wrist), float(cmd.elbow), float(cmd.base), float(cmd.shoulder))


def float_tuple_to_servo_command(t: tuple[float, float, float, float]) -> ServoCommand:
    return ServoCommand(
        wrist=int(round(t[0])),
        elbow=int(round(t[1])),
        base=int(round(t[2])),
        shoulder=int(round(t[3])),
    )


def sync_step_servo_float_toward(
    prev: tuple[float, float, float, float],
    target: ServoCommand,
    dt: float,
    max_dps: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """
    Same proportional sync as sync_step_servo_toward but keeps float degrees so sub-degree
    motion accumulates across many small steps (integer rounding each step would freeze motion
    when per-step delta is below 0.5 deg).
    """
    if dt <= 1e-9:
        return (
            float(target.wrist),
            float(target.elbow),
            float(target.base),
            float(target.shoulder),
        )
    pairs = (
        (prev[0], float(target.wrist), max_dps[0]),
        (prev[1], float(target.elbow), max_dps[1]),
        (prev[2], float(target.base), max_dps[2]),
        (prev[3], float(target.shoulder), max_dps[3]),
    )
    max_frac = 1.0
    for p, t, dps in pairs:
        d = t - p
        ad = abs(d)
        if ad < 1e-9:
            continue
        max_frac = min(max_frac, (dps * dt) / ad)
    if max_frac < 1e-12:
        return prev
    return (
        prev[0] + (float(target.wrist) - prev[0]) * max_frac,
        prev[1] + (float(target.elbow) - prev[1]) * max_frac,
        prev[2] + (float(target.base) - prev[2]) * max_frac,
        prev[3] + (float(target.shoulder) - prev[3]) * max_frac,
    )


def rate_float_toward_independent(
    prev: tuple[float, float, float, float],
    target: ServoCommand,
    dt: float,
    max_dps: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """
    Each joint steps toward its target by up to max_dps[i]*dt (no shared fraction). Joints that
    need motion all move together each tick; proportional sync used one joint's error to limit
    everyone, which made only one joint's integer command change at a time when errors differed.
    """
    if dt <= 1e-9:
        return (
            float(target.wrist),
            float(target.elbow),
            float(target.base),
            float(target.shoulder),
        )
    tt = (
        float(target.wrist),
        float(target.elbow),
        float(target.base),
        float(target.shoulder),
    )
    out: list[float] = []
    for i in range(4):
        p, t = prev[i], tt[i]
        max_step = max_dps[i] * dt
        d = t - p
        ad = abs(d)
        if ad < 1e-12:
            out.append(p)
        elif ad <= max_step:
            out.append(t)
        else:
            out.append(p + math.copysign(max_step, d))
    return (out[0], out[1], out[2], out[3])


def accel_limit_delta(
    state: JointRateState,
    new_cmd: ServoCommand,
    prev_cmd: ServoCommand,
    dt: float,
    max_accel_dps2: tuple[float, float, float, float],
) -> tuple[ServoCommand, JointRateState]:
    """Second-order: limit change in velocity (deg/s) per joint."""
    if dt <= 1e-9:
        return new_cmd, state
    # Current velocity estimate (deg/s)
    v_w = (float(new_cmd.wrist) - float(prev_cmd.wrist)) / dt
    v_e = (float(new_cmd.elbow) - float(prev_cmd.elbow)) / dt
    v_b = (float(new_cmd.base) - float(prev_cmd.base)) / dt
    v_s = (float(new_cmd.shoulder) - float(prev_cmd.shoulder)) / dt

    def limit_vel(v_cur: float, v_prev: float, a_max: float) -> float:
        dv = v_cur - v_prev
        max_dv = a_max * dt
        if abs(dv) <= max_dv:
            return v_cur
        return v_prev + math.copysign(max_dv, dv)

    v_w = limit_vel(v_w, state.wrist, max_accel_dps2[0])
    v_e = limit_vel(v_e, state.elbow, max_accel_dps2[1])
    v_b = limit_vel(v_b, state.base, max_accel_dps2[2])
    v_s = limit_vel(v_s, state.shoulder, max_accel_dps2[3])

    out = ServoCommand(
        wrist=int(round(float(prev_cmd.wrist) + v_w * dt)),
        elbow=int(round(float(prev_cmd.elbow) + v_e * dt)),
        base=int(round(float(prev_cmd.base) + v_b * dt)),
        shoulder=int(round(float(prev_cmd.shoulder) + v_s * dt)),
    )
    new_state = JointRateState(wrist=v_w, elbow=v_e, base=v_b, shoulder=v_s)
    return out, new_state


def command_delta_max_deg(a: ServoCommand, b: ServoCommand) -> float:
    return max(
        abs(a.wrist - b.wrist),
        abs(a.elbow - b.elbow),
        abs(a.base - b.base),
        abs(a.shoulder - b.shoulder),
    )
