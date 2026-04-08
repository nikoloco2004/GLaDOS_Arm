"""
Forward and inverse kinematics for the vertical chain (shoulder + elbow).

Plane convention matches `robot_model.TipPlane`: +x forward in median plane, +z up.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

from . import config

# --- IK branch hysteresis (visual servoing): avoid elbow_up/down flipping every frame ---
from .robot_model import TipPlane


@dataclass(frozen=True)
class FKResult:
    tip: TipPlane
    theta1_abs: float  # rad, absolute geometry angle
    theta2_abs: float  # rad, absolute (θ2 in x = L1 cos θ1 + L2 cos(θ1+θ2))


@dataclass(frozen=True)
class IKResult:
    ok: bool
    q_shoulder: float  # rad, offset from neutral
    q_elbow: float  # rad, offset from neutral
    theta1_abs: float
    theta2_abs: float
    reason: str
    solution: Literal["elbow_up", "elbow_down", "none"]


def forward_kinematics(
    q_shoulder: float,
    q_elbow: float,
    *,
    l1: float | None = None,
    l2: float | None = None,
    theta1_ref: float | None = None,
    theta2_ref: float | None = None,
    wrist_extra_bend: float | None = None,
) -> FKResult:
    """
    2R planar FK with offsets from neutral.

    Absolute angles: θ1 = theta1_ref + q_shoulder, θ2 = theta2_ref + q_elbow (+ wrist in v1 as constant).
    """
    l1 = l1 if l1 is not None else config.LINK_SHOULDER_ELBOW_MM
    l2 = l2 if l2 is not None else config.LINK_ELBOW_WRIST_MM
    t1r = theta1_ref if theta1_ref is not None else config.THETA1_REF_NEUTRAL_RAD
    t2r = theta2_ref if theta2_ref is not None else config.THETA2_REF_NEUTRAL_RAD
    w = wrist_extra_bend if wrist_extra_bend is not None else config.WRIST_BEND_IN_FK_RAD

    theta1 = t1r + q_shoulder
    theta2 = t2r + q_elbow + w  # wrist lumped as extra bend at tip for FK experiments

    x = l1 * math.cos(theta1) + l2 * math.cos(theta1 + theta2)
    z = l1 * math.sin(theta1) + l2 * math.sin(theta1 + theta2)
    return FKResult(tip=TipPlane(x=x, z=z), theta1_abs=theta1, theta2_abs=theta2)


def resolve_ik_preference(
    prefer: Literal["elbow_up", "elbow_down"],
    last_solution: Literal["elbow_up", "elbow_down", "none"] | None,
    err_y_norm: float,
    *,
    switch_threshold: float = 0.12,
) -> Literal["elbow_up", "elbow_down"]:
    """
    Keep IK branch stable when vertical error is small; allow switch when error is large.
    """
    if last_solution is None or last_solution == "none":
        return prefer
    if abs(err_y_norm) >= switch_threshold:
        return prefer
    return last_solution


def inverse_kinematics_plane(
    x: float,
    z: float,
    *,
    l1: float | None = None,
    l2: float | None = None,
    theta1_ref: float | None = None,
    theta2_ref: float | None = None,
    prefer: Literal["elbow_up", "elbow_down"] = "elbow_up",
) -> IKResult:
    """
    Solve for (q_shoulder, q_elbow) to reach (x,z) in the vertical plane.

    Uses standard 2R IK (two solutions). Returns offsets from neutral references.
    """
    l1 = l1 if l1 is not None else config.LINK_SHOULDER_ELBOW_MM
    l2 = l2 if l2 is not None else config.LINK_ELBOW_WRIST_MM
    t1r = theta1_ref if theta1_ref is not None else config.THETA1_REF_NEUTRAL_RAD
    t2r = theta2_ref if theta2_ref is not None else config.THETA2_REF_NEUTRAL_RAD

    d = math.hypot(x, z)
    if d < 1e-9:
        return IKResult(
            ok=False,
            q_shoulder=0.0,
            q_elbow=0.0,
            theta1_abs=t1r,
            theta2_abs=t2r,
            reason="target_at_singularity_origin",
            solution="none",
        )

    if d > l1 + l2 + 1e-6:
        return IKResult(
            ok=False,
            q_shoulder=0.0,
            q_elbow=0.0,
            theta1_abs=t1r,
            theta2_abs=t2r,
            reason="unreachable_too_far",
            solution="none",
        )
    if d < abs(l1 - l2) - 1e-6:
        return IKResult(
            ok=False,
            q_shoulder=0.0,
            q_elbow=0.0,
            theta1_abs=t1r,
            theta2_abs=t2r,
            reason="unreachable_too_close",
            solution="none",
        )

    # Elbow interior angle via cosine law: D = cos(theta2_interior)
    D = (d * d - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    if D < -1.0 - 1e-9 or D > 1.0 + 1e-9:
        return IKResult(
            ok=False,
            q_shoulder=0.0,
            q_elbow=0.0,
            theta1_abs=t1r,
            theta2_abs=t2r,
            reason="numeric_cos_elbow",
            solution="none",
        )
    D = max(-1.0, min(1.0, D))
    disc = max(0.0, 1.0 - D * D)
    sqrt_disc = math.sqrt(disc)
    # Same branches as ±acos(D): atan2(±sin, cos) with cos = D
    theta2_pos = math.atan2(sqrt_disc, D)
    theta2_neg = math.atan2(-sqrt_disc, D)

    def solve_theta1(theta2: float) -> float:
        phi = math.atan2(z, x)
        return phi - math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))

    use_a = prefer != "elbow_down"
    theta2_abs = theta2_pos if use_a else theta2_neg
    theta1_abs = solve_theta1(theta2_abs)
    q_shoulder = theta1_abs - t1r
    q_elbow = theta2_abs - t2r
    sol_name: Literal["elbow_up", "elbow_down"] = "elbow_up" if use_a else "elbow_down"

    return IKResult(
        ok=True,
        q_shoulder=q_shoulder,
        q_elbow=q_elbow,
        theta1_abs=theta1_abs,
        theta2_abs=theta2_abs,
        reason="ok",
        solution=sol_name,
    )


def workspace_radius_range(
    l1: float | None = None,
    l2: float | None = None,
) -> tuple[float, float]:
    l1 = l1 if l1 is not None else config.LINK_SHOULDER_ELBOW_MM
    l2 = l2 if l2 is not None else config.LINK_ELBOW_WRIST_MM
    return (abs(l1 - l2), l1 + l2)
