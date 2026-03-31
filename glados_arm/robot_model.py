"""
Mechanism model: axis decomposition, frames, and explicit assumptions.

This arm is **not** a symmetric planar XY manipulator driven equally by shoulder and elbow.

Horizontal aiming
-----------------
* Only the **base** joint contributes to left/right (azimuth / image X).
* Model: base yaw ψ about the vertical axis; ψ = 0 at neutral (aligned with software "forward").

Vertical aiming (primary v1)
----------------------------
* **Shoulder + elbow** form a 2R planar chain in the median vertical plane (after base rotation).
* **Wrist** is modeled as trim / future pitch compensation — not used in primary IK unless you set FK/trim.

Coordinate systems
--------------------
**World frame** (for high-level targeting):
* Origin at shoulder height on the base axis (intersection of base rotation axis with upper structure).
* +Y: vertical up.
* +X: horizontal to the robot's right (when viewed from above, base at center).
* +Z: "forward" / camera optical axis direction at ψ = 0 (adjust to your install).

**Base yaw ψ**: rotation about +Y (right-hand rule). Positive ψ rotates the vertical plane
toward +X (left/right aim). *You may flip the sign in mapping* to match your wiring.

**Vertical chain plane** (2D, before base rotation):
* Origin at shoulder pivot in that plane.
* +x_pl: "forward" in the arm plane (same direction as world +Z when ψ = 0).
* +z_pl: "up" in that plane (aligned with world +Y).

Forward kinematics returns tip position in **plane coordinates** (x_pl, z_pl) in mm.
Full 3D tip position (optional): rotate (x_pl, 0, z_pl) by ψ about world Y.

Assumptions (v1)
----------------
* Shoulder and elbow are revolute; wrist pitch is a small correction or held fixed.
* Link lengths are shoulder→elbow and elbow→wrist pivot (not potato CG).
* No dynamic compensation (gravity, compliance) — purely geometric.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class TipPlane:
    """Tip position in the vertical-chain plane (mm)."""

    x: float  # forward in plane
    z: float  # up in plane


@dataclass(frozen=True)
class TipWorld:
    """Tip in world frame (mm) — optional composition with base yaw."""

    x: float
    y: float
    z: float


def plane_to_world(x_pl: float, z_pl: float, yaw_rad: float) -> TipWorld:
    """Rotate plane forward axis into world; y vertical."""
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    # Plane forward (+x_pl) maps to world +Z at ψ=0; horizontal component splits to X,Z.
    wx = -x_pl * s
    wy = z_pl
    wz = x_pl * c
    return TipWorld(x=wx, y=wy, z=wz)
