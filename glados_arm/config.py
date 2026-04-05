"""
Hardware constants, link geometry, serial defaults, and calibration knobs.

All angles in *servo space* are integer degrees as sent to the Arduino unless noted.
Model-space angles use radians for shoulder/elbow/base yaw unless noted.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# --- Serial (USB to Arduino Uno R4 WiFi on Raspberry Pi) ---
SERIAL_DEFAULT_PORT = "/dev/ttyACM0"  # Pi default when Arduino enumerates as ACM; Windows: COMx
SERIAL_BAUD = 115200
SERIAL_TIMEOUT_S = 0.5

# --- Link geometry (mm) — vertical chain primary IK ---
# Shoulder joint to elbow pivot; elbow pivot to wrist pivot.
LINK_SHOULDER_ELBOW_MM = 130.0
LINK_ELBOW_WRIST_MM = 120.0

# --- Servo limits (degrees) — validated hardware truth ---
SERVO_WRIST_MIN = 20
SERVO_WRIST_MAX = 220

SERVO_ELBOW_MIN = 80
SERVO_ELBOW_MAX = 270

SERVO_BASE_MIN = 0
SERVO_BASE_MAX = 270

SERVO_SHOULDER_MIN = 0
SERVO_SHOULDER_MAX = 180

# --- Neutral pose (degrees) ---
NEUTRAL_WRIST = 120
NEUTRAL_ELBOW = 175
NEUTRAL_BASE = 135
NEUTRAL_SHOULDER = 90

# --- Model ↔ servo mapping (CALIBRATE against physical motion) ---
# Base: model yaw ψ (rad), 0 = neutral (straight ahead in software convention).
# Positive ψ increases → positive servo delta if BASE_YAW_SIGN = +1.
BASE_YAW_SIGN = 1.0
# How many servo degrees correspond to 1 radian of model yaw (approx: 180/pi).
BASE_RAD_TO_SERVO_DEG = 180.0 / math.pi

# Shoulder: model angle is offset from neutral (rad). 0 = neutral pose.
# Flipped to -1.0 based on IK benchmark evidence: large clipped_shoulder_min with elbow_up.
SHOULDER_SIGN = -1.0
SHOULDER_RAD_TO_SERVO_DEG = 180.0 / math.pi  # 1 model rad ≈ this many servo degrees (tune)

# Elbow (new hardware mapping): "up" model motion should increase servo command from neutral.
ELBOW_INVERT = False  # if False: servo = NEUTRAL + sign * f(q)
ELBOW_SIGN = 1.0
ELBOW_RAD_TO_SERVO_DEG = 180.0 / math.pi

# Wrist: trim / secondary DOF (not in primary IK v1).
WRIST_SIGN = 1.0
WRIST_RAD_TO_SERVO_DEG = 180.0 / math.pi

# --- Vertical-plane FK frame (radians) ---
# At *neutral* model offsets (q_shoulder=q_elbow=0), the kinematics frame uses these
# absolute link angles (shoulder link from +x, elbow as interior bend).
# **Calibrate** so FK matches a physical measurement at neutral.
THETA1_REF_NEUTRAL_RAD = 0.25  # final micro-pass: test if this further reduces shoulder-min clipping
THETA2_REF_NEUTRAL_RAD = 0.0  # elbow bend: angle from upper arm to lower arm

# Optional: include wrist as constant extra rotation at tip for FK (rad); v1 default 0.
WRIST_BEND_IN_FK_RAD = 0.0

# --- Notes (not used by code; documentation) ---
SERVO_MODEL_NOTES = (
    "All servos are the same beefier type. "
    "PCA channels: 0=wrist, 1=elbow, 2=shoulder, 3=base. "
    "SET_SERVO order remains: wrist, elbow, base, shoulder."
)


@dataclass(frozen=True)
class ServoLimits:
    wrist: tuple[int, int]
    elbow: tuple[int, int]
    base: tuple[int, int]
    shoulder: tuple[int, int]


DEFAULT_SERVO_LIMITS = ServoLimits(
    wrist=(SERVO_WRIST_MIN, SERVO_WRIST_MAX),
    elbow=(SERVO_ELBOW_MIN, SERVO_ELBOW_MAX),
    base=(SERVO_BASE_MIN, SERVO_BASE_MAX),
    shoulder=(SERVO_SHOULDER_MIN, SERVO_SHOULDER_MAX),
)
