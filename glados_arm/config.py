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
SERVO_WRIST_MIN = 0
SERVO_WRIST_MAX = 155

SERVO_ELBOW_MIN = 90
SERVO_ELBOW_MAX = 270

SERVO_BASE_MIN = 0
SERVO_BASE_MAX = 180

SERVO_SHOULDER_MIN = 0
SERVO_SHOULDER_MAX = 180

# --- Neutral pose (degrees) ---
NEUTRAL_WRIST = 60
NEUTRAL_ELBOW = 270
NEUTRAL_BASE = 90
NEUTRAL_SHOULDER = 0

# --- Model ↔ servo mapping (CALIBRATE against physical motion) ---
# Base: model yaw ψ (rad), 0 = neutral (straight ahead in software convention).
# Positive ψ increases → positive servo delta if BASE_YAW_SIGN = +1.
BASE_YAW_SIGN = 1.0
# How many servo degrees correspond to 1 radian of model yaw (approx: 180/pi).
BASE_RAD_TO_SERVO_DEG = 180.0 / math.pi

# Shoulder: model angle is offset from neutral (rad). 0 = neutral pose.
# Servo increases when model shoulder increases (tune sign if motion is reversed).
SHOULDER_SIGN = 1.0
SHOULDER_RAD_TO_SERVO_DEG = 180.0 / math.pi  # 1 model rad ≈ this many servo degrees (tune)

# Elbow: inverted — "up" motion decreases servo from 270. Model offset from neutral (rad).
ELBOW_INVERT = True  # if True: servo = NEUTRAL - sign * f(q)
ELBOW_SIGN = 1.0
ELBOW_RAD_TO_SERVO_DEG = 180.0 / math.pi

# Wrist: trim / secondary DOF (not in primary IK v1).
WRIST_SIGN = 1.0
WRIST_RAD_TO_SERVO_DEG = 180.0 / math.pi

# --- Vertical-plane FK frame (radians) ---
# At *neutral* model offsets (q_shoulder=q_elbow=0), the kinematics frame uses these
# absolute link angles (shoulder link from +x, elbow as interior bend).
# **Calibrate** so FK matches a physical measurement at neutral.
THETA1_REF_NEUTRAL_RAD = 0.0  # upper arm angle from +x in vertical plane
THETA2_REF_NEUTRAL_RAD = 0.0  # elbow bend: angle from upper arm to lower arm

# Optional: include wrist as constant extra rotation at tip for FK (rad); v1 default 0.
WRIST_BEND_IN_FK_RAD = 0.0

# --- Notes (not used by code; documentation) ---
SERVO_MODEL_NOTES = (
    "Base & wrist: MG996R. Shoulder & elbow: DS3225. "
    "Servo order on Arduino: 1=wrist, 2=elbow, 3=base, 4=shoulder."
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
