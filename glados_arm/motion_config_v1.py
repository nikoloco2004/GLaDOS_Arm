"""
V1 motion-control tuning (visual servoing, wrist stabilization, smoothing).

Does not replace vision_config camera/Haar keys; import alongside vision_config.
"""

from __future__ import annotations

# --- Wrist pitch stabilization (chicken-head) ---
# Camera: increasing wrist servo degree pitches camera up (mechanical convention).
WRIST_TRIM_MODE = "stab"  # "stab" | "legacy" — legacy uses image-based trim only
DESIRED_CAMERA_PITCH_RAD = 0.0  # world: level horizon in plane (tune install)
CAMERA_MOUNT_OFFSET_RAD = 0.0  # calibration: wrist zero vs camera boresight
BASE_YAW_COUPLING_GAIN = 0.0  # optional q_wrist += -gain * base_yaw_rad (small, 0 = off)

# Comfort band inside hard limits [NEUTRAL - half, NEUTRAL + half] (servo degrees)
WRIST_COMFORT_HALF_SPAN_DEG = 40.0
REBALANCE_TARGET_Z_MM = 2.5
REBALANCE_TARGET_X_MM = 2.0
REBALANCE_MAX_ITER = 2

# IK branch hysteresis: stick to last branch unless |err_y_norm| exceeds this
IK_BRANCH_SWITCH_ERR_NORM = 0.12

# Disable acceleration limiting in motion_smooth (use velocity limiting only)
MAX_JOINT_ACCEL_DPS2 = (0.0, 0.0, 0.0, 0.0)

# Smoothing: max slew deg/s per joint (wrist, elbow, base, shoulder)
MAX_JOINT_DPS = (120.0, 90.0, 60.0, 75.0)

# Low-pass on final integer command precursors (0 = off)
COMMAND_LPF_ALPHA = 0.35

# Skip serial send if all joint deltas below this (deg)
SEND_EPSILON_DEG = 0.5

# First-find and engage use vision_config; listed here for motion module defaults only
USE_MOTION_V1 = True
