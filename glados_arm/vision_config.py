"""
Picamera2 / face-tracking tuning (Raspberry Pi + Camera Module via libcamera).

Install on Pi: ``sudo apt install -y python3-picamera2 python3-opencv`` (or use pip for opencv).
"""

from __future__ import annotations

# Capture size — keep 4:3 to avoid the zoomed/cropped feel on Pi Cam v2.1.
# Pi Cam v3 is native 16:9; use a lighter 16:9 mode for wider-looking FOV + better FPS.
CAMERA_WIDTH = 960
CAMERA_HEIGHT = 540

# Optional cap on frame rate (Picamera2 controls); None = library default
CAMERA_FPS = 45

# Downscale width for Haar detection only (speed); full-res frame used for preview & overlay.
DETECT_MAX_WIDTH = 256
# Run Haar every N frames and reuse last bbox between detections.
DETECT_EVERY_N_FRAMES = 2

# Picamera2 color mode for raw frames: "bgr" or "rgb".
# On your stack face looked blue with RGB->BGR conversion, so default to BGR.
COLOR_MODE = "bgr"

# Control strategy for live tracking.
# - "ik": update base yaw + vertical target and solve IK every frame
# - "proportional": direct neutral+delta servo commands (legacy mode)
CONTROL_MODE = "ik"

# IK live-target tuning (used when CONTROL_MODE == "ik")
# Image X correction (normalized) -> base yaw delta (rad/frame)
TRACK_BASE_RAD_PER_NORM = 0.10
# Image Y correction (normalized) -> vertical target z delta (mm/frame)
# TEMP X-only tuning mode: disable vertical target updates.
TRACK_Z_MM_PER_NORM = 0.0
# Optional horizontal plane x target adjustment from image error (usually keep 0)
TRACK_X_MM_PER_NORM = 0.0
# Cross-axis coupling: x correction can influence vertical z correction (camera/geometry coupling).
# effective_y_for_z = y + SIGN_ERROR_X_TO_Z * TRACK_Z_FROM_X_MIX * x
TRACK_Z_FROM_X_MIX = 0.0
SIGN_ERROR_X_TO_Z = 1.0
IK_PREFER = "elbow_up"
IK_HOLD_LAST_ON_FAIL = True
IK_ACCEPT_CLAMPED = True

# Keep IK target inside practical workspace envelope.
TARGET_X_MIN_MM = 100.0
TARGET_X_MAX_MM = 230.0
TARGET_Z_MIN_MM = 0.0
TARGET_Z_MAX_MM = 170.0
# Additional controller bounds / smoothing
BASE_YAW_MAX_DEG = 180.0
MAX_BASE_YAW_STEP_RAD = 0.030
MAX_Z_STEP_MM = 0.0
MAX_X_STEP_MM = 3.0
FACE_CENTER_ALPHA = 0.18

# Distance control from face box size (applies in IK mode).
# We estimate relative distance from detected face width in pixels:
# larger face -> person is closer, smaller face -> person is farther.
# The controller nudges target_x_mm (range depth) toward DESIRED_FACE_WIDTH_PX.
DIST_CONTROL_ENABLE = False
DESIRED_FACE_WIDTH_PX = 160.0
DIST_DEADBAND_PX = 2.0
DIST_ERR_CLAMP_PX = 120.0
DIST_MM_PER_PX = 0.45
DIST_MAX_STEP_MM = 6.0
DIST_ALPHA = 0.45
DIST_ENABLE_AFTER_LOCK = True
# Distance sign: +1 means smaller face -> increase x target; -1 flips behavior.
DIST_SIGN_X = -1.0
# Optional distance->z coupling so shoulder participates from range changes.
DIST_SIGN_Z = 1.0
DIST_Z_MM_PER_PX = 0.15
DIST_Z_MAX_STEP_MM = 1.0

# Extra shoulder engagement in IK mode (applied on top of IK shoulder command).
TRACK_SHOULDER_ASSIST_DEG_PER_NORM = 0.0
TRACK_SHOULDER_ASSIST_MAX_DEG = 0
# Distance-driven shoulder assist (independent of Y).
DIST_SHOULDER_ASSIST_ENABLE = False
DIST_SHOULDER_DEG_PER_PX = 0.04
DIST_SHOULDER_MAX_DEG = 10
DIST_SIGN_SHOULDER = 1.0
DIST_SHOULDER_SMOOTH_ALPHA = 0.15
DIST_SHOULDER_MAX_STEP_PER_FRAME_DEG = 1
# Additional shoulder assist from measured z error (mm -> deg).
ZERR_SHOULDER_ASSIST_ENABLE = False
ZERR_SHOULDER_DEG_PER_MM = 0.10
ZERR_SHOULDER_MAX_DEG = 8
ZERR_SIGN_SHOULDER = 1.0
# Elbow assist in IK mode for vertical compensation.
TRACK_ELBOW_ASSIST_DEG_PER_NORM = 0.0
TRACK_ELBOW_ASSIST_MAX_DEG = 16
ELBOW_SMOOTH_ALPHA = 0.08
ELBOW_MAX_STEP_PER_FRAME_DEG = 1
ELBOW_CMD_MAX_STEP_PER_FRAME_DEG = 2

# Engagement smoothing to prevent snap-to-target when a face first appears.
LOCK_IN_FRAMES = 6
ENGAGE_UP_PER_FRAME = 0.20
ENGAGE_DOWN_PER_FRAME = 0.35

# Wrist participation for vertical correction in IK mode.
# Command is: sign * corr_y_ctrl * TRACK_WRIST_DEG_PER_NORM, with min active step/cap below.
TRACK_WRIST_DEG_PER_NORM = 0.0
TRACK_WRIST_MIN_STEP_DEG = 0
TRACK_WRIST_MAX_TRIM_DEG = 95
SIGN_ERROR_Y_WRIST = 1.0
WRIST_SMOOTH_ALPHA = 0.25
WRIST_MAX_STEP_PER_FRAME_DEG = 4

# When no face is detected, gently settle vertical chain toward neutral so it
# does not remain "looking up" at the last lock point.
NO_FACE_VERTICAL_RETURN_ENABLE = True
NO_FACE_Z_RETURN_MM_PER_FRAME = 2.5
NO_FACE_X_RETURN_MM_PER_FRAME = 3.0
NO_FACE_WRIST_RETURN_DEG_PER_FRAME = 4.0
NO_FACE_ELBOW_RETURN_DEG_PER_FRAME = 4.0
NO_FACE_SHOULDER_RETURN_DEG_PER_FRAME = 3.0

# Normalized error deadband (0..1) — ignore jitter inside this band
TRACK_DEADBAND = 0.055

# Adaptive ramping:
# Start with gentle correction, then ramp up if target stays outside center for multiple frames.
# This keeps first response smooth but makes persistent errors react faster.
RAMP_ENABLE = False
RAMP_START_ERROR = 0.10      # normalized error before ramp starts to build
RAMP_UP_PER_FRAME = 0.10     # how quickly gain ramps up while off-center
RAMP_DOWN_PER_FRAME = 0.20   # how quickly gain falls back near center
RAMP_MIN = 1.0               # initial correction multiplier
RAMP_MAX = 1.2               # max boosted multiplier

# Degrees added per frame per unit normalized error (after deadband). Tune on hardware.
# Horizontal: positive error_x = face to the right of frame center → increase base if SIGN_BASE matches.
TRACK_GAIN_BASE_DEG = 2.5

# Vertical chain: positive error_y = face above frame center → "up" (tune signs with INVERT_*)
TRACK_GAIN_SHOULDER_DEG = 1.5
TRACK_GAIN_ELBOW_DEG = 1.2

# Sign flips if your mount/camera orientation reverses left/right or up/down
# Tuned for your current mechanical/camera installation:
SIGN_ERROR_X_BASE = -1.0   # -1: face right -> rotate base to bring target back toward center
SIGN_ERROR_Y_SHOULDER = -1.0
SIGN_ERROR_Y_ELBOW = 1.0

# Haar detector (fast; for better accuracy consider YuNet / DNN later)
HAAR_SCALE_FACTOR = 1.15
HAAR_MIN_NEIGHBORS = 5
HAAR_MIN_SIZE = (40, 40)
