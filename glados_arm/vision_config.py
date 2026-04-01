"""
Picamera2 / face-tracking tuning (Raspberry Pi + Camera Module via libcamera).

Install on Pi: ``sudo apt install -y python3-picamera2 python3-opencv`` (or use pip for opencv).
"""

from __future__ import annotations

# Capture size — keep 4:3 to avoid the zoomed/cropped feel on Pi Cam v2.1.
# Lower than 1280x960 for better FPS.
CAMERA_WIDTH = 960
CAMERA_HEIGHT = 720

# Optional cap on frame rate (Picamera2 controls); None = library default
CAMERA_FPS = 30

# Downscale width for Haar detection only (speed); full-res frame used for preview & overlay.
DETECT_MAX_WIDTH = 320
# Run Haar every N frames and reuse last bbox between detections.
DETECT_EVERY_N_FRAMES = 1

# Picamera2 color mode for raw frames: "bgr" or "rgb".
# On your stack face looked blue with RGB->BGR conversion, so default to BGR.
COLOR_MODE = "bgr"

# Control strategy for live tracking.
# - "ik": update base yaw + vertical target and solve IK every frame
# - "proportional": direct neutral+delta servo commands (legacy mode)
CONTROL_MODE = "ik"

# IK live-target tuning (used when CONTROL_MODE == "ik")
# Image X correction (normalized) -> base yaw delta (rad/frame)
TRACK_BASE_RAD_PER_NORM = 0.20
# Image Y correction (normalized) -> vertical target z delta (mm/frame)
TRACK_Z_MM_PER_NORM = 12.0
# Optional horizontal plane x target adjustment from image error (usually keep 0)
TRACK_X_MM_PER_NORM = 0.0
# Cross-axis coupling: x correction can influence vertical z correction (camera/geometry coupling).
# effective_y_for_z = y + SIGN_ERROR_X_TO_Z * TRACK_Z_FROM_X_MIX * x
TRACK_Z_FROM_X_MIX = 0.25
SIGN_ERROR_X_TO_Z = 1.0
IK_PREFER = "elbow_up"
IK_HOLD_LAST_ON_FAIL = True
IK_ACCEPT_CLAMPED = True

# Keep IK target inside practical workspace envelope.
TARGET_X_MIN_MM = 100.0
TARGET_X_MAX_MM = 230.0
TARGET_Z_MIN_MM = 0.0
TARGET_Z_MAX_MM = 190.0
# Additional controller bounds / smoothing
BASE_YAW_MAX_DEG = 90.0
MAX_BASE_YAW_STEP_RAD = 0.08
MAX_Z_STEP_MM = 10.0
MAX_X_STEP_MM = 3.0
FACE_CENTER_ALPHA = 0.35

# Minimal wrist participation for vertical correction in IK mode (degrees/frame per normalized error).
# Keep small so shoulder+elbow do most of the work.
TRACK_WRIST_DEG_PER_NORM = 0.8
SIGN_ERROR_Y_WRIST = -1.0

# Normalized error deadband (0..1) — ignore jitter inside this band
TRACK_DEADBAND = 0.03

# Adaptive ramping:
# Start with gentle correction, then ramp up if target stays outside center for multiple frames.
# This keeps first response smooth but makes persistent errors react faster.
RAMP_ENABLE = True
RAMP_START_ERROR = 0.10      # normalized error before ramp starts to build
RAMP_UP_PER_FRAME = 0.10     # how quickly gain ramps up while off-center
RAMP_DOWN_PER_FRAME = 0.20   # how quickly gain falls back near center
RAMP_MIN = 1.0               # initial correction multiplier
RAMP_MAX = 2.2               # max boosted multiplier

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
SIGN_ERROR_Y_ELBOW = 1.0  # keep opposite to shoulder so chain bends in same physical vertical direction

# Haar detector (fast; for better accuracy consider YuNet / DNN later)
HAAR_SCALE_FACTOR = 1.15
HAAR_MIN_NEIGHBORS = 5
HAAR_MIN_SIZE = (40, 40)
