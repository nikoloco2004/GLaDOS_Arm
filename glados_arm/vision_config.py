"""
Picamera2 / face-tracking tuning (Raspberry Pi + Camera Module via libcamera).

Install on Pi: ``sudo apt install -y python3-picamera2 python3-opencv`` (or use pip for opencv).
"""

from __future__ import annotations

# Capture size — keep 4:3 to avoid the zoomed/cropped feel on Pi Cam v2.1.
# Wide-compat mode for Camera Module 3: small 16:9 stream to encourage full-FOV scaling.
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 360

# Optional cap on frame rate (Picamera2 controls); None = library default
CAMERA_FPS = 30

# Try to force a full-sensor-style binned mode before scaling to main size.
# If unsupported on your stack, code falls back automatically.
SENSOR_OUTPUT_SIZE = (4608, 2592)
# Additional sensor mode fallback chain (full-FOV-first) for stacks that reject one mode.
SENSOR_OUTPUT_SIZE_FALLBACKS = (
    (4608, 2592),
    (2304, 1296),
)

# Keep full-sensor crop enforced to avoid creeping zoom in some libcamera pipelines.
FORCE_MAX_SCALERCROP = True
SCALERCROP_REAPPLY_SECONDS = 999999.0
SCALERCROP_REAPPLY_EVERY_N_FRAMES = 1

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
TRACK_BASE_RAD_PER_NORM = 0.09
# Base X controller mode: "p" (legacy proportional) or "pid".
BASE_X_CTRL_MODE = "pid"
# PID tuning for base X (output is radians per frame, then clamped by MAX_BASE_YAW_STEP_RAD).
BASE_PID_KP = 0.130
BASE_PID_KI = 0.0002
BASE_PID_KD = 0.030
# Integral clamp (in normalized-error frame-sum units) and derivative smoothing.
BASE_PID_I_CLAMP = 2.0
BASE_PID_D_ALPHA = 0.45
BASE_PID_RESET_ON_LOSS = True
# Extra damping when error crosses zero (helps remove lingering overshoot).
BASE_PID_ZERO_CROSS_BRAKE = 0.45
# Two-zone damping near center: keep far response, suppress near-center overshoot.
BASE_PID_NEAR_ERROR = 0.10
BASE_PID_NEAR_STEP_SCALE = 0.55
BASE_PID_ZERO_CROSS_HOLD_FRAMES = 1
# Image Y correction (normalized) -> vertical target z delta (mm/frame)
TRACK_Z_MM_PER_NORM = 2.2
# Vertical Y->Z controller mode: "p" (legacy proportional) or "pid".
Y_Z_CTRL_MODE = "pid"
# When True: image Y integrates target_z_mm (IK plane) — classic vertical IK.
# When False: no Z from image Y; use VERTICAL_Y_PID (degrees) split across wrist+shoulder+elbow in tandem.
VERTICAL_IK_Z_FROM_IMAGE_ENABLE = False
# Tandem vertical PID (degrees total before ratio split). Tune for smooth/accurate Y tracking.
VERTICAL_Y_PID_KP = 10.0
VERTICAL_Y_PID_KI = 0.08
VERTICAL_Y_PID_KD = 1.8
VERTICAL_Y_PID_I_CLAMP = 6.0
VERTICAL_Y_PID_D_ALPHA = 0.4
VERTICAL_Y_PID_MAX_DEG = 16.0
VERTICAL_Y_PID_RESET_ON_LOSS = True
# How the PID output (deg) is shared across joints (same sign as wrist was using).
VERTICAL_Y_WRIST_RATIO = 0.34
VERTICAL_Y_SHOULDER_RATIO = 0.33
VERTICAL_Y_ELBOW_RATIO = 0.33
# Tandem elbow: by default scale by FIRST_FIND_BIAS_ELBOW/FIRST_FIND_BIAS_SHOULDER so elbow servo
# moves with the same relative sense as first-find (shoulder + / elbow - on this arm). Override with
# VERTICAL_Y_ELBOW_FOLLOW_FIRST_FIND_BIAS = False and tune VERTICAL_Y_ELBOW_SIGN (+/-1) if needed.
VERTICAL_Y_ELBOW_FOLLOW_FIRST_FIND_BIAS = True
VERTICAL_Y_ELBOW_SIGN = 1.0
# PID tuning for vertical correction (output in mm/frame, then clamped by MAX_Z_STEP_MM).
# Used only when VERTICAL_IK_Z_FROM_IMAGE_ENABLE is True.
Y_PID_KP = 0.82
Y_PID_KI = 0.006
Y_PID_KD = 0.65
Y_PID_I_CLAMP = 2.5
Y_PID_D_ALPHA = 0.50
Y_PID_RESET_ON_LOSS = True
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
MAX_BASE_YAW_STEP_RAD = 0.052
MAX_Z_STEP_MM = 0.85
MAX_X_STEP_MM = 3.0
FACE_CENTER_ALPHA = 0.25

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
TRACK_SHOULDER_ASSIST_DEG_PER_NORM = 1.0
TRACK_SHOULDER_ASSIST_MAX_DEG = 5
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
TRACK_ELBOW_ASSIST_DEG_PER_NORM = 2.0
TRACK_ELBOW_ASSIST_MAX_DEG = 10
ELBOW_SMOOTH_ALPHA = 0.06
ELBOW_MAX_STEP_PER_FRAME_DEG = 1
ELBOW_CMD_MAX_STEP_PER_FRAME_DEG = 2

# Engagement smoothing to prevent snap-to-target when a face first appears.
LOCK_IN_FRAMES = 6
ENGAGE_UP_PER_FRAME = 0.20
ENGAGE_DOWN_PER_FRAME = 0.35

# First acquisition after no-face: move shoulder/elbow only partway toward IK, slowly, then ramp to full.
# Resets each time the face is lost and found again.
FIRST_FIND_EXTEND_ENABLE = True
FIRST_FIND_EXTEND_FRACTION = 0.25  # first phase stops at 25% of (IK - neutral) on shoulder/elbow
FIRST_FIND_TO_QUARTER_PER_FRAME = 0.022
FIRST_FIND_TO_FULL_PER_FRAME = 0.04
# If vertical error is inside deadband (face near center), still nudge IK so shoulder/elbow have a target.
FIRST_FIND_MIN_VERTICAL_NORM = 0.12
# Extra degrees added to IK shoulder/elbow *targets* so first-find always has something to reach toward
# (IK alone is often still ~neutral when engage was masking error). Flip signs if the arm moves wrong way.
FIRST_FIND_BIAS_SHOULDER_DEG = 10.0
FIRST_FIND_BIAS_ELBOW_DEG = -8.0

# Wrist participation for vertical correction in IK mode.
# Command is: sign * corr_y_ctrl * TRACK_WRIST_DEG_PER_NORM, with min active step/cap below.
TRACK_WRIST_DEG_PER_NORM = 22.0
TRACK_WRIST_MIN_STEP_DEG = 1
TRACK_WRIST_MAX_TRIM_DEG = 95
SIGN_ERROR_Y_WRIST = 1.0  # wrist was correct; only shoulder/elbow use inverted Y sign below
WRIST_SMOOTH_ALPHA = 0.15
WRIST_MAX_STEP_PER_FRAME_DEG = 2

# When no face is detected, gently settle vertical chain toward neutral so it
# does not remain "looking up" at the last lock point.
NO_FACE_VERTICAL_RETURN_ENABLE = True
NO_FACE_RETURN_DELAY_S = 30.0
NO_FACE_Z_RETURN_MM_PER_FRAME = 2.5
NO_FACE_X_RETURN_MM_PER_FRAME = 3.0
NO_FACE_WRIST_RETURN_DEG_PER_FRAME = 4.0
NO_FACE_ELBOW_RETURN_DEG_PER_FRAME = 4.0
NO_FACE_SHOULDER_RETURN_DEG_PER_FRAME = 3.0

# Normalized error deadband (0..1) — ignore jitter inside this band
TRACK_DEADBAND = 0.03

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
SIGN_ERROR_Y_SHOULDER = 1.0
SIGN_ERROR_Y_ELBOW = 1.0

# Haar detector (fast; for better accuracy consider YuNet / DNN later)
HAAR_SCALE_FACTOR = 1.15
HAAR_MIN_NEIGHBORS = 5
HAAR_MIN_SIZE = (40, 40)
