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
# Run Haar every N frames and reuse last bbox between detections (=1 is more stable when the target moves).
DETECT_EVERY_N_FRAMES = 1

# Detection grayscale: global equalizeHist (legacy) vs CLAHE (often better in flat / mid lighting).
VISION_CLAHE_ENABLE = True
VISION_CLAHE_CLIP = 2.0
VISION_CLAHE_TILE = 8

# Full-res bbox low-pass (reduces overlay jitter; Haar still runs every frame when DETECT_EVERY_N_FRAMES==1).
FACE_BBOX_SMOOTH_ALPHA = 0.22
# When Haar returns no faces, keep last smoothed bbox this many frames (~12–18 @ 30fps ≈ 0.4–0.6s).
FACE_HOLD_MAX_FRAMES = 15

# Picamera2 color mode for raw frames: "bgr" or "rgb".
# On your stack face looked blue with RGB->BGR conversion, so default to BGR.
COLOR_MODE = "bgr"

# Control strategy for live tracking.
# - "ik": update base yaw + vertical target and solve IK every frame
# - "proportional": direct neutral+delta servo commands (legacy mode)
CONTROL_MODE = "ik"
# Temporary tuning switch: disable all Y/vertical correction so only X->base is active.
TRACK_DISABLE_Y_AXIS = False
# Temporary tuning switch: disable all X/base correction so only Y/vertical chain is active.
TRACK_DISABLE_X_AXIS = False

# IK live-target tuning (used when CONTROL_MODE == "ik")
# Image X correction (normalized) -> base yaw delta (rad/frame)
TRACK_BASE_RAD_PER_NORM = 0.09
# Base X controller mode: "p" (legacy proportional) or "pid".
BASE_X_CTRL_MODE = "pid"
# PID tuning for base X (output is radians per frame, then clamped by MAX_BASE_YAW_STEP_RAD).
BASE_PID_KP = 0.072
BASE_PID_KI = 0.0
BASE_PID_KD = 0.060
# Integral clamp (in normalized-error frame-sum units) and derivative smoothing.
BASE_PID_I_CLAMP = 2.0
BASE_PID_D_ALPHA = 0.55
BASE_PID_RESET_ON_LOSS = True
# Extra damping when error crosses zero (helps remove lingering overshoot).
BASE_PID_ZERO_CROSS_BRAKE = 0.28
# Two-zone damping near center: keep far response, suppress near-center overshoot.
BASE_PID_NEAR_ERROR = 0.14
BASE_PID_PREBRAKE_ERROR = 0.18
BASE_PID_PREBRAKE_SCALE = 0.72
BASE_PID_NEAR_STEP_SCALE = 0.45
BASE_PID_ZERO_CROSS_HOLD_FRAMES = 0
# Image Y correction (normalized) -> vertical target z delta (mm/frame) when Y_Z_CTRL_MODE == "p".
# Direct P on error each frame (no integral windup) — use this instead of PID when vertical feels "zoomy".
TRACK_Z_MM_PER_NORM = 3.4
# Vertical Y->Z controller mode: "p" (proportional, recommended) or "pid" (can overshoot / oscillate).
Y_Z_CTRL_MODE = "p"
# PID tuning (only if Y_Z_CTRL_MODE == "pid")
Y_PID_KP = 0.58
Y_PID_KI = 0.0
Y_PID_KD = 0.25
Y_PID_I_CLAMP = 2.0
Y_PID_D_ALPHA = 0.58
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
# For this calibration, neutral is already near the vertical lower bound (elbow at max, shoulder at min).
# Allowing controller z targets below this drives persistent clipped_elbow_max + clipped_shoulder_min.
TARGET_Z_MIN_MM = 118.0
TARGET_Z_MAX_MM = 170.0
# Additional controller bounds / smoothing
BASE_YAW_MAX_DEG = 180.0
MAX_BASE_YAW_STEP_RAD = 0.030
# First-lock anti-overshoot: cap base yaw step on initial face acquisition, then ramp up.
BASE_FIRST_LOCK_STEP_FRAMES = 16
BASE_FIRST_LOCK_STEP_SCALE = 0.18
MAX_Z_STEP_MM = 1.05
MAX_X_STEP_MM = 3.0
FACE_CENTER_ALPHA = 0.30
# Use unfiltered face Y for vertical error so the arm reacts immediately (filt_cy lags by seconds).
# Set False to use filt_cy for vertical if the chain jitters from bbox noise.
FACE_Y_USE_RAW = True

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
# Elbow was dominating; raise shoulder so the vertical chain lifts with both joints.
TRACK_SHOULDER_ASSIST_DEG_PER_NORM = 3.2
TRACK_SHOULDER_ASSIST_MAX_DEG = 14
# Distance-driven shoulder assist (independent of Y).
DIST_SHOULDER_ASSIST_ENABLE = False
DIST_SHOULDER_DEG_PER_PX = 0.04
DIST_SHOULDER_MAX_DEG = 10
DIST_SIGN_SHOULDER = 1.0
DIST_SHOULDER_SMOOTH_ALPHA = 0.15
DIST_SHOULDER_MAX_STEP_PER_FRAME_DEG = 1
# Additional shoulder assist from measured z error (mm -> deg).
# When IK mostly bends the elbow, this nudges shoulder so the tip actually reaches target_z.
ZERR_SHOULDER_ASSIST_ENABLE = True
ZERR_SHOULDER_DEG_PER_MM = 0.07
ZERR_SHOULDER_MAX_DEG = 10
ZERR_SIGN_SHOULDER = 1.0
# Lower-bound behavior: if Y asks down while chain is at lower bound, use wrist-only down trim.
LOWER_BOUND_WRIST_ONLY_ENABLE = True
LOWER_BOUND_WRIST_ONLY_MAX_DEG = 20.0
LOWER_BOUND_WRIST_ONLY_GAIN_DEG_PER_NORM = 80.0
# Elbow assist in IK mode for vertical compensation.
TRACK_ELBOW_ASSIST_DEG_PER_NORM = 1.7
TRACK_ELBOW_ASSIST_MAX_DEG = 10
ELBOW_SMOOTH_ALPHA = 0.10
ELBOW_MAX_STEP_PER_FRAME_DEG = 1
ELBOW_CMD_MAX_STEP_PER_FRAME_DEG = 2

# Engagement smoothing to prevent snap-to-target when a face first appears.
LOCK_IN_FRAMES = 2
ENGAGE_UP_PER_FRAME = 0.55
ENGAGE_DOWN_PER_FRAME = 0.28

# First acquisition after no-face: optional slow blend from neutral toward IK (can jerk on re-acquire).
# Off = full IK immediately after re-acquire (faster vertical response).
FIRST_FIND_EXTEND_ENABLE = False
FIRST_FIND_EXTEND_FRACTION = 0.18  # first phase stops at this fraction of (IK - neutral) on shoulder/elbow
FIRST_FIND_TO_QUARTER_PER_FRAME = 0.016
FIRST_FIND_TO_FULL_PER_FRAME = 0.028
# If vertical error is inside deadband (face near center), still nudge IK so shoulder/elbow have a target.
FIRST_FIND_MIN_VERTICAL_NORM = 0.06
# Extra degrees added to IK shoulder/elbow *targets* so first-find always has something to reach toward
# (IK alone is often still ~neutral when engage was masking error). Flip signs if the arm moves wrong way.
FIRST_FIND_BIAS_SHOULDER_DEG = 10.0
FIRST_FIND_BIAS_ELBOW_DEG = -8.0

# Wrist participation for vertical correction in IK mode.
# Command is: sign * corr_y_ctrl * TRACK_WRIST_DEG_PER_NORM, with min active step/cap below.
TRACK_WRIST_DEG_PER_NORM = 18.0
TRACK_WRIST_MIN_STEP_DEG = 1
TRACK_WRIST_MAX_TRIM_DEG = 95
SIGN_ERROR_Y_WRIST = 1.0  # wrist was correct; only shoulder/elbow use inverted Y sign below
WRIST_SMOOTH_ALPHA = 0.42
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

# Normalized error deadband for X (horizontal).
TRACK_DEADBAND = 0.02
# Y (vertical): tighter so the loop actually drives error toward zero instead of sitting in a dead zone.
TRACK_DEADBAND_Y = 0.004

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
