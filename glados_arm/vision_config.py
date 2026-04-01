"""
Picamera2 / face-tracking tuning (Raspberry Pi + Camera Module via libcamera).

Install on Pi: ``sudo apt install -y python3-picamera2 python3-opencv`` (or use pip for opencv).
"""

from __future__ import annotations

# Capture size — **larger = wider field of view** on Pi Camera (small sizes often crop / “zoom”).
# Tune down if FPS is too low on older Pi models.
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

# Optional cap on frame rate (Picamera2 controls); None = library default
CAMERA_FPS = 30

# Downscale width for Haar detection only (speed); full-res frame used for preview & overlay.
DETECT_MAX_WIDTH = 640

# Normalized error deadband (0..1) — ignore jitter inside this band
TRACK_DEADBAND = 0.06

# Degrees added per frame per unit normalized error (after deadband). Tune on hardware.
# Horizontal: positive error_x = face to the right of frame center → increase base if SIGN_BASE matches.
TRACK_GAIN_BASE_DEG = 2.5

# Vertical chain: positive error_y = face above frame center → "up" (tune signs with INVERT_*)
TRACK_GAIN_SHOULDER_DEG = 1.5
TRACK_GAIN_ELBOW_DEG = 1.2

# Sign flips if your mount/camera orientation reverses left/right or up/down
SIGN_ERROR_X_BASE = 1.0   # +1: face right → increase base command
SIGN_ERROR_Y_SHOULDER = 1.0
SIGN_ERROR_Y_ELBOW = -1.0  # elbow often inverted vs shoulder for same image-up cue

# Haar detector (fast; for better accuracy consider YuNet / DNN later)
HAAR_SCALE_FACTOR = 1.15
HAAR_MIN_NEIGHBORS = 5
HAAR_MIN_SIZE = (40, 40)
