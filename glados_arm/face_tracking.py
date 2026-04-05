"""
Face tracking with **Picamera2** (libcamera) + OpenCV Haar cascade.

Requires Raspberry Pi OS with working ``rpicam-hello`` / Picamera2 stack.

Architecture (matches your arm model):
  * Image X error → **base** (horizontal)
  * Image Y error → **shoulder + elbow** (vertical chain); wrist held at neutral

Supports:
  * IK target loop (default): image X -> base yaw, image Y -> vertical target (x,z) -> IK
  * Proportional legacy mode: direct neutral+delta servo commands
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import cv2
import numpy as np

from . import config, kinematics, vision_config
from .controller import RobotController, solve_vertical_plane
from .mapping import ServoCommand, clamp_servo, servo_to_model
from .serial_comm import ArmSerial

_HAAR_XML = "haarcascade_frontalface_default.xml"


def _haar_cascade_path() -> str:
    """
    Resolve Haar XML. Many pip wheels expose cv2.data.haarcascades; apt OpenCV on Pi often does not.
    """
    try:
        root = cv2.data.haarcascades  # type: ignore[attr-defined]
        p = os.path.join(root, _HAAR_XML)
        if os.path.isfile(p):
            return p
    except AttributeError:
        pass

    for root in (
        "/usr/share/opencv4/haarcascades",
        "/usr/local/share/opencv4/haarcascades",
        "/usr/share/opencv/haarcascades",
        "/usr/local/share/opencv/haarcascades",
    ):
        p = os.path.join(root, _HAAR_XML)
        if os.path.isfile(p):
            return p

    raise FileNotFoundError(
        f"Could not find {_HAAR_XML}. On Raspberry Pi OS try:\n"
        "  sudo apt install -y opencv-data\n"
        "  # or: sudo apt install -y libopencv-data\n"
        "Then confirm: ls /usr/share/opencv4/haarcascades/"
    )


def _neutral_command() -> ServoCommand:
    return ServoCommand(
        wrist=config.NEUTRAL_WRIST,
        elbow=config.NEUTRAL_ELBOW,
        base=config.NEUTRAL_BASE,
        shoulder=config.NEUTRAL_SHOULDER,
    )


def _apply_deadband(v: float, db: float) -> float:
    if abs(v) < db:
        return 0.0
    return v


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _step_toward(cur: float, target: float, max_step: float) -> float:
    d = target - cur
    if abs(d) <= max_step:
        return target
    return cur + max_step if d > 0 else cur - max_step


def _base_yaw_limit_rad() -> float:
    """
    Compute a safe yaw limit that cannot request base servo values outside hardware limits.
    Final limit = min(configured vision cap, physically reachable from neutral by mapping).
    """
    cfg_lim_rad = math.radians(abs(float(getattr(vision_config, "BASE_YAW_MAX_DEG", 90.0))))
    scale = abs(float(getattr(config, "BASE_RAD_TO_SERVO_DEG", 180.0 / math.pi)))
    if scale <= 1e-9:
        return cfg_lim_rad

    pos_rad = (float(config.SERVO_BASE_MAX) - float(config.NEUTRAL_BASE)) / scale
    neg_rad = (float(config.NEUTRAL_BASE) - float(config.SERVO_BASE_MIN)) / scale
    phys_lim_rad = max(0.0, min(pos_rad, neg_rad))
    return min(cfg_lim_rad, phys_lim_rad)


def resolve_preview_mode(explicit_preview: bool, explicit_no_preview: bool) -> bool:
    """
    Whether to call cv2.imshow. Default: on when DISPLAY is set (local Pi desktop).
    Use --preview to force on, --no-preview to force off (e.g. SSH without forwarding).
    """
    if explicit_preview and explicit_no_preview:
        raise ValueError("Use only one of --preview and --no-preview")
    if explicit_preview:
        return True
    if explicit_no_preview:
        return False
    disp = os.environ.get("DISPLAY", "").strip()
    return bool(disp)


def run_tracking(
    *,
    port: str,
    use_serial: bool,
    preview: bool,
    width: int,
    height: int,
    color_mode: str | None = None,
    control_mode: str | None = None,
) -> int:
    try:
        from picamera2 import Picamera2  # type: ignore[import-untyped]
    except ImportError:
        print(
            "Picamera2 not found. On Raspberry Pi OS install with:\n"
            "  sudo apt install -y python3-picamera2\n"
            "Verify camera first:  rpicam-hello -t 0",
            file=sys.stderr,
        )
        return 1

    vc = vision_config
    try:
        haar_path = _haar_cascade_path()
    except FileNotFoundError as e:
        print(str(e), file=sys.stderr)
        return 1

    face_cascade = cv2.CascadeClassifier(haar_path)
    if face_cascade.empty():
        print(f"Failed to load Haar cascade from: {haar_path}", file=sys.stderr)
        return 1

    picam2 = Picamera2()
    # Use RGB888 for consistent detector input; color_mode controls preview conversion path.
    cfg_kwargs: dict[str, object] = {
        "main": {"size": (width, height), "format": "RGB888"},
        "controls": {"FrameRate": float(getattr(vc, "CAMERA_FPS", 30))},
    }
    sensor_output = getattr(vc, "SENSOR_OUTPUT_SIZE", None)
    if isinstance(sensor_output, (tuple, list)) and len(sensor_output) == 2:
        try:
            so = (int(sensor_output[0]), int(sensor_output[1]))
            cfg_kwargs["sensor"] = {"output_size": so}
        except Exception:
            pass
    try:
        cfg = picam2.create_video_configuration(**cfg_kwargs)
    except Exception as e:
        # Fallback for stacks that reject explicit sensor mode hints.
        print(f"Video cfg with sensor hint failed ({e}); falling back.", flush=True)
        cfg = picam2.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"},
            controls={"FrameRate": float(getattr(vc, "CAMERA_FPS", 30))},
        )
    picam2.configure(cfg)
    picam2.start()

    # Request full sensor crop when available (reduces "zoomed-in" look).
    max_crop_tuple: tuple[int, int, int, int] | None = None
    try:
        max_crop = picam2.camera_properties.get("ScalerCropMaximum")
        if max_crop is not None and bool(getattr(vc, "FORCE_MAX_SCALERCROP", True)):
            max_crop_tuple = tuple(int(v) for v in max_crop)
            # Apply a few times; libcamera controls can take a couple frames to settle.
            for _ in range(6):
                picam2.set_controls({"ScalerCrop": max_crop_tuple})
                time.sleep(0.03)
            md = picam2.capture_metadata()
            cur_crop = md.get("ScalerCrop") if isinstance(md, dict) else None
            print(f"ScalerCrop max={max_crop_tuple} current={cur_crop}", flush=True)
        else:
            print("ScalerCropMaximum unavailable on this stack.", flush=True)
    except Exception as e:
        print(f"ScalerCrop full-FOV request failed: {e}", flush=True)

    crop_reapply_until = time.time() + float(getattr(vc, "SCALERCROP_REAPPLY_SECONDS", 8.0))
    crop_reapply_every_n = max(1, int(getattr(vc, "SCALERCROP_REAPPLY_EVERY_N_FRAMES", 12)))
    time.sleep(0.2)

    arm = ArmSerial(port=port) if use_serial else None
    controller = RobotController(serial=arm)
    if use_serial:
        controller.connect()
        if not controller.ping():
            print(f"No PONG from Arduino on {port} — check USB and port.", file=sys.stderr)
            picam2.stop()
            return 1
        controller.neutral()

    cmd = _neutral_command()
    last_valid_cmd = cmd

    # IK state (default control path)
    fk0 = kinematics.forward_kinematics(0.0, 0.0)
    target_x_mm = fk0.tip.x
    target_z_mm = fk0.tip.z
    base_yaw_rad = 0.0
    ik_status = "init"
    ik_clip_notes: list[str] = []
    base_yaw_lim = _base_yaw_limit_rad()

    ctl = (control_mode or getattr(vc, "CONTROL_MODE", "ik")).strip().lower()
    if ctl not in ("ik", "proportional"):
        print(f"Invalid control mode '{ctl}', using 'ik'.", flush=True)
        ctl = "ik"
    mode = (color_mode or getattr(vc, "COLOR_MODE", "bgr")).strip().lower()
    if mode not in ("bgr", "rgb"):
        print(f"Invalid color mode '{mode}', using 'bgr'.", flush=True)
        mode = "bgr"
    detect_every = max(1, int(getattr(vc, "DETECT_EVERY_N_FRAMES", 1)))
    print(f"Color mode: {mode} | detect_every_n_frames={detect_every} | control_mode={ctl}", flush=True)
    print(
        "Tracking: Ctrl+C to stop. Picamera2 + OpenCV; horizontal→base, vertical→shoulder/elbow.",
        flush=True,
    )
    if preview:
        print(
            "Preview: ON (OpenCV window). Press 'q' in the window to quit.",
            flush=True,
        )
    else:
        print(
            "Preview: OFF. For a window: run on the Pi desktop, or "
            "`export DISPLAY=:0`, or pass --preview (needs X11 / local desktop).",
            flush=True,
        )

    if preview:
        try:
            probe = np.zeros((8, 8, 3), dtype=np.uint8)
            cv2.imshow("_glados_opencv_gui_probe", probe)
            cv2.waitKey(1)
            cv2.destroyWindow("_glados_opencv_gui_probe")
        except cv2.error as e:
            print(
                f"Preview disabled: {e}\n"
                "Install GUI OpenCV on the Pi (e.g. sudo apt install python3-opencv), "
                "not opencv-python-headless from pip — headless builds cannot imshow.",
                file=sys.stderr,
                flush=True,
            )
            preview = False

    det_max_w = getattr(vc, "DETECT_MAX_WIDTH", 640)
    frame_idx = 0
    last_faces: list[tuple[int, int, int, int]] = []
    fps_ema = 0.0
    last_frame_t = time.time()
    filt_cx: float | None = None
    filt_cy: float | None = None
    filt_face_w: float | None = None
    x_ramp = float(getattr(vc, "RAMP_MIN", 1.0))
    y_ramp = float(getattr(vc, "RAMP_MIN", 1.0))
    wrist_trim_state = 0.0
    wrist_trim_last = 0
    elbow_assist_state = 0.0
    elbow_assist_last = 0
    elbow_cmd_last = int(config.NEUTRAL_ELBOW)
    shoulder_dist_state = 0.0
    shoulder_dist_last = 0
    face_lock_frames = 0
    engage = 0.0
    base_pid_i = 0.0
    base_pid_prev_e = 0.0
    base_pid_d = 0.0

    try:
        while True:
            now_t = time.time()
            dt = max(1e-6, now_t - last_frame_t)
            last_frame_t = now_t
            inst_fps = 1.0 / dt
            if fps_ema <= 1e-6:
                fps_ema = inst_fps
            else:
                fps_ema = 0.9 * fps_ema + 0.1 * inst_fps

            raw = picam2.capture_array("main")
            if raw is None or raw.size == 0:
                continue
            if (
                max_crop_tuple is not None
                and time.time() < crop_reapply_until
                and (frame_idx % crop_reapply_every_n == 0)
            ):
                try:
                    picam2.set_controls({"ScalerCrop": max_crop_tuple})
                except Exception:
                    pass
            # Convert to BGR for OpenCV only when raw is RGB.
            if mode == "rgb":
                frame_bgr = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
            else:
                frame_bgr = raw

            h, w = frame_bgr.shape[:2]
            # Optional downscale for Haar (faster); map boxes back to full resolution for control + preview.
            if w > det_max_w:
                sf = det_max_w / float(w)  # shrink factor full → small
                small_w = det_max_w
                small_h = max(1, int(round(h * sf)))
                gray_small = cv2.cvtColor(
                    cv2.resize(frame_bgr, (small_w, small_h), interpolation=cv2.INTER_AREA),
                    cv2.COLOR_BGR2GRAY,
                )
                inv_scale = 1.0 / sf
                min_face = (
                    max(1, int(round(vc.HAAR_MIN_SIZE[0] * sf))),
                    max(1, int(round(vc.HAAR_MIN_SIZE[1] * sf))),
                )
            else:
                gray_small = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
                inv_scale = 1.0
                min_face = vc.HAAR_MIN_SIZE

            gray_small = cv2.equalizeHist(gray_small)
            run_detect = (frame_idx % detect_every) == 0 or not last_faces
            if run_detect:
                detected = face_cascade.detectMultiScale(
                    gray_small,
                    scaleFactor=vc.HAAR_SCALE_FACTOR,
                    minNeighbors=vc.HAAR_MIN_NEIGHBORS,
                    minSize=min_face,
                )
                last_faces = [tuple(map(int, d)) for d in detected]
            faces = last_faces
            cx_img = w * 0.5
            cy_img = h * 0.5
            corr_x_norm = 0.0
            corr_y_norm = 0.0
            corr_x_px = 0.0
            corr_y_px = 0.0
            dist_err_px = 0.0
            dist_step_mm = 0.0
            dist_step_z_mm = 0.0
            z_err_mm = 0.0

            if len(faces) > 0:
                face_lock_frames += 1
                areas = [fw * fh for (_x, _y, fw, fh) in faces]
                i = int(np.argmax(areas))
                x, y, fw, fh = faces[i]
                x = int(round(x * inv_scale))
                y = int(round(y * inv_scale))
                fw = int(round(fw * inv_scale))
                fh = int(round(fh * inv_scale))
                cx = x + fw * 0.5
                cy = y + fh * 0.5

                alpha = float(getattr(vc, "FACE_CENTER_ALPHA", 0.35))
                if filt_cx is None or filt_cy is None:
                    filt_cx, filt_cy = cx, cy
                else:
                    filt_cx = (1.0 - alpha) * filt_cx + alpha * cx
                    filt_cy = (1.0 - alpha) * filt_cy + alpha * cy

                dist_alpha = float(getattr(vc, "DIST_ALPHA", 0.25))
                if filt_face_w is None:
                    filt_face_w = float(fw)
                else:
                    filt_face_w = (1.0 - dist_alpha) * filt_face_w + dist_alpha * float(fw)

                err_x = (filt_cx - cx_img) / max(cx_img, 1.0)  # face right => +
                err_y = (cy_img - filt_cy) / max(cy_img, 1.0)  # face above => +

                corr_x_norm = _apply_deadband(err_x, vc.TRACK_DEADBAND)
                corr_y_norm = _apply_deadband(err_y, vc.TRACK_DEADBAND)
                corr_x_px = corr_x_norm * cx_img
                corr_y_px = corr_y_norm * cy_img
                corr_x_ctrl = corr_x_norm
                corr_y_ctrl = corr_y_norm

                lock_need = max(1, int(getattr(vc, "LOCK_IN_FRAMES", 6)))
                engage_target = min(1.0, face_lock_frames / float(lock_need))
                engage = _step_toward(
                    engage,
                    engage_target,
                    max(1e-3, float(getattr(vc, "ENGAGE_UP_PER_FRAME", 0.20))),
                )

                if bool(getattr(vc, "RAMP_ENABLE", True)):
                    ramp_start = float(getattr(vc, "RAMP_START_ERROR", 0.10))
                    ramp_up = float(getattr(vc, "RAMP_UP_PER_FRAME", 0.10))
                    ramp_down = float(getattr(vc, "RAMP_DOWN_PER_FRAME", 0.20))
                    ramp_min = float(getattr(vc, "RAMP_MIN", 1.0))
                    ramp_max = float(getattr(vc, "RAMP_MAX", 2.2))

                    ax = abs(corr_x_norm)
                    ay = abs(corr_y_norm)
                    if ax > ramp_start:
                        x_ramp = min(ramp_max, x_ramp + ramp_up * (ax - ramp_start) / max(1e-6, (1.0 - ramp_start)))
                    else:
                        x_ramp = max(ramp_min, x_ramp - ramp_down)
                    if ay > ramp_start:
                        y_ramp = min(ramp_max, y_ramp + ramp_up * (ay - ramp_start) / max(1e-6, (1.0 - ramp_start)))
                    else:
                        y_ramp = max(ramp_min, y_ramp - ramp_down)

                    corr_x_ctrl = corr_x_norm * x_ramp
                    corr_y_ctrl = corr_y_norm * y_ramp
                # Smooth first-lock behavior so the arm does not snap/overshoot on reacquire.
                corr_x_ctrl *= engage
                corr_y_ctrl *= engage
                wrist_cmd = (
                    float(getattr(vc, "SIGN_ERROR_Y_WRIST", 1.0))
                    * corr_y_ctrl
                    * float(getattr(vc, "TRACK_WRIST_DEG_PER_NORM", 0.8))
                )
                wrist_trim_deg = int(round(wrist_cmd))
                wrist_min_step = max(0, int(getattr(vc, "TRACK_WRIST_MIN_STEP_DEG", 0)))
                if wrist_trim_deg == 0 and abs(corr_y_ctrl) > 1e-6 and wrist_min_step > 0:
                    wrist_trim_deg = wrist_min_step if wrist_cmd > 0.0 else -wrist_min_step
                wrist_max_trim = max(0, int(getattr(vc, "TRACK_WRIST_MAX_TRIM_DEG", 35)))
                wrist_trim_deg = max(-wrist_max_trim, min(wrist_max_trim, wrist_trim_deg))
                wrist_alpha = _clamp(float(getattr(vc, "WRIST_SMOOTH_ALPHA", 0.25)), 0.0, 1.0)
                wrist_trim_state = (1.0 - wrist_alpha) * wrist_trim_state + wrist_alpha * float(wrist_trim_deg)
                wrist_trim_deg = int(round(wrist_trim_state))
                wrist_step_max = max(1, int(getattr(vc, "WRIST_MAX_STEP_PER_FRAME_DEG", 4)))
                wrist_trim_deg = int(round(_step_toward(float(wrist_trim_last), float(wrist_trim_deg), float(wrist_step_max))))
                wrist_trim_last = wrist_trim_deg
                shoulder_assist_deg = int(
                    round(
                        float(getattr(vc, "SIGN_ERROR_Y_SHOULDER", 1.0))
                        * corr_y_ctrl
                        * float(getattr(vc, "TRACK_SHOULDER_ASSIST_DEG_PER_NORM", 0.0))
                    )
                )
                shoulder_assist_max = max(0, int(getattr(vc, "TRACK_SHOULDER_ASSIST_MAX_DEG", 0)))
                shoulder_assist_deg = max(-shoulder_assist_max, min(shoulder_assist_max, shoulder_assist_deg))
                elbow_assist_deg = int(
                    round(
                        float(getattr(vc, "SIGN_ERROR_Y_ELBOW", 1.0))
                        * corr_y_ctrl
                        * float(getattr(vc, "TRACK_ELBOW_ASSIST_DEG_PER_NORM", 0.0))
                    )
                )
                elbow_assist_max = max(0, int(getattr(vc, "TRACK_ELBOW_ASSIST_MAX_DEG", 0)))
                elbow_assist_deg = max(-elbow_assist_max, min(elbow_assist_max, elbow_assist_deg))
                elbow_alpha = _clamp(float(getattr(vc, "ELBOW_SMOOTH_ALPHA", 0.25)), 0.0, 1.0)
                elbow_assist_state = (1.0 - elbow_alpha) * elbow_assist_state + elbow_alpha * float(elbow_assist_deg)
                elbow_assist_deg = int(round(elbow_assist_state))
                elbow_step_max = max(1, int(getattr(vc, "ELBOW_MAX_STEP_PER_FRAME_DEG", 3)))
                elbow_assist_deg = int(
                    round(_step_toward(float(elbow_assist_last), float(elbow_assist_deg), float(elbow_step_max)))
                )
                elbow_assist_last = elbow_assist_deg

                if ctl == "ik":
                    shoulder_dist_assist_deg = 0
                    max_base_step = float(getattr(vc, "MAX_BASE_YAW_STEP_RAD", 0.08))
                    x_ctrl_mode = str(getattr(vc, "BASE_X_CTRL_MODE", "p")).strip().lower()
                    if x_ctrl_mode == "pid":
                        # PID on horizontal image error -> base yaw step (rad/frame).
                        e = float(vc.SIGN_ERROR_X_BASE) * corr_x_ctrl
                        kp = float(getattr(vc, "BASE_PID_KP", 0.07))
                        ki = float(getattr(vc, "BASE_PID_KI", 0.0))
                        kd = float(getattr(vc, "BASE_PID_KD", 0.02))
                        i_clamp = max(0.0, float(getattr(vc, "BASE_PID_I_CLAMP", 2.0)))
                        d_alpha = _clamp(float(getattr(vc, "BASE_PID_D_ALPHA", 0.35)), 0.0, 1.0)

                        base_pid_i += e
                        base_pid_i = _clamp(base_pid_i, -i_clamp, i_clamp)
                        d_raw = e - base_pid_prev_e
                        base_pid_d = (1.0 - d_alpha) * base_pid_d + d_alpha * d_raw
                        base_unclamped = kp * e + ki * base_pid_i + kd * base_pid_d
                        base_step = _clamp(base_unclamped, -max_base_step, max_base_step)
                        # Basic anti-windup: undo this frame's integral if output is saturating further.
                        if abs(base_unclamped - base_step) > 1e-9 and abs(e) > 1e-9:
                            if (base_unclamped > 0.0 and e > 0.0) or (base_unclamped < 0.0 and e < 0.0):
                                base_pid_i -= e
                        base_pid_prev_e = e
                    else:
                        base_step = (
                            vc.SIGN_ERROR_X_BASE * corr_x_ctrl * float(getattr(vc, "TRACK_BASE_RAD_PER_NORM", 0.04))
                        )
                    base_step = _clamp(
                        base_step,
                        -max_base_step,
                        max_base_step,
                    )
                    base_yaw_rad += base_step
                    base_yaw_rad = _clamp(base_yaw_rad, -base_yaw_lim, base_yaw_lim)

                    y_for_z = corr_y_ctrl + float(getattr(vc, "SIGN_ERROR_X_TO_Z", 1.0)) * float(
                        getattr(vc, "TRACK_Z_FROM_X_MIX", 0.0)
                    ) * corr_x_ctrl
                    y_for_z = _clamp(y_for_z, -1.0, 1.0)
                    z_step = (
                        vc.SIGN_ERROR_Y_SHOULDER * y_for_z * float(getattr(vc, "TRACK_Z_MM_PER_NORM", 10.0))
                    )
                    z_step = _clamp(
                        z_step,
                        -float(getattr(vc, "MAX_Z_STEP_MM", 10.0)),
                        float(getattr(vc, "MAX_Z_STEP_MM", 10.0)),
                    )
                    target_z_mm += z_step

                    x_step = (
                        vc.SIGN_ERROR_X_BASE * corr_x_ctrl * float(getattr(vc, "TRACK_X_MM_PER_NORM", 0.0))
                    )
                    x_step = _clamp(
                        x_step,
                        -float(getattr(vc, "MAX_X_STEP_MM", 3.0)),
                        float(getattr(vc, "MAX_X_STEP_MM", 3.0)),
                    )
                    target_x_mm += x_step

                    if bool(getattr(vc, "DIST_CONTROL_ENABLE", True)):
                        desired_face_w = float(getattr(vc, "DESIRED_FACE_WIDTH_PX", 160.0))
                        dist_db = max(0.0, float(getattr(vc, "DIST_DEADBAND_PX", 10.0)))
                        dist_err_limit = max(1.0, float(getattr(vc, "DIST_ERR_CLAMP_PX", 120.0)))
                        dist_mm_per_px = float(getattr(vc, "DIST_MM_PER_PX", 0.35))
                        dist_max_step = max(0.1, float(getattr(vc, "DIST_MAX_STEP_MM", 8.0)))
                        dist_z_mm_per_px = float(getattr(vc, "DIST_Z_MM_PER_PX", 0.0))
                        dist_z_max_step = max(0.0, float(getattr(vc, "DIST_Z_MAX_STEP_MM", 0.0)))

                        dist_allowed = (not bool(getattr(vc, "DIST_ENABLE_AFTER_LOCK", True))) or (engage >= 0.95)
                        if dist_allowed:
                            measured_face_w = float(filt_face_w if filt_face_w is not None else fw)
                            dist_err_px = desired_face_w - measured_face_w
                            if abs(dist_err_px) < dist_db:
                                dist_err_px = 0.0
                            dist_err_px = _clamp(dist_err_px, -dist_err_limit, dist_err_limit)
                            dist_step_mm = _clamp(
                                float(getattr(vc, "DIST_SIGN_X", 1.0)) * dist_err_px * dist_mm_per_px,
                                -dist_max_step,
                                dist_max_step,
                            )
                            # Smaller face (farther) => positive dist_err => increase x target (reach out).
                            target_x_mm += dist_step_mm
                            dist_step_z_mm = _clamp(
                                float(getattr(vc, "DIST_SIGN_Z", 1.0)) * dist_err_px * dist_z_mm_per_px,
                                -dist_z_max_step,
                                dist_z_max_step,
                            )
                            target_z_mm += dist_step_z_mm

                    if bool(getattr(vc, "DIST_SHOULDER_ASSIST_ENABLE", True)):
                        shoulder_dist_assist_deg = int(
                            round(
                                float(getattr(vc, "DIST_SIGN_SHOULDER", 1.0))
                                * dist_err_px
                                * float(getattr(vc, "DIST_SHOULDER_DEG_PER_PX", 0.0))
                            )
                        )
                        shoulder_dist_max = max(0, int(getattr(vc, "DIST_SHOULDER_MAX_DEG", 28)))
                        shoulder_dist_assist_deg = max(
                            -shoulder_dist_max, min(shoulder_dist_max, shoulder_dist_assist_deg)
                        )
                        shoulder_dist_alpha = _clamp(
                            float(getattr(vc, "DIST_SHOULDER_SMOOTH_ALPHA", 0.15)), 0.0, 1.0
                        )
                        shoulder_dist_state = (
                            (1.0 - shoulder_dist_alpha) * shoulder_dist_state
                            + shoulder_dist_alpha * float(shoulder_dist_assist_deg)
                        )
                        shoulder_dist_assist_deg = int(round(shoulder_dist_state))
                        shoulder_dist_step_max = max(
                            1, int(getattr(vc, "DIST_SHOULDER_MAX_STEP_PER_FRAME_DEG", 1))
                        )
                        shoulder_dist_assist_deg = int(
                            round(
                                _step_toward(
                                    float(shoulder_dist_last),
                                    float(shoulder_dist_assist_deg),
                                    float(shoulder_dist_step_max),
                                )
                            )
                        )
                        shoulder_dist_last = shoulder_dist_assist_deg

                    target_x_mm = max(float(getattr(vc, "TARGET_X_MIN_MM", 100.0)), min(float(getattr(vc, "TARGET_X_MAX_MM", 230.0)), target_x_mm))
                    target_z_mm = max(float(getattr(vc, "TARGET_Z_MIN_MM", 0.0)), min(float(getattr(vc, "TARGET_Z_MAX_MM", 190.0)), target_z_mm))

                    solved = solve_vertical_plane(
                        x_mm=target_x_mm,
                        z_mm=target_z_mm,
                        base_yaw_rad=base_yaw_rad,
                        q_wrist_rad=0.0,
                        prefer=str(getattr(vc, "IK_PREFER", "elbow_up")),
                    )
                    filtered_notes = [n for n in solved.clip_notes if not n.startswith("clipped_base_")]
                    if solved.ik.ok:
                        ik_status = "ok" if not filtered_notes else "servo_limits_clipped:" + ",".join(filtered_notes)
                    else:
                        ik_status = solved.message
                    ik_clip_notes = filtered_notes
                    # Anti-windup: if vertical chain hits shoulder minimum, stop integrating z further
                    # in the same "upward" direction for this frame.
                    if "clipped_shoulder_min" in solved.clip_notes and z_step > 0:
                        target_z_mm -= z_step
                        target_z_mm = max(
                            float(getattr(vc, "TARGET_Z_MIN_MM", 0.0)),
                            min(float(getattr(vc, "TARGET_Z_MAX_MM", 190.0)), target_z_mm),
                        )
                    vertical_ok = len(filtered_notes) == 0
                    if vertical_ok and solved.ik.ok:
                        cmd = ServoCommand(
                            wrist=config.NEUTRAL_WRIST + wrist_trim_deg,
                            elbow=solved.servo_clamped.elbow + elbow_assist_deg,
                            base=solved.servo_clamped.base,
                            shoulder=solved.servo_clamped.shoulder + shoulder_assist_deg + shoulder_dist_assist_deg,
                        )
                        cmd, _ = clamp_servo(cmd)
                        last_valid_cmd = cmd
                    elif bool(getattr(vc, "IK_ACCEPT_CLAMPED", True)) and solved.ik.ok:
                        # Accept clamped command so base/x can still move even when vertical chain clips.
                        cmd = ServoCommand(
                            wrist=config.NEUTRAL_WRIST + wrist_trim_deg,
                            elbow=solved.servo_clamped.elbow + elbow_assist_deg,
                            base=solved.servo_clamped.base,
                            shoulder=solved.servo_clamped.shoulder + shoulder_assist_deg + shoulder_dist_assist_deg,
                        )
                        cmd, _ = clamp_servo(cmd)
                        last_valid_cmd = cmd
                    elif bool(getattr(vc, "IK_HOLD_LAST_ON_FAIL", True)):
                        # Preserve horizontal/base correction even when holding vertical state.
                        cmd = ServoCommand(
                            wrist=config.NEUTRAL_WRIST + wrist_trim_deg,
                            elbow=last_valid_cmd.elbow + elbow_assist_deg,
                            base=solved.servo_clamped.base,
                            shoulder=last_valid_cmd.shoulder + shoulder_assist_deg + shoulder_dist_assist_deg,
                        )
                        cmd, _ = clamp_servo(cmd)
                    else:
                        cmd = ServoCommand(
                            wrist=config.NEUTRAL_WRIST + wrist_trim_deg,
                            elbow=solved.servo_clamped.elbow + elbow_assist_deg,
                            base=solved.servo_clamped.base,
                            shoulder=solved.servo_clamped.shoulder + shoulder_assist_deg + shoulder_dist_assist_deg,
                        )
                        cmd, _ = clamp_servo(cmd)

                    # Real Z error: target plane Z minus estimated Z from the *actual commanded* joints.
                    est_model = servo_to_model(cmd)
                    est_fk = kinematics.forward_kinematics(est_model.q_shoulder_rad, est_model.q_elbow_rad)
                    z_err_mm = target_z_mm - est_fk.tip.z
                    shoulder_zerr_assist_deg = 0
                    if bool(getattr(vc, "ZERR_SHOULDER_ASSIST_ENABLE", True)):
                        shoulder_zerr_assist_deg = int(
                            round(
                                float(getattr(vc, "ZERR_SIGN_SHOULDER", 1.0))
                                * z_err_mm
                                * float(getattr(vc, "ZERR_SHOULDER_DEG_PER_MM", 0.0))
                            )
                        )
                        shoulder_zerr_max = max(0, int(getattr(vc, "ZERR_SHOULDER_MAX_DEG", 35)))
                        shoulder_zerr_assist_deg = max(
                            -shoulder_zerr_max, min(shoulder_zerr_max, shoulder_zerr_assist_deg)
                        )
                        cmd = ServoCommand(
                            wrist=cmd.wrist,
                            elbow=cmd.elbow,
                            base=cmd.base,
                            shoulder=cmd.shoulder + shoulder_zerr_assist_deg,
                        )
                        cmd, _ = clamp_servo(cmd)
                    # Final elbow command rate-limit to suppress IK-induced snapping.
                    elbow_cmd_step = max(1, int(getattr(vc, "ELBOW_CMD_MAX_STEP_PER_FRAME_DEG", 2)))
                    elbow_cmd = int(
                        round(_step_toward(float(elbow_cmd_last), float(cmd.elbow), float(elbow_cmd_step)))
                    )
                    cmd = ServoCommand(
                        wrist=cmd.wrist,
                        elbow=elbow_cmd,
                        base=cmd.base,
                        shoulder=cmd.shoulder,
                    )
                    cmd, _ = clamp_servo(cmd)
                    elbow_cmd_last = cmd.elbow
                    last_valid_cmd = cmd
                else:
                    d_base = int(
                        round(vc.SIGN_ERROR_X_BASE * corr_x_ctrl * vc.TRACK_GAIN_BASE_DEG)
                    )
                    d_sh = int(
                        round(vc.SIGN_ERROR_Y_SHOULDER * corr_y_ctrl * vc.TRACK_GAIN_SHOULDER_DEG)
                    )
                    d_el = int(
                        round(vc.SIGN_ERROR_Y_ELBOW * corr_y_ctrl * vc.TRACK_GAIN_ELBOW_DEG)
                    )
                    cmd = ServoCommand(
                        wrist=config.NEUTRAL_WRIST,
                        elbow=config.NEUTRAL_ELBOW + d_el,
                        base=config.NEUTRAL_BASE + d_base,
                        shoulder=config.NEUTRAL_SHOULDER + d_sh,
                    )
                    cl, _notes = clamp_servo(cmd)
                    cmd = cl
                    elbow_cmd_last = cmd.elbow

                if use_serial:
                    controller.send_servo(cmd)

                if preview:
                    vis = frame_bgr.copy()
                    cv2.line(vis, (int(cx_img), 0), (int(cx_img), h), (180, 180, 180), 1)
                    cv2.line(vis, (0, int(cy_img)), (w, int(cy_img)), (180, 180, 180), 1)
                    cv2.rectangle(vis, (x, y), (x + fw, y + fh), (0, 255, 0), 2)
                    cv2.circle(vis, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                    cv2.putText(
                        vis,
                        f"mode={ctl} b={cmd.base} s={cmd.shoulder} e={cmd.elbow} w={cmd.wrist}",
                        (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )
                    cv2.putText(
                        vis,
                        f"fps={fps_ema:4.1f} detect_n={detect_every}",
                        (10, 48),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 0),
                        2,
                    )
                    cv2.putText(
                        vis,
                        f"corr x:{corr_x_px:+5.0f}px ({corr_x_norm:+.3f}->{corr_x_ctrl:+.3f}) y:{corr_y_px:+5.0f}px ({corr_y_norm:+.3f}->{corr_y_ctrl:+.3f})",
                        (10, 72),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (255, 200, 0),
                        2,
                    )
                    if ctl == "ik":
                        cv2.putText(
                            vis,
                            f"ik x={target_x_mm:5.1f} z={target_z_mm:5.1f} yaw={base_yaw_rad:+.3f} ramp x={x_ramp:.2f} y={y_ramp:.2f} status={ik_status}",
                            (10, 96),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (200, 255, 200),
                            2,
                        )
                        cv2.putText(
                            vis,
                            f"y_for_z={y_for_z:+.3f} (xmix={float(getattr(vc, 'TRACK_Z_FROM_X_MIX', 0.0)):+.2f})",
                            (10, 120),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (200, 255, 200),
                            2,
                        )
                        cv2.putText(
                            vis,
                            f"z_err={z_err_mm:+5.1f}mm shoulder_dist={shoulder_dist_assist_deg:+d}deg",
                            (10, 144),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (200, 255, 200),
                            2,
                        )
                        if bool(getattr(vc, "DIST_CONTROL_ENABLE", True)):
                            face_w_show = float(filt_face_w if filt_face_w is not None else fw)
                            cv2.putText(
                                vis,
                                f"engage={engage:.2f} dist face_w={face_w_show:5.1f}px target={float(getattr(vc, 'DESIRED_FACE_WIDTH_PX', 160.0)):5.1f}px err={dist_err_px:+5.1f}px dx={dist_step_mm:+4.1f}mm",
                                (10, 168),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (200, 255, 200),
                                2,
                            )
                            cv2.putText(
                                vis,
                                f"dist dz={dist_step_z_mm:+4.1f}mm",
                                (10, 192),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (200, 255, 200),
                                2,
                            )
                    cv2.imshow("GLaDOS face track", vis)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
            else:
                face_lock_frames = 0
                engage = _step_toward(
                    engage,
                    0.0,
                    max(1e-3, float(getattr(vc, "ENGAGE_DOWN_PER_FRAME", 0.35))),
                )
                if bool(getattr(vc, "BASE_PID_RESET_ON_LOSS", True)):
                    base_pid_i = 0.0
                    base_pid_prev_e = 0.0
                    base_pid_d = 0.0
                if ctl == "ik" and bool(getattr(vc, "NO_FACE_VERTICAL_RETURN_ENABLE", True)):
                    z_relax = max(0.0, float(getattr(vc, "NO_FACE_Z_RETURN_MM_PER_FRAME", 2.5)))
                    x_relax = max(0.0, float(getattr(vc, "NO_FACE_X_RETURN_MM_PER_FRAME", 3.0)))
                    target_z_mm = _step_toward(target_z_mm, fk0.tip.z, z_relax)
                    target_x_mm = _step_toward(target_x_mm, fk0.tip.x, x_relax)
                    target_z_mm = _clamp(
                        target_z_mm,
                        float(getattr(vc, "TARGET_Z_MIN_MM", 0.0)),
                        float(getattr(vc, "TARGET_Z_MAX_MM", 190.0)),
                    )
                    target_x_mm = _clamp(
                        target_x_mm,
                        float(getattr(vc, "TARGET_X_MIN_MM", 100.0)),
                        float(getattr(vc, "TARGET_X_MAX_MM", 230.0)),
                    )
                    cmd = ServoCommand(
                        wrist=int(round(_step_toward(float(last_valid_cmd.wrist), float(config.NEUTRAL_WRIST), float(getattr(vc, "NO_FACE_WRIST_RETURN_DEG_PER_FRAME", 4.0))))),
                        elbow=int(round(_step_toward(float(last_valid_cmd.elbow), float(config.NEUTRAL_ELBOW), float(getattr(vc, "NO_FACE_ELBOW_RETURN_DEG_PER_FRAME", 4.0))))),
                        base=last_valid_cmd.base,
                        shoulder=int(round(_step_toward(float(last_valid_cmd.shoulder), float(config.NEUTRAL_SHOULDER), float(getattr(vc, "NO_FACE_SHOULDER_RETURN_DEG_PER_FRAME", 3.0))))),
                    )
                    cmd, _ = clamp_servo(cmd)
                    elbow_cmd_last = cmd.elbow
                    last_valid_cmd = cmd
                    est_model = servo_to_model(cmd)
                    est_fk = kinematics.forward_kinematics(est_model.q_shoulder_rad, est_model.q_elbow_rad)
                    z_err_mm = target_z_mm - est_fk.tip.z
                    if use_serial:
                        controller.send_servo(cmd)
                if preview:
                    vis = frame_bgr.copy()
                    cv2.line(vis, (int(cx_img), 0), (int(cx_img), h), (180, 180, 180), 1)
                    cv2.line(vis, (0, int(cy_img)), (w, int(cy_img)), (180, 180, 180), 1)
                    cv2.putText(
                        vis,
                        "no face",
                        (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 0, 255),
                        2,
                    )
                    cv2.putText(
                        vis,
                        f"fps={fps_ema:4.1f} detect_n={detect_every}",
                        (10, 48),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 0),
                        2,
                    )
                    cv2.putText(
                        vis,
                        "corr x:+0px (+0.000) y:+0px (+0.000)",
                        (10, 72),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (255, 200, 0),
                        2,
                    )
                    if ctl == "ik":
                        clip_msg = ",".join(ik_clip_notes) if ik_clip_notes else "none"
                        cv2.putText(
                            vis,
                            f"ik x={target_x_mm:5.1f} z={target_z_mm:5.1f} clips={clip_msg}",
                            (10, 96),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (200, 255, 200),
                            2,
                        )
                    cv2.imshow("GLaDOS face track", vis)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
            frame_idx += 1

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if use_serial and arm:
            # Return to safe/known pose on shutdown.
            try:
                controller.neutral()
                time.sleep(0.25)
            except Exception as e:
                print(f"Neutral-on-exit failed: {e}", file=sys.stderr)
        picam2.stop()
        if preview:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        if use_serial and arm:
            arm.close()

    return 0


def main(argv: list[str] | None = None) -> int:
    argv = argv if argv is not None else sys.argv[1:]
    p = argparse.ArgumentParser(
        description="Picamera2 face tracking → GLaDOS arm (base + shoulder/elbow)",
    )
    p.add_argument(
        "--port",
        default=config.SERIAL_DEFAULT_PORT,
        help=f"Arduino serial device (default: {config.SERIAL_DEFAULT_PORT})",
    )
    p.add_argument(
        "--no-serial",
        action="store_true",
        help="Camera + detection only; do not open serial or move servos",
    )
    p.add_argument(
        "--preview",
        action="store_true",
        help="Force OpenCV preview window (needs GUI OpenCV + display)",
    )
    p.add_argument(
        "--no-preview",
        action="store_true",
        help="Never open a window (default when DISPLAY is unset)",
    )
    p.add_argument("--width", type=int, default=vision_config.CAMERA_WIDTH)
    p.add_argument("--height", type=int, default=vision_config.CAMERA_HEIGHT)
    p.add_argument(
        "--color-mode",
        choices=("bgr", "rgb"),
        default=getattr(vision_config, "COLOR_MODE", "bgr"),
        help="Interpret Picamera2 raw frame order before OpenCV: bgr or rgb",
    )
    p.add_argument(
        "--control-mode",
        choices=("ik", "proportional"),
        default=getattr(vision_config, "CONTROL_MODE", "ik"),
        help="Tracking control strategy",
    )
    args = p.parse_args(argv)
    try:
        want_preview = resolve_preview_mode(args.preview, args.no_preview)
    except ValueError as e:
        print(str(e), file=sys.stderr)
        return 2
    return run_tracking(
        port=args.port,
        use_serial=not args.no_serial,
        preview=want_preview,
        width=args.width,
        height=args.height,
        color_mode=args.color_mode,
        control_mode=args.control_mode,
    )


if __name__ == "__main__":
    raise SystemExit(main())
