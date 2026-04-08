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

from . import config, kinematics, motion_config_v1 as mv1, vision_config
from .controller import RobotController
from .mapping import ServoCommand, clamp_servo
from .motion_controller_v1 import MotionControllerV1, VisionMeasurement
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


def _preprocess_gray_for_detect(gray: np.ndarray, vc: object) -> np.ndarray:
    """Grayscale for Haar: global hist_eq or CLAHE (often better in flat / mid lighting)."""
    if bool(getattr(vc, "VISION_CLAHE_ENABLE", False)):
        clip = float(getattr(vc, "VISION_CLAHE_CLIP", 2.0))
        tile = max(2, int(getattr(vc, "VISION_CLAHE_TILE", 8)))
        clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(tile, tile))
        return clahe.apply(gray)
    return cv2.equalizeHist(gray)


def _step_toward(cur: float, target: float, max_step: float) -> float:
    d = target - cur
    if abs(d) <= max_step:
        return target
    return cur + max_step if d > 0 else cur - max_step


def _first_find_biased_targets(cmd: ServoCommand, vc: object) -> tuple[int, int]:
    """IK command + optional bias so shoulder/elbow are not both == neutral when IK is flat."""
    bs = float(getattr(vc, "FIRST_FIND_BIAS_SHOULDER_DEG", 0.0))
    be = float(getattr(vc, "FIRST_FIND_BIAS_ELBOW_DEG", 0.0))
    return int(round(float(cmd.shoulder) + bs)), int(round(float(cmd.elbow) + be))


def _servo_deg_toward_ideal(neutral_deg: int, ideal_deg: float) -> int:
    """
    Integer servo command from neutral→ideal. Slow ramps use fractional degrees; plain int(round())
    often stays on neutral for many frames — creep by at least 1° when ideal has diverged.
    """
    r = int(round(ideal_deg))
    if r != neutral_deg:
        return r
    if ideal_deg > float(neutral_deg) + 0.08:
        return neutral_deg + 1
    if ideal_deg < float(neutral_deg) - 0.08:
        return neutral_deg - 1
    return neutral_deg


def _apply_first_find_extend(
    cmd: ServoCommand,
    phase: str,
    ramp: float,
    vc: object,
) -> tuple[ServoCommand, str, float]:
    """
    On first face after none: shoulder/elbow blend neutral → 25% of IK delta (slow), then → full IK (slow).
    Base and wrist follow the current command unchanged.
    """
    nu = _neutral_command()
    t_sh, t_el = _first_find_biased_targets(cmd, vc)
    ef = float(getattr(vc, "FIRST_FIND_EXTEND_FRACTION", 0.25))
    ef = max(0.0, min(1.0, ef))
    pq = max(1e-6, float(getattr(vc, "FIRST_FIND_TO_QUARTER_PER_FRAME", 0.012)))
    pf = max(1e-6, float(getattr(vc, "FIRST_FIND_TO_FULL_PER_FRAME", 0.035)))

    if phase == "idle":
        return cmd, phase, ramp
    if phase == "to_quarter":
        ramp = min(1.0, ramp + pq)
        fac = ef * ramp
        ideal_sh = float(nu.shoulder) + fac * (float(t_sh) - float(nu.shoulder))
        ideal_el = float(nu.elbow) + fac * (float(t_el) - float(nu.elbow))
        sh = _servo_deg_toward_ideal(nu.shoulder, ideal_sh)
        el = _servo_deg_toward_ideal(nu.elbow, ideal_el)
        out = ServoCommand(wrist=cmd.wrist, elbow=el, base=cmd.base, shoulder=sh)
        out, _ = clamp_servo(out)
        if ramp >= 1.0 - 1e-9:
            return out, "to_full", 0.0
        return out, phase, ramp
    if phase == "to_full":
        ramp = min(1.0, ramp + pf)
        factor = ef + (1.0 - ef) * ramp
        ideal_sh = float(nu.shoulder) + factor * (float(t_sh) - float(nu.shoulder))
        ideal_el = float(nu.elbow) + factor * (float(t_el) - float(nu.elbow))
        sh = _servo_deg_toward_ideal(nu.shoulder, ideal_sh)
        el = _servo_deg_toward_ideal(nu.elbow, ideal_el)
        out = ServoCommand(wrist=cmd.wrist, elbow=el, base=cmd.base, shoulder=sh)
        out, _ = clamp_servo(out)
        if ramp >= 1.0 - 1e-9:
            return out, "idle", 0.0
        return out, phase, ramp
    return cmd, "idle", 0.0


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
    tried_sensor_modes: list[tuple[int, int]] = []
    cfg = None
    sensor_modes = getattr(vc, "SENSOR_OUTPUT_SIZE_FALLBACKS", None)
    if not isinstance(sensor_modes, (tuple, list)) or len(sensor_modes) == 0:
        sensor_output = getattr(vc, "SENSOR_OUTPUT_SIZE", None)
        if isinstance(sensor_output, (tuple, list)) and len(sensor_output) == 2:
            sensor_modes = (sensor_output,)
    if isinstance(sensor_modes, (tuple, list)):
        for mode in sensor_modes:
            if not isinstance(mode, (tuple, list)) or len(mode) != 2:
                continue
            try:
                so = (int(mode[0]), int(mode[1]))
                tried_sensor_modes.append(so)
                cfg_try = dict(cfg_kwargs)
                cfg_try["sensor"] = {"output_size": so}
                cfg = picam2.create_video_configuration(**cfg_try)
                print(f"Video cfg using sensor mode {so}", flush=True)
                break
            except Exception as e:
                print(f"Sensor mode {mode} rejected: {e}", flush=True)
    if cfg is None:
        if tried_sensor_modes:
            print("All requested sensor modes rejected; using default video config.", flush=True)
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
        # Use NEUTRAL (same pose as firmware kStartupDeg) — not SET_SERVO startup angles that
        # used to differ from validated neutral and caused a snap when a face was first tracked.
        controller.neutral()

    fk0 = kinematics.forward_kinematics(0.0, 0.0)
    base_yaw_lim = _base_yaw_limit_rad()
    motion = MotionControllerV1(
        vc,
        mv1,
        fk0_tip_x=fk0.tip.x,
        fk0_tip_z=fk0.tip.z,
        base_yaw_lim=base_yaw_lim,
    )

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
    face_lock_frames = 0
    engage = 0.0
    last_face_seen_t = time.time()
    no_face_neutral_sent = False
    prev_had_face = False
    first_find_phase = "idle"
    first_find_ramp = 0.0
    sm_bbox: tuple[float, float, float, float] | None = None
    hold_left = 0

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

            gray_small = _preprocess_gray_for_detect(gray_small, vc)
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

            raw_best: tuple[int, int, int, int] | None = None
            if len(faces) > 0:
                areas = [fw * fh for (_x, _y, fw, fh) in faces]
                i = int(np.argmax(areas))
                x0, y0, fw0, fh0 = faces[i]
                raw_best = (
                    int(round(x0 * inv_scale)),
                    int(round(y0 * inv_scale)),
                    int(round(fw0 * inv_scale)),
                    int(round(fh0 * inv_scale)),
                )

            bbox_sm_alpha = float(getattr(vc, "FACE_BBOX_SMOOTH_ALPHA", 0.25))
            hold_max = max(0, int(getattr(vc, "FACE_HOLD_MAX_FRAMES", 15)))
            track_rect: tuple[float, float, float, float] | None = None
            if raw_best is not None:
                rf = tuple(float(v) for v in raw_best)
                if sm_bbox is None:
                    sm_bbox = rf
                else:
                    sm_bbox = tuple(
                        (1.0 - bbox_sm_alpha) * s + bbox_sm_alpha * r
                        for s, r in zip(sm_bbox, rf)
                    )
                hold_left = hold_max
                track_rect = sm_bbox
            elif hold_left > 0 and sm_bbox is not None:
                hold_left -= 1
                track_rect = sm_bbox
            else:
                track_rect = None
                sm_bbox = None
                hold_left = 0

            coasting = raw_best is None and track_rect is not None

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

            if track_rect is not None:
                if not prev_had_face:
                    if bool(getattr(vc, "FIRST_FIND_EXTEND_ENABLE", False)):
                        first_find_phase = "to_quarter"
                        first_find_ramp = 0.0
                last_face_seen_t = now_t
                motion.last_face_seen_t = now_t
                no_face_neutral_sent = False
                motion.no_face_neutral_sent = False
                face_lock_frames += 1
                x = int(round(track_rect[0]))
                y = int(round(track_rect[1]))
                fw = int(round(track_rect[2]))
                fh = int(round(track_rect[3]))
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
                # Filtered cy lags real motion → vertical error builds slowly (arm feels seconds late).
                # Raw cy matches wrist responsiveness; horizontal still uses filt_cx for smooth pan.
                err_y_filt = (cy_img - filt_cy) / max(cy_img, 1.0)  # face above => +
                err_y_raw = (cy_img - cy) / max(cy_img, 1.0)
                use_raw_y = bool(getattr(vc, "FACE_Y_USE_RAW", False))
                err_y = err_y_raw if use_raw_y else err_y_filt

                db_x = float(getattr(vc, "TRACK_DEADBAND", 0.03))
                db_y = float(getattr(vc, "TRACK_DEADBAND_Y", db_x))
                corr_x_norm = _apply_deadband(err_x, db_x)
                corr_y_norm = _apply_deadband(err_y, db_y)
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
                # Pre-engage: ramped norm error before engage mask (wrist/assist use engaged corr below).
                corr_x_pre_eng = corr_x_ctrl
                corr_y_pre_eng = corr_y_ctrl
                corr_x_ctrl *= engage
                corr_y_ctrl *= engage
                # After lock-in, use full vertical error for IK/assists/wrist (engage only softens first N frames).
                locked_in = face_lock_frames >= lock_need
                corr_y_vert = corr_y_pre_eng if locked_in else corr_y_ctrl
                # IK vertical + first-find floor (when first-find off, IK always used full pre-engage error).
                if bool(getattr(vc, "FIRST_FIND_EXTEND_ENABLE", False)):
                    if first_find_phase != "idle":
                        corr_y_ik = corr_y_vert
                        min_v = float(getattr(vc, "FIRST_FIND_MIN_VERTICAL_NORM", 0.0))
                        if min_v > 0.0 and abs(corr_y_ik) < min_v:
                            sgn = 1.0 if corr_y_norm >= 0.0 else -1.0
                            if abs(corr_y_norm) < 1e-6:
                                sgn = 1.0
                            corr_y_ik = sgn * min_v
                    else:
                        corr_y_ik = corr_y_vert
                else:
                    corr_y_ik = corr_y_pre_eng
                vm = VisionMeasurement(
                    face_detected=True,
                    err_x_norm=corr_x_norm,
                    err_y_norm=corr_y_norm,
                    corr_x_norm_raw=err_x,
                    corr_y_norm_raw=err_y,
                    filt_face_w=filt_face_w,
                    face_w_px=fw,
                    t_seconds=now_t,
                )
                motion.face_lock_frames = face_lock_frames
                motion.engage = engage

                if ctl == "ik":
                    cmd = motion.process_ik(
                        vm,
                        corr_x_ctrl=corr_x_ctrl,
                        corr_y_vert=corr_y_vert,
                        corr_y_ik=corr_y_ik,
                        corr_y_norm=corr_y_norm,
                        engage=engage,
                        dt=dt,
                    )
                    target_x_mm = motion.target_x_mm
                    target_z_mm = motion.target_z_mm
                    base_yaw_rad = motion.base_yaw_rad
                    ik_status = motion.ik_status
                    ik_clip_notes = motion.ik_clip_notes
                    y_for_z = motion.y_for_z
                    z_err_mm = motion.z_err_mm
                    dist_err_px = motion.dist_err_px
                    dist_step_mm = 0.0
                    dist_step_z_mm = 0.0
                else:
                    cmd = motion.process_proportional(
                        vm,
                        corr_x_ctrl=corr_x_ctrl,
                        corr_y_vert=corr_y_vert,
                        dt=dt,
                    )
                    target_x_mm = motion.target_x_mm
                    target_z_mm = motion.target_z_mm
                    base_yaw_rad = motion.base_yaw_rad
                    ik_status = "proportional"
                    ik_clip_notes = []
                    y_for_z = 0.0
                    z_err_mm = 0.0
                    dist_err_px = 0.0
                    dist_step_mm = 0.0
                    dist_step_z_mm = 0.0
                last_valid_cmd = motion.last_valid_cmd

                if bool(getattr(vc, "FIRST_FIND_EXTEND_ENABLE", False)) and first_find_phase != "idle":
                    cmd, first_find_phase, first_find_ramp = _apply_first_find_extend(
                        cmd, first_find_phase, first_find_ramp, vc
                    )
                    motion.elbow_cmd_last = cmd.elbow

                if use_serial:
                    controller.send_servo(cmd)

                if preview:
                    vis = frame_bgr.copy()
                    cv2.line(vis, (int(cx_img), 0), (int(cx_img), h), (180, 180, 180), 1)
                    cv2.line(vis, (0, int(cy_img)), (w, int(cy_img)), (180, 180, 180), 1)
                    cv2.rectangle(vis, (x, y), (x + fw, y + fh), (0, 255, 0), 2)
                    if coasting:
                        cv2.putText(
                            vis,
                            "hold",
                            (min(x + fw + 4, w - 52), max(y + 18, 20)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 255),
                            1,
                            cv2.LINE_AA,
                        )
                    cv2.circle(vis, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                    cv2.putText(
                        vis,
                        f"mode={ctl} first_find={first_find_phase} b={cmd.base} s={cmd.shoulder} e={cmd.elbow} w={cmd.wrist}",
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
                        f"corr x:{corr_x_px:+5.0f}px ({corr_x_norm:+.3f}->{corr_x_ctrl:+.3f}) y:{corr_y_px:+5.0f}px ({corr_y_norm:+.3f}->{corr_y_vert:+.3f}v e={engage:.2f})",
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
                        y_ol = (
                            f"y_for_z={y_for_z:+.3f} (xmix={float(getattr(vc, 'TRACK_Z_FROM_X_MIX', 0.0)):+.2f})"
                        )
                        cv2.putText(
                            vis,
                            y_ol,
                            (10, 120),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (200, 255, 200),
                            2,
                        )
                        cv2.putText(
                            vis,
                            f"z_err={z_err_mm:+5.1f}mm shoulder_dist={motion.shoulder_dist_last:+d}deg",
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
                first_find_phase = "idle"
                first_find_ramp = 0.0
                face_lock_frames = 0
                engage = _step_toward(
                    engage,
                    0.0,
                    max(1e-3, float(getattr(vc, "ENGAGE_DOWN_PER_FRAME", 0.35))),
                )
                _nf_cmd, neutral_sent = motion.process_no_face(
                    vc, now_t=now_t, ctl=ctl, fk0_tip_x=fk0.tip.x, fk0_tip_z=fk0.tip.z
                )
                no_face_neutral_sent = motion.no_face_neutral_sent
                last_valid_cmd = motion.last_valid_cmd
                target_x_mm = motion.target_x_mm
                target_z_mm = motion.target_z_mm
                base_yaw_rad = motion.base_yaw_rad
                ik_clip_notes = motion.ik_clip_notes
                z_err_mm = motion.z_err_mm
                if neutral_sent and use_serial:
                    controller.neutral()
                no_face_delay_s = max(0.0, float(getattr(vc, "NO_FACE_RETURN_DELAY_S", 30.0)))
                face_missing_for_s = max(0.0, now_t - last_face_seen_t)
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
                        remaining = max(0.0, no_face_delay_s - face_missing_for_s)
                        cv2.putText(
                            vis,
                            f"no-face timer: {remaining:4.1f}s to neutral",
                            (10, 120),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (200, 255, 200),
                            2,
                        )
                    cv2.imshow("GLaDOS face track", vis)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
            prev_had_face = track_rect is not None
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
