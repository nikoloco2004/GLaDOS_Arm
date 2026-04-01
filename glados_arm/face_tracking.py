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
from .mapping import ServoCommand, clamp_servo
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
    cfg = picam2.create_preview_configuration(
        main={"size": (width, height), "format": "RGB888"},
        controls={"FrameRate": float(getattr(vc, "CAMERA_FPS", 30))},
    )
    picam2.configure(cfg)
    picam2.start()

    # Request full sensor crop when available (reduces "zoomed-in" look).
    try:
        max_crop = picam2.camera_properties.get("ScalerCropMaximum")
        if max_crop is not None:
            picam2.set_controls({"ScalerCrop": max_crop})
            print(f"ScalerCrop set to full sensor: {max_crop}", flush=True)
    except Exception:
        pass

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

            if len(faces) > 0:
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

                err_x = (filt_cx - cx_img) / max(cx_img, 1.0)  # face right => +
                err_y = (cy_img - filt_cy) / max(cy_img, 1.0)  # face above => +

                corr_x_norm = _apply_deadband(err_x, vc.TRACK_DEADBAND)
                corr_y_norm = _apply_deadband(err_y, vc.TRACK_DEADBAND)
                corr_x_px = corr_x_norm * cx_img
                corr_y_px = corr_y_norm * cy_img

                if ctl == "ik":
                    base_step = (
                        vc.SIGN_ERROR_X_BASE * corr_x_norm * float(getattr(vc, "TRACK_BASE_RAD_PER_NORM", 0.04))
                    )
                    base_step = _clamp(
                        base_step,
                        -float(getattr(vc, "MAX_BASE_YAW_STEP_RAD", 0.06)),
                        float(getattr(vc, "MAX_BASE_YAW_STEP_RAD", 0.06)),
                    )
                    base_yaw_rad += base_step
                    base_yaw_lim = math.radians(float(getattr(vc, "BASE_YAW_MAX_DEG", 75.0)))
                    base_yaw_rad = _clamp(base_yaw_rad, -base_yaw_lim, base_yaw_lim)

                    z_step = (
                        vc.SIGN_ERROR_Y_SHOULDER * corr_y_norm * float(getattr(vc, "TRACK_Z_MM_PER_NORM", 10.0))
                    )
                    z_step = _clamp(
                        z_step,
                        -float(getattr(vc, "MAX_Z_STEP_MM", 6.0)),
                        float(getattr(vc, "MAX_Z_STEP_MM", 6.0)),
                    )
                    target_z_mm += z_step

                    x_step = (
                        vc.SIGN_ERROR_X_BASE * corr_x_norm * float(getattr(vc, "TRACK_X_MM_PER_NORM", 0.0))
                    )
                    x_step = _clamp(
                        x_step,
                        -float(getattr(vc, "MAX_X_STEP_MM", 3.0)),
                        float(getattr(vc, "MAX_X_STEP_MM", 3.0)),
                    )
                    target_x_mm += x_step

                    target_x_mm = max(float(getattr(vc, "TARGET_X_MIN_MM", 100.0)), min(float(getattr(vc, "TARGET_X_MAX_MM", 230.0)), target_x_mm))
                    target_z_mm = max(float(getattr(vc, "TARGET_Z_MIN_MM", 0.0)), min(float(getattr(vc, "TARGET_Z_MAX_MM", 170.0)), target_z_mm))

                    solved = solve_vertical_plane(
                        x_mm=target_x_mm,
                        z_mm=target_z_mm,
                        base_yaw_rad=base_yaw_rad,
                        q_wrist_rad=0.0,
                        prefer=str(getattr(vc, "IK_PREFER", "elbow_up")),
                    )
                    ik_status = solved.message
                    ik_clip_notes = solved.clip_notes
                    if solved.ok:
                        cmd = solved.servo_clamped
                        last_valid_cmd = cmd
                    elif bool(getattr(vc, "IK_ACCEPT_CLAMPED", True)) and solved.ik.ok:
                        # Accept clamped command so base/x can still move even when vertical chain clips.
                        # Keep wrist at neutral by design.
                        cmd = ServoCommand(
                            wrist=config.NEUTRAL_WRIST,
                            elbow=solved.servo_clamped.elbow,
                            base=solved.servo_clamped.base,
                            shoulder=solved.servo_clamped.shoulder,
                        )
                        last_valid_cmd = cmd
                    elif bool(getattr(vc, "IK_HOLD_LAST_ON_FAIL", True)):
                        # Preserve horizontal/base correction even when holding vertical state.
                        cmd = ServoCommand(
                            wrist=last_valid_cmd.wrist,
                            elbow=last_valid_cmd.elbow,
                            base=solved.servo_clamped.base,
                            shoulder=last_valid_cmd.shoulder,
                        )
                    else:
                        cmd = solved.servo_clamped
                else:
                    d_base = int(
                        round(vc.SIGN_ERROR_X_BASE * corr_x_norm * vc.TRACK_GAIN_BASE_DEG)
                    )
                    d_sh = int(
                        round(vc.SIGN_ERROR_Y_SHOULDER * corr_y_norm * vc.TRACK_GAIN_SHOULDER_DEG)
                    )
                    d_el = int(
                        round(vc.SIGN_ERROR_Y_ELBOW * corr_y_norm * vc.TRACK_GAIN_ELBOW_DEG)
                    )
                    cmd = ServoCommand(
                        wrist=config.NEUTRAL_WRIST,
                        elbow=config.NEUTRAL_ELBOW + d_el,
                        base=config.NEUTRAL_BASE + d_base,
                        shoulder=config.NEUTRAL_SHOULDER + d_sh,
                    )
                    cl, _notes = clamp_servo(cmd)
                    cmd = cl

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
                        f"mode={ctl} b={cmd.base} s={cmd.shoulder} e={cmd.elbow}",
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
                        f"corr x:{corr_x_px:+5.0f}px ({corr_x_norm:+.3f}) y:{corr_y_px:+5.0f}px ({corr_y_norm:+.3f})",
                        (10, 72),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (255, 200, 0),
                        2,
                    )
                    if ctl == "ik":
                        cv2.putText(
                            vis,
                            f"ik x={target_x_mm:5.1f} z={target_z_mm:5.1f} yaw={base_yaw_rad:+.3f} status={ik_status}",
                            (10, 96),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (200, 255, 200),
                            2,
                        )
                    cv2.imshow("GLaDOS face track", vis)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
            else:
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
