"""
Face tracking with **Picamera2** (libcamera) + OpenCV Haar cascade → proportional arm commands.

Requires Raspberry Pi OS with working ``rpicam-hello`` / Picamera2 stack.

Architecture (matches your arm model):
  * Image X error → **base** (horizontal)
  * Image Y error → **shoulder + elbow** (vertical chain); wrist held at neutral

This is **proportional image-space control** with clamped servo commands, not full IK per frame.
Replace with calibrated angular mapping / IK when ready.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import cv2
import numpy as np

from . import config, vision_config
from .controller import RobotController
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
    mode = (color_mode or getattr(vc, "COLOR_MODE", "bgr")).strip().lower()
    if mode not in ("bgr", "rgb"):
        print(f"Invalid color mode '{mode}', using 'bgr'.", flush=True)
        mode = "bgr"
    detect_every = max(1, int(getattr(vc, "DETECT_EVERY_N_FRAMES", 1)))
    print(f"Color mode: {mode} | detect_every_n_frames={detect_every}", flush=True)
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

    try:
        while True:
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

                err_x = (cx - cx_img) / max(cx_img, 1.0)
                err_y = (cy_img - cy) / max(cy_img, 1.0)

                err_x = _apply_deadband(err_x, vc.TRACK_DEADBAND)
                err_y = _apply_deadband(err_y, vc.TRACK_DEADBAND)

                d_base = int(
                    round(vc.SIGN_ERROR_X_BASE * err_x * vc.TRACK_GAIN_BASE_DEG)
                )
                d_sh = int(
                    round(vc.SIGN_ERROR_Y_SHOULDER * err_y * vc.TRACK_GAIN_SHOULDER_DEG)
                )
                d_el = int(
                    round(vc.SIGN_ERROR_Y_ELBOW * err_y * vc.TRACK_GAIN_ELBOW_DEG)
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
                    cv2.rectangle(vis, (x, y), (x + fw, y + fh), (0, 255, 0), 2)
                    cv2.circle(vis, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                    cv2.putText(
                        vis,
                        f"b={cmd.base} s={cmd.shoulder} e={cmd.elbow}",
                        (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )
                    cv2.imshow("GLaDOS face track", vis)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
            else:
                if preview:
                    vis = frame_bgr.copy()
                    cv2.putText(
                        vis,
                        "no face",
                        (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 0, 255),
                        2,
                    )
                    cv2.imshow("GLaDOS face track", vis)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
            frame_idx += 1

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
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
    )


if __name__ == "__main__":
    raise SystemExit(main())
