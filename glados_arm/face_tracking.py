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
import sys
import time
import cv2
import numpy as np

from . import config, vision_config
from .controller import RobotController
from .mapping import ServoCommand, clamp_servo
from .serial_comm import ArmSerial


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


def run_tracking(
    *,
    port: str,
    use_serial: bool,
    preview: bool,
    width: int,
    height: int,
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
    face_cascade = cv2.CascadeClassifier(
        cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    )
    if face_cascade.empty():
        print("Failed to load Haar cascade XML.", file=sys.stderr)
        return 1

    picam2 = Picamera2()
    cfg = picam2.create_preview_configuration(
        main={"size": (width, height), "format": "RGB888"},
    )
    picam2.configure(cfg)
    picam2.start()
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
    print("Tracking: Ctrl+C to stop. Picamera2 + OpenCV; horizontal→base, vertical→shoulder/elbow.")

    try:
        while True:
            frame_rgb = picam2.capture_array("main")
            if frame_rgb is None or frame_rgb.size == 0:
                continue

            gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
            gray = cv2.equalizeHist(gray)
            faces = face_cascade.detectMultiScale(
                gray,
                scaleFactor=vc.HAAR_SCALE_FACTOR,
                minNeighbors=vc.HAAR_MIN_NEIGHBORS,
                minSize=vc.HAAR_MIN_SIZE,
            )

            h, w = gray.shape[:2]
            cx_img = w * 0.5
            cy_img = h * 0.5

            if len(faces) > 0:
                areas = [fw * fh for (_x, _y, fw, fh) in faces]
                i = int(np.argmax(areas))
                x, y, fw, fh = faces[i]
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
                    vis = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
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
                    vis = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
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
        help="Show OpenCV window (needs display on Pi)",
    )
    p.add_argument("--width", type=int, default=vision_config.CAMERA_WIDTH)
    p.add_argument("--height", type=int, default=vision_config.CAMERA_HEIGHT)
    args = p.parse_args(argv)
    return run_tracking(
        port=args.port,
        use_serial=not args.no_serial,
        preview=args.preview,
        width=args.width,
        height=args.height,
    )


if __name__ == "__main__":
    raise SystemExit(main())
