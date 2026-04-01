"""
CLI for servo debug, model mapping test, FK/IK checks, and optional serial send.
"""

from __future__ import annotations

import argparse
import math
import sys
from collections import Counter

from . import config, kinematics
from .controller import (
    VerticalSolveResult,
    explain_assumptions,
    format_servo_line,
    solve_azimuth_elevation,
    solve_vertical_plane,
)
from .mapping import ModelJointState, ServoCommand, clamp_servo, model_to_servo, servo_to_model
from .serial_comm import ArmSerial


def cmd_fk(args: argparse.Namespace) -> int:
    qs = math.radians(args.shoulder_deg)
    qe = math.radians(args.elbow_deg)
    fk = kinematics.forward_kinematics(qs, qe)
    print(f"tip_plane_mm x={fk.tip.x:.3f} z={fk.tip.z:.3f}")
    print(f"theta1_abs_rad={fk.theta1_abs:.5f} theta2_abs_rad={fk.theta2_abs:.5f}")
    return 0


def cmd_ik(args: argparse.Namespace) -> int:
    res = kinematics.inverse_kinematics_plane(args.x, args.z, prefer=args.prefer)
    print(f"ok={res.ok} reason={res.reason} solution={res.solution}")
    if res.ok:
        print(f"q_shoulder_rad={res.q_shoulder:.5f} q_elbow_rad={res.q_elbow:.5f}")
        print(f"theta1_abs_rad={res.theta1_abs:.5f} theta2_abs_rad={res.theta2_abs:.5f}")
    m = ModelJointState(
        base_yaw_rad=0.0,
        q_shoulder_rad=res.q_shoulder if res.ok else 0.0,
        q_elbow_rad=res.q_elbow if res.ok else 0.0,
        q_wrist_rad=0.0,
    )
    raw = model_to_servo(m)
    cl, notes = clamp_servo(raw)
    print(f"servo_raw {raw.wrist} {raw.elbow} {raw.base} {raw.shoulder}")
    print(f"servo_clamped {cl.wrist} {cl.elbow} {cl.base} {cl.shoulder} notes={notes}")
    return 0 if res.ok else 1


def cmd_model_to_servo(args: argparse.Namespace) -> int:
    m = ModelJointState(
        base_yaw_rad=args.base_yaw,
        q_shoulder_rad=args.q_shoulder,
        q_elbow_rad=args.q_elbow,
        q_wrist_rad=args.q_wrist,
    )
    raw = model_to_servo(m)
    cl, notes = clamp_servo(raw)
    print(f"servo_raw {raw.wrist} {raw.elbow} {raw.base} {raw.shoulder}")
    print(f"servo_clamped {cl.wrist} {cl.elbow} {cl.base} {cl.shoulder} notes={notes}")
    return 0 if not notes else 2


def cmd_servo_to_model(args: argparse.Namespace) -> int:
    s = ServoCommand(
        wrist=args.wrist,
        elbow=args.elbow,
        base=args.base,
        shoulder=args.shoulder,
    )
    m = servo_to_model(s)
    print(
        f"base_yaw_rad={m.base_yaw_rad:.5f} q_shoulder_rad={m.q_shoulder_rad:.5f} "
        f"q_elbow_rad={m.q_elbow_rad:.5f} q_wrist_rad={m.q_wrist_rad:.5f}"
    )
    return 0


def cmd_solve(args: argparse.Namespace) -> int:
    r: VerticalSolveResult = solve_vertical_plane(
        args.x,
        args.z,
        args.base_yaw,
        q_wrist_rad=args.q_wrist,
        prefer=args.prefer,
    )
    print(r.message)
    print(f"ik_ok={r.ik.ok} ik_reason={r.ik.reason} branch={r.ik.solution}")
    print(f"model {r.model}")
    print(f"servo_raw {r.servo_raw.wrist} {r.servo_raw.elbow} {r.servo_raw.base} {r.servo_raw.shoulder}")
    print(
        f"servo_clamped {r.servo_clamped.wrist} {r.servo_clamped.elbow} "
        f"{r.servo_clamped.base} {r.servo_clamped.shoulder} clips={r.clip_notes}"
    )
    return 0 if r.ok else 1


def cmd_solve_az_el(args: argparse.Namespace) -> int:
    r = solve_azimuth_elevation(
        args.azimuth,
        args.elevation,
        args.range_mm,
        q_wrist_rad=args.q_wrist,
        prefer=args.prefer,
    )
    print(r.message)
    print(f"ik_ok={r.ik.ok} ik_reason={r.ik.reason}")
    print(format_servo_line(r.servo_clamped))
    return 0 if r.ok else 1


def cmd_serial(args: argparse.Namespace) -> int:
    arm = ArmSerial(port=args.port)
    arm.connect()
    try:
        if args.cmd == "ping":
            arm.write_line("PING")
            print(arm.read_line())
        elif args.cmd == "neutral":
            arm.write_line("NEUTRAL")
            print(arm.read_line())
        elif args.cmd == "set_servo":
            line = f"SET_SERVO {args.wrist} {args.elbow} {args.base} {args.shoulder}"
            arm.write_line(line)
            print(arm.read_line())
        elif args.cmd == "raw":
            arm.write_line(args.line)
            print(arm.read_line())
    finally:
        arm.close()
    return 0


def cmd_assumptions(_: argparse.Namespace) -> int:
    for line in explain_assumptions():
        print(line)
    return 0


def cmd_track(args: argparse.Namespace) -> int:
    from . import vision_config
    from .face_tracking import resolve_preview_mode, run_tracking

    w = args.width if args.width is not None else vision_config.CAMERA_WIDTH
    h = args.height if args.height is not None else vision_config.CAMERA_HEIGHT
    try:
        want_preview = resolve_preview_mode(args.preview, args.no_preview)
    except ValueError as e:
        print(str(e), file=sys.stderr)
        return 2
    return run_tracking(
        port=args.port,
        use_serial=not args.no_serial,
        preview=want_preview,
        width=w,
        height=h,
        color_mode=args.color_mode,
        control_mode=args.control_mode,
    )


def cmd_ik_benchmark(args: argparse.Namespace) -> int:
    """
    Evaluate current IK + mapping quality over an x/z grid.
    Reports:
      - geometric IK success rate (math reachable)
      - servo-feasible rate (after model->servo mapping with limits)
      - FK reconstruction error stats for successful IK points
    """
    x_min = args.x_min
    x_max = args.x_max
    z_min = args.z_min
    z_max = args.z_max
    nx = max(2, int(args.nx))
    nz = max(2, int(args.nz))
    prefer = args.prefer

    total = 0
    ik_ok = 0
    servo_ok = 0
    errors_mm: list[float] = []
    fail_reasons: Counter[str] = Counter()
    clip_reasons: Counter[str] = Counter()

    for iz in range(nz):
        z = z_min + (z_max - z_min) * (iz / (nz - 1))
        for ix in range(nx):
            x = x_min + (x_max - x_min) * (ix / (nx - 1))
            total += 1

            ik = kinematics.inverse_kinematics_plane(x, z, prefer=prefer)
            if not ik.ok:
                fail_reasons[ik.reason] += 1
                continue
            ik_ok += 1

            fk = kinematics.forward_kinematics(ik.q_shoulder, ik.q_elbow)
            err = math.hypot(fk.tip.x - x, fk.tip.z - z)
            errors_mm.append(err)

            # Reuse existing controller path for model->servo feasibility checks.
            solved: VerticalSolveResult = solve_vertical_plane(
                x_mm=x,
                z_mm=z,
                base_yaw_rad=0.0,
                q_wrist_rad=0.0,
                prefer=prefer,
            )
            if solved.clip_notes:
                for n in solved.clip_notes:
                    clip_reasons[n] += 1
            if solved.ok:
                servo_ok += 1

    if total == 0:
        print("No samples.")
        return 2

    ik_rate = 100.0 * ik_ok / total
    servo_rate = 100.0 * servo_ok / total
    mean_err = (sum(errors_mm) / len(errors_mm)) if errors_mm else float("nan")
    max_err = (max(errors_mm)) if errors_mm else float("nan")

    print("IK benchmark (vertical plane)")
    print(f"grid: x=[{x_min:.1f},{x_max:.1f}] z=[{z_min:.1f},{z_max:.1f}] samples={nx}x{nz} total={total}")
    print(f"prefer_branch={prefer}")
    print(f"ik_ok={ik_ok}/{total} ({ik_rate:.1f}%)")
    print(f"servo_feasible={servo_ok}/{total} ({servo_rate:.1f}%)")
    print(f"fk_reconstruction_error_mm mean={mean_err:.4f} max={max_err:.4f}")

    if fail_reasons:
        print("ik_fail_reasons:")
        for k, v in fail_reasons.most_common():
            print(f"  {k}: {v}")
    if clip_reasons:
        print("servo_clip_reasons:")
        for k, v in clip_reasons.most_common():
            print(f"  {k}: {v}")
    else:
        print("servo_clip_reasons: none")

    # Non-zero exit if the benchmark quality is poor by simple thresholds.
    # This makes it script-friendly on Pi.
    if args.max_mean_err_mm is not None and mean_err > args.max_mean_err_mm:
        print(f"FAIL threshold: mean_err {mean_err:.4f} > {args.max_mean_err_mm:.4f}")
        return 3
    if args.min_servo_ok_rate is not None and servo_rate < args.min_servo_ok_rate:
        print(f"FAIL threshold: servo_feasible_rate {servo_rate:.2f}% < {args.min_servo_ok_rate:.2f}%")
        return 4
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="GLaDOS arm control / kinematics CLI")
    sub = p.add_subparsers(dest="command", required=True)

    fk = sub.add_parser("fk", help="forward kinematics (model offsets in radians)")
    fk.add_argument("--shoulder-deg", type=float, default=0.0, help="q_shoulder in degrees")
    fk.add_argument("--elbow-deg", type=float, default=0.0, help="q_elbow in degrees")
    fk.set_defaults(func=cmd_fk)

    ik = sub.add_parser("ik", help="inverse kinematics plane target mm")
    ik.add_argument("x", type=float)
    ik.add_argument("z", type=float)
    ik.add_argument("--prefer", choices=("elbow_up", "elbow_down"), default="elbow_up")
    ik.set_defaults(func=cmd_ik)

    m2s = sub.add_parser("model-to-servo", help="map model state to servo degrees")
    m2s.add_argument("--base-yaw", type=float, default=0.0)
    m2s.add_argument("--q-shoulder", type=float, default=0.0)
    m2s.add_argument("--q-elbow", type=float, default=0.0)
    m2s.add_argument("--q-wrist", type=float, default=0.0)
    m2s.set_defaults(func=cmd_model_to_servo)

    s2m = sub.add_parser("servo-to-model", help="map servo degrees to model state")
    s2m.add_argument("wrist", type=int)
    s2m.add_argument("elbow", type=int)
    s2m.add_argument("base", type=int)
    s2m.add_argument("shoulder", type=int)
    s2m.set_defaults(func=cmd_servo_to_model)

    sv = sub.add_parser("solve", help="full vertical solve + mapping")
    sv.add_argument("x", type=float)
    sv.add_argument("z", type=float)
    sv.add_argument("--base-yaw", type=float, default=0.0)
    sv.add_argument("--q-wrist", type=float, default=0.0)
    sv.add_argument("--prefer", choices=("elbow_up", "elbow_down"), default="elbow_up")
    sv.set_defaults(func=cmd_solve)

    sa = sub.add_parser("solve-az-el", help="direction aim using range in mm")
    sa.add_argument("azimuth", type=float, help="radians")
    sa.add_argument("elevation", type=float, help="radians")
    sa.add_argument("range_mm", type=float)
    sa.add_argument("--q-wrist", type=float, default=0.0)
    sa.add_argument("--prefer", choices=("elbow_up", "elbow_down"), default="elbow_up")
    sa.set_defaults(func=cmd_solve_az_el)

    ser = sub.add_parser("serial", help="talk to Arduino")
    ser.add_argument("--port", default=config.SERIAL_DEFAULT_PORT)
    ser.add_argument("cmd", choices=("ping", "neutral", "set_servo", "raw"))
    ser.add_argument("--wrist", type=int, default=60)
    ser.add_argument("--elbow", type=int, default=270)
    ser.add_argument("--base", type=int, default=90)
    ser.add_argument("--shoulder", type=int, default=0)
    ser.add_argument("--line", default="", help="for raw subcommand")
    ser.set_defaults(func=cmd_serial)

    sub.add_parser("assumptions", help="print kinematic assumptions").set_defaults(func=cmd_assumptions)

    tr = sub.add_parser(
        "track",
        help="Picamera2 face tracking → arm (Pi; run on Raspberry Pi with camera stack)",
    )
    tr.add_argument("--port", default=config.SERIAL_DEFAULT_PORT)
    tr.add_argument("--no-serial", action="store_true")
    tr.add_argument("--preview", action="store_true", help="force OpenCV window")
    tr.add_argument("--no-preview", action="store_true", help="never open window")
    tr.add_argument("--color-mode", choices=("bgr", "rgb"), default=None, help="override camera color order")
    tr.add_argument("--control-mode", choices=("ik", "proportional"), default=None, help="override tracking control strategy")
    tr.add_argument("--width", type=int, default=None)
    tr.add_argument("--height", type=int, default=None)
    tr.set_defaults(func=cmd_track)

    ikb = sub.add_parser("ik-benchmark", help="benchmark vertical-plane IK + mapping over grid")
    ikb.add_argument("--x-min", type=float, default=40.0)
    ikb.add_argument("--x-max", type=float, default=240.0)
    ikb.add_argument("--z-min", type=float, default=-80.0)
    ikb.add_argument("--z-max", type=float, default=180.0)
    ikb.add_argument("--nx", type=int, default=33)
    ikb.add_argument("--nz", type=int, default=33)
    ikb.add_argument("--prefer", choices=("elbow_up", "elbow_down"), default="elbow_up")
    ikb.add_argument("--max-mean-err-mm", type=float, default=None)
    ikb.add_argument("--min-servo-ok-rate", type=float, default=None, help="percent, e.g. 80")
    ikb.set_defaults(func=cmd_ik_benchmark)

    return p


def main(argv: list[str] | None = None) -> int:
    argv = argv if argv is not None else sys.argv[1:]
    p = build_parser()
    args = p.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
