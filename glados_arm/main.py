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
    RobotController,
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


def cmd_ik_vertical_test(args: argparse.Namespace) -> int:
    """
    Synthetic step-test for IK vertical correction dynamics.
    Applies fixed normalized x/y correction for N steps and prints target evolution + clip behavior.
    Useful for tuning TRACK_Z_MM_PER_NORM and TRACK_Z_FROM_X_MIX.
    """
    prefer = args.prefer
    steps = max(1, int(args.steps))
    corr_x = float(args.corr_x)
    corr_y = float(args.corr_y)
    x = float(args.start_x)
    z = float(args.start_z)
    yaw = float(args.start_yaw)

    from . import vision_config

    z_mix = float(getattr(vision_config, "TRACK_Z_FROM_X_MIX", 0.0))
    sign_x_to_z = float(getattr(vision_config, "SIGN_ERROR_X_TO_Z", 1.0))
    k_base = float(getattr(vision_config, "TRACK_BASE_RAD_PER_NORM", 0.04))
    k_z = float(getattr(vision_config, "TRACK_Z_MM_PER_NORM", 10.0))
    k_x = float(getattr(vision_config, "TRACK_X_MM_PER_NORM", 0.0))
    max_base_step = float(getattr(vision_config, "MAX_BASE_YAW_STEP_RAD", 0.08))
    max_z_step = float(getattr(vision_config, "MAX_Z_STEP_MM", 10.0))
    max_x_step = float(getattr(vision_config, "MAX_X_STEP_MM", 3.0))
    x_min = float(getattr(vision_config, "TARGET_X_MIN_MM", 100.0))
    x_max = float(getattr(vision_config, "TARGET_X_MAX_MM", 230.0))
    z_min = float(getattr(vision_config, "TARGET_Z_MIN_MM", 0.0))
    z_max = float(getattr(vision_config, "TARGET_Z_MAX_MM", 190.0))
    yaw_lim = math.radians(float(getattr(vision_config, "BASE_YAW_MAX_DEG", 90.0)))

    clips = 0
    oks = 0
    print(
        f"ik-vertical-test steps={steps} corr_x={corr_x:+.3f} corr_y={corr_y:+.3f} "
        f"start(x,z,yaw)=({x:.1f},{z:.1f},{yaw:+.3f})"
    )
    print(f"params k_base={k_base} k_z={k_z} z_mix={z_mix} sign_x_to_z={sign_x_to_z}")

    def _clamp(v: float, lo: float, hi: float) -> float:
        return lo if v < lo else hi if v > hi else v

    for i in range(steps):
        base_step = config.BASE_YAW_SIGN * corr_x * k_base
        base_step = _clamp(base_step, -max_base_step, max_base_step)
        yaw = _clamp(yaw + base_step, -yaw_lim, yaw_lim)

        y_for_z = _clamp(corr_y + sign_x_to_z * z_mix * corr_x, -1.0, 1.0)
        z_step = float(getattr(vision_config, "SIGN_ERROR_Y_SHOULDER", 1.0)) * y_for_z * k_z
        z_step = _clamp(z_step, -max_z_step, max_z_step)
        z = _clamp(z + z_step, z_min, z_max)

        x_step = config.BASE_YAW_SIGN * corr_x * k_x
        x_step = _clamp(x_step, -max_x_step, max_x_step)
        x = _clamp(x + x_step, x_min, x_max)

        r = solve_vertical_plane(x_mm=x, z_mm=z, base_yaw_rad=yaw, q_wrist_rad=0.0, prefer=prefer)
        if r.ok:
            oks += 1
        if r.clip_notes:
            clips += 1
        print(
            f"step={i+1:02d} x={x:6.1f} z={z:6.1f} yaw={yaw:+.3f} "
            f"ok={r.ok} clips={r.clip_notes} cmd=({r.servo_clamped.base},{r.servo_clamped.shoulder},{r.servo_clamped.elbow},{r.servo_clamped.wrist})"
        )

    print(f"summary ok_steps={oks}/{steps} clipped_steps={clips}/{steps}")
    return 0


def _wrist_stab_pitch_rad(
    q_shoulder_rad: float,
    q_elbow_rad: float,
    base_yaw_rad: float,
    mv1: object,
) -> float:
    """
    Same wrist model angle as MotionControllerV1 (stab): cancel link pitch so camera boresight
    matches DESIRED_CAMERA_PITCH_RAD (see motion_config_v1).
    """
    fk = kinematics.forward_kinematics(q_shoulder_rad, q_elbow_rad)
    desired = float(getattr(mv1, "DESIRED_CAMERA_PITCH_RAD", 0.0))
    mount = float(getattr(mv1, "CAMERA_MOUNT_OFFSET_RAD", 0.0))
    kg = float(getattr(mv1, "BASE_YAW_COUPLING_GAIN", 0.0))
    return desired - (fk.theta1_abs + fk.theta2_abs) - mount - kg * base_yaw_rad


def _solve_vertical_with_optional_wrist_stab(
    x_mm: float,
    z_mm: float,
    base_yaw_rad: float,
    prefer: str,
    mv1: object,
    wrist_stab: bool,
) -> VerticalSolveResult:
    """Planar IK, then optional second solve with q_wrist from stab (IK unchanged)."""
    r0 = solve_vertical_plane(
        x_mm=x_mm,
        z_mm=z_mm,
        base_yaw_rad=base_yaw_rad,
        q_wrist_rad=0.0,
        prefer=prefer,
    )
    if not r0.ik.ok or not wrist_stab:
        return r0
    qw = _wrist_stab_pitch_rad(
        r0.model.q_shoulder_rad,
        r0.model.q_elbow_rad,
        base_yaw_rad,
        mv1,
    )
    return solve_vertical_plane(
        x_mm=x_mm,
        z_mm=z_mm,
        base_yaw_rad=base_yaw_rad,
        q_wrist_rad=qw,
        prefer=prefer,
    )


def _vertical_path_zs(z0: float, z_top: float, step_mm: float, return_down: bool) -> list[float]:
    zs_up: list[float] = []
    z = z0
    while z < z_top - 1e-6:
        zs_up.append(z)
        z += step_mm
    zs_up.append(z_top)
    zs_path = list(zs_up)
    if return_down and len(zs_up) > 1:
        zs_path = zs_up + zs_up[-2::-1]
    return zs_path


def _clamp_ik_target(x: float, z: float, yaw: float, vc: object) -> tuple[float, float, float]:
    xmin = float(getattr(vc, "TARGET_X_MIN_MM", 100.0))
    xmax = float(getattr(vc, "TARGET_X_MAX_MM", 230.0))
    zmin = float(getattr(vc, "TARGET_Z_MIN_MM", 0.0))
    zmax = float(getattr(vc, "TARGET_Z_MAX_MM", 170.0))
    ylim = math.radians(float(getattr(vc, "BASE_YAW_MAX_DEG", 180.0)))
    x = max(xmin, min(xmax, x))
    z = max(zmin, min(zmax, z))
    yaw = max(-ylim, min(ylim, yaw))
    return x, z, yaw


def cmd_ik_servo_test(args: argparse.Namespace) -> int:
    """
    Scripted IK → SET_SERVO sequence on hardware: ping, neutral, small x/z/yaw probes, neutral.

    Use --dry-run on a PC to print the plan. Run on the Pi with the arm clear of people/obstacles.
    """
    import time

    from . import vision_config as vc

    prefer = args.prefer

    delay = max(0.05, float(args.delay_s))
    # Signed deltas from FK home tip: +x is farther in reach direction (often near workspace limit).
    dx = float(args.dx_mm)
    dz = float(args.dz_mm)
    dyaw = math.radians(float(args.dyaw_deg))

    fk0 = kinematics.forward_kinematics(0.0, 0.0)
    x0, z0 = float(fk0.tip.x), float(fk0.tip.z)
    x0, z0, _ = _clamp_ik_target(x0, z0, 0.0, vc)

    waypoints: list[tuple[str, float, float, float]] = [
        ("fk_home (model q=0 tip)", x0, z0, 0.0),
        ("plane x +dx (reach)", x0 + dx, z0, 0.0),
        ("fk_home", x0, z0, 0.0),
        ("plane z +dz (height)", x0, z0 + dz, 0.0),
        ("fk_home", x0, z0, 0.0),
        ("base yaw +dyaw (pan)", x0, z0, dyaw),
        ("fk_home", x0, z0, 0.0),
    ]

    print("IK servo test - stand clear; small motions from FK home tip.")
    print(f"FK(0,0) tip x={fk0.tip.x:.2f} z={fk0.tip.z:.2f} mm | prefer={prefer} delay={delay:.2f}s")
    print(f"steps dx={dx:+.1f} mm dz={dz:+.1f} mm dyaw={math.degrees(dyaw):+.2f} deg")

    def _one(label: str, x: float, z: float, yaw: float) -> VerticalSolveResult:
        xc, zc, yc = _clamp_ik_target(x, z, yaw, vc)
        r = solve_vertical_plane(
            x_mm=xc,
            z_mm=zc,
            base_yaw_rad=yc,
            q_wrist_rad=0.0,
            prefer=prefer,
        )
        clipped = ",".join(r.clip_notes) if r.clip_notes else "none"
        print(
            f"  {label:28s} x={xc:6.1f} z={zc:6.1f} yaw={yc:+.4f} "
            f"ik_ok={r.ik.ok} servo_feasible={r.ok} clips={clipped} "
            f"servo=({r.servo_clamped.wrist},{r.servo_clamped.elbow},{r.servo_clamped.base},{r.servo_clamped.shoulder})"
        )
        return r

    if args.dry_run:
        for label, x, z, yaw in waypoints:
            _one(label, x, z, yaw)
        print("dry-run: no serial motion.")
        return 0

    arm = ArmSerial(port=args.port)
    controller = RobotController(serial=arm)
    controller.connect()
    try:
        if not controller.ping():
            print("PING failed — check USB port and firmware.", file=sys.stderr)
            return 1
        print("PING ok. Sending NEUTRAL…")
        controller.neutral()
        time.sleep(max(0.15, delay * 0.5))

        for label, x, z, yaw in waypoints:
            r = _one(label, x, z, yaw)
            if not r.ik.ok:
                print("ABORT: IK failed ( unreachable target ). Check dx/dz/dyaw or envelope.", file=sys.stderr)
                controller.neutral()
                return 2
            controller.send_servo(r.servo_clamped)
            time.sleep(delay)

        print("Returning to NEUTRAL…")
        controller.neutral()
        time.sleep(max(0.15, delay * 0.5))
    finally:
        controller.close()

    print("Done.")
    return 0


def cmd_ik_servo_vertical_line(args: argparse.Namespace) -> int:
    """
    Fixed base yaw (no pan) and fixed plane x: move tip upward along +z in the *kinematic* plane.

    This is **Cartesian** motion: (x,z) targets go through planar 2R IK. The resulting **servo**
    shoulder/elbow angles are whatever solves that geometry (elbow_up branch), not a preset
    like "shoulder 90 deg / elbow 180 deg". If NEUTRAL_ELBOW is at the elbow servo max (270),
    many upward moves use **elbow** first; shoulder may hit SERVO_SHOULDER_MIN depending on
    calibration (THETA*_REF, NEUTRAL_*, SHOULDER_SIGN). Use --pull-back-mm to change reach depth.

    Wrist stays at model q_wrist=0. Face tracking uses separate wrist stab for horizon hold.
    """
    import time

    from . import vision_config as vc

    prefer = args.prefer
    delay = max(0.08, float(args.delay_s))
    step_mm = max(0.5, float(args.step_mm))
    scan_mm = max(0.25, float(args.scan_mm))
    yaw_fixed = 0.0

    fk0 = kinematics.forward_kinematics(0.0, 0.0)
    x0 = float(args.x_mm) if args.x_mm is not None else float(fk0.tip.x)
    x0 -= float(args.pull_back_mm)
    z0 = float(fk0.tip.z)
    x0, z0, yaw_fixed = _clamp_ik_target(x0, z0, yaw_fixed, vc)

    z_max_env = float(getattr(vc, "TARGET_Z_MAX_MM", 170.0))

    # Find highest z at this x with fixed yaw (coarse scan).
    z_top = z0
    z_scan = z0 + scan_mm
    while z_scan <= z_max_env + 1e-6:
        xc, zc, yc = _clamp_ik_target(x0, z_scan, yaw_fixed, vc)
        r = solve_vertical_plane(
            x_mm=xc,
            z_mm=zc,
            base_yaw_rad=yc,
            q_wrist_rad=0.0,
            prefer=prefer,
        )
        if not r.ik.ok:
            break
        z_top = zc
        z_scan += scan_mm

    zs_path = _vertical_path_zs(z0, z_top, step_mm, bool(args.return_down))

    print("IK vertical line - fixed base yaw (forward), fixed x, sweep +z then optional return.")
    print(
        f"FK(0,0) tip x={fk0.tip.x:.2f} z={fk0.tip.z:.2f} mm | using x={x0:.1f} mm "
        f"(pull_back={float(args.pull_back_mm):.1f} mm) yaw={yaw_fixed:+.4f} rad | prefer={prefer}"
    )
    print(f"scan: z_top={z_top:.2f} mm (env z_max={z_max_env:.1f}) | path steps={len(zs_path)} step={step_mm:.1f}mm delay={delay:.2f}s")

    def _fmt_line(r: VerticalSolveResult, zq: float, idx: str) -> str:
        clipped = ",".join(r.clip_notes) if r.clip_notes else "none"
        extra = ""
        if r.clip_notes:
            extra = f" raw_sh={r.servo_raw.shoulder}"
        return (
            f"  {idx} z={zq:6.2f} ik_ok={r.ik.ok} servo_feasible={r.ok} clips={clipped}{extra} "
            f"servo=({r.servo_clamped.wrist},{r.servo_clamped.elbow},{r.servo_clamped.base},{r.servo_clamped.shoulder})"
        )

    def _solve_at_z(zq: float) -> VerticalSolveResult:
        xc, zc, yc = _clamp_ik_target(x0, zq, yaw_fixed, vc)
        return solve_vertical_plane(
            x_mm=xc,
            z_mm=zc,
            base_yaw_rad=yc,
            q_wrist_rad=0.0,
            prefer=prefer,
        )

    if z_top > z0 + 1e-6:
        r_start = _solve_at_z(z0)
        rp = _solve_at_z(z0 + 0.5 * (z_top - z0))
        # Only warn when the *start* pose never asked for shoulder above min (misleading if
        # pull-back already moves shoulder, e.g. raw_sh=9 at z0).
        if (
            "clipped_shoulder_min" in rp.clip_notes
            and rp.servo_raw.shoulder < float(config.NEUTRAL_SHOULDER)
            and r_start.servo_raw.shoulder <= 0
        ):
            print(
                "NOTE: Mid-path IK wants shoulder below SERVO_SHOULDER_MIN (0 deg); "
                "clamped to 0 there. At z0 the model did not command positive shoulder. "
                "Try --pull-back-mm, or recalibrate NEUTRAL_* / THETA*_REF so neutral is not "
                "shoulder-at-min + elbow-at-max (elbow then does most of the vertical lift)."
            )

    if args.dry_run:
        for i, zq in enumerate(zs_path):
            r = _solve_at_z(zq)
            print(_fmt_line(r, zq, f"{i+1:03d}"))
        print("dry-run: no serial motion.")
        return 0

    arm = ArmSerial(port=args.port)
    controller = RobotController(serial=arm)
    controller.connect()
    try:
        if not controller.ping():
            print("PING failed - check USB port and firmware.", file=sys.stderr)
            return 1
        print("PING ok. Sending NEUTRAL…")
        controller.neutral()
        time.sleep(max(0.2, delay * 0.5))

        for i, zq in enumerate(zs_path):
            r = _solve_at_z(zq)
            print(_fmt_line(r, zq, f"{i+1:03d}/{len(zs_path)}"))
            if not r.ik.ok:
                print("ABORT: IK failed mid-path.", file=sys.stderr)
                controller.neutral()
                return 2
            controller.send_servo(r.servo_clamped)
            time.sleep(delay)

        print("Returning to NEUTRAL…")
        controller.neutral()
        time.sleep(max(0.2, delay * 0.5))
    finally:
        controller.close()

    print("Done.")
    return 0


def cmd_raise_camera_line(args: argparse.Namespace) -> int:
    """
    From firmware NEUTRAL (model q=0 at FK(0,0) tip): move tip in a straight vertical line in
    the kinematic plane (fixed x, fixed base yaw). Optionally apply wrist stabilization so the
    camera stays level (same math as face-tracking WRIST_TRIM_MODE=stab).
    """
    import time

    from . import motion_config_v1 as mv1
    from . import vision_config as vc

    prefer = args.prefer
    delay = max(0.1, float(args.delay_s))
    step_mm = max(0.5, float(args.step_mm))
    scan_mm = max(0.25, float(args.scan_mm))
    yaw_fixed = 0.0
    wrist_stab = bool(args.wrist_stab)

    fk0 = kinematics.forward_kinematics(0.0, 0.0)
    x0 = float(args.x_mm) if args.x_mm is not None else float(fk0.tip.x)
    x0 -= float(args.pull_back_mm)
    z0 = float(fk0.tip.z)
    x0, z0, yaw_fixed = _clamp_ik_target(x0, z0, yaw_fixed, vc)

    z_max_env = float(getattr(vc, "TARGET_Z_MAX_MM", 170.0))

    z_top = z0
    z_scan = z0 + scan_mm
    while z_scan <= z_max_env + 1e-6:
        xc, zc, yc = _clamp_ik_target(x0, z_scan, yaw_fixed, vc)
        r = solve_vertical_plane(
            x_mm=xc,
            z_mm=zc,
            base_yaw_rad=yc,
            q_wrist_rad=0.0,
            prefer=prefer,
        )
        if not r.ik.ok:
            break
        z_top = zc
        z_scan += scan_mm

    zs_path = _vertical_path_zs(z0, z_top, step_mm, bool(args.return_down))

    print("Raise camera: neutral tip -> Cartesian +z line; base yaw fixed (no pan).")
    print(
        f"FK(0,0) tip x={fk0.tip.x:.2f} z={fk0.tip.z:.2f} mm | path x={x0:.1f} mm "
        f"pull_back={float(args.pull_back_mm):.1f} mm | wrist_stab={wrist_stab} | prefer={prefer}"
    )
    print(
        f"z_top={z_top:.2f} mm (env z_max={z_max_env:.1f}) | steps={len(zs_path)} "
        f"step={step_mm:.1f} mm delay={delay:.2f} s"
    )

    def _fmt_line(r: VerticalSolveResult, zq: float, idx: str) -> str:
        clipped = ",".join(r.clip_notes) if r.clip_notes else "none"
        extra = ""
        if r.clip_notes:
            extra = f" raw_sh={r.servo_raw.shoulder}"
        return (
            f"  {idx} z={zq:6.2f} ik_ok={r.ik.ok} servo_feasible={r.ok} clips={clipped}{extra} "
            f"servo=({r.servo_clamped.wrist},{r.servo_clamped.elbow},{r.servo_clamped.base},{r.servo_clamped.shoulder})"
        )

    def _solve_at_z(zq: float) -> VerticalSolveResult:
        xc, zc, yc = _clamp_ik_target(x0, zq, yaw_fixed, vc)
        return _solve_vertical_with_optional_wrist_stab(
            xc,
            zc,
            yc,
            prefer,
            mv1,
            wrist_stab,
        )

    if z_top > z0 + 1e-6:
        r_start = _solve_at_z(z0)
        rp = _solve_at_z(z0 + 0.5 * (z_top - z0))
        if (
            "clipped_shoulder_min" in rp.clip_notes
            and rp.servo_raw.shoulder < float(config.NEUTRAL_SHOULDER)
            and r_start.servo_raw.shoulder <= 0
        ):
            print(
                "NOTE: Mid-path shoulder may clamp at min; elbow does most lift from this neutral. "
                "Try --pull-back-mm if you need more shoulder range."
            )

    if args.dry_run:
        for i, zq in enumerate(zs_path):
            print(_fmt_line(_solve_at_z(zq), zq, f"{i+1:03d}"))
        print("dry-run: no serial motion.")
        return 0

    arm = ArmSerial(port=args.port)
    controller = RobotController(serial=arm)
    controller.connect()
    try:
        if not controller.ping():
            print("PING failed - check USB port and firmware.", file=sys.stderr)
            return 1
        print("PING ok. Sending NEUTRAL (firmware pose = model q=0)…")
        controller.neutral()
        time.sleep(max(0.25, delay * 0.5))

        for i, zq in enumerate(zs_path):
            r = _solve_at_z(zq)
            print(_fmt_line(r, zq, f"{i+1:03d}/{len(zs_path)}"))
            if not r.ik.ok:
                print("ABORT: IK failed mid-path.", file=sys.stderr)
                controller.neutral()
                return 2
            controller.send_servo(r.servo_clamped)
            time.sleep(delay)

        print("Returning to NEUTRAL…")
        controller.neutral()
        time.sleep(max(0.25, delay * 0.5))
    finally:
        controller.close()

    print("Done.")
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

    ikv = sub.add_parser("ik-vertical-test", help="synthetic vertical correction step test for IK tuning")
    ikv.add_argument("--corr-x", type=float, default=0.0, help="normalized x correction input [-1..1]")
    ikv.add_argument("--corr-y", type=float, default=0.4, help="normalized y correction input [-1..1]")
    ikv.add_argument("--steps", type=int, default=20)
    ikv.add_argument("--start-x", type=float, default=170.0)
    ikv.add_argument("--start-z", type=float, default=90.0)
    ikv.add_argument("--start-yaw", type=float, default=0.0, help="radians")
    ikv.add_argument("--prefer", choices=("elbow_up", "elbow_down"), default="elbow_up")
    ikv.set_defaults(func=cmd_ik_vertical_test)

    iks = sub.add_parser(
        "ik-servo-test",
        help="scripted IK poses over serial (ping → neutral → small x/z/yaw → neutral); use --dry-run first",
    )
    iks.add_argument("--port", default=config.SERIAL_DEFAULT_PORT, help="Arduino serial device")
    iks.add_argument(
        "--delay-s",
        type=float,
        default=0.55,
        help="seconds between SET_SERVO commands (default: 0.55)",
    )
    iks.add_argument(
        "--dx-mm",
        type=float,
        default=-6.0,
        help="plane x delta from FK home (mm); negative pulls back (safer near max reach)",
    )
    iks.add_argument(
        "--dz-mm",
        type=float,
        default=6.0,
        help="plane z delta from FK home (mm); positive raises tip in plane",
    )
    iks.add_argument("--dyaw-deg", type=float, default=7.0, help="base pan delta (deg)")
    iks.add_argument(
        "--prefer",
        choices=("elbow_up", "elbow_down"),
        default="elbow_up",
        help="IK branch (vision_config IK_PREFER used if valid when omitted in code paths)",
    )
    iks.add_argument(
        "--dry-run",
        action="store_true",
        help="print IK/servo plan only; do not open serial",
    )
    iks.set_defaults(func=cmd_ik_servo_test)

    ikv2 = sub.add_parser(
        "ik-servo-vertical",
        help=(
            "Cartesian vertical line: fixed x + base yaw, sweep +z via planar IK (not fixed servo angles); "
            "see command docstring. --dry-run first."
        ),
    )
    ikv2.add_argument("--port", default=config.SERIAL_DEFAULT_PORT)
    ikv2.add_argument("--delay-s", type=float, default=0.65, help="seconds between poses (default: 0.65)")
    ikv2.add_argument(
        "--step-mm",
        type=float,
        default=3.0,
        help="interpolation step along z between start and z_top (default: 3)",
    )
    ikv2.add_argument(
        "--scan-mm",
        type=float,
        default=0.5,
        help="coarse scan step when finding max z (default: 0.5)",
    )
    ikv2.add_argument(
        "--x-mm",
        type=float,
        default=None,
        help="fixed plane x in mm (default: FK(0,0) tip x)",
    )
    ikv2.add_argument(
        "--pull-back-mm",
        type=float,
        default=0.0,
        help="subtract from x before clamp (tip closer to base; often frees shoulder from min clip)",
    )
    ikv2.add_argument(
        "--return-down",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="after reaching z_top, trace back to start (default: true)",
    )
    ikv2.add_argument("--prefer", choices=("elbow_up", "elbow_down"), default="elbow_up")
    ikv2.add_argument("--dry-run", action="store_true")
    ikv2.set_defaults(func=cmd_ik_servo_vertical_line)

    rc = sub.add_parser(
        "raise-camera",
        help="slow vertical line from NEUTRAL: fixed x + base yaw, +z scan; wrist stab keeps camera level",
    )
    rc.add_argument("--port", default=config.SERIAL_DEFAULT_PORT)
    rc.add_argument(
        "--delay-s",
        type=float,
        default=0.9,
        help="seconds between poses (default: 0.9, slower than ik-servo-vertical)",
    )
    rc.add_argument("--step-mm", type=float, default=2.0, help="z interpolation step (default: 2)")
    rc.add_argument("--scan-mm", type=float, default=0.5, help="coarse scan for z_top (default: 0.5)")
    rc.add_argument("--x-mm", type=float, default=None, help="fixed plane x mm (default: FK(0,0) tip x)")
    rc.add_argument(
        "--pull-back-mm",
        type=float,
        default=0.0,
        help="subtract from x (tip closer to base)",
    )
    rc.add_argument(
        "--return-down",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="trace back to start z after peak (default: true)",
    )
    rc.add_argument("--prefer", choices=("elbow_up", "elbow_down"), default="elbow_up")
    rc.add_argument(
        "--wrist-stab",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="level camera using motion_config_v1 (DESIRED_CAMERA_PITCH_RAD, etc.); use --no-wrist-stab to fix wrist at model 0",
    )
    rc.add_argument("--dry-run", action="store_true")
    rc.set_defaults(func=cmd_raise_camera_line)

    return p


def main(argv: list[str] | None = None) -> int:
    argv = argv if argv is not None else sys.argv[1:]
    p = build_parser()
    args = p.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
