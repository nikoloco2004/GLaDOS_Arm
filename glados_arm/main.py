"""
CLI for servo debug, model mapping test, FK/IK checks, and optional serial send.
"""

from __future__ import annotations

import argparse
import bisect
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
from .motion_smooth import (
    float_tuple_to_servo_command,
    lowpass_scalar,
    rate_limit_servo_deg_per_sec,
    servo_command_to_float_tuple,
    sync_step_servo_float_toward,
    sync_step_servo_toward,
)
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
        disable_y_axis=bool(args.disable_y_axis),
        disable_x_axis=bool(args.disable_x_axis),
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

    Hardware note: on this arm, **model q_wrist = 0** (servo at NEUTRAL_WRIST) is already level and
    facing forward; stab adds q_wrist as shoulder/elbow move so the camera **stays** that way.
    Tune CAMERA_MOUNT_OFFSET_RAD if the first pose after stab does not match neutral wrist.
    """
    fk = kinematics.forward_kinematics(q_shoulder_rad, q_elbow_rad)
    desired = float(getattr(mv1, "DESIRED_CAMERA_PITCH_RAD", 0.0))
    mount = float(getattr(mv1, "CAMERA_MOUNT_OFFSET_RAD", 0.0))
    kg = float(getattr(mv1, "BASE_YAW_COUPLING_GAIN", 0.0))
    gp = float(getattr(mv1, "WRIST_STAB_LINK_PITCH_GAIN", 1.0))
    return desired + gp * (fk.theta1_abs + fk.theta2_abs) - mount - kg * base_yaw_rad


def _wrist_stab_pitch_rad_delta_neutral(
    q_shoulder_rad: float,
    q_elbow_rad: float,
    base_yaw_rad: float,
    mv1: object,
) -> float:
    """
    Level reference = **neutral model pose** (q_shoulder=q_elbow=0): camera level with wrist at
    model zero. Compensate only the **change** in shoulder+elbow link pitch from that pose:
    q_wrist = desired - (sum_cur - sum_neutral) - mount - kg*base, so q_wrist=0 at neutral.
    """
    fk0 = kinematics.forward_kinematics(0.0, 0.0)
    fk = kinematics.forward_kinematics(q_shoulder_rad, q_elbow_rad)
    s0 = fk0.theta1_abs + fk0.theta2_abs
    s1 = fk.theta1_abs + fk.theta2_abs
    desired = float(getattr(mv1, "DESIRED_CAMERA_PITCH_RAD", 0.0))
    mount = float(getattr(mv1, "CAMERA_MOUNT_OFFSET_RAD", 0.0))
    kg = float(getattr(mv1, "BASE_YAW_COUPLING_GAIN", 0.0))
    gp = float(getattr(mv1, "WRIST_STAB_LINK_PITCH_GAIN", 1.0))
    return desired + gp * (s1 - s0) - mount - kg * base_yaw_rad


def _solve_vertical_with_optional_wrist_stab(
    x_mm: float,
    z_mm: float,
    base_yaw_rad: float,
    prefer: str,
    mv1: object,
    wrist_stab: bool,
    wrist_stab_mode: str = "absolute",
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
    if wrist_stab_mode == "delta_neutral":
        qw = _wrist_stab_pitch_rad_delta_neutral(
            r0.model.q_shoulder_rad,
            r0.model.q_elbow_rad,
            base_yaw_rad,
            mv1,
        )
    else:
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


def _scan_max_z_coarse_then_binary(
    x0: float,
    z0: float,
    yaw_fixed: float,
    z_max_env: float,
    scan_mm: float,
    prefer: str,
    vc: object,
) -> float:
    """Find maximum feasible z at fixed x (IK with q_wrist=0); coarse scan then binary refine."""
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
    # Refine between last OK and first fail (bracket within one coarse step)
    lo = z_top
    hi = min(z_top + scan_mm, z_max_env)
    if hi <= lo + 1e-4:
        return z_top
    for _ in range(16):
        mid = 0.5 * (lo + hi)
        xc, zc, yc = _clamp_ik_target(x0, mid, yaw_fixed, vc)
        r = solve_vertical_plane(
            x_mm=xc,
            z_mm=zc,
            base_yaw_rad=yc,
            q_wrist_rad=0.0,
            prefer=prefer,
        )
        if r.ik.ok:
            lo = mid
        else:
            hi = mid
    return lo


def _smooth_servo_toward(
    prev: ServoCommand,
    target: ServoCommand,
    dt: float,
    mv1: object,
) -> ServoCommand:
    """Match MotionControllerV1: LPF then deg/s rate limit."""
    alpha = float(getattr(mv1, "COMMAND_LPF_ALPHA", 0.35))
    max_dps = getattr(mv1, "MAX_JOINT_DPS", (120.0, 90.0, 60.0, 75.0))
    smoothed = ServoCommand(
        wrist=int(round(lowpass_scalar(float(prev.wrist), float(target.wrist), alpha))),
        elbow=int(round(lowpass_scalar(float(prev.elbow), float(target.elbow), alpha))),
        base=int(round(lowpass_scalar(float(prev.base), float(target.base), alpha))),
        shoulder=int(round(lowpass_scalar(float(prev.shoulder), float(target.shoulder), alpha))),
    )
    return rate_limit_servo_deg_per_sec(prev, smoothed, dt, max_dps)


def _raise_camera_interp_z_for_cost(cost_target: float, costs: list[float], z_grid: list[float]) -> float:
    """Linearly interpolate z for a target cumulative joint-space cost along a sampled path."""
    if not costs or not z_grid or len(costs) != len(z_grid):
        return z_grid[0] if z_grid else 0.0
    if cost_target <= costs[0]:
        return z_grid[0]
    if cost_target >= costs[-1]:
        return z_grid[-1]
    i = bisect.bisect_right(costs, cost_target) - 1
    i = max(0, min(i, len(costs) - 2))
    c0, c1 = costs[i], costs[i + 1]
    z0, z1 = z_grid[i], z_grid[i + 1]
    if c1 <= c0 + 1e-12:
        return z1
    w = (cost_target - c0) / (c1 - c0)
    return z0 + w * (z1 - z0)


def _raise_camera_build_arc_cost_grid(
    z_a: float,
    z_b: float,
    seg_start_cmd: ServoCommand,
    solve_fn,
    k_fine: int,
) -> tuple[list[float], list[float]]:
    """
    Walk z_a→z_b in uniform z samples; cumulative cost = sum of max |Δservo| between consecutive
    ideal poses (same wrist feedback chain as runtime). Used to re-time z(t) so motion is even
    in joint space (linear z in time causes joint speed to spike near workspace limits).
    """
    k_fine = max(3, min(400, k_fine))
    z_grid: list[float] = [z_a]
    costs: list[float] = [0.0]
    r_a = solve_fn(z_a, seg_start_cmd)
    cmd_prev = r_a.servo_clamped
    for j in range(1, k_fine):
        t = j / float(k_fine - 1)
        zq = z_a + t * (z_b - z_a)
        r = solve_fn(zq, cmd_prev)
        cmd = r.servo_clamped
        delta = max(
            abs(cmd.wrist - cmd_prev.wrist),
            abs(cmd.elbow - cmd_prev.elbow),
            abs(cmd.base - cmd_prev.base),
            abs(cmd.shoulder - cmd_prev.shoulder),
        )
        costs.append(costs[-1] + max(delta, 1e-3))
        z_grid.append(zq)
        cmd_prev = cmd
    return costs, z_grid


def _raise_camera_arc_z_schedule(
    z_a: float,
    z_b: float,
    seg_start_cmd: ServoCommand,
    solve_fn,
    n_frames: int,
    k_fine: int,
) -> list[float]:
    """One z sample per frame index 0..n_frames; uniform progress in cumulative joint cost."""
    costs, z_grid = _raise_camera_build_arc_cost_grid(z_a, z_b, seg_start_cmd, solve_fn, k_fine)
    total = costs[-1]
    if total < 1e-6:
        return [z_a + (z_b - z_a) * (i / float(max(1, n_frames))) for i in range(n_frames + 1)]
    out: list[float] = []
    for i in range(n_frames + 1):
        u = i / float(max(1, n_frames))
        ct = u * total
        out.append(_raise_camera_interp_z_for_cost(ct, costs, z_grid))
    return out


def _raise_camera_linear_z_schedule(z_a: float, z_b: float, n_frames: int) -> list[float]:
    return [z_a + (z_b - z_a) * (i / float(max(1, n_frames))) for i in range(n_frames + 1)]


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
    From firmware NEUTRAL: straight vertical line in the (x,z) plane (fixed x, base yaw 0).

    Default **smooth** mode: fixed **x**, **z(t)** (arc-length optional) = straight vertical line.
    **Default** is **direct IK**: send full ``servo_clamped`` every frame so all joints follow the
    same Cartesian solution (no inter-frame slew, which caused one joint at a time with integer
    servos). Use ``--raise-slew`` for rate-limited blending toward each IK target if needed.

    Wrist uses **delta_neutral** stab: level reference is neutral (q=0); q_wrist tracks the
    change in link pitch from that pose. Stab reads shoulder/elbow from the **previous
    commanded** servos so wrist stays tied to the smoothed arm motion (not a fixed angle from
    ideal IK while joints lag).

    Use --discrete for the older stepped path (large z steps, hold at each pose).
    """
    import time

    from . import motion_config_v1 as mv1
    from . import vision_config as vc

    prefer = args.prefer
    discrete = bool(args.discrete)
    delay = max(0.1, float(args.delay_s))
    step_mm = max(0.5, float(args.step_mm))
    scan_mm = max(0.15, float(args.scan_mm))
    yaw_fixed = 0.0
    wrist_stab = bool(args.wrist_stab)
    hz = max(5.0, float(args.hz))
    duration_up = max(0.5, float(args.duration_up))
    base_dps = tuple(float(x) for x in getattr(mv1, "MAX_JOINT_DPS", (120.0, 90.0, 60.0, 75.0)))
    raise_speed = max(0.5, float(args.raise_speed))
    max_dps_raise = tuple(min(220.0, d * raise_speed) for d in base_dps)
    duration_down = float(args.duration_up) if args.duration_down is None else float(args.duration_down)
    dt = 1.0 / hz
    substeps = max(1, int(args.raise_substeps))
    raise_slew = bool(args.raise_slew)

    fk0 = kinematics.forward_kinematics(0.0, 0.0)
    x0 = float(args.x_mm) if args.x_mm is not None else float(fk0.tip.x)
    x0 -= float(args.pull_back_mm)
    z0 = float(fk0.tip.z)
    x0, z0, yaw_fixed = _clamp_ik_target(x0, z0, yaw_fixed, vc)

    z_max_env = float(getattr(vc, "TARGET_Z_MAX_MM", 170.0))
    z_top = _scan_max_z_coarse_then_binary(x0, z0, yaw_fixed, z_max_env, scan_mm, prefer, vc)

    wmode = "delta_neutral" if wrist_stab else "absolute"

    def _fmt_line(r: VerticalSolveResult, zq: float, idx: str) -> str:
        clipped = ",".join(r.clip_notes) if r.clip_notes else "none"
        extra = ""
        if r.clip_notes:
            extra = f" raw_sh={r.servo_raw.shoulder}"
        return (
            f"  {idx} z={zq:6.2f} ik_ok={r.ik.ok} servo_feasible={r.ok} clips={clipped}{extra} "
            f"servo=({r.servo_clamped.wrist},{r.servo_clamped.elbow},{r.servo_clamped.base},{r.servo_clamped.shoulder})"
        )

    def _solve_at_z(zq: float, prev_cmd: ServoCommand) -> VerticalSolveResult:
        """
        Planar IK at z, then wrist.

        For delta_neutral stab: q_wrist cancels link pitch **relative to neutral** using the
        **actual** shoulder/elbow pose implied by prev_cmd (smoothed command). That matches how
        IK couples joints: wrist tracks the moving links instead of holding one angle computed
        from the ideal IK target while shoulder/elbow lag behind the LPF.
        """
        xc, zc, yc = _clamp_ik_target(x0, zq, yaw_fixed, vc)
        r0 = solve_vertical_plane(
            x_mm=xc,
            z_mm=zc,
            base_yaw_rad=yc,
            q_wrist_rad=0.0,
            prefer=prefer,
        )
        if not r0.ik.ok or not wrist_stab:
            return r0
        if wmode == "delta_neutral":
            m_act = servo_to_model(prev_cmd)
            qw = _wrist_stab_pitch_rad_delta_neutral(
                m_act.q_shoulder_rad, m_act.q_elbow_rad, yc, mv1
            )
            m = ModelJointState(
                base_yaw_rad=r0.model.base_yaw_rad,
                q_shoulder_rad=r0.model.q_shoulder_rad,
                q_elbow_rad=r0.model.q_elbow_rad,
                q_wrist_rad=qw,
            )
            raw = model_to_servo(m)
            cl, notes = clamp_servo(raw)
            ok = len(notes) == 0
            return VerticalSolveResult(
                ok=ok and r0.ik.ok,
                ik=r0.ik,
                model=m,
                servo_raw=raw,
                servo_clamped=cl,
                clip_notes=notes,
                message=("ok" if ok else "servo_limits_clipped:" + ",".join(notes)),
            )
        return _solve_vertical_with_optional_wrist_stab(
            xc,
            zc,
            yc,
            prefer,
            mv1,
            True,
            wrist_stab_mode="absolute",
        )

    print("Raise camera: neutral tip -> Cartesian +z; base yaw fixed; wrist=delta-from-neutral stab.")
    print(
        f"FK(0,0) tip x={fk0.tip.x:.2f} z={fk0.tip.z:.2f} mm | x={x0:.1f} mm "
        f"pull_back={float(args.pull_back_mm):.1f} mm | wrist_stab={wrist_stab} ({wmode}) | prefer={prefer}"
    )
    print(f"z_top={z_top:.3f} mm (refined; env z_max={z_max_env:.1f})")

    if not discrete:
        n_up = max(2, int(math.ceil(duration_up * hz)))
        n_dn = max(2, int(math.ceil(duration_down * hz))) if bool(args.return_down) else 0
    else:
        zs_path = _vertical_path_zs(z0, z_top, step_mm, bool(args.return_down))
        print(
            f"discrete: steps={len(zs_path)} step={step_mm:.1f} mm delay={delay:.2f} s "
            f"(use default smooth mode for simultaneous joint motion)"
        )

    if z_top > z0 + 1e-6:
        xc0, zc0, yc0 = _clamp_ik_target(x0, z0, yaw_fixed, vc)
        r_start = solve_vertical_plane(
            x_mm=xc0,
            z_mm=zc0,
            base_yaw_rad=yc0,
            q_wrist_rad=0.0,
            prefer=prefer,
        )
        zmid = z0 + 0.5 * (z_top - z0)
        xc1, zc1, yc1 = _clamp_ik_target(x0, zmid, yaw_fixed, vc)
        rp = solve_vertical_plane(
            x_mm=xc1,
            z_mm=zc1,
            base_yaw_rad=yc1,
            q_wrist_rad=0.0,
            prefer=prefer,
        )
        if (
            "clipped_shoulder_min" in rp.clip_notes
            and rp.servo_raw.shoulder < float(config.NEUTRAL_SHOULDER)
            and r_start.servo_raw.shoulder <= 0
        ):
            print(
                "NOTE: Mid-path shoulder may clamp at min; try --pull-back-mm for more shoulder range."
            )

    def _run_smooth_segment(
        z_schedule: list[float],
        n_frames: int,
        prev_cmd: ServoCommand,
        print_samples: bool,
        label: str,
        controller: RobotController | None,
    ) -> ServoCommand:
        """
        Fixed (x,z) from the schedule → straight vertical line in the plane.

        **Default (direct IK):** send the full IK `servo_clamped` every frame. All joints update
        from the same pose; small Δz per frame keeps motion smooth. Slew toward target + integer
        rounding made only one servo's degree tick at a time.

        **Optional --raise-slew:** proportional float sync toward IK (rate-limited), for hardware
        that cannot follow fast command streams.
        """
        if len(z_schedule) != n_frames + 1:
            raise ValueError("z_schedule length must be n_frames+1")
        cmd = prev_cmd
        if not raise_slew:
            for i in range(n_frames + 1):
                zq = z_schedule[i]
                r = _solve_at_z(zq, cmd)
                if not r.ik.ok:
                    print(f"ABORT: IK failed at z={zq:.2f} ({label})", file=sys.stderr)
                    raise RuntimeError("ik_fail")
                cmd = r.servo_clamped
                if controller is not None:
                    controller.send_servo(cmd)
                    time.sleep(dt)
                if print_samples and (
                    i == 0
                    or i == n_frames
                    or (n_frames > 8 and i % max(1, n_frames // 8) == 0)
                ):
                    print(_fmt_line(r, zq, f"{label} {i}/{n_frames}"))
            return cmd
        dt_sub = dt / float(max(1, substeps))
        cmd_f = servo_command_to_float_tuple(prev_cmd)
        for i in range(n_frames + 1):
            zq = z_schedule[i]
            cmd_int = float_tuple_to_servo_command(cmd_f)
            r = _solve_at_z(zq, cmd_int)
            if not r.ik.ok:
                print(f"ABORT: IK failed at z={zq:.2f} ({label})", file=sys.stderr)
                raise RuntimeError("ik_fail")
            tgt = r.servo_clamped
            for _ in range(max(1, substeps)):
                cmd_f = sync_step_servo_float_toward(cmd_f, tgt, dt_sub, max_dps_raise)
                cmd = float_tuple_to_servo_command(cmd_f)
                if controller is not None:
                    controller.send_servo(cmd)
                    time.sleep(dt_sub)
            if print_samples and (
                i == 0
                or i == n_frames
                or (n_frames > 8 and i % max(1, n_frames // 8) == 0)
            ):
                print(_fmt_line(r, zq, f"{label} {i}/{n_frames}"))
        return float_tuple_to_servo_command(cmd_f)

    neutral_cmd = ServoCommand(
        wrist=config.NEUTRAL_WRIST,
        elbow=config.NEUTRAL_ELBOW,
        base=config.NEUTRAL_BASE,
        shoulder=config.NEUTRAL_SHOULDER,
    )

    if not discrete:
        k_fine = args.arc_fine if args.arc_fine is not None else max(48, min(300, int(abs(z_top - z0) / 0.08)))
        use_arc = not bool(args.no_arc_length)
        print(
            f"smooth: hz={hz:.1f} dt={dt*1000:.1f}ms | up {duration_up:.1f}s ({n_up} frames) "
            f"{'+ down ' + str(round(duration_down, 1)) + 's' if bool(args.return_down) else ''} "
            f"| arc_z={use_arc} k_fine={k_fine} direct_ik={not raise_slew} "
            f"{'| slew substeps=' + str(substeps) + ' MAX_JOINT_DPS=' + str(max_dps_raise) if raise_slew else ''}"
        )
        z_sched_up = (
            _raise_camera_arc_z_schedule(z0, z_top, neutral_cmd, _solve_at_z, n_up, k_fine)
            if use_arc
            else _raise_camera_linear_z_schedule(z0, z_top, n_up)
        )
    else:
        z_sched_up = []

    if args.dry_run:
        if discrete:
            cmd_dr = neutral_cmd
            for i, zq in enumerate(_vertical_path_zs(z0, z_top, step_mm, bool(args.return_down))):
                r = _solve_at_z(zq, cmd_dr)
                print(_fmt_line(r, zq, f"{i+1:03d}"))
                cmd_dr = r.servo_clamped
        else:
            try:
                cmd = _run_smooth_segment(z_sched_up, n_up, neutral_cmd, True, "up", None)
                if bool(args.return_down):
                    z_sched_dn = (
                        _raise_camera_arc_z_schedule(z_top, z0, cmd, _solve_at_z, n_dn, k_fine)
                        if use_arc
                        else _raise_camera_linear_z_schedule(z_top, z0, n_dn)
                    )
                    _run_smooth_segment(z_sched_dn, n_dn, cmd, True, "dn", None)
            except RuntimeError:
                return 2
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
        time.sleep(0.3)

        if discrete:
            zs_path = _vertical_path_zs(z0, z_top, step_mm, bool(args.return_down))
            cmd = neutral_cmd
            cmd_f = servo_command_to_float_tuple(neutral_cmd) if raise_slew else None
            for i, zq in enumerate(zs_path):
                r = _solve_at_z(zq, cmd)
                print(_fmt_line(r, zq, f"{i+1:03d}/{len(zs_path)}"))
                if not r.ik.ok:
                    print("ABORT: IK failed mid-path.", file=sys.stderr)
                    controller.neutral()
                    return 2
                if raise_slew:
                    step_dt = max(dt, delay * 0.5)
                    cmd_f = sync_step_servo_float_toward(cmd_f, r.servo_clamped, step_dt, max_dps_raise)
                    cmd = float_tuple_to_servo_command(cmd_f)
                else:
                    cmd = r.servo_clamped
                controller.send_servo(cmd)
                time.sleep(delay)
        else:
            try:
                cmd = _run_smooth_segment(z_sched_up, n_up, neutral_cmd, True, "up", controller)
                if bool(args.return_down):
                    z_sched_dn = (
                        _raise_camera_arc_z_schedule(z_top, z0, cmd, _solve_at_z, n_dn, k_fine)
                        if use_arc
                        else _raise_camera_linear_z_schedule(z_top, z0, n_dn)
                    )
                    cmd = _run_smooth_segment(z_sched_dn, n_dn, cmd, True, "dn", controller)
            except RuntimeError:
                controller.neutral()
                return 2

        print("Returning to NEUTRAL…")
        controller.neutral()
        time.sleep(0.35)
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
    tr.add_argument("--disable-y-axis", action="store_true", help="disable vertical/Y correction; tune base only")
    tr.add_argument("--disable-x-axis", action="store_true", help="disable horizontal/X correction; tune vertical only")
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
        help=(
            "vertical line from NEUTRAL: default direct IK each frame (straight tip path); "
            "wrist=delta-from-neutral stab; optional --raise-slew. Use --discrete for stepped motion."
        ),
    )
    rc.add_argument("--port", default=config.SERIAL_DEFAULT_PORT)
    rc.add_argument(
        "--discrete",
        action="store_true",
        help="stepped z waypoints + delay-s (old behavior); default is smooth timed motion",
    )
    rc.add_argument(
        "--hz",
        type=float,
        default=100.0,
        help="outer control rate for smooth mode (default: 100); each frame uses --raise-substeps sends",
    )
    rc.add_argument(
        "--duration-up",
        type=float,
        default=12.0,
        help="seconds to interpolate z0 -> z_top in smooth mode (default: 12)",
    )
    rc.add_argument(
        "--duration-down",
        type=float,
        default=None,
        help="seconds for return z_top -> z0 (default: same as --duration-up)",
    )
    rc.add_argument(
        "--delay-s",
        type=float,
        default=0.9,
        help="only for --discrete: pause between poses (default: 0.9)",
    )
    rc.add_argument("--step-mm", type=float, default=2.0, help="only --discrete: z step (default: 2)")
    rc.add_argument("--scan-mm", type=float, default=0.25, help="coarse scan step before binary z_top refine")
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
        help="delta-from-neutral wrist (level ref at neutral); --no-wrist-stab fixes q_wrist=0",
    )
    rc.add_argument(
        "--no-arc-length",
        action="store_true",
        help="use linear z vs time (default: arc-length scheduling for even joint-space motion)",
    )
    rc.add_argument(
        "--raise-slew",
        action="store_true",
        help="rate-limit toward IK with float sync (default: send full IK pose every frame)",
    )
    rc.add_argument(
        "--raise-substeps",
        type=int,
        default=1,
        help="coupled IK steps per outer frame toward the same IK target (default: 1)",
    )
    rc.add_argument(
        "--raise-speed",
        type=float,
        default=1.75,
        help="multiplier on MAX_JOINT_DPS for this move (default: 1.75, capped per joint)",
    )
    rc.add_argument(
        "--arc-fine",
        type=int,
        default=None,
        help="fine samples for arc-length precompute (default: auto from z span)",
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
