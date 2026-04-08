"""
V1 persistent visual-servo motion controller: targets, IK, wrist stabilization, smoothing.

Consumes vision-layer signals only (no detector). See VisionMeasurement.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Literal

from . import config, kinematics, motion_config_v1 as mv1_defaults
from .controller import solve_vertical_plane
from .mapping import ServoCommand, clamp_servo, servo_to_model
from .motion_smooth import (
    JointRateState,
    accel_limit_delta,
    clamp,
    command_delta_max_deg,
    lowpass_scalar,
    rate_limit_servo_deg_per_sec,
    step_toward,
)


@dataclass(frozen=True)
class VisionMeasurement:
    """Layer-1 signals after face_tracking filtering (detector unchanged)."""

    face_detected: bool
    err_x_norm: float
    err_y_norm: float
    corr_x_norm_raw: float
    corr_y_norm_raw: float
    filt_face_w: float | None
    face_w_px: int
    t_seconds: float


class MotionControllerV1:
    """
    Persistent motion targets + IK + wrist stab + rate limits + optional posture rebalance.
    """

    def __init__(
        self,
        vc: object,
        mv1: object | None = None,
        *,
        fk0_tip_x: float,
        fk0_tip_z: float,
        base_yaw_lim: float,
    ) -> None:
        self._vc = vc
        self._mv1 = mv1 if mv1 is not None else mv1_defaults
        self.target_x_mm = float(fk0_tip_x)
        self.target_z_mm = float(fk0_tip_z)
        self.base_yaw_rad = 0.0
        self.base_yaw_lim = float(base_yaw_lim)

        self.face_lock_frames = 0
        self.engage = 0.0
        self.x_ramp = float(getattr(vc, "RAMP_MIN", 1.0))
        self.y_ramp = float(getattr(vc, "RAMP_MIN", 1.0))

        self.base_pid_i = 0.0
        self.base_pid_prev_e = 0.0
        self.base_pid_d = 0.0
        self.base_zero_cross_hold = 0

        self.y_pid_i = 0.0
        self.y_pid_prev_e = 0.0
        self.y_pid_d = 0.0

        self.wrist_trim_state = 0.0
        self.wrist_trim_last = 0
        self.elbow_assist_state = 0.0
        self.elbow_assist_last = 0
        self.elbow_cmd_last = int(config.NEUTRAL_ELBOW)
        self.shoulder_dist_state = 0.0
        self.shoulder_dist_last = 0

        self.last_valid_cmd = ServoCommand(
            wrist=config.NEUTRAL_WRIST,
            elbow=config.NEUTRAL_ELBOW,
            base=config.NEUTRAL_BASE,
            shoulder=config.NEUTRAL_SHOULDER,
        )
        self._cmd_lpf = self.last_valid_cmd
        self._joint_vel = JointRateState()
        self.last_ik_solution: Literal["elbow_up", "elbow_down", "none"] = "none"
        self.last_face_seen_t = time.time()
        self.no_face_neutral_sent = False

        self.ik_status = "init"
        self.ik_clip_notes: list[str] = []
        self.dist_err_px = 0.0
        self.z_err_mm = 0.0
        self.y_for_z = 0.0

    def _mv(self, name: str, default: Any) -> Any:
        return getattr(self._mv1, name, getattr(mv1_defaults, name, default))

    @staticmethod
    def _hard_clamp_base(cmd: ServoCommand) -> ServoCommand:
        """Never emit a base command outside configured servo bounds."""
        return ServoCommand(
            wrist=cmd.wrist,
            elbow=cmd.elbow,
            base=int(clamp(float(cmd.base), float(config.SERVO_BASE_MIN), float(config.SERVO_BASE_MAX))),
            shoulder=cmd.shoulder,
        )

    def _smooth_and_limit(self, cmd: ServoCommand, _t: float, dt: float) -> ServoCommand:
        mv = self._mv1
        max_dps = getattr(mv, "MAX_JOINT_DPS", mv1_defaults.MAX_JOINT_DPS)
        alpha = float(getattr(mv, "COMMAND_LPF_ALPHA", mv1_defaults.COMMAND_LPF_ALPHA))
        wrist_alpha = float(getattr(mv, "WRIST_COMMAND_LPF_ALPHA", alpha))
        wrist_alpha = clamp(wrist_alpha, 0.0, 1.0)
        prev = self._cmd_lpf
        # Low-pass on float precursors
        smoothed = ServoCommand(
            wrist=int(round(lowpass_scalar(float(prev.wrist), float(cmd.wrist), wrist_alpha))),
            elbow=int(round(lowpass_scalar(float(prev.elbow), float(cmd.elbow), alpha))),
            base=int(round(lowpass_scalar(float(prev.base), float(cmd.base), alpha))),
            shoulder=int(round(lowpass_scalar(float(prev.shoulder), float(cmd.shoulder), alpha))),
        )
        wrist_hold_deg = max(0.0, float(getattr(mv, "WRIST_HOLD_BAND_DEG", 0.0)))
        if abs(float(smoothed.wrist) - float(self.last_valid_cmd.wrist)) <= wrist_hold_deg:
            smoothed = ServoCommand(
                wrist=self.last_valid_cmd.wrist,
                elbow=smoothed.elbow,
                base=smoothed.base,
                shoulder=smoothed.shoulder,
            )
        self._cmd_lpf = smoothed
        wrist_max_dps = float(getattr(mv, "WRIST_MAX_DPS", float(max_dps[0])))
        dps = (wrist_max_dps, float(max_dps[1]), float(max_dps[2]), float(max_dps[3]))
        out = rate_limit_servo_deg_per_sec(self.last_valid_cmd, smoothed, dt, dps)
        max_a = getattr(mv, "MAX_JOINT_ACCEL_DPS2", mv1_defaults.MAX_JOINT_ACCEL_DPS2)
        if max(max_a) > 1e-6:
            out, self._joint_vel = accel_limit_delta(
                self._joint_vel, out, self.last_valid_cmd, dt, max_a
            )
        return out

    def process_proportional(
        self,
        vm: VisionMeasurement,
        *,
        corr_x_ctrl: float,
        corr_y_vert: float,
        dt: float,
    ) -> ServoCommand:
        vc = self._vc
        d_base = int(round(vc.SIGN_ERROR_X_BASE * corr_x_ctrl * vc.TRACK_GAIN_BASE_DEG))
        d_sh = int(round(vc.SIGN_ERROR_Y_SHOULDER * corr_y_vert * vc.TRACK_GAIN_SHOULDER_DEG))
        d_el = int(round(vc.SIGN_ERROR_Y_ELBOW * corr_y_vert * vc.TRACK_GAIN_ELBOW_DEG))
        cmd = ServoCommand(
            wrist=config.NEUTRAL_WRIST,
            elbow=config.NEUTRAL_ELBOW + d_el,
            base=config.NEUTRAL_BASE + d_base,
            shoulder=config.NEUTRAL_SHOULDER + d_sh,
        )
        cl, _ = clamp_servo(cmd)
        self.elbow_cmd_last = cl.elbow
        out = self._hard_clamp_base(self._smooth_and_limit(cl, vm.t_seconds, dt))
        self.last_valid_cmd = out
        return out

    def process_ik(
        self,
        vm: VisionMeasurement,
        *,
        corr_x_ctrl: float,
        corr_y_vert: float,
        corr_y_ik: float,
        corr_y_norm: float,
        engage: float,
        dt: float,
    ) -> ServoCommand:
        vc = self._vc
        shoulder_dist_assist_deg = 0
        max_base_step = float(getattr(vc, "MAX_BASE_YAW_STEP_RAD", 0.08))
        # First-lock overshoot guard: limit yaw step on initial acquisition and ramp to full cap.
        lock_frames = max(0, int(getattr(vc, "BASE_FIRST_LOCK_STEP_FRAMES", 10)))
        lock_scale = clamp(float(getattr(vc, "BASE_FIRST_LOCK_STEP_SCALE", 0.45)), 0.05, 1.0)
        if lock_frames > 0 and self.face_lock_frames < lock_frames:
            u = float(self.face_lock_frames) / float(lock_frames)
            step_scale = lock_scale + (1.0 - lock_scale) * clamp(u, 0.0, 1.0)
            max_base_step *= step_scale
        x_ctrl_mode = str(getattr(vc, "BASE_X_CTRL_MODE", "p")).strip().lower()
        if x_ctrl_mode == "pid":
            e = float(vc.SIGN_ERROR_X_BASE) * corr_x_ctrl
            kp = float(getattr(vc, "BASE_PID_KP", 0.07))
            ki = float(getattr(vc, "BASE_PID_KI", 0.0))
            kd = float(getattr(vc, "BASE_PID_KD", 0.02))
            i_clamp = max(0.0, float(getattr(vc, "BASE_PID_I_CLAMP", 2.0)))
            d_alpha = clamp(float(getattr(vc, "BASE_PID_D_ALPHA", 0.35)), 0.0, 1.0)

            crossed_zero = self.base_pid_prev_e * e < 0.0
            if crossed_zero:
                self.base_pid_i = 0.0
            self.base_pid_i += e
            self.base_pid_i = clamp(self.base_pid_i, -i_clamp, i_clamp)
            d_raw = e - self.base_pid_prev_e
            self.base_pid_d = (1.0 - d_alpha) * self.base_pid_d + d_alpha * d_raw
            base_unclamped = kp * e + ki * self.base_pid_i + kd * self.base_pid_d
            base_step = clamp(base_unclamped, -max_base_step, max_base_step)
            if abs(base_unclamped - base_step) > 1e-9 and abs(e) > 1e-9:
                if (base_unclamped > 0.0 and e > 0.0) or (base_unclamped < 0.0 and e < 0.0):
                    self.base_pid_i -= e
            if crossed_zero:
                cross_brake = clamp(float(getattr(vc, "BASE_PID_ZERO_CROSS_BRAKE", 0.45)), 0.0, 1.0)
                base_step *= cross_brake
                self.base_zero_cross_hold = max(0, int(getattr(vc, "BASE_PID_ZERO_CROSS_HOLD_FRAMES", 2)))
            pre_err = float(getattr(vc, "BASE_PID_PREBRAKE_ERROR", 0.26))
            if abs(e) < pre_err:
                pre_scale = clamp(float(getattr(vc, "BASE_PID_PREBRAKE_SCALE", 0.55)), 0.0, 1.0)
                base_step *= pre_scale
            near_err = abs(e) < float(getattr(vc, "BASE_PID_NEAR_ERROR", 0.10))
            if near_err:
                near_scale = clamp(float(getattr(vc, "BASE_PID_NEAR_STEP_SCALE", 0.35)), 0.0, 1.0)
                base_step *= near_scale
            if self.base_zero_cross_hold > 0 and near_err:
                base_step = 0.0
                self.base_zero_cross_hold -= 1
            self.base_pid_prev_e = e
        else:
            base_step = vc.SIGN_ERROR_X_BASE * corr_x_ctrl * float(getattr(vc, "TRACK_BASE_RAD_PER_NORM", 0.04))

        base_step = clamp(base_step, -max_base_step, max_base_step)
        self.base_yaw_rad += base_step
        self.base_yaw_rad = clamp(self.base_yaw_rad, -self.base_yaw_lim, self.base_yaw_lim)

        y_for_z = corr_y_ik + float(getattr(vc, "SIGN_ERROR_X_TO_Z", 1.0)) * float(
            getattr(vc, "TRACK_Z_FROM_X_MIX", 0.0)
        ) * corr_x_ctrl
        y_for_z = clamp(y_for_z, -1.0, 1.0)
        self.y_for_z = y_for_z

        y_ctrl_mode = str(getattr(vc, "Y_Z_CTRL_MODE", "p")).strip().lower()
        ye = float(vc.SIGN_ERROR_Y_SHOULDER) * y_for_z
        z_step = 0.0
        if y_ctrl_mode == "pid":
            ykp = float(getattr(vc, "Y_PID_KP", 1.8))
            yki = float(getattr(vc, "Y_PID_KI", 0.03))
            ykd = float(getattr(vc, "Y_PID_KD", 0.8))
            yi_clamp = max(0.0, float(getattr(vc, "Y_PID_I_CLAMP", 2.5)))
            yd_alpha = clamp(float(getattr(vc, "Y_PID_D_ALPHA", 0.35)), 0.0, 1.0)

            self.y_pid_i += ye
            self.y_pid_i = clamp(self.y_pid_i, -yi_clamp, yi_clamp)
            y_d_raw = ye - self.y_pid_prev_e
            self.y_pid_d = (1.0 - yd_alpha) * self.y_pid_d + yd_alpha * y_d_raw
            z_unclamped = ykp * ye + yki * self.y_pid_i + ykd * self.y_pid_d
            z_step = z_unclamped
            z_cap = float(getattr(vc, "MAX_Z_STEP_MM", 10.0))
            z_tmp = clamp(z_unclamped, -z_cap, z_cap)
            if abs(z_unclamped - z_tmp) > 1e-9 and abs(ye) > 1e-9:
                if (z_unclamped > 0.0 and ye > 0.0) or (z_unclamped < 0.0 and ye < 0.0):
                    self.y_pid_i -= ye
            self.y_pid_prev_e = ye
        else:
            z_step = vc.SIGN_ERROR_Y_SHOULDER * y_for_z * float(getattr(vc, "TRACK_Z_MM_PER_NORM", 10.0))

        z_step = clamp(z_step, -float(getattr(vc, "MAX_Z_STEP_MM", 10.0)), float(getattr(vc, "MAX_Z_STEP_MM", 10.0)))
        self.target_z_mm += z_step

        x_step = vc.SIGN_ERROR_X_BASE * corr_x_ctrl * float(getattr(vc, "TRACK_X_MM_PER_NORM", 0.0))
        x_step = clamp(x_step, -float(getattr(vc, "MAX_X_STEP_MM", 3.0)), float(getattr(vc, "MAX_X_STEP_MM", 3.0)))
        self.target_x_mm += x_step

        self.dist_err_px = 0.0
        if bool(getattr(vc, "DIST_CONTROL_ENABLE", False)):
            desired_face_w = float(getattr(vc, "DESIRED_FACE_WIDTH_PX", 160.0))
            dist_db = max(0.0, float(getattr(vc, "DIST_DEADBAND_PX", 10.0)))
            dist_err_limit = max(1.0, float(getattr(vc, "DIST_ERR_CLAMP_PX", 120.0)))
            dist_mm_per_px = float(getattr(vc, "DIST_MM_PER_PX", 0.35))
            dist_max_step = max(0.1, float(getattr(vc, "DIST_MAX_STEP_MM", 8.0)))
            dist_z_mm_per_px = float(getattr(vc, "DIST_Z_MM_PER_PX", 0.0))
            dist_z_max_step = max(0.0, float(getattr(vc, "DIST_Z_MAX_STEP_MM", 0.0)))

            dist_allowed = (not bool(getattr(vc, "DIST_ENABLE_AFTER_LOCK", True))) or (engage >= 0.95)
            if dist_allowed and vm.filt_face_w is not None:
                measured_face_w = float(vm.filt_face_w)
                self.dist_err_px = desired_face_w - measured_face_w
                if abs(self.dist_err_px) < dist_db:
                    self.dist_err_px = 0.0
                self.dist_err_px = clamp(self.dist_err_px, -dist_err_limit, dist_err_limit)
                dist_step_mm = clamp(
                    float(getattr(vc, "DIST_SIGN_X", 1.0)) * self.dist_err_px * dist_mm_per_px,
                    -dist_max_step,
                    dist_max_step,
                )
                self.target_x_mm += dist_step_mm
                dist_step_z_mm = clamp(
                    float(getattr(vc, "DIST_SIGN_Z", 1.0)) * self.dist_err_px * dist_z_mm_per_px,
                    -dist_z_max_step,
                    dist_z_max_step,
                )
                self.target_z_mm += dist_step_z_mm

        if bool(getattr(vc, "DIST_SHOULDER_ASSIST_ENABLE", False)):
            shoulder_dist_assist_deg = int(
                round(
                    float(getattr(vc, "DIST_SIGN_SHOULDER", 1.0))
                    * self.dist_err_px
                    * float(getattr(vc, "DIST_SHOULDER_DEG_PER_PX", 0.0))
                )
            )
            shoulder_dist_max = max(0, int(getattr(vc, "DIST_SHOULDER_MAX_DEG", 28)))
            shoulder_dist_assist_deg = max(-shoulder_dist_max, min(shoulder_dist_max, shoulder_dist_assist_deg))
            shoulder_dist_alpha = clamp(float(getattr(vc, "DIST_SHOULDER_SMOOTH_ALPHA", 0.15)), 0.0, 1.0)
            self.shoulder_dist_state = (
                (1.0 - shoulder_dist_alpha) * self.shoulder_dist_state
                + shoulder_dist_alpha * float(shoulder_dist_assist_deg)
            )
            shoulder_dist_assist_deg = int(round(self.shoulder_dist_state))
            shoulder_dist_step_max = max(1, int(getattr(vc, "DIST_SHOULDER_MAX_STEP_PER_FRAME_DEG", 1)))
            shoulder_dist_assist_deg = int(
                round(
                    step_toward(
                        float(self.shoulder_dist_last),
                        float(shoulder_dist_assist_deg),
                        float(shoulder_dist_step_max),
                    )
                )
            )
            self.shoulder_dist_last = shoulder_dist_assist_deg
        else:
            shoulder_dist_assist_deg = 0

        self.target_x_mm = max(
            float(getattr(vc, "TARGET_X_MIN_MM", 100.0)),
            min(float(getattr(vc, "TARGET_X_MAX_MM", 230.0)), self.target_x_mm),
        )
        self.target_z_mm = max(
            float(getattr(vc, "TARGET_Z_MIN_MM", 0.0)),
            min(float(getattr(vc, "TARGET_Z_MAX_MM", 190.0)), self.target_z_mm),
        )

        prefer_default = str(getattr(vc, "IK_PREFER", "elbow_up"))
        prefer: Literal["elbow_up", "elbow_down"] = (
            "elbow_down" if prefer_default == "elbow_down" else "elbow_up"
        )
        ik_prefer = kinematics.resolve_ik_preference(
            prefer,
            self.last_ik_solution,
            corr_y_norm,
            switch_threshold=float(self._mv("IK_BRANCH_SWITCH_ERR_NORM", 0.12)),
        )

        wrist_trim_deg, shoulder_assist_deg, elbow_assist_deg = self._legacy_assists(corr_y_vert)

        cmd = self._solve_ik_chain(
            vc=vc,
            ik_prefer=ik_prefer,
            wrist_trim_deg=wrist_trim_deg,
            shoulder_assist_deg=shoulder_assist_deg,
            elbow_assist_deg=elbow_assist_deg,
            shoulder_dist_assist_deg=shoulder_dist_assist_deg,
            z_step=z_step,
        )

        # Lower-bound takeover: when vertical chain is pinned at lower limit, let wrist
        # handle further downward correction instead of fighting clipped shoulder/elbow.
        if bool(getattr(vc, "LOWER_BOUND_WRIST_ONLY_ENABLE", False)):
            z_min = float(getattr(vc, "TARGET_Z_MIN_MM", 0.0))
            at_lower = self.target_z_mm <= z_min + 1e-3
            lower_clip = (
                "clipped_elbow_max" in self.ik_clip_notes and "clipped_shoulder_min" in self.ik_clip_notes
            )
            pin_margin = max(0, int(getattr(vc, "LOWER_BOUND_PIN_MARGIN_DEG", 2)))
            elbow_pinned = int(self.last_valid_cmd.elbow) >= int(config.SERVO_ELBOW_MAX) - pin_margin
            shoulder_pinned = int(self.last_valid_cmd.shoulder) <= int(config.SERVO_SHOULDER_MIN) + pin_margin
            lower_pinned = elbow_pinned and shoulder_pinned
            # Use solved z direction (after sign/config) so takeover works regardless of
            # corr_y sign conventions.
            down_eps_mm = max(0.0, float(getattr(vc, "LOWER_BOUND_WRIST_ONLY_DOWN_ZSTEP_EPS_MM", 0.02)))
            wants_down = float(z_step) < -down_eps_mm
            if wants_down and (at_lower or lower_clip or lower_pinned):
                max_deg = max(0.0, float(getattr(vc, "LOWER_BOUND_WRIST_ONLY_MAX_DEG", 20.0)))
                gain = max(0.0, float(getattr(vc, "LOWER_BOUND_WRIST_ONLY_GAIN_DEG_PER_NORM", 80.0)))
                down_deg = clamp(abs(self.y_for_z) * gain, 0.0, max_deg)
                wrist_cmd = int(round(float(config.NEUTRAL_WRIST) - down_deg))
                cmd = ServoCommand(
                    wrist=wrist_cmd,
                    elbow=self.last_valid_cmd.elbow,
                    base=cmd.base,
                    shoulder=self.last_valid_cmd.shoulder,
                )
                cmd, _ = clamp_servo(cmd)
                self.ik_status = f"{self.ik_status}|lower_bound_wrist_only"

        # Upward monotonic guard:
        # If Y asks up, never let shoulder command decrease vs last command due to clipped/raw solve.
        if bool(getattr(vc, "UPWARD_MONOTONIC_GUARD_ENABLE", True)):
            up_eps = max(0.0, float(getattr(vc, "UPWARD_MONOTONIC_EPS_NORM", 0.03)))
            if corr_y_ik > up_eps:
                new_sh = max(int(cmd.shoulder), int(self.last_valid_cmd.shoulder))
                new_el = int(cmd.elbow)
                # If still pinned at lower-bound clip pair, force tiny coupled escape.
                if (
                    "clipped_elbow_max" in self.ik_clip_notes
                    and "clipped_shoulder_min" in self.ik_clip_notes
                ):
                    step = max(1, int(getattr(vc, "UPWARD_UNSTICK_STEP_DEG", 1)))
                    new_sh = max(new_sh, int(self.last_valid_cmd.shoulder) + step)
                    new_el = min(new_el, int(self.last_valid_cmd.elbow) - step)
                cmd = ServoCommand(
                    wrist=cmd.wrist,
                    elbow=new_el,
                    base=cmd.base,
                    shoulder=new_sh,
                )
                cmd, _ = clamp_servo(cmd)

        # Proximal-first vertical motion: keep wrist from leading while shoulder/elbow
        # are still moving toward the new IK target. This makes the chain "lift first"
        # and wrist follow after proximal joints settle.
        if bool(getattr(vc, "VERTICAL_PROXIMAL_FIRST_ENABLE", True)):
            shoulder_thresh = max(
                0.0, float(getattr(vc, "VERTICAL_PROXIMAL_FIRST_SHOULDER_THRESH_DEG", 1.0))
            )
            elbow_thresh = max(
                0.0, float(getattr(vc, "VERTICAL_PROXIMAL_FIRST_ELBOW_THRESH_DEG", 1.0))
            )
            shoulder_delta = abs(float(cmd.shoulder) - float(self.last_valid_cmd.shoulder))
            elbow_delta = abs(float(cmd.elbow) - float(self.last_valid_cmd.elbow))
            lower_bound_wrist_only_active = "lower_bound_wrist_only" in self.ik_status
            if (
                not lower_bound_wrist_only_active
                and (shoulder_delta > shoulder_thresh or elbow_delta > elbow_thresh)
            ):
                cmd = ServoCommand(
                    wrist=self.last_valid_cmd.wrist,
                    elbow=cmd.elbow,
                    base=cmd.base,
                    shoulder=cmd.shoulder,
                )
                cmd, _ = clamp_servo(cmd)

        out = self._hard_clamp_base(self._smooth_and_limit(cmd, vm.t_seconds, dt))
        self.last_valid_cmd = out
        return out

    def _legacy_assists(self, corr_y_vert: float) -> tuple[int, int, int]:
        vc = self._vc
        shoulder_assist_deg = int(
            round(
                float(getattr(vc, "SIGN_ERROR_Y_SHOULDER", 1.0))
                * corr_y_vert
                * float(getattr(vc, "TRACK_SHOULDER_ASSIST_DEG_PER_NORM", 0.0))
            )
        )
        shoulder_assist_max = max(0, int(getattr(vc, "TRACK_SHOULDER_ASSIST_MAX_DEG", 0)))
        shoulder_assist_deg = max(-shoulder_assist_max, min(shoulder_assist_max, shoulder_assist_deg))

        elbow_assist_deg = int(
            round(
                float(getattr(vc, "SIGN_ERROR_Y_ELBOW", 1.0))
                * corr_y_vert
                * float(getattr(vc, "TRACK_ELBOW_ASSIST_DEG_PER_NORM", 0.0))
            )
        )
        elbow_assist_max = max(0, int(getattr(vc, "TRACK_ELBOW_ASSIST_MAX_DEG", 0)))
        elbow_assist_deg = max(-elbow_assist_max, min(elbow_assist_max, elbow_assist_deg))
        elbow_alpha = clamp(float(getattr(vc, "ELBOW_SMOOTH_ALPHA", 0.25)), 0.0, 1.0)
        self.elbow_assist_state = (1.0 - elbow_alpha) * self.elbow_assist_state + elbow_alpha * float(
            elbow_assist_deg
        )
        elbow_assist_deg = int(round(self.elbow_assist_state))
        elbow_step_max = max(1, int(getattr(vc, "ELBOW_MAX_STEP_PER_FRAME_DEG", 3)))
        elbow_assist_deg = int(
            round(step_toward(float(self.elbow_assist_last), float(elbow_assist_deg), float(elbow_step_max)))
        )
        self.elbow_assist_last = elbow_assist_deg

        wrist_cmd = (
            float(getattr(vc, "SIGN_ERROR_Y_WRIST", 1.0))
            * corr_y_vert
            * float(getattr(vc, "TRACK_WRIST_DEG_PER_NORM", 0.8))
        )
        wrist_deadband = max(0.0, float(getattr(vc, "TRACK_WRIST_DEADBAND_NORM", 0.0)))
        if abs(corr_y_vert) < wrist_deadband:
            wrist_cmd = 0.0
        wrist_trim_deg = int(round(wrist_cmd))
        wrist_min_step = max(0, int(getattr(vc, "TRACK_WRIST_MIN_STEP_DEG", 0)))
        if wrist_trim_deg == 0 and abs(corr_y_vert) > 1e-6 and wrist_min_step > 0:
            wrist_trim_deg = wrist_min_step if wrist_cmd > 0.0 else -wrist_min_step
        wrist_max_trim = max(0, int(getattr(vc, "TRACK_WRIST_MAX_TRIM_DEG", 35)))
        wrist_trim_deg = max(-wrist_max_trim, min(wrist_max_trim, wrist_trim_deg))
        wrist_alpha = clamp(float(getattr(vc, "WRIST_SMOOTH_ALPHA", 0.25)), 0.0, 1.0)
        self.wrist_trim_state = (
            (1.0 - wrist_alpha) * self.wrist_trim_state + wrist_alpha * float(wrist_trim_deg)
        )
        wrist_trim_deg = int(round(self.wrist_trim_state))
        wrist_step_max = max(1, int(getattr(vc, "WRIST_MAX_STEP_PER_FRAME_DEG", 4)))
        wrist_trim_deg = int(
            round(step_toward(float(self.wrist_trim_last), float(wrist_trim_deg), float(wrist_step_max)))
        )
        self.wrist_trim_last = wrist_trim_deg

        return wrist_trim_deg, shoulder_assist_deg, elbow_assist_deg

    def _solve_ik_chain(
        self,
        *,
        vc: object,
        ik_prefer: Literal["elbow_up", "elbow_down"],
        wrist_trim_deg: int,
        shoulder_assist_deg: int,
        elbow_assist_deg: int,
        shoulder_dist_assist_deg: int,
        z_step: float,
    ) -> ServoCommand:
        tx = self.target_x_mm
        tz = self.target_z_mm
        rebal_iters = int(self._mv("REBALANCE_MAX_ITER", 2))
        dz = float(self._mv("REBALANCE_TARGET_Z_MM", 2.5))
        comfort = float(self._mv("WRIST_COMFORT_HALF_SPAN_DEG", 40.0))
        trim_mode = str(self._mv("WRIST_TRIM_MODE", "stab")).strip().lower()

        cmd_out: ServoCommand | None = None
        for iteration in range(rebal_iters + 1):
            solved = solve_vertical_plane(
                x_mm=tx,
                z_mm=tz,
                base_yaw_rad=self.base_yaw_rad,
                q_wrist_rad=0.0,
                prefer=ik_prefer,
            )
            filtered_notes = [n for n in solved.clip_notes if not n.startswith("clipped_base_")]
            if solved.ik.ok:
                self.ik_status = "ok" if not filtered_notes else "servo_limits_clipped:" + ",".join(filtered_notes)
            else:
                self.ik_status = solved.message
            self.ik_clip_notes = filtered_notes

            # If upward demand pushed target beyond reachable geometry, back off z and retry.
            if (not solved.ik.ok) and z_step > 0:
                backoff = max(abs(float(z_step)), max(0.5, float(dz)))
                self.target_z_mm -= backoff
                self.target_z_mm = max(
                    float(getattr(vc, "TARGET_Z_MIN_MM", 0.0)),
                    min(float(getattr(vc, "TARGET_Z_MAX_MM", 190.0)), self.target_z_mm),
                )
                tz = self.target_z_mm
                continue

            if "clipped_shoulder_min" in solved.clip_notes and z_step > 0:
                self.target_z_mm -= z_step
                self.target_z_mm = max(
                    float(getattr(vc, "TARGET_Z_MIN_MM", 0.0)),
                    min(float(getattr(vc, "TARGET_Z_MAX_MM", 190.0)), self.target_z_mm),
                )
                tz = self.target_z_mm
                continue

            vertical_ok = len(filtered_notes) == 0
            q_wrist_use = 0.0
            if trim_mode != "legacy" and solved.ik.ok:
                fk0 = kinematics.forward_kinematics(0.0, 0.0)
                fk = kinematics.forward_kinematics(solved.model.q_shoulder_rad, solved.model.q_elbow_rad)
                s0 = fk0.theta1_abs + fk0.theta2_abs
                s1 = fk.theta1_abs + fk.theta2_abs
                desired = float(self._mv("DESIRED_CAMERA_PITCH_RAD", 0.0))
                mount = float(self._mv("CAMERA_MOUNT_OFFSET_RAD", 0.0))
                kg = float(self._mv("BASE_YAW_COUPLING_GAIN", 0.0))
                gp = float(self._mv("WRIST_STAB_LINK_PITCH_GAIN", 1.0))
                q_wrist_use = (
                    desired + gp * (s1 - s0) - mount - kg * float(self.base_yaw_rad)
                )

            solved2 = solve_vertical_plane(
                x_mm=tx,
                z_mm=tz,
                base_yaw_rad=self.base_yaw_rad,
                q_wrist_rad=q_wrist_use,
                prefer=ik_prefer,
            )
            if (not solved2.ik.ok) and z_step > 0:
                backoff = max(abs(float(z_step)), max(0.5, float(dz)))
                self.target_z_mm -= backoff
                self.target_z_mm = max(
                    float(getattr(vc, "TARGET_Z_MIN_MM", 0.0)),
                    min(float(getattr(vc, "TARGET_Z_MAX_MM", 190.0)), self.target_z_mm),
                )
                tz = self.target_z_mm
                continue
            if solved2.ik.ok:
                self.last_ik_solution = solved2.ik.solution

            if solved2.ik.ok:
                if vertical_ok or bool(getattr(vc, "IK_ACCEPT_CLAMPED", True)):
                    raw = solved2.servo_clamped
                    wrist_val = raw.wrist + (wrist_trim_deg if trim_mode == "legacy" else 0)
                    cmd_try = ServoCommand(
                        wrist=wrist_val,
                        elbow=raw.elbow + elbow_assist_deg,
                        base=raw.base,
                        shoulder=raw.shoulder + shoulder_assist_deg + shoulder_dist_assist_deg,
                    )
                    cmd_try, _ = clamp_servo(cmd_try)
                elif bool(getattr(vc, "IK_HOLD_LAST_ON_FAIL", True)):
                    # Prefer the latest reachable planar IK pose over "hold last"; this keeps
                    # shoulder/elbow advancing when a single frame's refined solve is invalid.
                    if solved.ik.ok:
                        raw = solved.servo_clamped
                        wrist_val = raw.wrist + (wrist_trim_deg if trim_mode == "legacy" else 0)
                        cmd_try = ServoCommand(
                            wrist=wrist_val,
                            elbow=raw.elbow + elbow_assist_deg,
                            base=raw.base,
                            shoulder=raw.shoulder + shoulder_assist_deg + shoulder_dist_assist_deg,
                        )
                        cmd_try, _ = clamp_servo(cmd_try)
                    else:
                        cmd_try = ServoCommand(
                            wrist=config.NEUTRAL_WRIST + wrist_trim_deg,
                            elbow=self.last_valid_cmd.elbow + elbow_assist_deg,
                            base=solved2.servo_clamped.base,
                            shoulder=self.last_valid_cmd.shoulder
                            + shoulder_assist_deg
                            + shoulder_dist_assist_deg,
                        )
                        cmd_try, _ = clamp_servo(cmd_try)
                else:
                    cmd_try = ServoCommand(
                        wrist=config.NEUTRAL_WRIST + wrist_trim_deg,
                        elbow=solved2.servo_clamped.elbow + elbow_assist_deg,
                        base=solved2.servo_clamped.base,
                        shoulder=solved2.servo_clamped.shoulder
                        + shoulder_assist_deg
                        + shoulder_dist_assist_deg,
                    )
                    cmd_try, _ = clamp_servo(cmd_try)
            else:
                cmd_try = self.last_valid_cmd

            est_model = servo_to_model(cmd_try)
            est_fk = kinematics.forward_kinematics(est_model.q_shoulder_rad, est_model.q_elbow_rad)
            self.z_err_mm = tz - est_fk.tip.z
            if bool(getattr(vc, "ZERR_SHOULDER_ASSIST_ENABLE", True)):
                shoulder_zerr_assist_deg = int(
                    round(
                        float(getattr(vc, "ZERR_SIGN_SHOULDER", 1.0))
                        * self.z_err_mm
                        * float(getattr(vc, "ZERR_SHOULDER_DEG_PER_MM", 0.0))
                    )
                )
                shoulder_zerr_max = max(0, int(getattr(vc, "ZERR_SHOULDER_MAX_DEG", 35)))
                shoulder_zerr_assist_deg = max(
                    -shoulder_zerr_max, min(shoulder_zerr_max, shoulder_zerr_assist_deg)
                )
                cmd_try = ServoCommand(
                    wrist=cmd_try.wrist,
                    elbow=cmd_try.elbow,
                    base=cmd_try.base,
                    shoulder=cmd_try.shoulder + shoulder_zerr_assist_deg,
                )
                cmd_try, _ = clamp_servo(cmd_try)

            elbow_cmd_step = max(1, int(getattr(vc, "ELBOW_CMD_MAX_STEP_PER_FRAME_DEG", 2)))
            elbow_cmd = int(
                round(step_toward(float(self.elbow_cmd_last), float(cmd_try.elbow), float(elbow_cmd_step)))
            )
            cmd_try = ServoCommand(
                wrist=cmd_try.wrist, elbow=elbow_cmd, base=cmd_try.base, shoulder=cmd_try.shoulder
            )
            cmd_try, _ = clamp_servo(cmd_try)
            self.elbow_cmd_last = cmd_try.elbow

            if trim_mode != "legacy" and solved2.ik.ok and iteration < rebal_iters:
                mid = float(config.NEUTRAL_WRIST)
                if abs(float(cmd_try.wrist) - mid) > comfort:
                    if float(cmd_try.wrist) > mid:
                        tz -= dz
                    else:
                        tz += dz
                    self.target_z_mm = max(
                        float(getattr(vc, "TARGET_Z_MIN_MM", 0.0)),
                        min(float(getattr(vc, "TARGET_Z_MAX_MM", 190.0)), tz),
                    )
                    tz = self.target_z_mm
                    continue

            cmd_out = cmd_try
            break

        if cmd_out is None:
            cmd_out = self.last_valid_cmd
        return cmd_out

    def process_no_face(
        self,
        vc: object,
        *,
        now_t: float,
        ctl: str,
        fk0_tip_x: float,
        fk0_tip_z: float,
    ) -> tuple[ServoCommand | None, bool]:
        """
        Returns (command or None to hold hardware, whether neutral was sent this call).
        """
        if bool(getattr(vc, "BASE_PID_RESET_ON_LOSS", True)):
            self.base_pid_i = 0.0
            self.base_pid_prev_e = 0.0
            self.base_pid_d = 0.0
            self.base_zero_cross_hold = 0
        if bool(getattr(vc, "Y_PID_RESET_ON_LOSS", True)):
            self.y_pid_i = 0.0
            self.y_pid_prev_e = 0.0
            self.y_pid_d = 0.0

        no_face_delay_s = max(0.0, float(getattr(vc, "NO_FACE_RETURN_DELAY_S", 30.0)))
        face_missing_for_s = max(0.0, now_t - self.last_face_seen_t)
        should_return = face_missing_for_s >= no_face_delay_s

        if (
            ctl == "ik"
            and bool(getattr(vc, "NO_FACE_VERTICAL_RETURN_ENABLE", True))
            and should_return
            and not self.no_face_neutral_sent
        ):
            cmd = ServoCommand(
                wrist=config.NEUTRAL_WRIST,
                elbow=config.NEUTRAL_ELBOW,
                base=config.NEUTRAL_BASE,
                shoulder=config.NEUTRAL_SHOULDER,
            )
            cmd, _ = clamp_servo(cmd)
            self.last_valid_cmd = cmd
            self.elbow_cmd_last = cmd.elbow
            self.target_x_mm = fk0_tip_x
            self.target_z_mm = fk0_tip_z
            self.z_err_mm = 0.0
            self.no_face_neutral_sent = True
            return cmd, True

        return None, False

    def should_send(self, cmd: ServoCommand, eps: float | None = None) -> bool:
        eps = eps if eps is not None else float(self._mv("SEND_EPSILON_DEG", 0.5))
        return command_delta_max_deg(cmd, self.last_valid_cmd) >= eps
