"""Motion V1 helpers (no hardware)."""

from __future__ import annotations

import unittest

from glados_arm import config, kinematics, vision_config
from glados_arm.mapping import ServoCommand
from glados_arm.motion_controller_v1 import MotionControllerV1, VisionMeasurement
from glados_arm.motion_smooth import (
    servo_command_to_float_tuple,
    sync_step_servo_float_toward,
    sync_step_servo_toward,
)


class TestSyncStepServoToward(unittest.TestCase):
    def test_same_fraction_when_one_joint_limits(self) -> None:
        prev = ServoCommand(wrist=0, elbow=0, base=0, shoulder=0)
        target = ServoCommand(wrist=0, elbow=100, base=0, shoulder=10)
        max_dps = (100.0, 50.0, 60.0, 75.0)
        dt = 0.1
        out = sync_step_servo_toward(prev, target, dt, max_dps)
        # Elbow max step 5 -> fraction 0.05; shoulder moves 0.5
        self.assertEqual(out.elbow, 5)
        self.assertEqual(out.shoulder, 0)

    def test_full_step_when_within_limits(self) -> None:
        prev = ServoCommand(wrist=0, elbow=0, base=90, shoulder=0)
        # Deltas must fit all joints' dps*dt in one frame (shoulder 75 dps -> 7.5 deg / 0.1s)
        target = ServoCommand(wrist=0, elbow=5, base=90, shoulder=5)
        max_dps = (120.0, 90.0, 60.0, 75.0)
        dt = 0.1
        out = sync_step_servo_toward(prev, target, dt, max_dps)
        self.assertEqual(out.elbow, 5)
        self.assertEqual(out.shoulder, 5)

    def test_float_accumulates_sub_degree_steps(self) -> None:
        """Integer sync would stall when each step is < 0.5 deg; float must reach target."""
        prev = ServoCommand(wrist=0, elbow=0, base=0, shoulder=0)
        target = ServoCommand(wrist=0, elbow=30, base=0, shoulder=0)
        max_dps = (120.0, 90.0, 60.0, 75.0)
        dt = 0.01
        f = servo_command_to_float_tuple(prev)
        for _ in range(500):
            f = sync_step_servo_float_toward(f, target, dt, max_dps)
        self.assertAlmostEqual(f[1], 30.0, places=3)

    def test_proportional_sync_moves_all_joints_toward_target(self) -> None:
        """One max_frac for all joints — elbow and shoulder both change when both have error."""
        prev = (200.0, 270.0, 135.0, 0.0)
        target = ServoCommand(wrist=200, elbow=260, base=135, shoulder=5)
        max_dps = (120.0, 90.0, 60.0, 75.0)
        dt = 0.05
        out = sync_step_servo_float_toward(prev, target, dt, max_dps)
        self.assertLess(out[1], prev[1])
        self.assertGreater(out[3], prev[3])


class TestIKBranchHysteresis(unittest.TestCase):
    def test_prefers_last_when_small_error(self) -> None:
        p = kinematics.resolve_ik_preference(
            "elbow_up",
            "elbow_down",
            0.02,
            switch_threshold=0.12,
        )
        self.assertEqual(p, "elbow_down")

    def test_switches_when_large_error(self) -> None:
        p = kinematics.resolve_ik_preference(
            "elbow_up",
            "elbow_down",
            0.5,
            switch_threshold=0.12,
        )
        self.assertEqual(p, "elbow_up")

    def test_none_falls_back(self) -> None:
        p = kinematics.resolve_ik_preference("elbow_down", None, 0.0, switch_threshold=0.12)
        self.assertEqual(p, "elbow_down")


class _VCProxy:
    """Test helper: override selected vision_config values only."""

    def __init__(self, **overrides: float | int | bool) -> None:
        self._overrides = dict(overrides)

    def __getattr__(self, name: str):  # noqa: ANN204 - dynamic proxy for config constants
        if name in self._overrides:
            return self._overrides[name]
        return getattr(vision_config, name)


class _StubMotionController(MotionControllerV1):
    """MotionController with deterministic IK output for logic-only tests."""

    def __init__(self, vc: object, stub_cmd: ServoCommand) -> None:
        fk0 = kinematics.forward_kinematics(0.0, 0.0)
        super().__init__(vc, None, fk0_tip_x=fk0.tip.x, fk0_tip_z=fk0.tip.z, base_yaw_lim=1.0)
        self.stub_cmd = stub_cmd

    def _solve_ik_chain(self, **kwargs) -> ServoCommand:  # type: ignore[override]
        self.ik_status = "ok"
        self.ik_clip_notes = []
        return self.stub_cmd

    def _smooth_and_limit(self, cmd: ServoCommand, _t: float, dt: float) -> ServoCommand:  # type: ignore[override]
        return cmd


class TestAsymmetricVerticalBehavior(unittest.TestCase):
    def _vm(self) -> VisionMeasurement:
        return VisionMeasurement(
            face_detected=True,
            err_x_norm=0.0,
            err_y_norm=0.0,
            corr_x_norm_raw=0.0,
            corr_y_norm_raw=0.0,
            filt_face_w=None,
            face_w_px=120,
            t_seconds=0.0,
        )

    def test_lower_bound_uses_wrist_and_holds_proximal(self) -> None:
        vc = _VCProxy(UPPER_BOUND_PROXIMAL_EXTEND_ENABLE=False)
        mc = _StubMotionController(
            vc,
            ServoCommand(
                wrist=config.NEUTRAL_WRIST - 15,
                elbow=config.NEUTRAL_ELBOW - 10,
                base=config.NEUTRAL_BASE,
                shoulder=config.NEUTRAL_SHOULDER + 10,
            ),
        )
        mc.target_z_mm = float(vc.TARGET_Z_MIN_MM)
        mc.last_valid_cmd = ServoCommand(
            wrist=config.NEUTRAL_WRIST,
            elbow=config.NEUTRAL_ELBOW,
            base=config.NEUTRAL_BASE,
            shoulder=config.NEUTRAL_SHOULDER,
        )
        prev = mc.last_valid_cmd

        out = mc.process_ik(
            self._vm(),
            corr_x_ctrl=0.0,
            corr_y_vert=-0.50,
            corr_y_ik=-0.50,
            corr_y_norm=-0.50,
            engage=1.0,
            dt=0.033,
        )

        self.assertLess(out.wrist, prev.wrist)
        self.assertEqual(out.elbow, prev.elbow)
        self.assertEqual(out.shoulder, prev.shoulder)
        self.assertIn("lower_bound_wrist_only", mc.ik_status)

    def test_upper_bound_prefers_proximal_and_freezes_wrist_trim(self) -> None:
        vc = _VCProxy(UPPER_BOUND_PROXIMAL_EXTEND_ENABLE=True)
        mc = _StubMotionController(
            vc,
            ServoCommand(
                wrist=config.NEUTRAL_WRIST - 25,
                elbow=148,
                base=config.NEUTRAL_BASE,
                shoulder=101,
            ),
        )
        mc.target_z_mm = float(vc.TARGET_Z_MAX_MM)
        mc.last_valid_cmd = ServoCommand(
            wrist=config.NEUTRAL_WRIST,
            elbow=150,
            base=config.NEUTRAL_BASE,
            shoulder=100,
        )

        out = mc.process_ik(
            self._vm(),
            corr_x_ctrl=0.0,
            corr_y_vert=0.40,
            corr_y_ik=0.40,
            corr_y_norm=0.40,
            engage=1.0,
            dt=0.033,
        )

        self.assertEqual(out.wrist, config.NEUTRAL_WRIST)
        self.assertGreaterEqual(out.shoulder, 102)
        self.assertLessEqual(out.elbow, 149)
        self.assertIn("upper_bound_proximal_extend", mc.ik_status)


if __name__ == "__main__":
    unittest.main()
