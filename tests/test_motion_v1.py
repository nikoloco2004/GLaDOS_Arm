"""Motion V1 helpers (no hardware)."""

from __future__ import annotations

import unittest

from glados_arm import kinematics
from glados_arm.mapping import ServoCommand
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


if __name__ == "__main__":
    unittest.main()
