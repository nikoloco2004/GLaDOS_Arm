"""FK/IK sanity tests (no hardware)."""

from __future__ import annotations

import math
import unittest

from glados_arm import config, kinematics
from glados_arm.mapping import ModelJointState, model_to_servo, servo_to_model


class TestFKIK(unittest.TestCase):
    def test_fk_ik_roundtrip_zero(self) -> None:
        fk = kinematics.forward_kinematics(0.0, 0.0)
        ik = kinematics.inverse_kinematics_plane(fk.tip.x, fk.tip.z)
        self.assertTrue(ik.ok)
        self.assertAlmostEqual(ik.q_shoulder, 0.0, places=4)
        self.assertAlmostEqual(ik.q_elbow, 0.0, places=4)

    def test_fk_ik_roundtrip_offset(self) -> None:
        qs = math.radians(12.0)
        qe = math.radians(-8.0)
        fk = kinematics.forward_kinematics(qs, qe)
        ik = kinematics.inverse_kinematics_plane(fk.tip.x, fk.tip.z, prefer="elbow_up")
        self.assertTrue(ik.ok)
        # Two IK branches can match the same tip — assert geometric consistency.
        fk2 = kinematics.forward_kinematics(ik.q_shoulder, ik.q_elbow)
        self.assertAlmostEqual(fk2.tip.x, fk.tip.x, places=3)
        self.assertAlmostEqual(fk2.tip.z, fk.tip.z, places=3)

    def test_unreachable(self) -> None:
        r = config.LINK_SHOULDER_ELBOW_MM + config.LINK_ELBOW_WRIST_MM
        ik = kinematics.inverse_kinematics_plane(r + 500.0, 0.0)
        self.assertFalse(ik.ok)
        self.assertEqual(ik.reason, "unreachable_too_far")

    def test_mapping_roundtrip_neutral(self) -> None:
        m0 = ModelJointState(0.0, 0.0, 0.0, 0.0)
        s = model_to_servo(m0)
        self.assertEqual(s.wrist, config.NEUTRAL_WRIST)
        self.assertEqual(s.elbow, config.NEUTRAL_ELBOW)
        self.assertEqual(s.base, config.NEUTRAL_BASE)
        self.assertEqual(s.shoulder, config.NEUTRAL_SHOULDER)
        m1 = servo_to_model(s)
        self.assertAlmostEqual(m1.base_yaw_rad, 0.0, places=5)
        self.assertAlmostEqual(m1.q_shoulder_rad, 0.0, places=5)
        self.assertAlmostEqual(m1.q_elbow_rad, 0.0, places=5)


if __name__ == "__main__":
    unittest.main()
