"""Motion V1 helpers (no hardware)."""

from __future__ import annotations

import unittest

from glados_arm import kinematics


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
