"""Acceptance checks must reject a motionless or unsupported 'grasp'."""

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from dexmate_grasp_regression import evaluate_grasp  # noqa: E402


class GraspAcceptanceTests(unittest.TestCase):
    def fixture(self):
        phases = np.array(["grasp_approach"] * 30 + ["grasp_close"] * 30
                          + ["grasp_hold"] * 240 + ["grasp_release"] * 60 + ["grasp_retreat"])
        n = len(phases)
        hold = phases == "grasp_hold"
        position = np.tile([.35, .22, .841], (n, 1))
        position[hold, 2] += .05
        loaded = np.zeros((n, 2), dtype=bool)
        loaded[hold] = True
        tactile = np.zeros((n, 4))
        tactile[hold, :2] = .8
        return {
            "phase": phases, "object_position_m": position,
            "object_in_gripper_m": np.tile([0, 0, .15], (n, 1)),
            "finger_loaded": loaded, "tactile_peak": tactile, "object_velocity_m_s": np.zeros((n, 3)),
        }

    def test_motionless_object_fails_even_with_contact_and_tactile(self):
        data = self.fixture()
        self.assertTrue(evaluate_grasp(data, {"phase_contacts": {}})["passed"])
        data["object_position_m"][:, 2] = .841
        self.assertFalse(evaluate_grasp(data, {"phase_contacts": {}})["checks"]["lift_50mm_within_10mm"])

    def test_tactile_without_sustained_physical_contact_fails(self):
        data = self.fixture()
        data["finger_loaded"][:] = False
        data["finger_loaded"][60] = True
        result = evaluate_grasp(data, {"phase_contacts": {}})
        self.assertFalse(result["passed"])
        self.assertFalse(result["checks"]["both_fingers_loaded_during_hold"])

    def test_slip_and_unexpected_collision_fail(self):
        data = self.fixture()
        data["object_in_gripper_m"][100, 0] += .01
        report = {"phase_contacts": {"grasp_hold": {
            "/World/Dexmate/L_arm_l5 | /World/WorktopBoard": {"max_impulse_Ns": .01}
        }}}
        result = evaluate_grasp(data, report)
        self.assertFalse(result["checks"]["slip_under_8mm"])
        self.assertFalse(result["checks"]["no_unexpected_robot_contacts"])

    def test_early_termination_is_reported_as_failure(self):
        data = {key: value[:30] for key, value in self.fixture().items()}
        result = evaluate_grasp(data, {"phase_contacts": {}})
        self.assertFalse(result["passed"])
        self.assertIn("grasp_hold", result["missing_phases"])
