"""Pose variations preserve the physical grasp relation without cumulative drift."""

import sys
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from dexmate_variations import pose_sweep, resolve_grasp_config  # noqa: E402


class PoseVariationTests(unittest.TestCase):
    baseline = {"position_m": [.34935, .22, .841], "gripper_base_position_m": [.2, .22, .8545]}

    def test_yaw_rotates_about_object_not_world_origin(self):
        config = resolve_grasp_config(self.baseline, {"offset_xy_m": [.01, -.01], "yaw_deg": 10})
        centre = np.asarray(config["position_m"])
        delta = np.asarray(config["gripper_base_position_m"]) - centre
        rotation = Rotation.from_euler("z", 10, degrees=True)
        np.testing.assert_allclose(rotation.inv().apply(delta), [-.14935, 0, .0135], atol=1e-12)
        np.testing.assert_allclose(centre, [.35935, .21, .841])

    def test_cases_are_bounded_and_final_baseline_does_not_accumulate(self):
        original = {k: list(v) for k, v in self.baseline.items()}
        cases = pose_sweep()
        self.assertEqual(len(cases), 10)
        for case in cases:
            resolve_grasp_config(self.baseline, case)
        self.assertEqual(self.baseline, original)
        last = resolve_grasp_config(self.baseline, cases[-1])
        for key, value in original.items():
            np.testing.assert_allclose(last[key], value)

    def test_invalid_or_out_of_scope_variation_fails(self):
        for case in ({"offset_xy_m": [.5, 0]}, {"yaw_deg": 90}, {"yaw_deg": float("nan")},
                     {"offset_xy_m": [0]}):
            with self.assertRaises(ValueError):
                resolve_grasp_config(self.baseline, case)
