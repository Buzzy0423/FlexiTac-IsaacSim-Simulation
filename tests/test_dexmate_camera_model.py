"""Nominal C960 optics must preserve diagonal FOV and pixel aspect ratio."""

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from dexmate_camera_model import camera_parameters  # noqa: E402


class CameraModelTests(unittest.TestCase):
    def test_diagonal_not_horizontal_fov_and_uniform_downscale(self):
        spec = {"resolution": [1280, 720], "diagonal_fov_deg": 90, "focal_length_mm": 2.88}
        full = camera_parameters(spec)
        half = camera_parameters({**spec, "resolution": [640, 360]})
        f = full["K"][0][0]
        self.assertAlmostEqual(f, full["K"][1][1])
        self.assertAlmostEqual(math.degrees(2 * math.atan(math.hypot(640, 360) / f)), 90)
        self.assertAlmostEqual(math.degrees(2 * math.atan(640 / f)), 82.1492203)
        for i in range(2):
            for j in range(3):
                self.assertAlmostEqual(full["K"][i][j], 2 * half["K"][i][j])
        self.assertAlmostEqual(full["horizontal_aperture_mm"] / full["vertical_aperture_mm"], 16 / 9)
        self.assertAlmostEqual(full["horizontal_aperture_mm"], half["horizontal_aperture_mm"])

    def test_legacy_head_camera_unchanged(self):
        result = camera_parameters()
        self.assertEqual(result["resolution"], [640, 480])
        self.assertEqual(result["K"][0][2], 320)
        self.assertEqual(result["K"][1][2], 240)
        self.assertAlmostEqual(result["K"][0][0], 426.6666666667)

    def test_invalid_camera_parameters_rejected(self):
        valid = {"resolution": [640, 360], "diagonal_fov_deg": 90, "focal_length_mm": 2.88}
        for override in ({"resolution": [0, 360]}, {"diagonal_fov_deg": 180},
                         {"focal_length_mm": float("nan")}):
            with self.assertRaises(ValueError):
                camera_parameters({**valid, **override})
