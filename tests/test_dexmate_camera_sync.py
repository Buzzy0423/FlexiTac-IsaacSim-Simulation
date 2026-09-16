"""A clock mapping can advance while the RGB render reference remains stale."""

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
from dexmate_performance import Timings  # noqa: E402

from Isaacsim_tactile_env.dexmate_env import DexmateEnv  # noqa: E402


class CameraSyncTests(unittest.TestCase):
    def test_retimed_old_reference_requires_a_new_render(self):
        env = DexmateEnv.__new__(DexmateEnv)
        env.render_mode = "physics"
        env.timings, env.flush_histogram = Timings(), {}
        env.rgb_reference_time = {"camera": (9, 30)}
        frame = [9]

        def render():
            frame[0] += 1

        env.world = SimpleNamespace(current_time_step_index=4, current_time=1., render=render)
        env.core_clock = SimpleNamespace(get_sim_time_at_time=lambda _: 1.)
        env.clocks = {"camera": SimpleNamespace(get_data=lambda: {
            "referenceTimeNumerator": frame[0], "referenceTimeDenominator": 30})}
        env.readers = {"camera": SimpleNamespace(get_data=lambda: np.zeros((2, 2, 3), dtype=np.uint8))}
        env.camera_prims = {"camera": None}
        env.step_count, env.time_origin = 4, 0.
        pxr = MagicMock()
        pxr.UsdGeom.XformCache.return_value.GetLocalToWorldTransform.return_value = np.eye(4)
        with patch.dict(sys.modules, {"pxr": pxr}):
            env._capture()
        self.assertEqual(env.render_flushes, 1)
        self.assertEqual(env.rgb_reference_time["camera"], (10, 30))
        self.assertEqual(env.world.current_time_step_index, 4)
