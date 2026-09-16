"""30 Hz persistence must preserve alignment and full-rate failure diagnostics."""

import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from dexmate_episode_recording import EpisodeWriter  # noqa: E402


class RecordingTests(unittest.TestCase):
    def test_camera_samples_and_between_frame_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = EpisodeWriter.__new__(EpisodeWriter)
            writer.output = Path(directory)
            writer.env = SimpleNamespace(robot=SimpleNamespace(dof_names=["joint"]),
                                         action_names=["joint"], report={"grasp_config": {}})
            writer.closed, writer.reset_state = True, {}
            writer.names = ["head_left"]
            writer.frames = [{
                "step": s, "physics_step": s, "time_s": s / 120,
                "action": np.array([s]), "joint_position": np.array([s + .5]),
                "tactile": np.full((4, 12, 32), s / 10),
                "phase": "grasp_approach", "phase_step": s - 1,
                "terminated": s == 9, "truncated": False, "task_success": False,
            } for s in range(1, 10)]
            writer.camera_rows = [{
                "state_row": s - 1, "step": s, "time_s": s / 120,
                "sim_time_s": 10 + s / 120, "render_sim_time_s": [10 + s / 120],
                "reference_time": [[s, 30]],
            } for s in (4, 8)]
            with patch("dexmate_grasp_regression.evaluate_grasp", return_value={"passed": False}) as evaluate, \
                    patch("dexmate_tactile_preview.export_views"), \
                    patch("dexmate_episode_recording.subprocess.check_output", return_value="2"):
                report = writer.finish()
            self.assertEqual(len(evaluate.call_args.args[0]["step"]), 9)
            self.assertFalse(report["passed"])
            self.assertTrue(all(report["sync_checks"].values()))
            self.assertEqual(report["frame_count"], 2)
            self.assertEqual(report["validation_frame_count"], 9)
            self.assertEqual(report["final_physics_state"]["step"], 9)
            self.assertTrue(report["final_physics_state"]["terminated"])
            self.assertEqual(report["data_contract"]["signal_hz"], 30)
            with np.load(writer.output / "trajectory.npz") as data, \
                    np.load(writer.output / "camera_index.npz") as index:
                np.testing.assert_array_equal(index["state_row"], [0, 1])
                np.testing.assert_array_equal(data["step"], [4, 8])
                np.testing.assert_array_equal(data["action"][:, 0], [4, 8])
                np.testing.assert_array_equal(data["tactile"][:, 0, 0, 0], [.4, .8])
                np.testing.assert_array_equal(data["time_s"], index["time_s"])
