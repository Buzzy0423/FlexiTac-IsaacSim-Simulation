"""Guard collision coverage when fixed URDF frames are retained during import."""

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from tools.dexmate_collision import adjacent_body_pairs, record_contact_phase


class CollisionCoverageTests(unittest.TestCase):
    def test_dexmate_keeps_opposing_fingers_and_nonadjacent_bodies(self):
        root = Path(__file__).resolve().parents[1]
        pairs = set(adjacent_body_pairs(root / "Isaacsim_tactile_env/assets/dexmate/vega_1u_gripper_flexitac.urdf"))
        for side in "LR":
            self.assertNotIn((f"{side}_gripper_l1", f"{side}_gripper_l2"), pairs)
            self.assertNotIn((f"{side}_arm_l5", f"{side}_arm_l7"), pairs)
            self.assertIn((f"{side}_arm_l1", "torso_flip_link"), pairs)
        self.assertNotIn(("base_link", "torso_flip_link"), pairs)
        self.assertNotIn(("head_l2", "torso_flip_link"), pairs)
        self.assertIn(("head_l1", "torso_flip_link"), pairs)
        self.assertFalse(any(a.startswith("L_") and b.startswith("R_") for a, b in pairs))

    def test_empty_fixed_frame_does_not_hide_adjacent_joint(self):
        # Two moving fingers sharing a fixed mount remain able to hit each other.
        urdf = """<robot name="fixture">
          <link name="body"/><link name="mount"/><link name="finger_a"/><link name="finger_b"/>
          <joint name="mount" type="fixed"><parent link="body"/><child link="mount"/></joint>
          <joint name="a" type="revolute"><parent link="mount"/><child link="finger_a"/></joint>
          <joint name="b" type="revolute"><parent link="mount"/><child link="finger_b"/></joint>
        </robot>"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "fixture.urdf"
            path.write_text(urdf)
            pairs = set(adjacent_body_pairs(path))
        self.assertIn(("body", "finger_a"), pairs)
        self.assertIn(("body", "finger_b"), pairs)
        self.assertNotIn(("finger_a", "finger_b"), pairs)

    def test_candidate_contact_without_impulse_does_not_count_as_loaded(self):
        report = {"physics_phase": {"name": "release", "step": 230}, "phase_contacts": {}}
        header = SimpleNamespace(num_contact_data=1, contact_data_offset=0)
        candidate = SimpleNamespace(separation=0.0001, impulse=SimpleNamespace(x=0, y=0, z=0))
        record_contact_phase(report, ["finger", "probe"], header, [candidate])
        stats = report["phase_contacts"]["release"]["finger | probe"]
        self.assertEqual(stats["last_step"], 230)
        self.assertEqual(stats["last_impulse_step"], -1)
        loaded = SimpleNamespace(separation=-0.000001, impulse=SimpleNamespace(x=.001, y=0, z=0))
        record_contact_phase(report, ["finger", "probe"], header, [loaded])
        self.assertEqual(stats["last_impulse_step"], 230)

    def test_contact_summary_respects_offsets_peak_and_previous_loaded_step(self):
        report = {"physics_phase": {"name": "hold", "step": 4}, "phase_contacts": {}}
        def point(separation, x, y, z):
            return SimpleNamespace(separation=separation, impulse=SimpleNamespace(x=x, y=y, z=z))
        data = [point(-1, 99, 99, 99), point(-.001, 0, -.003, .004), point(.0002, 0, 0, 0)]
        header = SimpleNamespace(num_contact_data=2, contact_data_offset=1)
        record_contact_phase(report, ["finger", "cube"], header, data)
        stats = report["phase_contacts"]["hold"]["finger | cube"]
        self.assertEqual(stats["points"], 2)
        self.assertAlmostEqual(stats["max_impulse_Ns"], .005)
        self.assertEqual(stats["min_separation_m"], -.001)
        report["physics_phase"]["step"] = 5
        header.num_contact_data, header.contact_data_offset = 1, 2
        record_contact_phase(report, ["finger", "cube"], header, data)
        self.assertEqual(stats["points"], 3)
        self.assertEqual(stats["last_impulse_step"], 4)
        self.assertEqual(stats["last_step"], 5)
        self.assertAlmostEqual(stats["max_impulse_Ns"], .005)
