"""Geometry and distance-response checks, independent of the GPU scene regression."""

import json
import unittest
from pathlib import Path

import numpy as np
import torch
import trimesh
import warp as wp

from Isaacsim_tactile_env.dexmate_flexitac import build_local_patches, load_shared_kernels

ROOT = Path(__file__).resolve().parents[1]


class TactileTests(unittest.TestCase):
    def test_four_patches_fit_and_point_outward(self):
        assets = ROOT / "Isaacsim_tactile_env/assets/dexmate"
        cfg = json.loads((ROOT / "Isaacsim_tactile_env/dexmate_workspace.json").read_text())["tactile"]
        points, normals = build_local_patches(assets, cfg)
        self.assertEqual(points.shape, (4, 384, 3))
        np.testing.assert_allclose(normals, [[1, 0, 0], [-1, 0, 0], [1, 0, 0], [-1, 0, 0]], atol=1e-6)
        manifest = json.loads((assets / "manifest.json").read_text())
        for index, finger in enumerate(("gripper_l1", "gripper_l2") * 2):
            transform = np.asarray(manifest["finger_mapping"][finger]["source_metres_to_link"])
            canonical = (points[index] - transform[:3, 3]) @ np.linalg.inv(transform[:3, :3]).T
            np.testing.assert_allclose(canonical[:, 0], 0.02579656791687 + 0.0021, atol=1e-8)
            np.testing.assert_allclose(np.ptp(canonical[:, 1:], axis=0), [.022, .062], atol=1e-8)
            np.testing.assert_allclose(canonical[:, 1:].mean(0), [-0.0000049610138, .1220692901611], atol=1e-8)
            grid = canonical.reshape(12, 32, 3)
            np.testing.assert_allclose(np.diff(grid[:, 0, 1]), .002, atol=1e-8)
            np.testing.assert_allclose(np.diff(grid[0, :, 2]), .002, atol=2e-8)

    def test_signed_mesh_response_uses_metres_and_saturates(self):
        kernels = load_shared_kernels()
        box = trimesh.creation.box(extents=[.05, .04, .03])
        indices = wp.array(box.faces.astype(np.int32).ravel(), dtype=wp.int32, device="cpu")
        mesh = wp.Mesh(points=wp.array(box.vertices, dtype=wp.vec3, device="cpu"), indices=indices)
        normals = wp.array(np.asarray(box.vertex_normals), dtype=wp.vec3, device="cpu")
        queries = wp.array([[x, 0, 0] for x in [.026, .025, .0245, .024, .023, .021]],
                           dtype=wp.vec3, device="cpu")
        distance = wp.empty(6, dtype=wp.float32, device="cpu")
        wp.launch(kernels.mesh_distance_kernel, dim=6, device="cpu",
                  inputs=[queries, mesh.id, indices, normals, .2, 1, 1, distance])
        np.testing.assert_allclose(distance.numpy(), [.001, 0, -.0005, -.001, -.002, -.004], atol=1e-8)
        response = kernels.spring_response(torch.from_numpy(distance.numpy()), 5000, 10)
        np.testing.assert_allclose(response.numpy(), [0, 0, .25, .5, 1, 1], atol=1e-5)

    def test_shared_spring_preserves_aloha_signed_and_unsigned_modes(self):
        kernels = load_shared_kernels()
        distances = torch.linspace(-.005, .005, 101)
        for shell in (None, .001):
            penetration = (-distances if shell is None else shell - distances).clamp_min(0)
            old_force = (5000 * penetration).clamp(0, 10)
            for normalize in (False, True):
                expected = old_force / 10 if normalize else old_force
                torch.testing.assert_close(
                    kernels.spring_response(distances, 5000, 10, normalize, shell), expected, rtol=0, atol=0
                )


if __name__ == "__main__":
    unittest.main()
