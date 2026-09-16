"""Four Dexmate FlexiTac patches on native Isaac Sim, sharing ALOHA's Warp kernels.

Call update() after each physics step. Output order: L1, L2, R1, R2;
float32 shape (4, 12, 32), normalized to [0, 1]. No PhysX force is used.
"""

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
KERNEL_PATH = ROOT / "source/isaaclab/isaaclab/sensors/warp_sdf_tactile/warp_sdf_kernels.py"
SLOTS = ("L_gripper_l1", "L_gripper_l2", "R_gripper_l1", "R_gripper_l2")


def load_shared_kernels():
    # Load the same file ALOHA imports, without importing Isaac Lab's extension
    # graph into the standalone native-Isaac-Sim process.
    name = "dexmate_shared_flexitac_kernels"
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, KERNEL_PATH)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


def build_local_patches(assets, cfg):
    """Generate row=width, column=length samples and transform both mirrored fingers."""
    manifest = json.loads((assets / "manifest.json").read_text())
    mesh = trimesh.load(assets / manifest["custom_source"]["path"], force="mesh", process=True)
    face = mesh.vertices[np.isclose(mesh.vertices[:, 0], mesh.bounds[1, 0], atol=1e-5, rtol=0)] * 0.001
    if len(face) != 4:
        raise ValueError("Expected the four plateau corners in the canonical finger STL")
    rows, cols, pitch = cfg["num_rows"], cfg["num_cols"], cfg["point_distance_m"]
    if rows < 1 or cols < 1 or pitch <= 0 or cfg["outward_offset_m"] < 0:
        raise ValueError("Invalid tactile grid or offset")
    if (rows - 1) * pitch > np.ptp(face[:, 1]) or (cols - 1) * pitch > np.ptp(face[:, 2]):
        raise ValueError("Taxels extend past the contact plateau")
    u = (np.arange(rows) - (rows - 1) / 2) * pitch
    v = (np.arange(cols) - (cols - 1) / 2) * pitch
    uu, vv = np.meshgrid(u, v, indexing="ij")
    points = np.tile(face.mean(0), (rows * cols, 1))
    points[:, 0] += cfg["outward_offset_m"]
    points[:, 1] += uu.ravel()
    points[:, 2] += vv.ravel()
    patches, normals = [], []
    for slot in SLOTS:
        transform = np.asarray(manifest["finger_mapping"][slot[2:]]["source_metres_to_link"])
        patches.append(points @ transform[:3, :3].T + transform[:3, 3])
        normals.append(transform[:3, 0])
    return np.asarray(patches, dtype=np.float32), np.asarray(normals, dtype=np.float32)


def world_matrix(prim, cache):
    return np.asarray(cache.GetLocalToWorldTransform(prim), dtype=np.float64).T


def rigid_matrix(matrix):
    """Separate rigid pose from scale, which is baked into mesh vertices in metres."""
    u, _, vt = np.linalg.svd(matrix[:3, :3])
    result = np.eye(4)
    result[:3, :3] = u @ vt
    result[:3, 3] = matrix[:3, 3]
    return result


def transform_points(points, matrix):
    return points @ matrix[:3, :3].T + matrix[:3, 3]


class MeshTarget:
    """A rigid target with all its mesh parts baked into a metre-scaled query mesh."""

    def __init__(self, stage, path, device, wp):
        from pxr import Usd, UsdGeom, UsdPhysics

        self.prim = stage.GetPrimAtPath(path)
        if not self.prim.IsValid():
            raise ValueError(f"Missing tactile target: {path}")
        if len([p for p in Usd.PrimRange(self.prim) if p.HasAPI(UsdPhysics.RigidBodyAPI)]) > 1:
            raise ValueError("Register articulated target bodies separately")
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        matrix = world_matrix(self.prim, cache)
        inverse_pose = np.linalg.inv(rigid_matrix(matrix))
        self.local_affine = inverse_pose @ matrix
        parts = []
        for prim in Usd.PrimRange(self.prim, Usd.TraverseInstanceProxies()):
            if prim.IsA(UsdGeom.Mesh):
                mesh = UsdGeom.Mesh(prim)
                points = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float64)
                counts = mesh.GetFaceVertexCountsAttr().Get()
                indices = mesh.GetFaceVertexIndicesAttr().Get()
                faces, offset = [], 0
                for count in counts:
                    polygon = indices[offset:offset + count]
                    faces.extend([polygon[0], polygon[j], polygon[j + 1]] for j in range(1, count - 1))
                    offset += count
                part = trimesh.Trimesh(points, faces, process=True)
            elif prim.IsA(UsdGeom.Cube):
                size = float(UsdGeom.Cube(prim).GetSizeAttr().Get())
                part = trimesh.creation.box(extents=[size] * 3)
            else:
                continue
            # Collision duplicates must not be combined with visual surfaces.
            # URDF importer marks collision roots as guide purpose.
            if UsdGeom.Imageable(prim).ComputePurpose() == UsdGeom.Tokens.guide:
                continue
            part.apply_transform(inverse_pose @ world_matrix(prim, cache))
            if not part.is_watertight or not part.is_winding_consistent or part.volume <= 0:
                raise ValueError(f"Signed-distance target must be a closed oriented solid: {prim.GetPath()}")
            parts.append(part)
        if not parts:
            raise ValueError(f"No supported Mesh/Cube geometry under tactile target: {path}")
        mesh = trimesh.util.concatenate(parts)
        self.mesh = wp.Mesh(
            points=wp.array(mesh.vertices.astype(np.float32), dtype=wp.vec3, device=device),
            indices=wp.array(mesh.faces.astype(np.int32).ravel(), dtype=wp.int32, device=device),
        )
        self.indices = wp.array(mesh.faces.astype(np.int32).ravel(), dtype=wp.int32, device=device)
        self.normals = wp.array(np.asarray(mesh.vertex_normals, dtype=np.float32), dtype=wp.vec3, device=device)
        self.part_count = len(parts)

    def inverse_pose(self, cache):
        matrix = world_matrix(self.prim, cache)
        inverse = np.linalg.inv(rigid_matrix(matrix))
        if not np.allclose(inverse @ matrix, self.local_affine, atol=1e-6):
            raise ValueError("Tactile target scale changed; rebuild the target query mesh")
        return inverse


class DexmateFlexiTac:
    """Pose-bound taxels and explicit object queries; robot geometry is never a target."""

    def __init__(self, stage, robot_path, assets, cfg, target_paths=None):
        import torch
        import warp as wp

        if cfg["stiffness"] <= 0 or cfg["max_force"] <= 0 or cfg["mesh_max_dist_m"] <= 0:
            raise ValueError("Tactile stiffness, force limit and query distance must be positive")
        self.stage, self.cfg = stage, cfg
        self.wp, self.torch = wp, torch
        self.kernels = load_shared_kernels()
        self.device = cfg["device"]
        self.local_points, self.local_normals = build_local_patches(assets, cfg)
        self.links = [stage.GetPrimAtPath(f"{robot_path}/{name}") for name in SLOTS]
        if not all(p.IsValid() for p in self.links):
            raise ValueError("Missing one or more Dexmate finger links")
        paths = list(dict.fromkeys(cfg["target_prim_paths"] if target_paths is None else target_paths))
        for path in paths:
            if path == robot_path or path.startswith(robot_path + "/") or robot_path.startswith(path.rstrip("/") + "/"):
                raise ValueError("Robot bodies cannot be tactile query targets")
        self.targets = [MeshTarget(stage, path, self.device, wp) for path in paths]
        self.target_paths = paths
        count = self.local_points.shape[0] * self.local_points.shape[1]
        self.distance = torch.empty(count, device=self.device, dtype=torch.float32)
        self.observation = np.zeros((4, cfg["num_rows"], cfg["num_cols"]), dtype=np.float32)
        self.points_world = np.zeros_like(self.local_points)
        self.tactile_points_w = np.zeros((1, count, 4), dtype=np.float32)

    def update(self):
        from pxr import Usd, UsdGeom

        wp, torch = self.wp, self.torch
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        for index, prim in enumerate(self.links):
            self.points_world[index] = transform_points(self.local_points[index], world_matrix(prim, cache))
        points = self.points_world.reshape(-1, 3)
        self.distance.fill_(float(self.cfg["mesh_max_dist_m"]))
        stream = wp.stream_from_torch(torch.cuda.current_stream(self.device)) if self.distance.is_cuda else None
        for target in self.targets:
            query = torch.as_tensor(
                transform_points(points, target.inverse_pose(cache)), dtype=torch.float32, device=self.device
            ).contiguous()
            distance = torch.empty_like(self.distance)
            wp.launch(
                self.kernels.mesh_distance_kernel, dim=len(points), device=self.device, stream=stream,
                inputs=[wp.from_torch(query, dtype=wp.vec3), target.mesh.id, target.indices, target.normals,
                        float(self.cfg["mesh_max_dist_m"]), 1, 1, wp.from_torch(distance, dtype=wp.float32)],
            )
            self.distance = torch.minimum(self.distance, distance)
        force = self.kernels.spring_response(self.distance, self.cfg["stiffness"], self.cfg["max_force"], True)
        self.observation = force.cpu().numpy().reshape(self.observation.shape)
        self.tactile_points_w[0, :, :3] = points
        self.tactile_points_w[0, :, 3] = self.observation.ravel()
        return self.observation.copy()

    def metadata(self):
        return {
            "implementation": "Native Isaac Sim adapter, shared ALOHA Warp kernels and spring_response",
            "slots": list(SLOTS), "shape": list(self.observation.shape), "config": self.cfg,
            "target_paths": self.target_paths, "target_mesh_parts": [t.part_count for t in self.targets],
            "local_normals": self.local_normals.tolist(),
            "local_sample_bounds_m": [[p.min(0).tolist(), p.max(0).tolist()] for p in self.local_points],
            "units": "normalized per-taxel distance response in [0,1], not measured pressure or PhysX force",
        }
