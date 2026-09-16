"""Generate offline mesh previews and a sampled gripper clearance audit (no Isaac Sim required)."""

import argparse
import base64
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ASSETS = Path(__file__).resolve().parents[1] / "Isaacsim_tactile_env/assets/dexmate"
BLUE = np.array([0.15, 0.55, 0.85])
ORANGE = np.array([0.95, 0.47, 0.12])


def origin(element):
    if element is None:
        return np.eye(4)
    rpy = np.fromstring(element.get("rpy", "0 0 0"), sep=" ")
    matrix = trimesh.transformations.euler_matrix(*rpy)
    matrix[:3, 3] = np.fromstring(element.get("xyz", "0 0 0"), sep=" ")
    return matrix


class Robot:
    def __init__(self, path):
        self.path = path
        self.root = ET.parse(path).getroot()
        self.geometry = {}
        for link in self.root.findall("link"):
            entries = []
            for visual in link.findall("visual"):
                spec = visual.find("geometry/mesh")
                if spec is None:
                    continue
                scene = trimesh.load_scene(path.parent / spec.get("filename"), process=False)
                for node in scene.graph.nodes_geometry:
                    transform, name = scene.graph[node]
                    mesh = scene.geometry[name].copy()
                    mesh.apply_transform(transform)
                    mesh.apply_scale(np.fromstring(spec.get("scale", "1 1 1"), sep=" "))
                    mesh.apply_transform(origin(visual.find("origin")))
                    entries.append((name, mesh))
            self.geometry[link.get("name")] = entries

    def collision_mesh(self, link, component):
        collision = self.root.find(f"link[@name='{link}']/collision[@name='{component}_collision']")
        spec = collision.find("geometry/mesh")
        mesh = trimesh.load_mesh(self.path.parent / spec.get("filename"))
        mesh.apply_scale(np.fromstring(spec.get("scale", "1 1 1"), sep=" "))
        mesh.apply_transform(origin(collision.find("origin")))
        return mesh

    def poses(self, q):
        result = {"base_link": np.eye(4)}
        pending = list(self.root.findall("joint"))
        while pending:
            progressed = False
            for joint in list(pending):
                parent = joint.find("parent").get("link")
                if parent not in result:
                    continue
                value = q.get(joint.get("name"), 0.0)
                mimic = joint.find("mimic")
                if mimic is not None:
                    value = q.get(mimic.get("joint"), 0.0) * float(mimic.get("multiplier", 1))
                    value += float(mimic.get("offset", 0))
                motion = np.eye(4)
                if joint.get("type") != "fixed":
                    axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                    if joint.get("type") == "prismatic":
                        motion[:3, 3] = axis * value
                    else:
                        motion = trimesh.transformations.rotation_matrix(value, axis)
                result[joint.find("child").get("link")] = result[parent] @ origin(joint.find("origin")) @ motion
                pending.remove(joint)
                progressed = True
            if not progressed:
                raise ValueError("Disconnected joint tree")
        return result

    def meshes(self, q, hand=None):
        poses = self.poses(q)
        reference = np.linalg.inv(poses[f"{hand}_gripper_base"]) if hand else np.eye(4)
        output = []
        for link, entries in self.geometry.items():
            if hand and not link.startswith(f"{hand}_gripper"):
                continue
            for name, mesh in entries:
                world = mesh.copy()
                world.apply_transform(reference @ poses[link])
                color = BLUE if name == "custom_tpu" else np.array([0.60, 0.64, 0.69])
                output.append((world, color))
        return output


def draw_3d(ax, meshes, title, elev=22, azim=-65):
    bounds = []
    light = np.array([0.3, -0.7, 0.65])
    light /= np.linalg.norm(light)
    for mesh, color in meshes:
        shade = 0.45 + 0.55 * np.abs(mesh.face_normals @ light)
        colors = np.clip(shade[:, None] * color, 0, 1)
        ax.add_collection3d(Poly3DCollection(mesh.triangles * 1000, facecolors=colors, linewidths=0))
        bounds.extend(mesh.bounds * 1000)
    bounds = np.asarray(bounds)
    center = (bounds.max(0) + bounds.min(0)) / 2
    radius = np.max(np.ptp(bounds, axis=0)) * 0.54
    ax.set(xlim=(center[0] - radius, center[0] + radius), ylim=(center[1] - radius, center[1] + radius),
           zlim=(center[2] - radius, center[2] + radius), title=title)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()


def finger_polygons(robot, q):
    poses = robot.poses({"L_gripper_j1": q})
    reference = np.linalg.inv(poses["L_gripper_base"])
    result = []
    for i in (1, 2):
        mesh = robot.collision_mesh(f"L_gripper_l{i}", "tpu")
        mesh.apply_transform(reference @ poses[f"L_gripper_l{i}"])
        section = mesh.section(plane_normal=[0, 1, 0], plane_origin=[0, 0, 0])
        result.append([curve[:, [0, 2]] * 1000 for curve in section.discrete])
    return result


def intervals(curves, z):
    crossings = []
    for curve in curves:
        first, second = curve[:-1], curve[1:]
        selected = (first[:, 1] > z) != (second[:, 1] > z)
        a, b = first[selected], second[selected]
        crossings.extend(a[:, 0] + (z - a[:, 1]) * (b[:, 0] - a[:, 0]) / (b[:, 1] - a[:, 1]))
    ordered = sorted(crossings)
    if len(ordered) % 2:
        raise ValueError("Odd number of section crossings; cannot infer solid intervals")
    return list(zip(ordered[::2], ordered[1::2]))


def clearance(robot, q):
    left, right = finger_polygons(robot, q)
    points = np.concatenate(left + right)
    overlaps = []
    minimum_gap = float("inf")
    for z in np.arange(points[:, 1].min() + 0.007, points[:, 1].max(), 0.025):
        for a, b in intervals(left, z):
            for c, d in intervals(right, z):
                width = min(b, d) - max(a, c)
                minimum_gap = min(minimum_gap, max(c - b, a - d, 0))
                if width > 0:
                    overlaps.append((width, z, max(a, c), min(b, d)))
    return {
        "joint_rad": q,
        "max_sampled_overlap_x_mm": max((v[0] for v in overlaps), default=0),
        "min_sampled_gap_x_mm": minimum_gap if np.isfinite(minimum_gap) else None,
        "overlap_sample_z_range_mm": [min(v[1] for v in overlaps), max(v[1] for v in overlaps)] if overlaps else None,
    }


def make_previews(robot, output):
    source = ASSETS / "upstream/robots/hands/dexs_gripper/meshes/visual/gripper_l1.glb"
    scene = trimesh.load_scene(source)
    node = next(n for n in scene.graph.nodes_geometry if scene.graph[n][1] == "Mesh_1.002")
    matrix, name = scene.graph[node]
    old = scene.geometry[name].copy()
    old.apply_transform(matrix)
    new = robot.collision_mesh("L_gripper_l1", "tpu")
    fig = plt.figure(figsize=(15, 8), facecolor="white")
    draw_3d(fig.add_subplot(231, projection="3d"), [(old, ORANGE)], "Original TPU | ~20 mm wide")
    draw_3d(fig.add_subplot(232, projection="3d"), [(new, BLUE)], "Custom TPU | 26 mm wide")
    ax = fig.add_subplot(233)
    for mesh, color, label in [(old, ORANGE, "Original"), (new, BLUE, "Custom")]:
        section = mesh.section(plane_normal=[0, 1, 0], plane_origin=[0, 0, 0])
        for j, curve in enumerate(section.discrete):
            ax.plot(curve[:, 0] * 1000, curve[:, 2] * 1000, color=color, lw=0.65, label=label if j == 0 else None)
    ax.set(title="Profile overlay | Y = 0", xlabel="X (mm)", ylabel="Z (mm)")
    ax.axis("equal")
    ax.legend()
    for index, q in enumerate([0.0, 0.4, 0.7854], start=4):
        ax = fig.add_subplot(2, 3, index, projection="3d")
        draw_3d(ax, robot.meshes({"L_gripper_j1": q}, hand="L"), f"Assembly | q = {q:.4f} rad")
    fig.suptitle("Dexmate custom finger review | actual URDF + mesh geometry", fontsize=17)
    fig.text(
        0.5, 0.015, "Blue = custom TPU. Grey = retained hardware. Offline geometry preview; no physics simulation.",
        ha="center", fontsize=10,
    )
    fig.tight_layout(rect=(0, 0.03, 1, 0.95))
    fig.savefig(output / "finger_review.png", dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(1, 3, figsize=(15, 6))
    for ax, q in zip(axes, [0.0, 0.005, 0.01]):
        for curves, color in zip(finger_polygons(robot, q), [BLUE, ORANGE]):
            for curve in curves:
                ax.plot(curve[:, 0], curve[:, 1], color=color, lw=1)
        ax.set(xlim=(-5, 5), ylim=(155, 181), xlabel="X (mm)", ylabel="Z (mm)", title=f"q = {q:.3f} rad")
        ax.set_aspect("equal")
        ax.axvline(0, color="#999999", lw=0.5)
        ax.grid(alpha=0.2)
    fig.suptitle("Closing clearance | centre section Y = 0 | blue/orange = opposing custom fingers")
    fig.tight_layout()
    fig.savefig(output / "closing_clearance.png", dpi=150)
    plt.close(fig)
    fig = plt.figure(figsize=(12, 8))
    q = {"head_j1": 0.5, "L_gripper_j1": 0.4, "R_gripper_j1": 0.4}
    draw_3d(fig.add_subplot(121, projection="3d"), robot.meshes(q), "Full robot | front oblique", azim=-55)
    draw_3d(fig.add_subplot(122, projection="3d"), robot.meshes(q), "Full robot | reverse oblique", azim=125)
    fig.suptitle("Original mount transforms retained | head = [0.5, 0, 0], arms = 0 rad")
    fig.tight_layout()
    fig.savefig(output / "robot_mounts.png", dpi=140)
    plt.close(fig)
    printing_path = ASSETS / "meshes/bump_v2/printing/gripper_finger_26mm_bump_minus_0p8mm_print.stl"
    if printing_path.exists():
        printing = trimesh.load_mesh(printing_path)
        printing.apply_scale(0.001)
        initial = trimesh.load_mesh(ASSETS / "meshes/custom/gripper_finger_tpu_26mm_uniform_width_raise_2mm.stl")
        initial.apply_scale(0.001)
        green = np.array([0.12, 0.65, 0.43])
        fig = plt.figure(figsize=(14, 7))
        draw_3d(fig.add_subplot(131, projection="3d"), [(new, BLUE)], "Simulation | bump -0.5 mm")
        draw_3d(fig.add_subplot(132, projection="3d"), [(printing, green)], "Print STL | bump -0.8 mm")
        ax = fig.add_subplot(133)
        for mesh, color, label in [(initial, ORANGE, "Previous"), (new, BLUE, "Sim -0.5 mm"),
                                   (printing, green, "Print -0.8 mm")]:
            section = mesh.section(plane_normal=[0, 1, 0], plane_origin=[0, 0, 0])
            for index, curve in enumerate(section.discrete):
                ax.plot(curve[:, 0] * 1000, curve[:, 2] * 1000, color=color, lw=1.5,
                        label=label if index == 0 else None)
        ax.set(xlim=(23.6, 26.7), ylim=(147, 150.7), xlabel="X (mm)", ylabel="Z (mm)",
               title="Raised contact face | tip detail")
        ax.set_aspect("equal")
        ax.legend(loc="lower left")
        ax.grid(alpha=0.2)
        fig.suptitle("Approved variants | 26 mm width and mounting geometry preserved", fontsize=15)
        fig.tight_layout()
        fig.savefig(output / "bump_revision.png", dpi=160)
        plt.close(fig)


def interactive_preview(robot, output):
    def pack(array):
        return base64.b64encode(np.asarray(array, dtype="<f4").tobytes()).decode("ascii")

    upstream = Robot(ASSETS / "upstream/robots/humanoid/vega_1u/vega_1u_gripper.urdf")
    objects = []
    for mode, model in [("custom", robot), ("original", upstream)]:
        for link, entries in model.geometry.items():
            if not link.startswith("L_gripper"):
                continue
            joint = next(j for j in model.root.findall("joint") if j.find("child").get("link") == link)
            base = link == "L_gripper_base"
            transform = np.eye(4) if base else origin(joint.find("origin"))
            axis = [0, 0, 0] if base else np.fromstring(joint.find("axis").get("xyz"), sep=" ").tolist()
            if base:
                color = [0.58, 0.62, 0.68]
            else:
                color = BLUE.tolist() if link.endswith("l1") else ORANGE.tolist()
            for _, mesh in entries:
                objects.append({
                    "mode": mode, "base": base, "color": color,
                    "transform": transform.T.flatten().tolist(), "axis": axis,
                    "positions": pack(mesh.triangles), "normals": pack(np.repeat(mesh.face_normals, 3, axis=0)),
                })
            if mode == "custom" and not base:
                for component in ("tpu", "metal"):
                    mesh = robot.collision_mesh(link, component)
                    objects.append({
                        "mode": "collision", "base": False, "color": color,
                        "transform": transform.T.flatten().tolist(), "axis": axis,
                        "positions": pack(mesh.triangles), "normals": pack(np.repeat(mesh.face_normals, 3, axis=0)),
                    })
    samples = json.loads((output / "clearance.json").read_text())["samples"]
    descriptions = []
    for sample in samples[:3]:
        overlap = sample["max_sampled_overlap_x_mm"]
        result = f"最大 X 向重叠约 {overlap:.3f} mm" if overlap else "该截面采样未发现重叠"
        if not overlap and sample.get("min_sampled_gap_x_mm") is not None:
            result += f"，最小 X 向间隙约 {sample['min_sampled_gap_x_mm']:.3f} mm"
        descriptions.append(f"q = {sample['joint_rad']:.3f} rad 时，{result}。")
    template = Path(__file__).with_name("dexmate_preview.html").read_text()
    template = template.replace("__CLOSURE_SUMMARY__", " ".join(descriptions))
    (output / "interactive.html").write_text(template.replace("__MESH_DATA__", json.dumps(objects)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    robot = Robot(ASSETS / "vega_1u_gripper_flexitac.urdf")
    report = {
        "urdf_sha256": hashlib.sha256((ASSETS / "vega_1u_gripper_flexitac.urdf").read_bytes()).hexdigest(),
        "asset_manifest_sha256": hashlib.sha256((ASSETS / "manifest.json").read_bytes()).hexdigest(),
        "method": "TPU centre-section Y=0; solid intervals sampled every 0.025 mm along Z; not a 3D collision solver",
        "samples": [clearance(robot, q) for q in [0, 0.005, 0.01, 0.02, 0.4, 0.7854]],
        "physics_validated": False,
    }
    (args.output / "clearance.json").write_text(json.dumps(report, indent=2) + "\n")
    make_previews(robot, args.output)
    interactive_preview(robot, args.output)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
