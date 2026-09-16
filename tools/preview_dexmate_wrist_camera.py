"""Review the supplied wrist-camera assembly without changing maintained assets.

Run in dex_wire (trimesh 4, scipy, matplotlib and coal). Outputs are offline
visuals and triangle-surface distance checks, not a physics qualification.
"""

import argparse
import base64
import hashlib
import json
from pathlib import Path

import coal
from preview_dexmate import ASSETS, Robot, draw_3d, np, origin, plt, trimesh
from scipy.spatial import cKDTree


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def collision_object(mesh):
    vertices, faces = coal.StdVec_Vec3s(), coal.StdVec_Triangle()
    for vertex in mesh.vertices:
        vertices.append(vertex)
    for face in mesh.faces:
        faces.append(coal.Triangle(*map(int, face)))
    model = coal.BVHModelOBBRSS()
    model.beginModel(len(mesh.faces), len(mesh.vertices))
    model.addSubModel(vertices, faces)
    model.endModel()
    return coal.CollisionObject(model)


def distance(a, b):
    request, result = coal.DistanceRequest(), coal.DistanceResult()
    value = coal.distance(a, b, request, result)
    return float(value * 1000)


def sampled_penetration_mm(surface, solid):
    """Check vertices/face centres by solid angle; no optional rtree dependency.

    Both input meshes must be watertight and in millimetres. This is a finite
    sampling check, not an exact Boolean intersection-volume measurement.
    """
    if not surface.is_watertight or not solid.is_watertight:
        raise ValueError("Solid-angle check requires closed meshes")
    points = np.concatenate([surface.vertices, surface.triangles_center])
    points = points[np.all((points >= solid.bounds[0]) & (points <= solid.bounds[1]), axis=1)]
    inside = []
    for index in range(0, len(points), 32):
        batch = points[index:index + 32]
        offsets = solid.triangles[None, :, :, :] - batch[:, None, None, :]
        a, b, c = offsets[:, :, 0, :], offsets[:, :, 1, :], offsets[:, :, 2, :]
        la, lb, lc = (np.linalg.norm(v, axis=-1) for v in (a, b, c))
        numerator = np.einsum("ijk,ijk->ij", a, np.cross(b, c))
        denominator = (la * lb * lc + np.einsum("ijk,ijk->ij", a, b) * lc
                       + np.einsum("ijk,ijk->ij", b, c) * la + np.einsum("ijk,ijk->ij", c, a) * lb)
        winding = np.arctan2(numerator, denominator).sum(axis=1) / (2 * np.pi)
        inside.extend(batch[np.abs(winding) > 0.5])
    distances = []
    for index in range(0, len(inside), 16):
        distances.extend(trimesh.proximity.closest_point_naive(solid, np.array(inside[index:index + 16]))[1])
    distances = np.array(distances)
    return {
        "samples_in_other_bbox": len(points),
        "inside_over_0p01mm": int((distances > 0.01).sum()),
        "maximum_sampled_penetration_mm": float(distances.max()) if len(distances) else 0.0,
        "method": "Vertices and triangle centres; solid angle and nearest triangle, tolerance 0.01 mm",
    }


def components(robot):
    """Remove two disconnected official camera-mount parts, not a spatial crop.

    Identification uses measured CAD-frame component bounds. Abort if the
    upstream topology changes, so unrelated base hardware cannot be removed.
    """
    visual = robot.root.find("link[@name='L_gripper_base']/visual[2]")
    transform = origin(visual.find("origin"))
    path = ASSETS / visual.find("geometry/mesh").get("filename")
    scene = trimesh.load_scene(path)
    kept, removed = [], []
    expected = [
        np.array([[-96.14729, -18.90316, -41.33494], [9.20064, 49.56585, 40.94223]]),
        np.array([[-71.91305, 3.94663, -17.29989], [-50.56300, 41.14174, 16.89999]]),
    ]
    for node in scene.graph.nodes_geometry:
        matrix, name = scene.graph[node]
        mesh = scene.geometry[name].copy()
        mesh.apply_transform(matrix)
        mesh.merge_vertices()
        for part in mesh.split(only_watertight=False):
            if any(np.allclose(part.bounds * 1000, bounds, atol=0.01) for bounds in expected):
                removed.append(part)
            else:
                kept.append(part)
    if len(removed) != 2:
        raise ValueError(f"Expected exactly two old mount components; found {len(removed)}")
    return transform, kept, removed, path


def hand_entries(robot, kept, transform, q, custom):
    poses = robot.poses({"L_gripper_j1": q})
    reference = np.linalg.inv(poses["L_gripper_base"])
    entries = []
    for link, meshes in robot.geometry.items():
        if not (link.startswith("L_gripper") or link == "L_camera_link"):
            continue
        if custom and link == "L_camera_link":
            continue
        if custom and link == "L_gripper_base":
            # First visual is the retained arm connector; remaining entries are
            # the four material groups of gripper_base.glb.
            connector = robot.root.find("link[@name='L_gripper_base']/visual")
            scene = trimesh.load_scene(ASSETS / connector.find("geometry/mesh").get("filename"))
            meshes = []
            for node in scene.graph.nodes_geometry:
                matrix, name = scene.graph[node]
                mesh = scene.geometry[name].copy()
                mesh.apply_transform(origin(connector.find("origin")) @ matrix)
                meshes.append((name, mesh))
            mesh = trimesh.util.concatenate(kept)
            mesh.apply_transform(transform)
            meshes.append(("retained_base", mesh))
        for name, mesh in meshes:
            mesh = mesh.copy()
            mesh.apply_transform(reference @ poses[link])
            color = [0.15, 0.55, 0.85] if name == "custom_tpu" else [0.60, 0.64, 0.69]
            entries.append((link, mesh, np.array(color)))
    return entries


def write_html(robot, kept, transform, additions, output):
    def pack(array):
        return base64.b64encode(np.asarray(array, dtype="<f4").tobytes()).decode("ascii")

    objects = []
    for mode in ("original", "custom"):
        entries = hand_entries(robot, kept, transform, 0, mode == "custom")
        if mode == "custom":
            entries += additions
        for link, mesh, color in entries:
            matrix, axis = np.eye(4), np.zeros(3)
            if link in ("L_gripper_l1", "L_gripper_l2"):
                joint = robot.root.find(f"joint[@name='L_gripper_j{link[-1]}']")
                matrix = origin(joint.find("origin"))
                axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                mesh = mesh.copy()
                mesh.apply_transform(np.linalg.inv(matrix))
            objects.append({
                "mode": mode, "base": link == "L_gripper_base",
                "positions": pack(mesh.triangles.reshape(-1, 3)),
                "normals": pack(np.repeat(mesh.face_normals, 3, axis=0)),
                "color": color.tolist(), "transform": matrix.T.flatten().tolist(), "axis": axis.tolist(),
            })
    template = Path(__file__).with_name("dexmate_preview.html").read_text()
    start = template.index("<header>")
    end = template.index('<script id="mesh-data"')
    template = template[:start] + '''<header><h1>Dexmate · 自装腕相机替换预览</h1>
<p>真实 STL / GLB 几何；橙色为新支架，深色为相机，蓝色为现有 TPU。拖动旋转，滚轮缩放，无需网络。</p></header>
<div class="controls"><select id="mode" aria-label="模型"><option value="custom">新支架 + C960 外壳</option>
<option value="original">官方腕相机对照</option></select>
<label><input type="checkbox" id="base" checked> 显示夹爪底座</label>
<label>开合 q <input id="q" type="range" min="0" max="0.7854" step="0.0001" value="0.4"></label>
<span id="status"></span><button id="closed">闭合</button><button id="small">q = 0.01</button>
<button id="front">正视图</button><button id="side">侧视图</button><button id="reset">复位视角</button></div>
<canvas id="canvas"></canvas><div class="note">已移除官方 camera_link 外观及底座内两块旧安装组件。
本页是离线几何预览，不包含物理仿真、标定光轴或完整运动避碰验证；正式 URDF 保持原样。<br>
<a href="comparison.png">新旧对照</a> · <a href="custom_views.png">新安装多视角</a> ·
<a href="review.json">尺寸、来源和几何检查</a> · <a href="custom_assembly.glb">安装 GLB</a></div>
''' + template[end:]
    template = template.replace("Dexmate 夹爪几何确认", "Dexmate 腕相机替换预览")
    template = template.replace("__MESH_DATA__", json.dumps(objects, separators=(",", ":")))
    (output / "review.html").write_text(template)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    baseline = ASSETS / "meshes/wrist_c960_v1/previous_urdf.xml"
    robot = Robot(baseline if baseline.exists() else ASSETS / "vega_1u_gripper_flexitac.urdf", asset_root=ASSETS)
    transform, kept, removed, base_path = components(robot)
    camera_path = ASSETS / "meshes/custom/camera_in_mount_frame_mm.stl"
    mount_path = ASSETS / "meshes/custom" / (
        "dexmate_c960_base_camera_mount_skate_ramp_full_base_coverage_"
        "left_dual_limit_2mm_w16mm_tangent_YZ24deg.stl"
    )
    report = {
        "scope": "Offline geometry; no maintained asset changes, physics or optical calibration",
        "sources_sha256": {str(p.relative_to(ASSETS)): sha256(p) for p in
                           [camera_path, mount_path, base_path, robot.path]},
        "source_units": "millimetres; scale 0.001 to metres",
        "cad_to_gripper_base_matrix": transform.tolist(),
        "removed_official_components_cad_bounds_mm": [(m.bounds * 1000).tolist() for m in removed],
        "parts": {},
    }
    additions = []
    for name, path, color in [("custom_camera", camera_path, [0.16, 0.22, 0.28]),
                              ("custom_mount", mount_path, [0.95, 0.50, 0.13])]:
        mesh = trimesh.load_mesh(path)
        report["parts"][name] = {"bounds_mm": mesh.bounds.tolist(), "extents_mm": mesh.extents.tolist(),
                                  "watertight": mesh.is_watertight, "triangles": len(mesh.faces)}
        mesh.apply_scale(0.001)
        if name == "custom_mount":
            old = max(removed, key=lambda m: len(m.faces))
            distances = cKDTree(mesh.vertices).query(old.vertices)[0]
            report["old_bracket_vertex_match"] = {
                "within_0p01mm": int((distances < 0.00001).sum()), "old_vertices": len(old.vertices),
                "method": "Nearest new vertex, in original CAD frame; supports unchanged installation interface",
            }
        mesh.apply_transform(transform)
        report["parts"][name]["gripper_base_bounds_mm"] = (mesh.bounds * 1000).tolist()
        additions.append((name, mesh, np.array(color)))
    # The two hands use identical local gripper and camera transforms in this URDF.
    left = robot.root.find("joint[@name='L_camera_joint']/origin").attrib
    right = robot.root.find("joint[@name='R_camera_joint']/origin").attrib
    report["left_right_official_camera_origins_identical"] = left == right
    camera_mm, mount_mm = trimesh.load_mesh(camera_path), trimesh.load_mesh(mount_path)
    report["camera_mount_sampled_penetration"] = {
        "camera_in_mount": sampled_penetration_mm(camera_mm, mount_mm),
        "mount_in_camera": sampled_penetration_mm(mount_mm, camera_mm),
    }
    print("Building triangle-distance audit", flush=True)
    fixed = [(name, collision_object(mesh)) for name, mesh, _ in additions]
    report["camera_mount_surface_distance_mm"] = distance(fixed[0][1], fixed[1][1])
    report["fixed_hardware_surface_distances_mm"] = {}
    base_entries = hand_entries(robot, kept, transform, 0, True)
    hardware = trimesh.util.concatenate([m for link, m, _ in base_entries if link == "L_gripper_base"])
    hardware_object = collision_object(hardware)
    for name, obj in fixed:
        report["fixed_hardware_surface_distances_mm"][name] = distance(obj, hardware_object)
    report["finger_sweep"] = []
    for q in np.linspace(0, 0.7854, 17):
        entries = hand_entries(robot, kept, transform, float(q), True)
        fingers = trimesh.util.concatenate([m for link, m, _ in entries if link != "L_gripper_base"])
        moving = collision_object(fingers)
        report["finger_sweep"].append({"q_rad": float(q), **{
            name + "_surface_distance_mm": distance(obj, moving) for name, obj in fixed}})
    report["distance_limitations"] = (
        "Triangle-surface distances on visual meshes (coal BVH); <=0 indicates surface contact/intersection. "
        "No solid-containment proof, fastener fit, deformation, PhysX, continuous-motion or whole-arm clearance test."
    )
    (args.output / "review.json").write_text(json.dumps(report, indent=2) + "\n")
    print("Rendering actual meshes", flush=True)
    official = hand_entries(robot, kept, transform, 0.4, False)
    custom = hand_entries(robot, kept, transform, 0.4, True) + additions
    fig = plt.figure(figsize=(14, 8), facecolor="white")
    for index, (entries, title) in enumerate([(official, "Official wrist camera"),
                                             (custom, "Custom mount + camera")], 1):
        ax = fig.add_subplot(1, 2, index, projection="3d")
        draw_3d(ax, [(m, c) for _, m, c in entries], title, elev=18, azim=-48)
        ax.set_proj_type("ortho")
        ax.set_box_aspect((1, 1, 1), zoom=1.3)
    fig.suptitle("Dexmate wrist-camera replacement | actual geometry", fontsize=18)
    fig.text(0.5, 0.035, "Orange: new mount | Dark: camera | Blue: existing TPU | Offline preview, q = 0.4 rad",
             ha="center", fontsize=11)
    fig.subplots_adjust(left=0, right=1, top=0.9, bottom=0.08, wspace=0)
    fig.savefig(args.output / "comparison.png", dpi=160)
    plt.close(fig)
    fig = plt.figure(figsize=(15, 11), facecolor="white")
    for index, (entries, title, elev, azim) in enumerate([
        (custom, "Installed | front oblique", 15, -48),
        (custom, "Installed | side", 0, 0),
        (custom, "Installed | reverse", 20, 130),
        (additions, "Two supplied parts | mounting detail", 25, -125),
    ], 1):
        ax = fig.add_subplot(2, 2, index, projection="3d")
        draw_3d(ax, [(m, c) for _, m, c in entries], title, elev=elev, azim=azim)
        ax.set_proj_type("ortho")
        ax.set_box_aspect((1, 1, 1), zoom=1.3)
    fig.suptitle("Custom wrist-camera installation | old mount removed", fontsize=18)
    fig.text(0.5, 0.018,
             "Actual meshes; mounting frame checked against shared CAD vertices. No calibrated optical axis.",
             ha="center", fontsize=10)
    fig.subplots_adjust(left=0, right=1, top=0.92, bottom=0.04, wspace=0, hspace=0.08)
    fig.savefig(args.output / "custom_views.png", dpi=150)
    plt.close(fig)
    scene = trimesh.Scene()
    for index, (name, mesh, color) in enumerate(custom):
        mesh = mesh.copy()
        mesh.visual = trimesh.visual.ColorVisuals(mesh, face_colors=np.r_[color * 255, 255].astype(np.uint8))
        scene.add_geometry(mesh, node_name=f"{name}_{index}", geom_name=f"{name}_{index}")
    scene.export(args.output / "custom_assembly.glb")
    write_html(robot, kept, transform, additions, args.output)
    print(json.dumps({k: report[k] for k in
                      ["camera_mount_surface_distance_mm", "fixed_hardware_surface_distances_mm"]}), flush=True)
    print(args.output / "review.html", flush=True)


if __name__ == "__main__":
    main()
