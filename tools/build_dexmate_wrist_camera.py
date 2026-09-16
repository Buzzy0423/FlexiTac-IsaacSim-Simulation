"""Build an isolated C960 candidate; activate the reviewed output with --activate."""

import argparse
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from preview_dexmate import ASSETS, Robot
from preview_dexmate_wrist_camera import components, sha256

REVISION = "wrist_c960_v1"


def mesh_element(link, kind, path, name):
    item = ET.SubElement(link, kind, name=name)
    ET.SubElement(item, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(ET.SubElement(item, "geometry"), "mesh", filename=str(path))


def write_visual(mesh, path, color):
    mesh = mesh.copy()
    mesh.visual = trimesh.visual.texture.TextureVisuals(material=trimesh.visual.material.PBRMaterial(
        baseColorFactor=color, metallicFactor=0.0, roughnessFactor=0.7))
    trimesh.Scene(mesh).export(path)


def build(output):
    output.mkdir(parents=True, exist_ok=False)
    baseline = ASSETS / "meshes" / REVISION / "previous_urdf.xml"
    urdf = baseline if baseline.exists() else ASSETS / "vega_1u_gripper_flexitac.urdf"
    robot = Robot(urdf, asset_root=ASSETS)
    transform, kept, removed, base_path = components(robot)
    camera_path = ASSETS / "meshes/custom/camera_in_mount_frame_mm.stl"
    mount_path = ASSETS / "meshes/custom" / (
        "dexmate_c960_base_camera_mount_skate_ramp_full_base_coverage_"
        "left_dual_limit_2mm_w16mm_tangent_YZ24deg.stl")
    record = {"revision": REVISION, "source_sha256": {
        str(p.relative_to(ASSETS)): sha256(p) for p in (urdf, camera_path, mount_path, base_path)},
        "status": "Nominal model, not physical calibration"}
    camera = trimesh.load_mesh(camera_path)
    forward = np.array([np.sin(np.deg2rad(24)), -np.cos(np.deg2rad(24)), 0])
    lens = (camera.face_normals @ forward > .999) & (camera.triangles_center @ forward > -36.72)
    if lens.sum() != 64:
        raise ValueError("Source lens-front topology changed; review optical origin")
    lens_center = np.average(camera.triangles_center[lens], axis=0, weights=camera.area_faces[lens])
    optical_cad = np.eye(4)
    right = np.array([0, 0, -1])
    optical_cad[:3, :3] = np.column_stack((right, np.cross(forward, right), forward))
    optical_cad[:3, 3] = lens_center * .001
    optical = transform @ optical_cad
    assert np.isclose(np.linalg.det(optical[:3, :3]), 1)
    rpy = trimesh.transformations.euler_from_matrix(optical)
    record.update(optical_to_gripper_base=optical.tolist(), lens_front_cad_mm=lens_center.tolist(),
                  optical_origin_policy="Lens front disc centroid; optical +Z outward, +X gripper right, +Y down",
                  removed_old_mount_bounds_mm=[(m.bounds * 1000).tolist() for m in removed])
    base = trimesh.util.concatenate(kept)
    base.apply_transform(transform)
    write_visual(base, output / "gripper_base_visual.glb", [155, 165, 180, 255])
    # Retain each substantial disconnected body as a separate convex collider.
    # Tiny disconnected tessellation fragments stay in the visual only.
    collision_paths = []
    for part in kept:
        if len(part.faces) <= 100:
            continue
        hull = part.convex_hull
        if hull.volume < 1e-10:
            continue
        hull.apply_transform(transform)
        path = output / f"base_collision_{len(collision_paths)}.obj"
        hull.export(path)
        collision_paths.append(path)
    record["base_collision_policy"] = {
        "type": "Per retained substantial CAD body convex hull, replacing the old mount-containing OBJ",
        "count": len(collision_paths), "limitation": "Approximation; original small recesses may be filled"}
    mount = trimesh.load_mesh(mount_path)
    mount.apply_scale(.001)
    mount.apply_transform(transform)
    write_visual(mount, output / "mount_visual.glb", [225, 115, 30, 255])
    mount.export(output / "mount_collision.obj")
    camera.apply_scale(.001)
    camera.apply_transform(np.linalg.inv(optical) @ transform)
    write_visual(camera, output / "camera_visual.glb", [40, 50, 62, 255])
    camera.export(output / "camera_collision.obj")
    tree = ET.parse(urdf, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    root = tree.getroot()
    for element in root:
        if not isinstance(element.tag, str) and element.text and "Kinematics and inertias retained" in element.text:
            element.text = (
                " Project derivative: custom TPU fingers and C960 wrist cameras. "
                "Arm/finger kinematics retained; wrist optical frames and camera inertia approximated. "
                "See manifest and scoped validation reports; camera parameters are not calibrated. "
            )
    for spec in root.findall(".//mesh"):
        spec.set("filename", str((ASSETS / spec.get("filename")).resolve()))
    for hand in ("L", "R"):
        link = root.find(f"link[@name='{hand}_gripper_base']")
        visual = link.findall("visual")[1]
        link.remove(visual)
        for item in list(link.findall("collision")):
            link.remove(item)
        mesh_element(link, "visual", output / "gripper_base_visual.glb", "retained_base")
        mesh_element(link, "visual", output / "mount_visual.glb", "c960_mount")
        for index, path in enumerate(collision_paths):
            mesh_element(link, "collision", path, f"retained_base_{index}")
        mesh_element(link, "collision", output / "mount_collision.obj", "c960_mount")
        joint_origin = root.find(f"joint[@name='{hand}_camera_joint']/origin")
        joint_origin.set("xyz", " ".join(f"{v:.12g}" for v in optical[:3, 3]))
        joint_origin.set("rpy", " ".join(f"{v:.12g}" for v in rpy))
        link = root.find(f"link[@name='{hand}_camera_link']")
        for item in list(link.findall("visual")) + list(link.findall("collision")):
            link.remove(item)
        mesh_element(link, "visual", output / "camera_visual.glb", "c960_camera")
        mesh_element(link, "collision", output / "camera_collision.obj", "c960_camera")
        mass = float(link.find("inertial/mass").get("value"))
        inertia = camera.moment_inertia * mass / camera.volume
        center = camera.center_mass
        link.find("inertial/origin").set("xyz", " ".join(str(v) for v in center))
        tensor = link.find("inertial/inertia")
        for key, i, j in [("ixx", 0, 0), ("iyy", 1, 1), ("izz", 2, 2),
                          ("ixy", 0, 1), ("ixz", 0, 2), ("iyz", 1, 2)]:
            tensor.set(key, str(inertia[i, j]))
    record["inertia_policy"] = (
        "Retain official camera mass 0.0253335276780861 kg as provisional; "
        "uniform mesh inertia about its centroid in new optical frame. Base mass/inertia inherited; "
        "new mount mass not independently identified. Not measured C960 mass."
    )
    ET.indent(tree)
    tree.write(output / "candidate.urdf", encoding="utf-8", xml_declaration=True)
    # The tactile sampler resolves its canonical finger source next to the URDF.
    candidate_manifest = json.loads((ASSETS / "manifest.json").read_text())
    candidate_manifest["custom_source"]["path"] = str(ASSETS / candidate_manifest["custom_source"]["path"])
    (output / "manifest.json").write_text(json.dumps(candidate_manifest, indent=2) + "\n")
    record["generated_sha256"] = {p.name: sha256(p) for p in sorted(output.iterdir()) if p.is_file()}
    (output / "revision.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({"output": str(output), "optical_xyz_m": optical[:3, 3].tolist(),
                      "optical_rpy_rad": rpy, "base_colliders": len(collision_paths)}, indent=2))


def activate(output):
    record = json.loads((output / "revision.json").read_text())
    for name, value in record["source_sha256"].items():
        if sha256(ASSETS / name) != value:
            raise ValueError(f"Source changed since candidate generation: {name}")
    for name, value in record["generated_sha256"].items():
        if sha256(output / name) != value:
            raise ValueError(f"Candidate changed: {name}")
    destination = ASSETS / "meshes" / REVISION
    destination.mkdir(exist_ok=False)
    for path in output.iterdir():
        if path.suffix in (".obj", ".glb"):
            shutil.copyfile(path, destination / path.name)
    urdf = ASSETS / "vega_1u_gripper_flexitac.urdf"
    manifest_path = ASSETS / "manifest.json"
    shutil.copyfile(urdf, destination / "previous_urdf.xml")
    shutil.copyfile(manifest_path, destination / "previous_manifest.json")
    tree = ET.parse(output / "candidate.urdf", parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    for mesh in tree.getroot().findall(".//mesh"):
        path = Path(mesh.get("filename"))
        target = destination / path.name if path.parent == output else path
        mesh.set("filename", str(target.relative_to(ASSETS)))
    ET.indent(tree)
    tree.write(urdf, encoding="utf-8", xml_declaration=True)
    record["generated_sha256"] = {p.name: sha256(p) for p in destination.iterdir() if p.suffix in (".obj", ".glb")}
    (destination / "revision.json").write_text(json.dumps(record, indent=2) + "\n")
    manifest = json.loads(manifest_path.read_text())
    manifest["wrist_camera_revision"] = str((destination / "revision.json").relative_to(ASSETS))
    manifest["status"].update(
        kinematics=("Arm/finger kinematics unchanged; wrist camera fixed optical frames "
                    "replaced by STL-based nominal poses"),
        inertia="Base/finger inertias inherited; camera uses inherited provisional mass and new uniform-mesh inertia",
        collision="Custom TPU/metal plus retained-base convex hulls and new mount/camera surfaces; provisional",
        isaac_sim="See scoped validation reports; asset generation alone does not validate dynamics",
        tactile_patches="Runtime FlexiTac patches configured through dexmate_workspace.json",
    )
    manifest["files_sha256"] = {str(p.relative_to(ASSETS)): sha256(p) for p in sorted(ASSETS.rglob("*"))
                                 if p.is_file() and p != manifest_path and p.suffix != ".md"}
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Activated {destination}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--activate", action="store_true")
    args = parser.parse_args()
    (activate if args.activate else build)(args.output.resolve())
