"""Build the project-owned Vega-1U geometry snapshot; requires numpy and trimesh."""

import argparse
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh

URDF_REL = Path("robots/humanoid/vega_1u/vega_1u_gripper.urdf")
URDF_SHA256 = "79c95c997589b3af3b7057947b4417f155b23520f3c924a0fb82caea22443e40"
TPU_NAMES = {1: "Mesh_1.002", 2: "Mesh_1.001"}


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def geometry_in_link(scene, geometry_name):
    nodes = [n for n in scene.graph.nodes_geometry if scene.graph[n][1] == geometry_name]
    if len(nodes) != 1:
        raise ValueError(f"Expected one node for {geometry_name}, got {nodes}")
    transform, _ = scene.graph[nodes[0]]
    mesh = scene.geometry[geometry_name].copy()
    mesh.apply_transform(transform)
    return mesh, transform


def snapshot(package, source_stl, output):
    source_urdf = package / URDF_REL
    if sha256(source_urdf) != URDF_SHA256:
        raise ValueError("Expected the audited dexmate-urdf 0.8.4 URDF")
    root = ET.parse(source_urdf).getroot()
    referenced = {(source_urdf.parent / m.attrib["filename"]).resolve() for m in root.findall(".//mesh")}
    referenced.add(source_urdf)
    source_files = {}
    for src in sorted(referenced):
        relative = src.relative_to(package)
        dest = output / "upstream" / relative
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src, dest)
        source_files[str(relative)] = sha256(src)
    license_path = package.parent / "dexmate_urdf-0.8.4.dist-info/licenses/LICENSE"
    shutil.copyfile(license_path, output / "upstream/LICENSE")
    custom = output / "meshes/custom"
    custom.mkdir(parents=True)
    shutil.copyfile(source_stl, custom / source_stl.name)
    return root, custom, source_files


def build_fingers(package, source_stl, custom, *, distal_extension_mm=0.0):
    finger = trimesh.load_mesh(source_stl, process=True)
    # STL carries no unit declaration; its 26-unit width identifies this input as millimetres.
    if not np.isclose(finger.extents[1], 26.0, atol=1e-3) or not finger.is_watertight:
        raise ValueError("Expected a watertight 26 mm wide finger STL")
    finger.apply_scale(0.001)
    mesh_dir = package / "robots/hands/dexs_gripper/meshes/visual"
    scenes = {i: trimesh.load_scene(mesh_dir / f"gripper_l{i}.glb", process=False) for i in (1, 2)}
    original, first_transform = geometry_in_link(scenes[1], TPU_NAMES[1])
    # Preserve the mounting frame; an explicit approved revision may extend the tip.
    expected_z = original.bounds[:, 2].copy()
    expected_z[1] += distal_extension_mm * 0.001
    if not np.allclose(finger.bounds[:, 2], expected_z, atol=1e-6):
        raise ValueError("Replacement Z interval differs from the original finger frame")
    if not np.isclose(finger.bounds[0, 0], original.bounds[0, 0], atol=1e-6):
        raise ValueError("Replacement X origin differs from the original finger frame")
    mapping = {}
    for i, scene in scenes.items():
        _, transform = geometry_in_link(scene, TPU_NAMES[i])
        source_to_link = transform @ np.linalg.inv(first_transform)
        replacement = finger.copy()
        replacement.apply_transform(source_to_link)
        metal, _ = geometry_in_link(scene, TPU_NAMES[i] + "_1")
        # Preserve the original metal component and its material, replacing only the TPU node.
        replacement.visual = trimesh.visual.ColorVisuals(
            mesh=replacement, face_colors=[65, 65, 65, 255]
        )
        visual = trimesh.Scene()
        visual.add_geometry(metal, geom_name="original_metal", node_name="original_metal")
        visual.add_geometry(replacement, geom_name="custom_tpu", node_name="custom_tpu")
        visual.export(custom / f"gripper_l{i}_visual.glb")
        replacement.export(custom / f"gripper_l{i}_tpu_collision.obj")
        # An explicit provisional proxy, avoiding the old full-finger collision envelope.
        metal.convex_hull.export(custom / f"gripper_l{i}_metal_collision.obj")
        mapping[f"gripper_l{i}"] = {
            "source_metres_to_link": source_to_link.tolist(),
            "custom_tpu_bounds_m": replacement.bounds.tolist(),
            "retained_metal_geometry": TPU_NAMES[i] + "_1",
        }
    return mapping


def customize(root):
    root.set("name", "vega_1u_gripper_flexitac")
    for mesh in root.findall(".//mesh"):
        source = (URDF_REL.parent / mesh.attrib["filename"])
        # Normalize the original relative traversal without retaining machine-specific paths.
        parts = []
        for part in source.parts:
            if part == "..":
                parts.pop()
            else:
                parts.append(part)
        mesh.set("filename", str(Path("upstream", *parts)))
    for side in ("L", "R"):
        for i in (1, 2):
            link = root.find(f"link[@name='{side}_gripper_l{i}']")
            link.find("visual/geometry/mesh").set("filename", f"meshes/custom/gripper_l{i}_visual.glb")
            for collision in list(link.findall("collision")):
                link.remove(collision)
            for component in ("tpu", "metal"):
                collision = ET.SubElement(link, "collision", name=f"{component}_collision")
                ET.SubElement(collision, "origin", xyz="0 0 0", rpy="0 0 0")
                geometry = ET.SubElement(collision, "geometry")
                ET.SubElement(geometry, "mesh", filename=f"meshes/custom/gripper_l{i}_{component}_collision.obj")
    root.insert(0, ET.Comment(
        " Project derivative of dexmate-urdf 0.8.4: 26 mm TPU fingers; see README.zh-CN.md. "
        "Kinematics and inertias retained; dynamics and camera calibration NOT validated. "
    ))
    ET.indent(root, space="  ")
    return ET.ElementTree(root)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-root", type=Path, required=True)
    parser.add_argument("--finger-stl", type=Path, required=True)
    parser.add_argument(
        "--output", type=Path, required=True, help="New directory; existing output is never overwritten"
    )
    args = parser.parse_args()
    package = args.package_root.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    root, custom, source_files = snapshot(package, args.finger_stl, output)
    mapping = build_fingers(package, args.finger_stl, custom)
    customize(root).write(output / "vega_1u_gripper_flexitac.urdf", encoding="utf-8", xml_declaration=True)
    manifest = {
        "schema_version": 1,
        "upstream_package": "dexmate-urdf",
        "upstream_version": "0.8.4",
        "upstream_urdf": str(URDF_REL),
        "upstream_files_sha256": source_files,
        "custom_source": {"filename": args.finger_stl.name, "sha256": sha256(args.finger_stl), "units": "mm"},
        "build_versions": {"numpy": np.__version__, "trimesh": trimesh.__version__},
        "finger_mapping": mapping,
        "status": {
            "kinematics": "unchanged",
            "inertia": "inherited from upstream; not recalculated for custom TPU",
            "collision": "custom TPU surface plus convex hull of retained metal; provisional",
            "isaac_sim": "not validated",
            "camera_calibration": "not validated",
            "tactile_patches": "not added",
        },
        "files_sha256": {str(p.relative_to(output)): sha256(p) for p in sorted(output.rglob("*")) if p.is_file()},
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(output / "vega_1u_gripper_flexitac.urdf")


if __name__ == "__main__":
    main()
