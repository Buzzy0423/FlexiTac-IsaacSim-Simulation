"""Extend the existing sim/print contact regions to 66 x 26 mm, preserving their heights and mounts."""

import argparse
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from build_dexmate_assets import build_fingers, sha256
from reduce_dexmate_bump import PRINT_NAME, SIM_NAME

REVISION = "pad_66x26_v3"
NAMES = {
    "simulation": "gripper_finger_pad_66x26_bump_minus_0p5mm_sim.stl",
    "printing": "gripper_finger_pad_66x26_bump_minus_0p8mm_print.stl",
}


def extend_contact_region(mesh):
    """Lengthen only the distal contact section along Z; the proximal mounting section is fixed."""
    vertices = mesh.vertices.copy()
    face = np.isclose(vertices[:, 0], vertices[:, 0].max(), rtol=0, atol=1e-5)
    if face.sum() != 4:
        raise ValueError("Expected four corners on the rectangular contact plateau")
    start, end = vertices[face, 2].min(), vertices[face, 2].max()
    length = end - start
    if not np.isclose(length, 60.764068603515625, atol=1e-5) or not np.isclose(mesh.extents[1], 26.0):
        raise ValueError("Unexpected source contact dimensions")
    extension = 66.0 - length
    result = mesh.copy()
    fraction = np.clip((vertices[:, 2] - start) / length, 0.0, 1.0)
    result.vertices[:, 2] += extension * fraction
    if not np.array_equal(result.vertices[:, :2], vertices[:, :2]):
        raise ValueError("Width or surface height changed")
    if not np.array_equal(result.vertices[vertices[:, 2] <= start], vertices[vertices[:, 2] <= start]):
        raise ValueError("Proximal mounting geometry changed")
    if not result.is_watertight or not result.is_winding_consistent or result.volume <= 0:
        raise ValueError("Extended mesh must remain a closed, consistently oriented solid")
    if not np.isclose(np.ptp(result.vertices[face, 2]), 66.0, atol=1e-5):
        raise ValueError("Contact plateau is not 66 mm long")
    return result, {
        "old_plateau_length_mm": float(length), "plateau_length_mm": 66.0, "width_mm": 26.0,
        "extension_mm": float(extension), "unchanged_proximal_z_max_mm": float(start),
        "plateau_z_mm": [float(start), float(start + 66)],
        "plateau_x_mm": float(result.bounds[1, 0]),
        "bounds_mm": result.bounds.tolist(), "volume_mm3": float(result.volume),
        "source_volume_mm3": float(mesh.volume), "unchanged_proximal_vertices": int((vertices[:, 2] <= start).sum()),
        "watertight": bool(result.is_watertight), "winding_consistent": bool(result.is_winding_consistent),
    }


def activate(assets, revision, mapping):
    urdf = assets / "vega_1u_gripper_flexitac.urdf"
    manifest_path = assets / "manifest.json"
    shutil.copyfile(urdf, revision / "previous_urdf.xml")
    shutil.copyfile(manifest_path, revision / "previous_manifest.json")
    tree = ET.parse(urdf, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    updated = 0
    for mesh in tree.getroot().findall(".//mesh"):
        old = Path(mesh.get("filename"))
        if old.parent == Path("meshes/bump_v2/simulation") and old.name.startswith("gripper_l"):
            mesh.set("filename", str(Path("meshes") / REVISION / "simulation" / old.name))
            updated += 1
    if updated != 12:
        raise ValueError(f"Expected 12 visual/collision references across four fingers, got {updated}")
    tree.write(urdf, encoding="utf-8", xml_declaration=True)
    manifest = json.loads(manifest_path.read_text())
    manifest["schema_version"] = 3
    manifest["geometry_revision"] = REVISION
    manifest["geometry_revision_record"] = str((revision / "revision.json").relative_to(assets))
    sim = revision / "simulation" / NAMES["simulation"]
    manifest["custom_source"] = {
        "filename": sim.name, "path": str(sim.relative_to(assets)), "sha256": sha256(sim), "units": "mm",
    }
    manifest["finger_mapping"] = mapping
    manifest["files_sha256"] = {
        str(p.relative_to(assets)): sha256(p) for p in sorted(assets.rglob("*"))
        if p.is_file() and p != manifest_path and p.suffix != ".md"
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")


def preview(assets, output):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    output.mkdir(parents=True, exist_ok=False)
    revision = assets / "meshes" / REVISION
    fig, (side, front) = plt.subplots(1, 2, figsize=(10, 9))
    for path, label, color in (
        (assets / "meshes/bump_v2/simulation" / SIM_NAME, "Previous sim (60.76 mm)", "#888888"),
        (revision / "simulation" / NAMES["simulation"], "New sim (66 mm)", "#167aba"),
        (revision / "printing" / NAMES["printing"], "New print (66 mm)", "#dc742d"),
    ):
        mesh = trimesh.load_mesh(path, process=True)
        section = mesh.section(plane_normal=[0, 1, 0], plane_origin=[0, 0, 0])
        for i, line in enumerate(section.discrete):
            side.plot(line[:, 0], line[:, 2], color=color, label=label if i == 0 else None,
                      linewidth=1.4, linestyle="--" if color == "#888888" else "-")
    sim = trimesh.load_mesh(revision / "simulation" / NAMES["simulation"], process=True)
    face = sim.vertices[np.isclose(sim.vertices[:, 0], sim.bounds[1, 0], rtol=0, atol=1e-5)]
    y0, z0 = face[:, 1].min(), face[:, 2].min()
    front.add_patch(Rectangle((y0, z0), 26, 66, facecolor="#e2f0fa", edgecolor="#167aba", linewidth=2))
    front.add_patch(Rectangle((y0 + 1, z0 + 1), 24, 64, fill=False, edgecolor="#319255", linewidth=2))
    front.axhline(z0 + 60.764068603515625, color="#888888", linestyle="--", label="Previous plateau tip")
    front.text(y0 + 13, z0 + 34, "FlexiTac outline\n64 x 24 mm\n\n1 mm margin", ha="center")
    front.set_xlim(y0 - 4, y0 + 30)
    front.set_ylim(z0 - 3, z0 + 70)
    side.set_title("Centre section: height and mounting preserved")
    side.set_xlabel("Local X (mm)")
    front.set_title("New raised face: 66 x 26 mm")
    front.set_xlabel("Local Y (mm)")
    for axis in (side, front):
        axis.set_aspect("equal")
        axis.set_ylabel("Local Z (mm), towards fingertip")
        axis.grid(alpha=0.2)
        axis.legend(loc="lower left", fontsize=8)
    fig.suptitle("Dexmate finger revision | sim -0.5 mm / print -0.8 mm")
    fig.tight_layout()
    fig.savefig(output / "finger_66x26_review.png", dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assets", type=Path,
                        default=Path(__file__).resolve().parents[1] / "Isaacsim_tactile_env/assets/dexmate")
    parser.add_argument("--activate", action="store_true")
    parser.add_argument("--preview-output", type=Path, help="New directory for the mesh comparison figure")
    parser.add_argument("--preview-only", action="store_true", help="Preview an already built revision")
    args = parser.parse_args()
    assets = args.assets.resolve()
    if args.preview_only:
        if args.preview_output is None:
            parser.error("--preview-only requires --preview-output")
        preview(assets, args.preview_output)
        return
    previous = assets / "meshes/bump_v2"
    hashes = json.loads((previous / "revision.json").read_text())["files_sha256"]
    record = {
        "schema_version": 1, "units_stl": "mm", "sensor_outline_mm": [64.0, 24.0],
        "sensor_edge_margin_mm": 1.0,
        "operation": "Keep Z<=plateau start fixed; stretch contact section along Z to 66 mm; translate distal cap",
        "height_policy": "Preserve bump_v2: simulation -0.5 mm, printing -0.8 mm relative to original user STL",
        "inertia_status": "URDF mass/inertias retained as provisional; not recomputed from unknown material density",
        "variants": {},
    }
    meshes = {}
    for kind, name in (("simulation", SIM_NAME), ("printing", PRINT_NAME)):
        source = previous / kind / name
        if sha256(source) != hashes[f"{kind}/{name}"]:
            raise ValueError(f"Source hash mismatch: {source}")
        meshes[kind], info = extend_contact_region(trimesh.load_mesh(source, process=True))
        record["variants"][kind] = dict(info, source=str(source.relative_to(assets)), source_sha256=sha256(source))
    revision = assets / "meshes" / REVISION
    revision.mkdir(exist_ok=False)
    for kind, mesh in meshes.items():
        directory = revision / kind
        directory.mkdir()
        mesh.export(directory / NAMES[kind])
    mapping = build_fingers(
        assets / "upstream", revision / "simulation" / NAMES["simulation"], revision / "simulation",
        distal_extension_mm=record["variants"]["simulation"]["extension_mm"],
    )
    record["files_sha256"] = {
        str(p.relative_to(revision)): sha256(p) for p in sorted(revision.rglob("*")) if p.is_file()
    }
    (revision / "revision.json").write_text(json.dumps(record, indent=2) + "\n")
    if args.activate:
        activate(assets, revision, mapping)
    if args.preview_output:
        preview(assets, args.preview_output)
    print(json.dumps(record, indent=2))


if __name__ == "__main__":
    main()
