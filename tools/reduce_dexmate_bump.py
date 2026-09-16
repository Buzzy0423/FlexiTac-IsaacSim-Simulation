"""Build and activate the approved -0.5 mm simulation / -0.8 mm printing finger revision."""

import argparse
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from build_dexmate_assets import build_fingers, sha256

SOURCE_NAME = "gripper_finger_tpu_26mm_uniform_width_raise_2mm.stl"
SOURCE_HASH = "6e9cf1534a1a4153a59a2303c0c7f2e63afb1bc11936219ceb45a89a7955d62e"
SIM_NAME = "gripper_finger_26mm_bump_minus_0p5mm_sim.stl"
PRINT_NAME = "gripper_finger_26mm_bump_minus_0p8mm_print.stl"


def reduce_bump(source, reduction_mm):
    """Move only the four corners of the raised rectangular contact face, in source millimetres."""
    mesh = source.copy()
    selected = np.isclose(mesh.vertices[:, 0], 26.296567916870117, atol=1e-5, rtol=0)
    if selected.sum() != 4:
        raise ValueError("Expected the four corners of the audited raised contact face")
    expected_z = [89.06929016113281, 149.83335876464844]
    if not np.allclose(np.unique(mesh.vertices[selected, 2]), expected_z, atol=1e-5):
        raise ValueError("Unexpected raised contact face interval")
    before = mesh.vertices.copy()
    mesh.vertices[selected, 0] -= reduction_mm
    if not mesh.is_watertight or not mesh.is_winding_consistent or mesh.volume <= 0:
        raise ValueError("Modified print mesh is not a valid closed solid")
    if not np.array_equal(mesh.vertices[~selected], before[~selected]):
        raise ValueError("Unexpected modification outside the contact face")
    if not np.isclose(mesh.extents[1], 26.0, atol=1e-5):
        raise ValueError("Finger width changed")
    return mesh


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--assets", type=Path,
        default=Path(__file__).resolve().parents[1] / "Isaacsim_tactile_env/assets/dexmate",
    )
    parser.add_argument("--activate", action="store_true", help="Point the project URDF at the new simulation meshes")
    args = parser.parse_args()
    assets = args.assets.resolve()
    source_path = assets / "meshes/custom" / SOURCE_NAME
    if sha256(source_path) != SOURCE_HASH:
        raise ValueError("Input STL differs from the audited user-supplied source")
    revision = assets / "meshes/bump_v2"
    revision.mkdir(parents=True, exist_ok=False)
    sim_dir = revision / "simulation"
    print_dir = revision / "printing"
    sim_dir.mkdir()
    print_dir.mkdir()
    source = trimesh.load_mesh(source_path, process=True)
    simulation = reduce_bump(source, 0.5)
    printing = reduce_bump(source, 0.8)
    simulation.export(sim_dir / SIM_NAME)
    printing.export(print_dir / PRINT_NAME)
    mapping = build_fingers(assets / "upstream", sim_dir / SIM_NAME, sim_dir)
    record = {
        "schema_version": 1,
        "source": str(source_path.relative_to(assets)),
        "source_sha256": SOURCE_HASH,
        "units_stl": "mm",
        "operation": "Translate four contact-face corners along local -X; preserve all other vertices and topology",
        "simulation_reduction_mm": 0.5,
        "printing_reduction_mm": 0.8,
        "simulation_plateau_x_mm": float(simulation.bounds[1, 0]),
        "printing_plateau_x_mm": float(printing.bounds[1, 0]),
        "simulation_bounds_mm": simulation.bounds.tolist(),
        "printing_bounds_mm": printing.bounds.tolist(),
        "print_vs_sim_surface_recess_mm": 0.3,
        "padding_note": "Print surface is 0.3 mm below sim; adding a 1 mm pad projects 0.7 mm beyond sim per finger",
        "files_sha256": {
            str(p.relative_to(revision)): sha256(p) for p in sorted(revision.rglob("*")) if p.is_file()
        },
    }
    (revision / "revision.json").write_text(json.dumps(record, indent=2) + "\n")
    if args.activate:
        urdf_path = assets / "vega_1u_gripper_flexitac.urdf"
        manifest_path = assets / "manifest.json"
        shutil.copyfile(urdf_path, revision / "previous_urdf.xml")
        shutil.copyfile(manifest_path, revision / "previous_manifest.json")
        tree = ET.parse(urdf_path, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
        for mesh in tree.getroot().findall(".//mesh"):
            filename = Path(mesh.get("filename"))
            if filename.parent == Path("meshes/custom") and filename.name.startswith("gripper_l"):
                mesh.set("filename", str((sim_dir / filename.name).relative_to(assets)))
        tree.write(urdf_path, encoding="utf-8", xml_declaration=True)
        manifest = json.loads(manifest_path.read_text())
        manifest["schema_version"] = 2
        manifest["geometry_revision"] = "bump_v2"
        manifest["geometry_revision_record"] = str((revision / "revision.json").relative_to(assets))
        manifest["custom_source"] = {
            "filename": SIM_NAME, "path": str((sim_dir / SIM_NAME).relative_to(assets)),
            "sha256": sha256(sim_dir / SIM_NAME), "units": "mm",
        }
        manifest["finger_mapping"] = mapping
        manifest["files_sha256"] = {
            str(p.relative_to(assets)): sha256(p) for p in sorted(assets.rglob("*"))
            if p.is_file() and p != manifest_path and p.suffix != ".md"
        }
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(record, indent=2))


if __name__ == "__main__":
    main()
