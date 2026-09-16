"""Tessellate the user-supplied official Vention 312098 v6 STEP without changing its scale."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SHELL, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS


def tessellate(solid):
    vertices, faces = [], []
    explorer = TopExp_Explorer(solid, TopAbs_FACE)
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        location = TopLoc_Location()
        triangulation = BRep_Tool.Triangulation_s(face, location)
        if triangulation is None:
            raise ValueError("STEP face could not be triangulated")
        offset = len(vertices)
        for i in range(1, triangulation.NbNodes() + 1):
            point = triangulation.Node(i).Transformed(location.Transformation())
            vertices.append([point.X(), point.Y(), point.Z()])
        for i in range(1, triangulation.NbTriangles() + 1):
            indices = list(triangulation.Triangle(i).Get())
            if face.Orientation() == TopAbs_REVERSED:
                indices.reverse()
            faces.append([offset + v - 1 for v in indices])
        explorer.Next()
    return trimesh.Trimesh(vertices=vertices, faces=faces, process=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    reader = STEPControl_Reader()
    if reader.ReadFile(str(args.input)) != IFSelect_RetDone:
        raise ValueError("Could not read STEP")
    reader.TransferRoots()
    shape = reader.OneShape()
    BRepMesh_IncrementalMesh(shape, 0.35, False, 0.25, True)
    solids = []
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    while explorer.More():
        solids.append(tessellate(explorer.Current()))
        explorer.Next()
    if len(solids) != 124:
        raise ValueError(f"Expected the audited v6 assembly's 124 solids, found {len(solids)}")
    solid_count = len(solids)
    # Feet and caps in this official export include open shells; omitting them would
    # shorten the assembly. Keep geometry outside solids as well.
    explorer = TopExp_Explorer(shape, TopAbs_SHELL, TopAbs_SOLID)
    while explorer.More():
        solids.append(tessellate(explorer.Current()))
        explorer.Next()
    explorer = TopExp_Explorer(shape, TopAbs_FACE, TopAbs_SHELL)
    while explorer.More():
        solids.append(tessellate(explorer.Current()))
        explorer.Next()
    # This STEP is in mm, +Y up, +Z along the 1530 mm tabletop length.
    # Identify the six official tabletop extrusions, rather than fitting/scaling the whole assembly.
    top = [m for m in solids if np.allclose(m.extents, [180, 22.5, 1530], atol=0.2)]
    if len(top) != 6:
        raise ValueError(f"Expected six 180 x 22.5 x 1530 mm tabletop extrusions, found {len(top)}")
    bounds = trimesh.util.concatenate(solids).bounds
    top_bounds = trimesh.util.concatenate(top).bounds
    transform = np.eye(4)
    transform[:3, :3] = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]]) * 0.001
    transform[:3, 3] = -np.array([top_bounds[0, 2], top_bounds[:, 0].mean(), bounds[0, 1]]) * 0.001
    scene = trimesh.Scene()
    arrays = {}
    solid_info = []
    for i, mesh in enumerate(solids):
        mesh.apply_transform(transform)
        name = f"solid_{i:03d}"
        # Neutral visualization material; original CAD colors are not imported by STEPControl.
        mesh.visual.face_colors = [160, 167, 175, 255]
        scene.add_geometry(mesh, node_name=name, geom_name=name)
        arrays[f"{name}_vertices"] = mesh.vertices.astype(np.float32)
        arrays[f"{name}_faces"] = mesh.faces.astype(np.int32)
        solid_info.append({"name": name, "triangles": len(mesh.faces), "bounds_m": mesh.bounds.tolist()})
    scene.export(args.output / "vention_312098_v6.glb")
    np.savez_compressed(args.output / "vention_312098_v6_meshes.npz", **arrays)
    manifest = {
        "schema_version": 1,
        "source_file": args.input.name,
        "source_sha256": hashlib.sha256(args.input.read_bytes()).hexdigest(),
        "official_design": "AS-JF-312098 v6",
        "source_url": "https://vention.com/designs/t-slot-mounting-table-1530-x-1080-x-790-mm-312098",
        "source_units": "mm",
        "output_units": "m",
        "linear_deflection_mm": 0.35,
        "angular_deflection_rad": 0.25,
        "source_bounds_mm": bounds.tolist(),
        "step_to_workspace_transform": transform.tolist(),
        "frame": "Origin at nominal tabletop near short edge center, on floor; +X into table, +Z up",
        "tabletop_height_m": float((top_bounds[1, 1] - bounds[0, 1]) * 0.001),
        "bounds_m": scene.bounds.tolist(),
        "extents_m": scene.extents.tolist(),
        "solid_count": solid_count,
        "component_count": len(solids),
        "triangle_count": sum(len(m.faces) for m in solids),
        "material_status": "Neutral gray visualization; official geometry preserved, CAD colors not imported",
        "solids": solid_info,
        "outputs_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in args.output.iterdir()},
    }
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({k: manifest[k] for k in ["solid_count", "triangle_count", "extents_m", "tabletop_height_m"]}))


if __name__ == "__main__":
    main()
