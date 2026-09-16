"""Import the project Dexmate URDF and validate driven grippers in Isaac Sim 5.1."""

import argparse
import copy
import hashlib
import json
import subprocess
import traceback
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from dexmate_camera_model import camera_parameters
from dexmate_collision import configure_collision, setup_contact_reporting
from dexmate_contact_regression import create_probes, run_contact_regression
from dexmate_grasp_regression import create_grasp_object, run_grasp_regression
from dexmate_tactile_validation import create_recorder, finish_tactile, sample_tactile

ROOT = Path(__file__).resolve().parents[1]


def prepare_urdf(source, output):
    """Keep the owned asset immutable; resolve paths and replace zero drive limits in a run copy."""
    root = ET.parse(source).getroot()
    overrides = {}
    mesh_dir = output.parent / "import_meshes"
    mesh_dir.mkdir()
    # Bake GLB scene-node transforms explicitly. The native importer does not reproduce
    # all nested GLB transforms in the official robot visuals.
    for link in root.findall("link"):
        for kind in ("visual", "collision"):
            for index, element in enumerate(list(link.findall(kind))):
                spec = element.find("geometry/mesh")
                if spec is None:
                    continue
                scene = trimesh.load(source.parent / spec.get("filename"), force="scene", process=False)
                scale = np.fromstring(spec.get("scale", "1 1 1"), sep=" ")
                for node_index, node in enumerate(scene.graph.nodes_geometry):
                    transform, geometry_name = scene.graph[node]
                    mesh = scene.geometry[geometry_name].copy()
                    mesh.apply_transform(transform)
                    mesh.apply_scale(scale)
                    target = mesh_dir / f"{link.get('name')}_{kind}_{index}_{node_index}.obj"
                    mesh.export(target, include_texture=False)
                    replacement = copy.deepcopy(element)
                    replacement.set("name", target.stem)
                    mesh_spec = replacement.find("geometry/mesh")
                    mesh_spec.set("filename", str(target))
                    mesh_spec.set("scale", "1 1 1")
                    if kind == "visual":
                        for material in list(replacement.findall("material")):
                            replacement.remove(material)
                        material = ET.SubElement(replacement, "material", name=target.stem)
                        source_material = getattr(mesh.visual, "material", None)
                        color = getattr(source_material, "baseColorFactor", None)
                        color = np.array(color if color is not None else [150, 160, 175, 255]) / 255.0
                        ET.SubElement(material, "color", rgba=" ".join(str(float(c)) for c in color))
                    link.append(replacement)
                link.remove(element)
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if limit is None:
            continue
        name = joint.get("name")
        if float(limit.get("effort", "0")) == 0:
            effort = 1.0 if "gripper" in name else 6000.0 if name == "Lift" else 300.0
            limit.set("effort", str(effort))
            overrides.setdefault(name, {})["effort"] = effort
        if float(limit.get("velocity", "0")) == 0:
            limit.set("velocity", "0.5")
            overrides.setdefault(name, {})["velocity"] = 0.5
    ET.indent(root)
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)
    return overrides


def spawn_official_table(stage, archive):
    """Static triangle colliders preserve the slots and frame of the official CAD."""
    from pxr import UsdGeom, UsdPhysics

    manifest = json.loads((archive.parent / "manifest.json").read_text())
    expected_hash = manifest["outputs_sha256"][archive.name]
    if hashlib.sha256(archive.read_bytes()).hexdigest() != expected_hash:
        raise ValueError("Table mesh hash does not match its conversion manifest")
    stage.DefinePrim("/World/VentionTable", "Xform")
    with np.load(archive, allow_pickle=False) as data:
        for item in manifest["solids"]:
            name = item["name"]
            mesh = UsdGeom.Mesh.Define(stage, f"/World/VentionTable/{name}")
            vertices, faces = data[f"{name}_vertices"], data[f"{name}_faces"]
            mesh.CreatePointsAttr(vertices)
            mesh.CreateFaceVertexIndicesAttr(faces.flatten())
            mesh.CreateFaceVertexCountsAttr(np.full(len(faces), 3, dtype=np.int32))
            mesh.CreateSubdivisionSchemeAttr("none")
            mesh.CreateDisplayColorAttr([(0.52, 0.55, 0.59)])
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    return {k: manifest[k] for k in ("source_sha256", "component_count", "triangle_count", "tabletop_height_m")}


def spawn_worktop_board(world, board, height, report):
    """Place the configured static board directly on the tabletop."""
    from isaacsim.core.api.objects import FixedCuboid

    support_height = height
    if board:
        if height is None:
            raise ValueError("Worktop board requires a table")
        dimensions = np.asarray(board["dimensions_m"], dtype=float)
        if dimensions.shape != (3,) or not np.isfinite(dimensions).all() or np.any(dimensions <= 0):
            raise ValueError("Board dimensions must be three positive finite values")
        center = np.array([*board["center_xy_m"], height + dimensions[2] / 2])
        world.scene.add(
            FixedCuboid(
                prim_path="/World/WorktopBoard",
                name="worktop_board",
                position=center,
                scale=dimensions,
                color=np.asarray(board["display_color"]),
            )
        )
        support_height = height + dimensions[2]
        report["worktop_board"] = {
            "dimensions_m": dimensions.tolist(),
            "center_m": center.tolist(),
            "bounds_m": [(center - dimensions / 2).tolist(), (center + dimensions / 2).tolist()],
            "top_height_m": float(support_height),
            "collision": "Static box",
        }
    report["cube_support_surface"] = "worktop_board" if board else "table"
    return support_height


def driver_arguments(args, report):
    """Guard the documented R535 Vulkan version workaround."""
    extra_args = []
    if args.workaround_r535_vulkan:
        driver = (
            subprocess.check_output(["nvidia-smi", "--query-gpu=driver_version", "--format=csv,noheader"], text=True)
            .strip()
            .splitlines()[0]
        )
        parts = [int(p) for p in driver.split(".")]
        if parts[0] != 535 or parts[1] < 256:
            raise ValueError("The documented R535 workaround does not apply to this driver")
        extra_args.append("--/rtx/verifyDriverVersion/enabled=false")
        report["driver_workaround"] = driver
    return extra_args


def create_scene(args, report, save):
    """Shared scene construction for validation and the reusable environment."""
    import omni.kit.commands
    import omni.replicator.core as rep
    from isaacsim.asset.importer.urdf import _urdf
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
    from isaacsim.core.prims import SingleArticulation
    from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

    world = World(stage_units_in_meters=1.0, physics_dt=1 / 120, rendering_dt=1 / 120)
    physics = world.get_physics_context()
    if getattr(args, "physics_mode", "cpu") == "gpu":
        # Keep CPU readback and USD transforms for the current NumPy/tactile API.
        physics.enable_gpu_dynamics(True)
        physics.set_broadphase_type("GPU")
        # The Dexmate convex decomposition exceeds the default 1024 pair events
        # during reset (PhysX reports a minimum of 1313). Keep headroom.
        physics.set_gpu_found_lost_aggregate_pairs_capacity(4096)
    if getattr(args, "ccd", None) is not None:
        physics.enable_ccd(args.ccd)
    stage = world.stage
    scene = report["workspace"]
    source = (args.workspace.parent / scene["robot"]["urdf"]).resolve()
    report["source_urdf_sha256"] = hashlib.sha256(source.read_bytes()).hexdigest()
    import_urdf = args.output / "runtime.urdf"
    report["provisional_drive_overrides"] = prepare_urdf(source, import_urdf)
    _, config = omni.kit.commands.execute("URDFCreateImportConfig")
    config.set_fix_base(True)
    config.set_merge_fixed_joints(False)
    config.set_make_default_prim(True)
    config.set_create_physics_scene(False)
    config.set_convex_decomp(True)
    config.set_self_collision(not args.disable_self_collision)
    report["self_collision_enabled"] = not args.disable_self_collision
    config.set_parse_mimic(True)
    ok, model = omni.kit.commands.execute("URDFParseFile", urdf_path=str(import_urdf), import_config=config)
    if not ok:
        raise RuntimeError("URDFParseFile failed")
    for name, joint in model.joints.items():
        joint.drive.set_drive_type(_urdf.UrdfJointDriveType.JOINT_DRIVE_FORCE)
        joint.drive.set_target_type(_urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION)
        joint.drive.set_strength(80.0 if "gripper" in name else 2000.0)
        joint.drive.set_damping(4.0 if "gripper" in name else 100.0)
        if "gripper_j2" in name:
            joint.drive.set_target_type(_urdf.UrdfJointTargetType.JOINT_DRIVE_NONE)
    report["status"] = "urdf_parsed"
    save()
    usd_path = args.output / "robot.usd"
    ok, _ = omni.kit.commands.execute(
        "URDFImportRobot",
        urdf_path=str(import_urdf),
        urdf_robot=model,
        import_config=config,
        dest_path=str(usd_path),
    )
    if not ok or not usd_path.exists():
        raise RuntimeError("URDFImportRobot failed")
    robot_prim = stage.DefinePrim("/World/Dexmate", "Xform")
    robot_prim.GetReferences().AddReference(str(usd_path))
    robot_xform = UsdGeom.Xformable(robot_prim)
    robot_xform.ClearXformOpOrder()
    robot_xform.AddTranslateOp().Set(Gf.Vec3d(*scene["robot"]["position_m"]))
    quat = scene["robot"]["orientation_wxyz"]
    robot_xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(quat[0], Gf.Vec3d(*quat[1:])))
    roots = [p.GetPath().pathString for p in Usd.PrimRange(robot_prim) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(roots) != 1:
        raise RuntimeError(f"Expected one articulation root, found {roots}")
    report["articulation_root"] = roots[0]
    report["mimic_schemas"] = {
        str(p.GetPath()): p.GetAppliedSchemas()
        for p in Usd.PrimRange(robot_prim)
        if any("Mimic" in schema for schema in p.GetAppliedSchemas())
    }
    meshes = [p for p in Usd.PrimRange(robot_prim, Usd.TraverseInstanceProxies()) if p.IsA(UsdGeom.Mesh)]
    report["imported_mesh_count"] = len(meshes)
    report["empty_meshes"] = [str(p.GetPath()) for p in meshes if not UsdGeom.Mesh(p).GetPointsAttr().Get()]
    if not meshes or report["empty_meshes"]:
        raise RuntimeError("Imported robot has missing geometry")
    configure_collision(stage, robot_prim, source, report, raw=args.raw_collisions)
    contact_subscription, contact_pairs = setup_contact_reporting(robot_prim, report)
    robot = world.scene.add(SingleArticulation(prim_path=roots[0], name="dexmate"))
    world.scene.add(
        FixedCuboid(
            prim_path="/World/Ground",
            name="ground",
            position=np.array([0, 0, -0.05]),
            scale=np.array([6.0, 6.0, 0.1]),
            color=np.array([0.25, 0.25, 0.25]),
        )
    )
    table_archive = scene["table"]["official_asset_path"]
    if table_archive and not args.table_proxy:
        archive = (args.workspace.parent / table_archive).resolve()
        report["official_table"] = spawn_official_table(stage, archive)
        report["table_geometry"] = "Official Vention STEP tessellation; static triangle collision"
        height = report["official_table"]["tabletop_height_m"]
    elif args.table_proxy:
        length, width, height = scene["table"]["nominal_dimensions_m"]
        thickness = scene["table"]["proxy_top_thickness_m"]
        world.scene.add(
            FixedCuboid(
                prim_path="/World/TablePlacementProxy",
                name="table_proxy",
                position=np.array([length / 2, 0, height - thickness / 2]),
                scale=np.array([length, width, thickness]),
                color=np.array([0.35, 0.4, 0.5]),
            )
        )
        report["table_geometry"] = "Explicit placement proxy; NOT the official Vention CAD"
    else:
        height = None
        report["table_geometry"] = "Absent"
    board = scene.get("worktop_board")
    support_height = spawn_worktop_board(world, board, height, report)
    if table_archive or args.table_proxy:
        world.scene.add(
            DynamicCuboid(
                prim_path="/World/TestCube",
                name="test_cube",
                position=np.array([*(board["center_xy_m"] if board else [0.4, 0]), support_height + 0.08]),
                scale=np.array([0.05, 0.05, 0.05]),
                color=np.array([0.8, 0.3, 0.12]),
                mass=0.05,
            )
        )
    light = UsdLux.DomeLight.Define(stage, "/World/Light")
    light.CreateIntensityAttr(1500)
    rgb = None
    placement_readers = {}
    # Placement diagnostics are needed by the standalone scene validator only.
    if getattr(args, "diagnostic_cameras", True):
        overview = rep.create.camera(position=(3.2, 3.0, 2.5), look_at=(0.2, 0, 0.8))
        product = rep.create.render_product(overview, (1200, 900))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach(product)
        for name, position, target in (
            ("scene_side", (0.3, -3.8, 1.1), (0.3, 0, 0.85)),
            ("scene_top", (0.45, 0, 4.3), (0.45, 0, 0)),
        ):
            camera = rep.create.camera(position=position, look_at=target)
            reader = rep.AnnotatorRegistry.get_annotator("rgb")
            reader.attach(rep.create.render_product(camera, (1200, 900)))
            placement_readers[name] = reader
    camera_readers = {}
    camera_products = {}
    report["camera_models"] = {}
    if args.four_cameras:
        for name, link in {
            "head_left": "zed_left_camera",
            "head_right": "zed_right_camera",
            "wrist_left": "L_camera_link",
            "wrist_right": "R_camera_link",
        }.items():
            path = f"/World/Dexmate/{link}/RGB"
            if not stage.GetPrimAtPath(f"/World/Dexmate/{link}").IsValid():
                raise RuntimeError(f"Missing camera mount {link}")
            camera = UsdGeom.Camera.Define(stage, path)
            # URDF optical +Z forward/+Y down -> USD camera -Z forward/+Y up.
            UsdGeom.Xformable(camera).AddRotateXOp().Set(180.0)
            spec = scene.get("cameras", {}).get(name)
            parameters = camera_parameters(spec)
            report["camera_models"][name] = parameters
            camera.CreateFocalLengthAttr(parameters["focal_length_mm"])
            camera.CreateHorizontalApertureAttr(parameters["horizontal_aperture_mm"])
            camera.CreateVerticalApertureAttr(parameters["vertical_aperture_mm"])
            # Wrist mount origin is inside its opaque housing (front surface at
            # local Z=13.73 mm). Clip that housing instead of moving the mount.
            clip = spec["clipping_range_m"] if spec else [0.02 if name.startswith("wrist") else 0.01, 20.0]
            camera.CreateClippingRangeAttr(Gf.Vec2f(*clip))
            camera_product = rep.create.render_product(path, tuple(parameters["resolution"]))
            reader = rep.AnnotatorRegistry.get_annotator("rgb")
            reader.attach(camera_product)
            camera_readers[name] = reader
            camera_products[name] = camera_product
        report["camera_status"] = "Four URDF mounts; provisional intrinsics/optical convention, not calibrated"
    report["status"] = "scene_created"
    probes = create_probes(world, args.contact_regression)
    grasp_cube = create_grasp_object(world, args.grasp_regression, report, support_height)
    save()
    world.reset()
    names = robot.dof_names
    report["joint_names"] = names
    initial = np.zeros(len(names), dtype=np.float32)
    for i, name in enumerate(names):
        if "gripper" in name:
            initial[i] = 0.4
        elif name == "head_j1":
            initial[i] = 0.5
    robot.set_joint_positions(initial)
    robot.set_joint_velocities(np.zeros_like(initial))
    robot.set_solver_position_iteration_count(32)
    robot.set_solver_velocity_iteration_count(4)
    tactile = create_recorder(stage, source.parent, scene, probes, args.output, world)
    driven_ids = np.array([i for i, name in enumerate(names) if "gripper_j2" not in name])
    return {
        "world": world,
        "stage": stage,
        "robot": robot,
        "initial": initial,
        "driven_ids": driven_ids,
        "source": source,
        "tactile": tactile,
        "probes": probes,
        "grasp_cube": grasp_cube,
        "camera_readers": camera_readers,
        "camera_products": camera_products,
        "rgb": rgb,
        "placement_readers": placement_readers,
        "contact_subscription": contact_subscription,
        "contact_pairs": contact_pairs,
        "table_archive": table_archive,
        "support_height": support_height,
    }


def run(args, report, save):
    from isaacsim import SimulationApp

    extra_args = driver_arguments(args, report)
    app = SimulationApp({"headless": True, "extra_args": extra_args})
    try:
        from PIL import Image

        import omni.replicator.core as rep
        from isaacsim.core.utils.extensions import enable_extension
        from isaacsim.core.utils.types import ArticulationAction

        enable_extension("isaacsim.asset.importer.urdf")

        handles = create_scene(args, report, save)
        names = handles["robot"].dof_names
        world = handles["world"]
        stage = handles["stage"]
        robot = handles["robot"]
        initial = handles["initial"]
        driven_ids = handles["driven_ids"]
        source = handles["source"]
        tactile = handles["tactile"]
        probes = handles["probes"]
        grasp_cube = handles["grasp_cube"]
        camera_readers = handles["camera_readers"]
        rgb = handles["rgb"]
        placement_readers = handles["placement_readers"]
        contact_subscription = handles["contact_subscription"]
        contact_pairs = handles["contact_pairs"]
        table_archive = handles["table_archive"]
        support_height = handles["support_height"]
        records = []
        # Position-drive tests in free space. No claim of successful object grasping.
        for phase_index, target in enumerate((0.4, 0.0, 0.4)):
            desired = initial.copy()
            for i, name in enumerate(names):
                if "gripper" in name:
                    desired[i] = target
            robot.apply_action(ArticulationAction(joint_positions=desired[driven_ids], joint_indices=driven_ids))
            for step in range(args.steps_per_target):
                report["physics_phase"] = {"name": f"free_{phase_index}", "target_rad": target, "step": step}
                world.step(render=True)
                sample_tactile(tactile, report)
            actual = robot.get_joint_positions()
            records.append(
                {
                    "target_rad": target,
                    "actual": actual.tolist(),
                    "max_error_rad_or_m": float(np.max(np.abs(actual - desired))),
                    "self_contact_pair_events": dict(contact_pairs),
                }
            )
            if not np.isfinite(actual).all():
                raise RuntimeError("Non-finite articulation state")
            if target == 0:
                rep.orchestrator.step(rt_subframes=4, pause_timeline=False)
                Image.fromarray(rgb.get_data()).save(args.output / "closed.png")
        report["drive_tests"] = records
        report["self_contact_pair_events"] = contact_pairs
        report["drive_test_passed"] = all(r["max_error_rad_or_m"] < 0.03 for r in records)
        run_contact_regression(world, robot, probes, initial, driven_ids, report, args.output, tactile=tactile)
        run_grasp_regression(world, robot, grasp_cube, initial, driven_ids, report, args.output, source, tactile)
        finish_tactile(tactile, report)
        del contact_subscription
        if table_archive or args.table_proxy:
            cube_position, _ = world.scene.get_object("test_cube").get_world_pose()
            report["cube_rest_position_m"] = cube_position.tolist()
            report["expected_cube_rest_height_m"] = float(support_height + 0.025)
            report["table_contact_passed"] = bool(abs(cube_position[2] - (support_height + 0.025)) < 0.002)
        rep.orchestrator.step(rt_subframes=4, pause_timeline=False)
        pixels = np.asarray(rgb.get_data())
        report["render_passed"] = bool(pixels.size and pixels[..., :3].std() > 1)
        Image.fromarray(pixels).save(args.output / "scene.png")
        for name, reader in placement_readers.items():
            Image.fromarray(reader.get_data()).save(args.output / f"{name}.png")
        report["camera_outputs"] = {}
        for name, reader in camera_readers.items():
            pixels = np.asarray(reader.get_data())
            Image.fromarray(pixels).save(args.output / f"{name}.png")
            report["camera_outputs"][name] = {"shape": list(pixels.shape), "std": float(pixels[..., :3].std())}
        world.stop()
        stage.Export(str(args.output / "scene.usda"))
        report["status"] = "passed" if report["drive_test_passed"] and report["render_passed"] else "failed"
        if not all((report.get("table_contact_passed", True),
                    report.get("contact_regression", {}).get("passed", True),
                    report.get("grasp_regression", {}).get("passed", True),
                    report.get("tactile", {}).get("passed", True))):
            report["status"] = "failed"
        if args.disable_self_collision:
            report["validation_scope"] = "Drive/render/table-contact checks with robot self-collision disabled"
        else:
            report["validation_scope"] = "Drive/render/table-contact checks with robot self-collision enabled"
        report["validation_scope"] += "; tactile/contact checks as individually declared in this report"
        report["not_yet_validated"] = [
            "general grasp tasks beyond the declared regression and physically calibrated tactile response",
            "four calibrated cameras",
        ]
        save()
        if report["status"] != "passed":
            raise RuntimeError("Scene validation failed; inspect report and images")
    except BaseException:
        report["status"] = "failed"
        report["error"] = traceback.format_exc()
        save()
        raise
    finally:
        # Kit's full extension-by-extension shutdown can hang on this driver.
        # Preserve a meaningful exit status before its default fast shutdown.
        import omni.kit.app

        omni.kit.app.get_app().post_quit(0 if report.get("status") == "passed" else 1)
        app.close(skip_cleanup=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=ROOT / "Isaacsim_tactile_env/dexmate_workspace.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--table-proxy", action="store_true", help="Explicitly show a temporary table placement proxy")
    parser.add_argument("--workaround-r535-vulkan", action="store_true")
    parser.add_argument("--disable-self-collision", action="store_true", help="Diagnostic isolation only")
    parser.add_argument("--four-cameras", action="store_true", help="Render the four nominal URDF camera views")
    parser.add_argument("--steps-per-target", type=int, default=240, help="Shorter runs diagnose initial contacts")
    parser.add_argument("--raw-collisions", action="store_true", help="Diagnostic: bypass local collision corrections")
    parser.add_argument("--grasp-regression", action="store_true", help="Left-hand dynamic-object lift and release")
    parser.add_argument(
        "--contact-regression", action="store_true", help="Verify both hands contact and release fixed probes"
    )
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    report = {"status": "starting", "workspace": json.loads(args.workspace.read_text())}

    def save():
        (args.output / "validation.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")

    save()
    try:
        run(args, report, save)
    except BaseException:
        report["status"] = "failed"
        report["error"] = traceback.format_exc()
        save()
        raise


if __name__ == "__main__":
    main()
