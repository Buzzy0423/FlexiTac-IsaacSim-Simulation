"""Dynamic left-hand grasp, 5 cm lift, two-second hold, replace and release."""

import json

import numpy as np
from dexmate_grasp_motion import LeftArmKinematics, interpolate_path
from dexmate_tactile_preview import render_frame
from dexmate_tactile_validation import sample_tactile


def create_grasp_object(world, enabled, report, support_height):
    if not enabled:
        return None
    from isaacsim.core.api.objects import DynamicCuboid
    from pxr import PhysxSchema, Usd, UsdPhysics, UsdShade

    if not report["self_collision_enabled"] or support_height is None:
        raise ValueError("Grasp regression requires self-collision and a supporting table")
    cfg = {
        "dimensions_m": [.04, .03, .08], "mass_kg": .05,
        "position_m": [.34935, .22, support_height + .04],
        "gripper_base_position_m": [.2, .22, support_height + .0535],
        "static_friction": .8, "dynamic_friction": .6, "lift_m": .05, "hold_s": 2.0,
    }
    cube = world.scene.add(DynamicCuboid(
        prim_path="/World/GraspCube", name="grasp_cube", position=np.asarray(cfg["position_m"]),
        scale=np.asarray(cfg["dimensions_m"]), mass=cfg["mass_kg"], color=np.array([.15, .6, .85]),
    ))
    material = UsdShade.Material.Define(world.stage, "/World/GraspMaterial")
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr(cfg["static_friction"])
    physics.CreateDynamicFrictionAttr(cfg["dynamic_friction"])
    physics.CreateRestitutionAttr(0.0)
    prims = [cube.prim]
    for finger in (1, 2):
        root = world.stage.GetPrimAtPath(f"/World/Dexmate/L_gripper_l{finger}")
        prims.extend(p for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.CollisionAPI))
    for prim in prims:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose="physics")
    collision = PhysxSchema.PhysxCollisionAPI.Apply(cube.prim)
    collision.CreateContactOffsetAttr(.0002)
    collision.CreateRestOffsetAttr(0.0)
    PhysxSchema.PhysxContactReportAPI.Apply(cube.prim).CreateThresholdAttr(0.0)
    report["grasp_config"] = cfg
    report["workspace"]["tactile"]["target_prim_paths"].append("/World/GraspCube")
    return cube


def contact_flags(report, phase, step):
    pairs = report["phase_contacts"].get(phase, {})
    return [pairs.get(" | ".join(sorted([f"/World/Dexmate/L_gripper_l{i}", "/World/GraspCube"])),
                      {}).get("last_impulse_step", -1) == step for i in (1, 2)]


def evaluate_grasp(data, report):
    phases = data["phase"]
    required = ("grasp_approach", "grasp_close", "grasp_hold", "grasp_release")
    missing = [phase for phase in required if not np.any(phases == phase)]
    if missing:
        return {"passed": False, "checks": {"required_phases_present": False},
                "failure_reason": "Episode ended before required grasp phases", "missing_phases": missing,
                "frame_count": len(phases)}
    hold = phases == "grasp_hold"
    settled = np.flatnonzero(phases == "grasp_approach")[-30:]
    closed = np.flatnonzero(phases == "grasp_close")[-30:]
    released = np.flatnonzero(phases == "grasp_release")[-60:]
    baseline = np.median(data["object_position_m"][settled], axis=0)
    lift = data["object_position_m"][hold, 2] - baseline[2]
    relative = data["object_in_gripper_m"]
    slip = np.linalg.norm(relative[hold] - np.median(relative[closed], axis=0), axis=1)
    bad_contacts = {}
    for phase, pairs in report["phase_contacts"].items():
        if not phase.startswith("grasp_"):
            continue
        for pair, stats in pairs.items():
            robot_count = pair.count("/World/Dexmate/")
            same_gripper = any(all(f"{side}_gripper_l{i}" in pair for i in (1, 2)) for side in "LR")
            if robot_count and "GraspCube" not in pair and not same_gripper and stats["max_impulse_Ns"] > 1e-5:
                bad_contacts[f"{phase}: {pair}"] = stats["max_impulse_Ns"]
    checks = {
        "lift_50mm_within_10mm": bool(lift.min() > .04 and lift.max() < .06),
        "hold_2_seconds": int(hold.sum()) == 240,
        "slip_under_8mm": bool(slip.max() < .008),
        "both_fingers_loaded_during_hold": bool(np.all(data["finger_loaded"][hold].mean(0) > .9)),
        "both_pads_active_during_hold": bool(np.all((data["tactile_peak"][hold, :2] > .001).mean(0) > .9)),
        "released_contact_and_tactile": bool(not data["finger_loaded"][released].any()
                                              and data["tactile_peak"][released, :2].max() < 1e-6),
        "returned_to_table": bool(np.max(np.abs(data["object_position_m"][released, 2] - baseline[2])) < .003),
        "returned_near_start_xy": bool(np.linalg.norm(data["object_position_m"][-1, :2] - baseline[:2]) < .02),
        "released_object_stable": bool(np.linalg.norm(data["object_velocity_m_s"][released], axis=1).max() < .02),
        "no_unexpected_robot_contacts": not bad_contacts,
    }
    return {
        "kind": "Dynamic object, gravity and friction only; left-arm position drives; no object attachment",
        "checks": checks, "passed": all(checks.values()), "object_baseline_m": baseline.tolist(),
        "hold_lift_range_m": [float(lift.min()), float(lift.max())], "max_relative_slip_m": float(slip.max()),
        "hold_finger_contact_fraction": data["finger_loaded"][hold].mean(0).tolist(),
        "unexpected_robot_contacts": bad_contacts, "frame_count": len(phases),
    }


def run_grasp_regression(world, robot, cube, initial, driven_ids, report, output, source, tactile):
    if cube is None:
        return
    from PIL import Image, ImageDraw

    import omni.replicator.core as rep
    from isaacsim.core.utils.types import ArticulationAction
    from pxr import Usd, UsdGeom

    if tactile is None:
        raise ValueError("Grasp regression requires tactile enabled")
    kin = LeftArmKinematics(source, report["workspace"]["robot"])
    xyz = np.asarray(report["grasp_config"]["gripper_base_position_m"])
    pre = xyz + [0, 0, .12]
    pre_q = kin.solve(pre, np.array([1.5, .1, .1, -1.8, -.1, .3, .1]))
    approach = kin.line(pre, xyz, pre_q)
    lift = kin.line(xyz, xyz + [0, 0, .05], approach[-1])
    arm_ids = [robot.get_dof_index(f"L_arm_j{i}") for i in range(1, 8)]
    finger_ids = [robot.get_dof_index(f"L_gripper_j{i}") for i in (1, 2)]
    report["grasp_waypoints_rad"] = {"pre": pre_q.tolist(), "grasp": approach[-1].tolist(), "lift": lift[-1].tolist()}
    camera = rep.create.camera(position=(.9, 1.05, 1.25), look_at=(.30, .22, .92), clipping_range=(.01, 20.))
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach(rep.create.render_product(camera, (800, 600)))
    frames, movies = [], []
    phases = [
        ("grasp_preposition", np.array([robot.get_joint_positions()[arm_ids], pre_q]), .4, 480),
        ("grasp_approach", approach, .4, 360),
        ("grasp_close", np.array([approach[-1]] * 2), 0., 180),
        ("grasp_lift", lift, 0., 240),
        ("grasp_hold", np.array([lift[-1]] * 2), 0., 240),
        ("grasp_lower", lift[::-1], 0., 240),
        ("grasp_release", np.array([approach[-1]] * 2), .4, 180),
        ("grasp_retreat", approach[::-1], .4, 240),
    ]
    for phase, path, opening, count in phases:
        for step in range(count):
            desired = initial.copy()
            desired[arm_ids] = interpolate_path(path, (step + 1) / count)
            desired[finger_ids] = opening
            robot.apply_action(ArticulationAction(joint_positions=desired[driven_ids], joint_indices=driven_ids))
            report["physics_phase"] = {"name": phase, "step": step}
            world.step(render=True)
            sample_tactile(tactile, report)
            pos, quat = cube.get_world_pose()
            prim = world.stage.GetPrimAtPath("/World/Dexmate/L_gripper_base")
            pose = np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
            frames.append({
                "phase": phase, "step": step, "time_s": world.current_time,
                "object_position_m": pos, "object_orientation_wxyz": quat,
                "object_velocity_m_s": cube.get_linear_velocity(), "gripper_pose": pose,
                "object_in_gripper_m": (np.linalg.inv(pose) @ np.r_[pos, 1])[:3],
                "joint_position": robot.get_joint_positions(), "joint_target": desired,
                "finger_loaded": contact_flags(report, phase, step),
                "tactile_peak": tactile.sensor.observation.max(axis=(1, 2)),
            })
            if step % 12 == 0 or step == count - 1:
                pixels = np.asarray(rgb.get_data())
                if pixels.size:
                    image = Image.fromarray(pixels[..., :3]).resize((960, 720))
                    canvas = Image.new("RGB", (960, 899), "white")
                    canvas.paste(image, (0, 0))
                    heat = render_frame(tactile.sensor.observation, f"{phase} | {world.current_time:.2f}s")
                    canvas.paste(heat.resize((960, 179)), (0, 720))
                    ImageDraw.Draw(canvas).text((10, 10), f"{phase} | object Z={pos[2]:.3f} m", fill="white")
                    movies.append(canvas)
                    if step == count - 1:
                        canvas.save(output / f"{phase}.png")
        world.stage.Export(str(output / "grasp_latest.usda"))
    data = {key: np.asarray([f[key] for f in frames]) for key in frames[0]}
    np.savez_compressed(output / "grasp_sequence.npz", **data, joint_names=np.asarray(robot.dof_names))
    result = evaluate_grasp(data, report)
    report["grasp_regression"] = result
    (output / "grasp_report.json").write_text(json.dumps(result, indent=2) + "\n")
    movies[0].save(output / "grasp_replay.gif", save_all=True, append_images=movies[1:], duration=100, loop=0)
