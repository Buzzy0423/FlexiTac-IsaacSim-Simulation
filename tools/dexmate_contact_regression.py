"""Positive contact regression using fixed probes; this is not an object-grasp task."""

import numpy as np
from dexmate_tactile_validation import sample_tactile


def create_probes(world, enabled):
    from isaacsim.core.api.objects import FixedCuboid
    from pxr import PhysxSchema

    if not enabled:
        return {}
    probes = {
        side: world.scene.add(FixedCuboid(
            prim_path=f"/World/{side}_ContactProbe",
            name=f"{side}_contact_probe",
            position=np.array([3.0, 2.0 if side == "L" else -2.0, 1.0]),
            scale=np.array([0.02, 0.018, 0.025]),
            color=np.array([0.8, 0.25, 0.12]),
        ))
        for side in "LR"
    }
    for probe in probes.values():
        collision = PhysxSchema.PhysxCollisionAPI.Apply(probe.prim)
        collision.CreateContactOffsetAttr(0.0002)
        collision.CreateRestOffsetAttr(0.0)
    return probes


def run_contact_regression(world, robot, probes, initial, driven_ids, report, output, tactile=None):
    from isaacsim.core.utils.types import ArticulationAction
    from pxr import Gf, Usd, UsdGeom

    if not probes:
        return
    if not report["self_collision_enabled"]:
        raise ValueError("Contact regression requires self-collision enabled")
    result = {
        "kind": "Fixed 20 mm probes at two lateral positions: load each finger and release; not a free-object grasp",
        "placements": {},
    }
    shifted_positions = {}
    for side, probe in probes.items():
        prim = world.stage.GetPrimAtPath(f"/World/Dexmate/{side}_gripper_base")
        transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        # Middle of the opposing raised surfaces, in the gripper base frame.
        position = np.array(transform.Transform(Gf.Vec3d(0, -0.0015, 0.14935)))
        q = transform.ExtractRotationQuat()
        orientation = np.array([q.GetReal(), *q.GetImaginary()])
        probe.set_world_pose(position=position, orientation=orientation)
        result["placements"][side] = position.tolist()
        # A fixed probe cannot self-center like a free object. Repeat 2 mm toward
        # finger 1 to exercise sustained loading on both opposing surfaces.
        shifted_positions[side] = np.array(transform.Transform(Gf.Vec3d(-0.002, -0.0015, 0.14935)))
    for phase, target in (
        ("probe_close", 0.0), ("probe_release_1", 0.4), ("probe_shift_close", 0.0), ("probe_release", 0.4)
    ):
        if phase == "probe_shift_close":
            for side, probe in probes.items():
                probe.set_world_pose(position=shifted_positions[side])
        desired = initial.copy()
        for index, name in enumerate(robot.dof_names):
            if "gripper" in name:
                desired[index] = target
        robot.apply_action(ArticulationAction(joint_positions=desired[driven_ids], joint_indices=driven_ids))
        for step in range(240):
            report["physics_phase"] = {"name": phase, "step": step}
            world.step(render=True)
            sample_tactile(tactile, report)
        result[phase] = robot.get_joint_positions().tolist()
        if phase == "probe_close":
            world.stage.Export(str(output / "contact_hold.usda"))
    checks = {}
    for side in "LR":
        for finger in (1, 2):
            key = " | ".join(sorted([f"/World/Dexmate/{side}_gripper_l{finger}", f"/World/{side}_ContactProbe"]))
            close = report["phase_contacts"].get("probe_close", {}).get(key, {})
            shifted = report["phase_contacts"].get("probe_shift_close", {}).get(key, {})
            release = report["phase_contacts"].get("probe_release", {}).get(key, {})
            checks[f"{side}_finger_{finger}"] = {
                "hold_contact": max(close.get("last_impulse_step", -1), shifted.get("last_impulse_step", -1)) >= 220,
                "released": release.get("last_impulse_step", -1) < 220,
            }
    error = float(np.max(np.abs(np.array(result["probe_release"]) - initial)))
    result["checks"] = checks
    result["release_max_joint_error_rad_or_m"] = error
    result["passed"] = all(c["hold_contact"] and c["released"] for c in checks.values()) and error < 0.03
    report["contact_regression"] = result
    for side, probe in probes.items():
        probe.set_world_pose(position=np.array([3.0, 2.0 if side == "L" else -2.0, 1.0]))
