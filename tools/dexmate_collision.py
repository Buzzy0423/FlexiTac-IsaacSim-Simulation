"""Local collision and initialization policy for the Dexmate import (source URDF unchanged)."""

import itertools
import math
import xml.etree.ElementTree as ET


def adjacent_body_pairs(urdf):
    """Extend adjacent-joint filtering through fixed links, including empty frames."""
    root = ET.parse(urdf).getroot()
    parents = {link.get("name"): link.get("name") for link in root.findall("link")}

    def group(name):
        while parents[name] != name:
            name = parents[name]
        return name

    joints = root.findall("joint")
    for joint in joints:
        if joint.get("type") == "fixed":
            a, b = joint.find("parent").get("link"), joint.find("child").get("link")
            parents[group(b)] = group(a)
    groups = {}
    for name in parents:
        groups.setdefault(group(name), []).append(name)
    pairs = set()
    for names in groups.values():
        pairs.update(tuple(sorted(pair)) for pair in itertools.combinations(names, 2))
    for joint in joints:
        a, b = joint.find("parent").get("link"), joint.find("child").get("link")
        if group(a) != group(b):
            pairs.update(tuple(sorted(pair)) for pair in itertools.product(groups[group(a)], groups[group(b)]))
    return sorted(pairs)


def configure_collision(stage, robot_prim, urdf, report, raw=False):
    from pxr import PhysxSchema, Usd, UsdPhysics

    if raw:
        return
    prefix = str(robot_prim.GetPath())
    filtered = []
    for a, b in adjacent_body_pairs(urdf):
        pa, pb = stage.GetPrimAtPath(f"{prefix}/{a}"), stage.GetPrimAtPath(f"{prefix}/{b}")
        if pa.HasAPI(UsdPhysics.RigidBodyAPI) and pb.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.FilteredPairsAPI.Apply(pa).CreateFilteredPairsRel().AddTarget(pb.GetPath())
            filtered.append([a, b])
    report["collision_policy"] = {"adjacent_fixed_cluster_pairs": filtered}
    # Native importer instances collider subtrees. Regular PrimRange skips their
    # contents, so the old contact-offset loop silently touched zero colliders.
    # Make only collision instances editable; visuals remain instanced.
    editable = []
    for prim in list(Usd.PrimRange(robot_prim)):
        if prim.IsInstance() and prim.GetName() == "collisions":
            prim.SetInstanceable(False)
            editable.append(str(prim.GetPath()))
    report["collision_policy"]["editable_collision_subtrees"] = editable
    refined = []
    for prim in Usd.PrimRange(robot_prim):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            decomposition = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
            decomposition.CreateShrinkWrapAttr(True)
            decomposition.CreateMinThicknessAttr(0.00005)
            decomposition.CreateErrorPercentageAttr(1.0)
            decomposition.CreateMaxConvexHullsAttr(64)
            refined.append(str(prim.GetPath()))
    report["collision_policy"]["refined_convex_colliders"] = refined
    if not refined:
        raise RuntimeError("No editable colliders found; contact settings would be ineffective")
    report["collision_policy"]["convex_decomposition"] = {
        "shrink_wrap": True, "min_thickness_m": 0.00005, "error_percentage": 1.0, "max_convex_hulls": 64,
    }
    report["collision_policy"]["authored_self_collision"] = {
        str(p.GetPath()): PhysxSchema.PhysxArticulationAPI(p).GetEnabledSelfCollisionsAttr().Get()
        for p in Usd.PrimRange(robot_prim) if p.HasAPI(PhysxSchema.PhysxArticulationAPI)
    }
    # The importer defaults (25 rad/s, damping ratio 0.005) act as a soft spring:
    # under contact a follower can lag by ~0.1 rad. Model the mechanical coupling
    # as a stiff, critically damped constraint, not a second independent actuator.
    for prim in Usd.PrimRange(robot_prim):
        for schema in prim.GetAppliedSchemas():
            if schema.startswith("PhysxMimicJointAPI:"):
                axis = schema.split(":")[-1]
                prim.GetAttribute(f"physxMimicJoint:{axis}:naturalFrequency").Set(5000.0)
                prim.GetAttribute(f"physxMimicJoint:{axis}:dampingRatio").Set(1.0)
    report["collision_policy"]["mimic_parameters"] = {
        str(p.GetPath()): {a.GetName(): str(a.Get()) for a in p.GetAttributes() if "mimic" in a.GetName().lower()}
        for p in Usd.PrimRange(robot_prim) if p.GetName()[2:] == "gripper_j2"
    }
    # World.reset performs a physics step internally. Author the intended starting
    # positions before that step rather than letting the closed/zero pose collide.
    for prim in Usd.PrimRange(robot_prim):
        if prim.IsA(UsdPhysics.RevoluteJoint):
            name = prim.GetName()
            value = 0.4 if "gripper" in name else 0.5 if name == "head_j1" else 0.0
            state = PhysxSchema.JointStateAPI.Apply(prim, "angular")
            state.CreatePositionAttr(math.degrees(value))
            state.CreateVelocityAttr(0.0)
            if prim.HasAPI(UsdPhysics.DriveAPI, "angular"):
                UsdPhysics.DriveAPI(prim, "angular").GetTargetPositionAttr().Set(math.degrees(value))


def setup_contact_reporting(robot_prim, report):
    import omni.physx
    from pxr import PhysicsSchemaTools, PhysxSchema, Usd, UsdPhysics

    configured = []
    for prim in Usd.PrimRange(robot_prim):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            collision.CreateContactOffsetAttr(0.0002)
            collision.CreateRestOffsetAttr(0.0)
            configured.append(str(prim.GetPath()))
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
    contact_pairs = {}
    report["contact_offset_configured_colliders"] = configured
    expected = report.get("collision_policy", {}).get("refined_convex_colliders")
    if expected is not None and set(expected) != set(configured):
        raise RuntimeError("Contact offsets were not configured on every robot collider")
    report["first_self_contacts"] = {}
    report["phase_contacts"] = {}

    def on_contacts(headers, data):
        for header in headers:
            pair = sorted(
                [
                    str(PhysicsSchemaTools.intToSdfPath(header.actor0)),
                    str(PhysicsSchemaTools.intToSdfPath(header.actor1)),
                ]
            )
            record_contact_phase(report, pair, header, data)
            if all(p.startswith("/World/Dexmate/") for p in pair):
                key = " | ".join(pair)
                contact_pairs[key] = contact_pairs.get(key, 0) + 1
                if key not in report["first_self_contacts"]:
                    report["first_self_contacts"][key] = {
                        "colliders": [
                            str(PhysicsSchemaTools.intToSdfPath(c)) for c in (header.collider0, header.collider1)
                        ],
                        "phase": report.get("physics_phase", "initializing"),
                        "separations_m": [
                            float(data[i].separation)
                            for i in range(
                                header.contact_data_offset, header.contact_data_offset + header.num_contact_data
                            )
                        ],
                    }

    subscription = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(on_contacts)
    return subscription, contact_pairs


def record_contact_phase(report, pair, header, data):
    """Count actual contact points (not lost-contact notifications), including external probes."""
    if not header.num_contact_data:
        return
    phase = report.get("physics_phase", {})
    stats = report["phase_contacts"].setdefault(phase.get("name", "initializing"), {})
    entry = stats.setdefault(" | ".join(pair), {
        "points": 0, "min_separation_m": 1.0, "max_impulse_Ns": 0.0, "last_step": -1, "last_impulse_step": -1,
    })
    minimum = entry["min_separation_m"]
    peak_squared = 0.0
    for index in range(header.contact_data_offset, header.contact_data_offset + header.num_contact_data):
        contact = data[index]
        separation = float(contact.separation)
        if separation < minimum:
            minimum = separation
        # carb.Float3's Python sequence iteration is costly for millions of
        # candidate points. Read the three native members directly instead.
        vector = contact.impulse
        x, y, z = vector.x, vector.y, vector.z
        squared = x * x + y * y + z * z
        if squared > peak_squared:
            peak_squared = squared
    impulse = math.sqrt(peak_squared)
    entry["points"] += header.num_contact_data
    entry["min_separation_m"] = minimum
    entry["max_impulse_Ns"] = max(entry["max_impulse_Ns"], impulse)
    if impulse > 1e-7:
        entry["last_impulse_step"] = phase.get("step", -1)
    entry["last_step"] = phase.get("step", -1)
