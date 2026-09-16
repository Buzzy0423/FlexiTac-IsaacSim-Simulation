"""Small deterministic pose sweep about the validated left-hand grasp."""

import copy

import numpy as np
from scipy.spatial.transform import Rotation


def pose_sweep():
    return [
        {"name": name, "offset_xy_m": [x, y], "yaw_deg": yaw}
        for name, x, y, yaw in (
            ("baseline", 0, 0, 0),
            ("near_20mm", -.02, 0, 0), ("far_20mm", .02, 0, 0),
            ("left_20mm", 0, .02, 0), ("right_20mm", 0, -.02, 0),
            ("yaw_minus_10deg", 0, 0, -10), ("yaw_plus_10deg", 0, 0, 10),
            ("near_left_yaw_minus_5deg", -.01, .01, -5),
            ("far_right_yaw_plus_5deg", .01, -.01, 5),
            ("baseline_repeat", 0, 0, 0),
        )
    ]


def resolve_grasp_config(baseline, variation):
    """Rotate gripper and object together about object centre, then translate XY."""
    offset = np.asarray(variation.get("offset_xy_m", [0, 0]), dtype=float)
    yaw = float(variation.get("yaw_deg", 0))
    if offset.shape != (2,) or not np.isfinite(offset).all() or not np.isfinite(yaw):
        raise ValueError("Expected finite XY offset and yaw")
    if np.max(np.abs(offset)) > .03 or abs(yaw) > 15:
        raise ValueError("Initial pose sweep is bounded to +/-30 mm XY and +/-15 deg yaw")
    config = copy.deepcopy(baseline)
    rotation = Rotation.from_euler("z", yaw, degrees=True)
    centre = np.asarray(baseline["position_m"])
    shifted = centre + np.r_[offset, 0]
    grasp = shifted + rotation.apply(np.asarray(baseline["gripper_base_position_m"]) - centre)
    quat = rotation.as_quat()
    config.update(position_m=shifted.tolist(), gripper_base_position_m=grasp.tolist(),
                  orientation_wxyz=[float(quat[3]), *quat[:3].tolist()], yaw_deg=yaw,
                  variation={"name": variation.get("name", "custom"), "offset_xy_m": offset.tolist(), "yaw_deg": yaw})
    return config
