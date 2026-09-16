# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause
"""Shared ALOHA/Dexmate Warp distance kernels and per-taxel spring response.

No Isaac Lab runtime imports: usable from native Isaac Sim as well as SensorBase.
"""

import warp as wp


@wp.kernel(enable_backward=False)
def mesh_distance_kernel(
    queries_l: wp.array(dtype=wp.vec3),
    mesh: wp.uint64,
    tri_indices: wp.array(dtype=wp.int32),
    vertex_normals: wp.array(dtype=wp.vec3),
    max_dist: float,
    signed_mode: int,
    smooth_normals: int,
    dist_out: wp.array(dtype=wp.float32),
):
    tid = wp.tid()
    q = queries_l[tid]

    sign = float(0.0)
    face_idx = int(0)
    face_u = float(0.0)
    face_v = float(0.0)

    hit = wp.mesh_query_point(mesh, q, max_dist, sign, face_idx, face_u, face_v)
    if hit:
        p = wp.mesh_eval_position(mesh, face_idx, face_u, face_v)
        delta = q - p
        d = wp.length(delta)

        if signed_mode == 0:
            # unsigned distance
            dist_out[tid] = d
        elif signed_mode == 1:
            # Warp's winding-number based sign (requires watertight mesh)
            dist_out[tid] = sign * d
        else:
            # Normal-based sign (works for open meshes but depends on consistent normals)
            # Compute triangle normal (flat)
            p0 = wp.mesh_eval_position(mesh, face_idx, 0.0, 0.0)
            p1 = wp.mesh_eval_position(mesh, face_idx, 1.0, 0.0)
            p2 = wp.mesh_eval_position(mesh, face_idx, 0.0, 1.0)
            n_face = wp.cross(p1 - p0, p2 - p0)
            n_face_len = wp.length(n_face)
            if n_face_len > 1.0e-12:
                n_face = n_face / n_face_len
            else:
                n_face = wp.vec3(0.0, 0.0, 1.0)

            n = n_face
            if smooth_normals == 1:
                # Interpolate vertex normals using barycentric coords
                base = face_idx * 3
                i0 = tri_indices[base + 0]
                i1 = tri_indices[base + 1]
                i2 = tri_indices[base + 2]
                w0 = 1.0 - face_u - face_v
                w1 = face_u
                w2 = face_v
                n_interp = w0 * vertex_normals[i0] + w1 * vertex_normals[i1] + w2 * vertex_normals[i2]
                n_len = wp.length(n_interp)
                if n_len > 1.0e-12:
                    n_interp = n_interp / n_len
                else:
                    n_interp = n_face
                # Align interpolated normal with the triangle's orientation
                if wp.dot(n_interp, n_face) < 0.0:
                    n_interp = -n_interp
                n = n_interp

            sd = wp.dot(delta, n)
            s = float(1.0)
            if sd < 0.0:
                s = float(-1.0)
            dist_out[tid] = s * d
    else:
        dist_out[tid] = max_dist


@wp.func
def _quat_rotate_inv(q: wp.vec4, v: wp.vec3) -> wp.vec3:
    # q is (w, x, y, z). Compute inverse rotation by using conjugate.
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]
    # conjugate
    cx = -qx
    cy = -qy
    cz = -qz
    # quat * v
    # treat v as pure quaternion (0, v)
    tx = qw * v[0] + cy * v[2] - cz * v[1]
    ty = qw * v[1] + cz * v[0] - cx * v[2]
    tz = qw * v[2] + cx * v[1] - cy * v[0]
    tw = -cx * v[0] - cy * v[1] - cz * v[2]
    # result = (t) * conj(q)
    rx = tw * cx + tx * qw + ty * cz - tz * cy
    ry = tw * cy - tx * cz + ty * qw + tz * cx
    rz = tw * cz + tx * cy - ty * cx + tz * qw
    return wp.vec3(rx, ry, rz)


@wp.kernel(enable_backward=False)
def box_sdf_kernel(
    points_w: wp.array(dtype=wp.vec3),
    box_pos_w: wp.vec3,
    box_quat_w: wp.vec4,
    half_extents: wp.vec3,
    sdf_out: wp.array(dtype=wp.float32),
):
    tid = wp.tid()
    p_w = points_w[tid]
    # transform point into box local frame
    p_l = _quat_rotate_inv(box_quat_w, p_w - box_pos_w)

    qx = wp.abs(p_l.x) - half_extents.x
    qy = wp.abs(p_l.y) - half_extents.y
    qz = wp.abs(p_l.z) - half_extents.z

    # outside distance
    ox = wp.max(qx, 0.0)
    oy = wp.max(qy, 0.0)
    oz = wp.max(qz, 0.0)
    outside = wp.sqrt(ox * ox + oy * oy + oz * oz)

    # inside distance (negative)
    m = wp.max(qx, qy)
    m = wp.max(m, qz)
    inside = wp.min(m, 0.0)

    sdf_out[tid] = outside + inside


def spring_response(distance, stiffness, max_force, normalize=True, shell_thickness=None):
    """ALOHA's pointwise response; input distances must use metres."""
    penetration = (-distance if shell_thickness is None else shell_thickness - distance).clamp_min(0.0)
    force = (float(stiffness) * penetration).clamp(0.0, float(max_force))
    return force / float(max_force) if normalize else force
