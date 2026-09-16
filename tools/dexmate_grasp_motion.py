"""URDF forward/inverse kinematics for the bounded left-hand grasp regression."""

import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


class LeftArmKinematics:
    def __init__(self, urdf, robot_config):
        root = ET.parse(urdf).getroot()
        parents = {j.find("child").get("link"): j for j in root.findall("joint")}
        chain, link = [], "L_gripper_base"
        while link in parents:
            joint = parents[link]
            chain.insert(0, joint)
            link = joint.find("parent").get("link")
        self.chain, self.lower, self.upper = [], [], []
        self.base = np.eye(4)
        self.base[:3, 3] = robot_config["position_m"]
        q = robot_config["orientation_wxyz"]
        self.base[:3, :3] = Rotation.from_quat([*q[1:], q[0]]).as_matrix()
        for joint in chain:
            origin = joint.find("origin")
            transform = np.eye(4)
            transform[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
            transform[:3, :3] = Rotation.from_euler(
                "xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
            ).as_matrix()
            active = joint.get("name").startswith("L_arm_j") and joint.get("type") == "revolute"
            index = int(joint.get("name")[-1]) - 1 if active else None
            axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ") if active else np.zeros(3)
            self.chain.append((transform, axis, index))
            if active:
                self.lower.append(float(joint.find("limit").get("lower")))
                self.upper.append(float(joint.find("limit").get("upper")))
        self.orientation = self.forward(np.zeros(7))[:3, :3]

    def forward(self, joints):
        pose = self.base.copy()
        for origin, axis, index in self.chain:
            pose = pose @ origin
            if index is not None:
                motion = np.eye(4)
                motion[:3, :3] = Rotation.from_rotvec(axis * joints[index]).as_matrix()
                pose = pose @ motion
        return pose

    def solve(self, xyz, seed):
        def residual(joints):
            pose = self.forward(joints)
            return np.r_[pose[:3, 3] - xyz,
                         .2 * Rotation.from_matrix(self.orientation @ pose[:3, :3].T).as_rotvec()]

        solution = least_squares(residual, seed, bounds=(self.lower, self.upper), max_nfev=1000,
                                 ftol=1e-10, xtol=1e-10, gtol=1e-10)
        if np.linalg.norm(solution.fun) > 1e-5:
            raise ValueError(f"Grasp waypoint is unreachable: {xyz}; residual {solution.fun}")
        return solution.x

    def line(self, start, end, seed):
        points = []
        for xyz in np.linspace(start, end, 31):
            seed = self.solve(xyz, seed)
            points.append(seed)
        return np.asarray(points)


def interpolate_path(path, fraction):
    # Quintic timing gives zero speed/acceleration at phase boundaries.
    u = np.clip(fraction, 0, 1)
    u = u**3 * (10 - 15 * u + 6 * u**2)
    index = u * (len(path) - 1)
    low = min(int(index), len(path) - 2)
    return path[low] * (1 - (index - low)) + path[low + 1] * (index - low)
