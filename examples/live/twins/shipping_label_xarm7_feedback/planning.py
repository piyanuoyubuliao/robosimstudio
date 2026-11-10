import json
from pathlib import Path

import mujoco
import numpy as np
from config import DEFAULT
from observation import unit
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parent
DOWN = Rotation.from_matrix([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])


class Refusal(RuntimeError):
    def __init__(self, reason, detail=""):
        self.reason = reason
        super().__init__(f"{reason}: {detail}")


class Planner:
    def __init__(self, config=DEFAULT):
        self.config = config
        spec = mujoco.MjSpec.from_file(str(ROOT / "assets" / "dual_xarm7.xml"))
        spec.worldbody.add_geom(
            name="policy_table",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[1.125, 0.4, 0.025],
            pos=[-0.475, 0.15, config.table_z - 0.025],
        )
        spec.worldbody.add_geom(
            name="policy_pillar", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.14, 0.115, 0.91], pos=[0.15, -0.76, 0.99]
        )
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self.indices = {
            side: np.array([self.model.joint(f"{side}_joint{number}").qposadr[0] for number in range(1, 8)])
            for side in ("left", "right")
        }
        self.bounds = {
            side: np.array([self.model.joint(f"{side}_joint{number}").range for number in range(1, 8)])
            for side in self.indices
        }
        self.bodies = {side: self.model.body(f"{side}_xarm_gripper_base_link").id for side in self.indices}
        self.groups = {}
        for side in self.indices:
            self.groups[side] = [
                index
                for index in range(self.model.ngeom)
                if self.model.body(self.model.geom_bodyid[index]).name.startswith(side + "_")
                and self.model.geom_contype[index] != 0
            ]
        self.obstacles = [self.model.geom(name).id for name in ("policy_table", "policy_pillar")]
        self.pairs = [(left, right) for left in self.groups["left"] for right in self.groups["right"]]
        self.pairs += [
            (geom, obstacle)
            for side in self.groups
            for geom in self.groups[side]
            for obstacle in self.obstacles
            if "carriage" not in self.model.geom(geom).name
        ]

    def set_state(self, joints, grip):
        for side, values in joints.items():
            self.data.qpos[self.indices[side]] = values
            for finger in ("left", "right"):
                for suffix in ("driver_joint", "finger_joint", "inner_knuckle_joint"):
                    self.data.qpos[self.model.joint(f"{side}_{finger}_{suffix}").qposadr[0]] = grip[side] * 0.85
        mujoco.mj_forward(self.model, self.data)

    def pose(self, side, joints):
        self.data.qpos[self.indices[side]] = joints
        mujoco.mj_kinematics(self.model, self.data)
        body = self.bodies[side]
        matrix = self.data.xmat[body].reshape(3, 3)
        return (self.data.xpos[body] + matrix @ [0, 0, 0.155]).copy(), Rotation.from_matrix(matrix.copy())

    def solve(self, side, target, orientation, seed):
        target, seed = np.asarray(target), np.asarray(seed)

        def residual(values):
            position, rotation = self.pose(side, values)
            return np.concatenate(
                ((position - target) * 10, (orientation.inv() * rotation).as_rotvec(), (values - seed) * 0.002)
            )

        bounds = self.bounds[side]
        fit = least_squares(
            residual,
            np.clip(seed, bounds[:, 0] + 1e-5, bounds[:, 1] - 1e-5),
            bounds=(bounds[:, 0] + 1e-6, bounds[:, 1] - 1e-6),
            max_nfev=200,
            xtol=1e-9,
            ftol=1e-9,
            gtol=1e-9,
        )
        error = residual(fit.x)
        if np.linalg.norm(error[:3]) > 0.015 or np.linalg.norm(error[3:6]) > 0.03:
            raise Refusal("unreachable", f"{side}: {target.tolist()}, residual={error[:6].tolist()}")
        return fit.x

    def check(self, joints, grip):
        self.set_state(joints, grip)
        margin = self.config.collision_margin
        for first, second in self.pairs:
            bound = self.model.geom_rbound[first] + self.model.geom_rbound[second] + margin
            if np.linalg.norm(self.data.geom_xpos[first] - self.data.geom_xpos[second]) > bound:
                continue
            distance = mujoco.mj_geomDistance(self.model, self.data, first, second, margin, None)
            if distance < margin:
                names = [
                    self.model.geom(index).name or self.model.body(self.model.geom_bodyid[index]).name
                    for index in (first, second)
                ]
                raise Refusal("collision", f"{names}: {distance:.5f} m")

    def path(self, side, target, orientation, joints, grip, hz):
        start, rotation = self.pose(side, joints[side])
        target = np.asarray(target)
        distance = np.linalg.norm(target - start)
        angle = (orientation * rotation.inv()).magnitude()
        count = max(2, int(np.ceil(distance / 0.012)), int(np.ceil(angle / 0.07)))
        rotations = Slerp([0, 1], Rotation.concatenate([rotation, orientation]))
        previous = np.asarray(joints[side]).copy()
        previous_pose = start, rotation
        trajectory = []
        fractions = list(np.linspace(0, 1, count + 1)[1:])
        previous_fraction = 0.0
        while fractions:
            fraction = fractions.pop(0)
            position = start + (target - start) * fraction
            current_rotation = rotations(fraction)
            current = self.solve(side, position, current_rotation, previous)
            if np.max(np.abs(current - previous)) > 0.40:
                if fraction - previous_fraction < 1e-4:
                    raise Refusal("ik_discontinuity", f"{side}: target={position.tolist()}")
                fractions[:0] = [(previous_fraction + fraction) / 2, fraction]
                continue
            duration = max(
                np.max(np.abs(current - previous)) / self.config.max_joint_speed,
                np.linalg.norm(position - previous_pose[0]) / self.config.max_tcp_speed,
                (current_rotation * previous_pose[1].inv()).magnitude() / self.config.max_angular_speed,
            )
            steps = max(1, int(np.ceil(duration * hz)))
            for blend in np.linspace(0, 1, steps + 1)[1:]:
                values = previous + (current - previous) * blend
                self.check({**joints, side: values}, grip)
                trajectory.append(values)
            previous, previous_pose = current, (position, current_rotation)
            previous_fraction = fraction
        return trajectory


class FlipTemplate:
    def __init__(self, config=DEFAULT):
        self.config = config
        self.source = ROOT / "assets" / "teaching" / "20261110T035457_916925Z.json"
        document = json.loads(self.source.read_text())
        if document["status"] != "completed" or document["frame"] != "world":
            raise ValueError("A completed world-frame teaching is required")
        self.points = [point["arms"]["left"] for point in document["keyframes"]]
        self.origin = np.array(self.points[0]["tcp_position"])
        self.orientation = Rotation.from_quat(self.points[0]["tcp_quaternion_xyzw"])

    def grasp_pose(self, observation):
        upward = observation.grasp_normal.copy()
        if upward[2] < 0:
            upward *= -1
        tangent = observation.grasp_tangent.copy()
        if tangent[1] < 0:
            tangent *= -1
        inward = unit(np.cross(tangent, upward))
        frame = Rotation.from_matrix(np.column_stack((inward, unit(np.cross(upward, inward)), upward)))
        target = observation.grasp + inward * self.config.grasp_inset
        target[2] += self.config.grasp_height_offset
        return target, frame * self.orientation, frame

    def target(self, index, grasp, frame, extra_height=0.0):
        point = self.points[index]
        target = grasp + frame.apply(np.asarray(point["tcp_position"]) - self.origin)
        target[2] += extra_height
        rotation = frame * Rotation.from_quat(point["tcp_quaternion_xyzw"])
        return target, rotation


def push_contact(observation, offset=0.035):
    corners = observation.corners
    candidates = []
    for index in range(4):
        first, second = corners[index], corners[(index + 1) % 4]
        delta_y = second[1] - first[1]
        if abs(delta_y) < 1e-8:
            continue
        fraction = (observation.center[1] - first[1]) / delta_y
        if 0 <= fraction <= 1:
            candidates.append(first + fraction * (second - first))
    if not candidates:
        raise Refusal("degenerate_boundary")
    boundary = max(candidates, key=lambda point: point[0])
    contact = boundary.copy()
    contact[0] += offset
    contact[2] = observation.center[2] + 0.0175
    return contact
