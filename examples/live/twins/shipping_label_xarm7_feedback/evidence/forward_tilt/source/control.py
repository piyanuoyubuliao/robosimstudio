import json
from pathlib import Path

import numpy as np
from config import DEFAULT
from planning import Planner, Refusal
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent


class RobotControl:
    def __init__(self, sim, config=DEFAULT):
        self.sim, self.config = sim, config
        self.planner = Planner(config)
        self.sides = tuple(sim.scene.embodiment.iks)
        self.hz = sim.request.physics.control_hz
        self.home = json.loads((ROOT / "assets" / "motion.json").read_text())
        self.grip = dict(self.home["home_grip"])
        self.joints = self.measured_joints()
        labels = list(map(str, sim.model.body_label))
        self.base_ids = [
            next(index for index, label in enumerate(labels) if label.endswith(f"/{side}_stage_base"))
            for side in self.sides
        ]
        self.base_positions = sim.state.body_q.numpy()[self.base_ids, :3].copy()

    def measured_joints(self):
        values = self.sim.state.joint_q.numpy()
        return {side: values[self.sim.scene.embodiment.iks[side].arm_coords].astype(float) for side in self.sides}

    def pose(self, side):
        position, quaternion = self.sim.scene.embodiment.iks[side].ee_pose(self.sim.state)
        return np.asarray(position, dtype=float), Rotation.from_quat(np.asarray(quaternion))

    def step(self):
        action = np.concatenate([np.append(self.joints[side], self.grip[side]) for side in self.sides])
        self.sim.step(action.astype(np.float32), mode="joint_position")
        current = self.sim.state.body_q.numpy()[self.base_ids, :3]
        if not np.allclose(current, self.base_positions, atol=1e-7, rtol=0):
            raise Refusal("base_moved")
        yield

    def wait(self, seconds):
        for _ in range(max(1, round(seconds * self.hz))):
            yield from self.step()

    def move(self, side, target, orientation, monitor=None):
        trajectory = self.planner.path(side, target, orientation, self.joints, self.grip, self.hz)
        for index, values in enumerate(trajectory):
            self.joints[side] = values
            yield from self.step()
            if monitor is not None and index % 6 == 0:
                monitor()
        yield from self.wait(0.10)
        position, rotation = self.pose(side)
        if np.linalg.norm(position - target) > 0.012 or (orientation.inv() * rotation).magnitude() > 0.12:
            raise Refusal("tracking_error", f"{side}: actual={position.tolist()}, target={np.asarray(target).tolist()}")
        self.joints = self.measured_joints()

    def set_grip(self, side, closure, seconds=0.8):
        previous = self.grip[side]
        for blend in np.linspace(0, 1, max(2, round(seconds * self.hz))):
            self.grip[side] = previous + blend * (closure - previous)
            self.planner.check(self.joints, self.grip)
            yield from self.step()
        yield from self.wait(0.25)

    def jaw_gap(self, side):
        body_q = self.sim.state.body_q.numpy()
        labels = list(map(str, self.sim.model.body_label))
        pads = []
        for finger, direction in (("left", -1), ("right", 1)):
            body = next(index for index, label in enumerate(labels) if label.endswith(f"/{side}_{finger}_finger"))
            transform = body_q[body]
            point = transform[:3] + Rotation.from_quat(transform[3:]).apply([0, direction * 0.024003, 0.050])
            pads.append(point)
        return float(np.linalg.norm(pads[0] - pads[1]) - 0.004)

    def retreat(self, side):
        position, rotation = self.pose(side)
        if side == "right":
            # The inner handoff edge is reachable at table height, but a vertical
            # rise there crosses the right arm's workspace boundary.
            position[0] = max(position[0], 0.25)
        position[2] = max(position[2], 1.09)
        yield from self.move(side, position, rotation)

    def park(self, side):
        yield from self.retreat(side)
        target, rotation = self.planner.pose(side, self.home["home"][side])
        yield from self.move(side, target, rotation)
