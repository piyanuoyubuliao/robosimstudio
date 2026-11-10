import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp
from taught_motion import teaching_phases

ROOT = Path(__file__).resolve().parent
DOWN = Rotation.from_matrix([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
GRASP = Rotation.from_euler("z", 45, degrees=True) * DOWN


class Planner:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / "assets" / "dual_xarm7.xml"))
        self.data = mujoco.MjData(self.model)

    def pose(self, side, joints):
        indices = [self.model.joint(f"{side}_joint{index}").qposadr[0] for index in range(1, 8)]
        self.data.qpos[indices] = joints
        mujoco.mj_forward(self.model, self.data)
        body = self.model.body(f"{side}_xarm_gripper_base_link").id
        matrix = self.data.xmat[body].reshape(3, 3).copy()
        position = self.data.xpos[body] + matrix @ np.array([0.0, 0.0, 0.155])
        return position, Rotation.from_matrix(matrix)

    def path(self, side, target, orientation, seed, count, step_tolerance=0.15):
        start, rotation = self.pose(side, seed)
        rotations = Slerp([0, 1], Rotation.concatenate([rotation, orientation]))
        current = np.asarray(seed)
        trajectory = []
        for index in range(1, count + 1):
            fraction = index / count
            blend = fraction * fraction * (3 - 2 * fraction)
            position = start + (np.asarray(target) - start) * blend
            values = np.asarray(self.solve(side, position, rotations(blend), current, elbow_floor=0.90))
            if np.max(np.abs(values - current)) > step_tolerance:
                raise RuntimeError(f"{side} IK discontinuity along Cartesian path to {target}")
            trajectory.append(values)
            current = values
        return np.asarray(trajectory)

    def solve(self, side, target, orientation, seed, elbow_floor=1.16):
        # The stage bodies are authored at fixed world poses.  Only the seven
        # rotary joints are planned, so a waypoint cannot move a carriage.
        names = [f"{side}_joint{index}" for index in range(1, 8)]
        indices = [self.model.joint(name).qposadr[0] for name in names]
        bounds = np.array([self.model.joint(name).range for name in names])
        body = self.model.body(f"{side}_xarm_gripper_base_link").id
        target = np.asarray(target)
        elbow = self.model.body(f"{side}_link4").id

        def residual(values):
            self.data.qpos[indices] = values
            mujoco.mj_forward(self.model, self.data)
            matrix = self.data.xmat[body].reshape(3, 3)
            position = self.data.xpos[body] + matrix @ np.array([0.0, 0.0, 0.155])
            rotation = Rotation.from_matrix(matrix)
            return np.concatenate(
                [
                    (position - target) * 10,
                    (orientation.inv() * rotation).as_rotvec(),
                    [max(0.0, elbow_floor - self.data.xpos[elbow, 2]) * 8 if target[2] < 1.0 else 0.0],
                    (values - np.asarray(seed)) * 0.002,
                ]
            )

        fit = least_squares(
            residual,
            seed,
            bounds=(bounds[:, 0] + 1e-6, bounds[:, 1] - 1e-6),
            max_nfev=800,
            xtol=1e-10,
            ftol=1e-10,
            gtol=1e-10,
        )
        error = residual(fit.x)
        if np.linalg.norm(error[:3]) > 0.015 or np.linalg.norm(error[3:6]) > 0.03:
            raise RuntimeError(f"{side} IK at {target}: {error[:6]}")
        return fit.x.tolist()


def generate():
    planner = Planner()
    seed = [0.0, -0.247, 0.0, 0.909, 0.0, 1.15644, 0.0]
    fallback_seed = seed[:]

    # The right arm starts as the horizontal pusher.  The left arm is staged
    # at the parcel's receiving corner and is open until the transfer.
    home = {
        "right": planner.solve("right", (0.27, -0.12, 1.08), DOWN, seed),
        "left": planner.solve("left", (-0.40, -0.045, 1.08), DOWN, seed),
    }
    previous = {side: values[:] for side, values in home.items()}
    grip = {"right": 1.0, "left": 0.0}
    phases = []

    def waypoint(name, seconds, side=None, position=None, orientation=None, grips=None, elbow_floor=1.16):
        nonlocal previous, grip
        targets = {arm: values[:] for arm, values in previous.items()}
        if side:
            try:
                targets[side] = planner.solve(side, position, orientation, previous[side], elbow_floor=elbow_floor)
            except RuntimeError:
                # A long clearance-to-belt move can land in a local IK basin;
                # restart that waypoint from the neutral arm seed without
                # changing the requested Cartesian pose.
                targets[side] = planner.solve(side, position, orientation, fallback_seed, elbow_floor=elbow_floor)
        if grips:
            grip = {**grip, **grips}
        phases.append({"name": name, "seconds": seconds, "side": side, "arms": targets, "grip": grip.copy()})
        previous = targets

    waypoint("ready", 0.7)

    # Push from the parcel's right edge toward the left receiving hand.  The
    # contact and eight short strokes retain the original left-push cadence.
    waypoint("right_approach_push", 1.5, "right", (0.25, 0.12, 1.02), DOWN)
    waypoint("right_contact_edge", 1.2, "right", (0.25, 0.12, 0.82), DOWN)
    for index, coordinate in enumerate(np.linspace(0.25, 0.15, 9)[1:]):
        waypoint(f"right_push_{index + 1:02d}", 0.45, "right", (float(coordinate), 0.12, 0.82), DOWN)
    waypoint("right_retract", 1.4, "right", (0.42, 0.12, 1.07), DOWN)
    waypoint("right_release_transfer", 0.6, grips={"right": 0.0})

    taught, teaching = teaching_phases(planner, previous, grip)
    phases.extend(taught)
    previous = {side: values[:] for side, values in taught[-1]["arms"].items()}
    grip = dict(taught[-1]["grip"])

    # The receiving left hand then uses the same pusher style to place the
    # label-up parcel across the desktop onto the stationary conveyor.
    waypoint("left_approach_belt_push", 1.7, "left", (0.25, 0.015, 1.02), DOWN, grips={"left": 1.0})
    waypoint("left_contact_belt_edge", 1.2, "left", (0.25, 0.015, 0.82), DOWN)
    for index, coordinate in enumerate(np.linspace(0.25, -0.54, 9)[1:]):
        waypoint(f"left_belt_push_{index + 1:02d}", 0.45, "left", (float(coordinate), 0.015, 0.82), DOWN)
    waypoint("left_belt_retract", 1.4, "left", (-0.50, 0.015, 1.07), DOWN)
    waypoint("belt_settle", 1.0)

    motion = {"home": home, "home_grip": {"right": 1.0, "left": 0.0}, "phases": phases, "teaching": teaching}
    (ROOT / "assets" / "motion.json").write_text(json.dumps(motion, indent=2) + "\n")
    print(
        json.dumps(
            {"home": home, "phases": len(phases), "seconds": sum(phase["seconds"] for phase in phases)}, indent=2
        )
    )
    return motion


if __name__ == "__main__":
    generate()
