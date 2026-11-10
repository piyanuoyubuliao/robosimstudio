import mujoco
import numpy as np
from plan_motion import Planner
from scipy.spatial.transform import Rotation


class TeachingArm:
    def __init__(self, panel):
        self.panel = panel
        self.planner = Planner()
        model = self.planner.model
        joints = [model.joint(f"{panel.name}_joint{index}") for index in range(1, 8)]
        self.dofs = [int(joint.dofadr[0]) for joint in joints]
        self.lower = np.array([joint.range[0] for joint in joints])
        self.upper = np.array([joint.range[1] for joint in joints])
        self.body = model.body(f"{panel.name}_xarm_gripper_base_link").id
        self.position_jacobian = np.zeros((3, model.nv))
        self.rotation_jacobian = np.zeros((3, model.nv))
        self.dt = 1.0 / panel.control_hz
        self.command = panel.env.state.joint_q.numpy()[panel.ik.arm_coords].astype(float)
        self.goal_position = np.array(panel.cursor_pos, dtype=float)
        self.goal_rotation = Rotation.from_quat(panel.cursor_quat)
        self.moving = False

    def drive(self, env):
        position = np.asarray(self.panel.cursor_pos, dtype=float)
        quaternion = np.asarray(self.panel.cursor_quat, dtype=float)
        if not np.isfinite(position).all() or not np.isfinite(quaternion).all() or np.linalg.norm(quaternion) < 1e-8:
            self.panel.status = "无效目标，保持当前关节"
            return
        rotation = Rotation.from_quat(quaternion)
        if (
            np.linalg.norm(position - self.goal_position) > 1e-6
            or (rotation * self.goal_rotation.inv()).magnitude() > 1e-5
        ):
            self.goal_position = position.copy()
            self.goal_rotation = rotation
            self.moving = True
        if self.moving:
            self._advance(env)
        self.panel.ik.set_joint_targets(self.command, env.control.joint_target_q)
        actual_position, actual_quaternion = self.panel.ik.ee_pose(env.state)
        gap = float(np.linalg.norm(actual_position - self.goal_position))
        angle = float((self.goal_rotation * Rotation.from_quat(actual_quaternion).inv()).magnitude())
        self.panel.status = f"目标误差 {gap * 1000:.1f} mm / {np.degrees(angle):.1f}°"

    def _advance(self, env):
        position, rotation = self.planner.pose(self.panel.name, self.command)
        translation = self.goal_position - position
        turn = (self.goal_rotation * rotation.inv()).as_rotvec()
        if np.linalg.norm(translation) < 0.0002 and np.linalg.norm(turn) < 0.002:
            self.moving = False
            return
        translation *= min(1.0, 0.04 * self.dt / max(np.linalg.norm(translation), 1e-12))
        turn *= min(1.0, 0.3 * self.dt / max(np.linalg.norm(turn), 1e-12))
        mujoco.mj_jac(
            self.planner.model,
            self.planner.data,
            self.position_jacobian,
            self.rotation_jacobian,
            position,
            self.body,
        )
        jacobian = np.vstack((self.position_jacobian[:, self.dofs], 0.1 * self.rotation_jacobian[:, self.dofs]))
        error = np.concatenate((translation, 0.1 * turn))
        delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + 0.005**2 * np.eye(6), error)
        delta *= min(1.0, 0.35 * self.dt / max(np.max(np.abs(delta)), 1e-12))
        candidate = np.clip(self.command + delta, self.lower, self.upper)
        actual = env.state.joint_q.numpy()[self.panel.ik.arm_coords]
        if not np.isfinite(candidate).all() or np.max(np.abs(candidate - actual)) > 0.08:
            return
        self.command = candidate
