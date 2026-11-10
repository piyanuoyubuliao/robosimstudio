from pathlib import Path

import robosimstudio as rss
from robosimstudio.robots import get_gripper, register_gripper
from robosimstudio.specs import GripperSpec

ROOT = Path(__file__).resolve().parent
HOME = (-0.247, 0, 0.909, 0, 1.15644, 0, 0)


def robot_spec(home=None, grip=0.0):
    spec = GripperSpec(
        name="shipping_xarm_gripper",
        finger_names=("{arm}_left_driver_joint", "{arm}_right_driver_joint"),
        finger_open_q=(0.0, 0.0),
        finger_closed_q=(0.85, 0.85),
        linear_travel=False,
        span=0.085,
        tcp_body="{arm}_xarm_gripper_base_link",
        tcp_offset=(0, 0, 0.155),
        close_axis=(0, 1, 0),
        finger_bodies=(("{arm}_left_finger",), ("{arm}_right_finger",)),
    )
    try:
        get_gripper(spec.name)
    except KeyError:
        register_gripper(spec)

    def home_grip(side):
        value = grip[side] * 0.85 if isinstance(grip, dict) else grip
        return (value, value)

    arms = tuple(
        rss.ArmSpec(
            name=side,
            # The carriages are fixed bodies in the MJCF.  There is deliberately no
            # rail/lift DOF here: the two stage bases must remain at their authored
            # world z for the entire routine.
            joint_names=tuple(f"{side}_joint{index}" for index in range(1, 8)),
            base_body_name=f"{side}_stage_base",
            ee_body_name=f"{side}_link7",
            gripper="shipping_xarm_gripper",
            home_q=tuple(home[side]) if home else HOME,
            home_gripper_q=home_grip(side),
        )
        for side in ("left", "right")
    )
    return rss.EmbodimentSpec(
        name="dual_xarm7_shared_pillar",
        asset=rss.AssetSpec(path=str(ROOT / "assets" / "dual_xarm7.xml")),
        arms=arms,
        collapse_fixed_joints=False,
        gravcomp=True,
        enable_self_collisions=False,
        cross_arm_collisions=True,
        ik_iterations=32,
    )
