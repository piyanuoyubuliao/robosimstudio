import hashlib
import json
from itertools import pairwise
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "assets" / "teaching" / "20261110T035457_916925Z.json"
FLIP_CLEARANCE = 0.050
PINCH_CLOSURE = 0.95


def teaching_phases(planner, previous, grip):
    source = SOURCE.read_bytes()
    document = json.loads(source)
    if document["status"] != "completed" or document["frame"] != "world":
        raise ValueError("The flip requires a completed teaching in world coordinates")
    points = document["keyframes"]
    if [point["name"] for point in points] != ["抓取点", "提起点", "旋转后下压", "拖拽", "开爪", "离开"]:
        raise ValueError("The six taught flip points are missing or reordered")
    phases = []
    previous = {side: np.asarray(values) for side, values in previous.items()}
    grip = dict(grip)

    def waypoint(name, seconds, joints=None, closure=None, keyframe=None, tcp_offset_z=0.0):
        nonlocal previous, grip
        targets = {side: values.copy() for side, values in previous.items()}
        if joints is not None:
            targets["left"] = np.asarray(joints)
            seconds = retime(planner, "left", previous["left"], targets["left"], seconds, 60) / 60
        if closure is not None:
            grip["left"] = closure * PINCH_CLOSURE
        phases.append(
            {
                "name": name,
                "seconds": seconds,
                "side": "left" if joints is not None else None,
                "arms": {side: values.tolist() for side, values in targets.items()},
                "grip": grip.copy(),
                "taught": True,
                "keyframe": keyframe,
                "tcp_offset_z": tcp_offset_z,
            }
        )
        previous = targets

    takeover = document["takeover"]["arms"]["left"]
    grasp = points[0]["arms"]["left"]
    approach = list(grasp["tcp_position"])
    approach[2] = 1.055
    approach_joints = planner.solve(
        "left", approach, Rotation.from_quat(grasp["tcp_quaternion_xyzw"]), grasp["joint_positions"], elbow_floor=0.90
    )
    waypoint("left_teaching_entry", 2.0, takeover["joint_positions"], 0.0)
    waypoint("left_approach_corner", 2.0, approach_joints)
    waypoint("left_lower_corner", 4.0, grasp["joint_positions"])
    waypoint("left_clamp_corner", 1.8, closure=1.0, keyframe=points[0]["name"])
    for name, seconds, point in zip(
        ("left_flip_rise", "left_flip_rotate_press", "left_flip_drag", "left_release_face_up", "left_depart"),
        (4.5, 7.0, 4.0, 1.2, 1.5),
        points[1:],
        strict=True,
    ):
        arm = point["arms"]["left"]
        joints = arm["joint_positions"]
        duration = seconds
        if name in {"left_flip_rise", "left_flip_rotate_press"}:
            raised_position = np.array(arm["tcp_position"])
            raised_position[2] += FLIP_CLEARANCE
            raised_joints = planner.solve(
                "left", raised_position, Rotation.from_quat(arm["tcp_quaternion_xyzw"]), joints, elbow_floor=0.90
            )
            if name == "left_flip_rise":
                waypoint(name, seconds, joints, arm["grip_command"], point["name"])
                waypoint("left_flip_extra_lift", 1.5, raised_joints, arm["grip_command"], point["name"], FLIP_CLEARANCE)
                waypoint("left_flip_lift_hold", 0.8)
                continue
            waypoint("left_flip_rotate", seconds, raised_joints, arm["grip_command"], point["name"], FLIP_CLEARANCE)
            duration = 1.5
        waypoint(name, duration, joints, arm["grip_command"], point["name"])
        if name == "left_flip_rotate_press":
            waypoint("left_flip_contact_hold", 0.6)
    waypoint("parcel_settle", 2.0)
    metadata = {
        "file": SOURCE.relative_to(ROOT).as_posix(),
        "sha256": hashlib.sha256(source).hexdigest(),
        "created_at": document["created_at"],
        "interpolation": "smoothstep_joint",
        "flip_clearance_m": FLIP_CLEARANCE,
        "pinch_closure": PINCH_CLOSURE,
        "max_joint_speed_rad_s": 0.35,
        "max_tcp_speed_m_s": 0.06,
        "max_tcp_angular_speed_rad_s": 0.30,
    }
    return phases, metadata


def retime(planner, side, start, end, seconds, control_hz):
    fractions = np.linspace(0.0, 1.0, 101)
    blends = fractions**2 * (3.0 - 2.0 * fractions)
    samples = np.asarray(start) + blends[:, None] * (np.asarray(end) - start)
    poses = [planner.pose(side, sample) for sample in samples]
    joint_peak = np.max(np.abs(np.diff(samples, axis=0))) * 100.0
    positions = np.asarray([pose[0] for pose in poses])
    linear_peak = np.max(np.linalg.norm(np.diff(positions, axis=0), axis=1)) * 100.0
    angular_peak = max((second[1] * first[1].inv()).magnitude() for first, second in pairwise(poses)) * 100.0
    duration = max(seconds, joint_peak / 0.35, linear_peak / 0.06, angular_peak / 0.30)
    return max(1, int(np.ceil(duration * control_hz - 1e-9)))
