"""Contact-only corner flip and conveyor transfer."""

import json
from pathlib import Path

import numpy as np
from plan_motion import DOWN, Planner
from taught_motion import retime

ROOT = Path(__file__).resolve().parent


def mailer_state(sim):
    points = np.asarray(sim["shipping_mailer"].particle_q, dtype=float)
    center = points.mean(axis=0)
    count = len(points) // 2
    normal = (points[count:] - points[:count]).mean(axis=0)
    normal /= max(np.linalg.norm(normal), 1e-9)
    belt = (points[:, 0] >= -1.60) & (points[:, 0] <= -0.40)
    surface = (points[:, 0] >= -1.60) & (points[:, 0] <= 0.65)
    lateral = (points[:, 1] >= -0.25) & (points[:, 1] <= 0.55)
    return {
        "center": center.tolist(),
        "min": points.min(axis=0).tolist(),
        "max": points.max(axis=0).tolist(),
        "belt_coverage": float(np.mean(belt & lateral)),
        "surface_coverage": float(np.mean(surface & lateral)),
        "face_up": float(normal[2]),
        "normal": normal.tolist(),
        "finite": bool(np.isfinite(points).all()),
    }


def routine(sim, session=None):
    if session is not None:
        from live_controls import install

        install(session).prepare_routine()
    sim._teaching_progress = {"phase": "planning", "phase_index": 0, "phase_step": 0}
    motion = json.loads((ROOT / "assets" / "motion.json").read_text())
    sides = tuple(sim.scene.embodiment.iks)
    previous = {side: np.asarray(motion["home"][side], dtype=float) for side in sides}
    previous_grip = dict(motion["home_grip"])
    planner = Planner()
    readings = []
    push_start = None
    push_lateral = None
    labels = list(sim.model.body_label)
    base_ids = {
        side: next(index for index, label in enumerate(labels) if str(label).endswith(f"/{side}_stage_base"))
        for side in sides
    }
    base_xyz = {side: sim.state.body_q.numpy()[body_id, :3].copy() for side, body_id in base_ids.items()}

    for phase_index, phase in enumerate(motion["phases"]):
        name = phase["name"]
        sim._teaching_progress = {"phase": name, "phase_index": phase_index, "phase_step": 0}
        state = mailer_state(sim)
        targets = {side: values.copy() for side, values in previous.items()}
        moving_side = phase.get("side")
        target = None
        orientation = None
        cartesian = False
        if moving_side:
            targets[moving_side] = np.asarray(phase["arms"][moving_side])
        if phase.get("taught"):
            pass
        elif name.startswith("right_push") or name in {"right_approach_push", "right_contact_edge"}:
            if push_start is None:
                push_start = min(state["max"][0] + 0.17, 0.25)
                push_lateral = float(np.clip(state["center"][1] - 0.14, 0.08, 0.15))
            if name == "right_approach_push":
                target = (push_start, push_lateral, 1.02)
            elif name == "right_contact_edge":
                target = (push_start, push_lateral, 0.82)
            else:
                progress = int(name.rsplit("_", 1)[1]) / 8
                target = (push_start - 0.10 * progress, push_lateral, 0.82)
            orientation = DOWN
        elif name == "right_retract":
            target, orientation = (0.42, push_lateral, 1.07), DOWN
        elif name.startswith("left_belt_push") or name in {"left_approach_belt_push", "left_contact_belt_edge"}:
            if name == "left_approach_belt_push":
                push_start = min(state["max"][0] + 0.05, 0.30)
                push_lateral = float(np.clip(state["center"][1], -0.05, 0.135))
                target = (push_start, push_lateral, 1.04)
            elif name == "left_contact_belt_edge":
                target = (push_start, push_lateral, 0.82)
            else:
                progress = int(name.rsplit("_", 1)[1]) / 8
                target = (
                    push_start + (-0.54 - push_start) * progress,
                    push_lateral,
                    0.82,
                )
            orientation = DOWN
            cartesian = True
        elif name == "left_belt_retract":
            target, orientation = (-0.50, push_lateral, 1.07), DOWN
            cartesian = True

        count = max(1, round(phase["seconds"] * sim.request.physics.control_hz))
        if phase.get("taught") and moving_side:
            count = retime(
                planner,
                moving_side,
                previous[moving_side],
                targets[moving_side],
                phase["seconds"],
                sim.request.physics.control_hz,
            )
        trajectory = None
        if target is not None:
            if cartesian:
                trajectory = planner.path(moving_side, target, orientation, previous[moving_side], count)
                targets[moving_side] = trajectory[-1]
            else:
                targets[moving_side] = np.asarray(
                    planner.solve(moving_side, target, orientation, previous[moving_side])
                )
        for index in range(1, count + 1):
            fraction = index / count
            blend = fraction * fraction * (3 - 2 * fraction)
            action = []
            for side in sides:
                joints = previous[side] + (targets[side] - previous[side]) * blend
                if trajectory is not None and side == moving_side:
                    joints = trajectory[index - 1]
                action.extend(joints)
                action.append(previous_grip[side] + (phase["grip"][side] - previous_grip[side]) * blend)
            sim.step(np.asarray(action, dtype=np.float32), mode="joint_position")
            sim._teaching_progress.update(phase_step=index, phase_steps=count)
            body_q = sim.state.body_q.numpy()
            for side, body_id in base_ids.items():
                if np.linalg.norm(body_q[body_id, :3] - base_xyz[side]) > 1e-7:
                    raise RuntimeError(f"{side} robot base moved on its rail")
            yield

        previous = targets
        previous_grip = dict(phase["grip"])
        state = mailer_state(sim)
        left_position, left_quaternion = sim.scene.embodiment.iks["left"].ee_pose(sim.state)
        readings.append(
            {
                "phase": name,
                "time": sim.time,
                "steps": count,
                "keyframe": phase.get("keyframe"),
                "left_tcp_position": np.asarray(left_position).tolist(),
                "left_tcp_quaternion_xyzw": np.asarray(left_quaternion).tolist(),
                **state,
            }
        )
        (ROOT / "run" / "current_probe.json").write_text(json.dumps({"phases": readings}, indent=2) + "\n")
        print(
            f"{name}: center={np.round(state['center'], 4).tolist()}, belt={state['belt_coverage']:.1%}, face_up={state['face_up']:.3f}",
            flush=True,
        )
        if not state["finite"]:
            raise RuntimeError("The parcel state is not finite")
        if name == "left_flip_rise" and state["center"][2] < 0.84:
            raise RuntimeError("The left-hand corner pinch did not lift the parcel")
        if name == "parcel_settle" and not (
            state["face_up"] > 0.8 and 0.78 < state["center"][2] < 0.84 and state["surface_coverage"] > 0.98
        ):
            raise RuntimeError(f"The parcel did not settle label-up on the work surface: {state}")

    final = mailer_state(sim)
    result = {
        "teaching": motion.get("teaching"),
        "phases": readings,
        "final": final,
        "success": final["belt_coverage"] >= 0.90 and final["face_up"] > 0.8 and 0.78 < final["center"][2] < 0.84,
    }
    (ROOT / "run" / "last_result.json").write_text(json.dumps(result, indent=2) + "\n")
    if not result["success"]:
        raise RuntimeError(f"Parcel did not finish label-up on the conveyor: {final}")
