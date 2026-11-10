import json
import queue
import threading
import time
from collections import Counter
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from config import DEFAULT
from control import RobotControl
from observation import ObservationError, ParticleObserver, top_first
from planning import DOWN, FlipTemplate, Refusal, push_contact
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent


class FeedbackPolicy:
    def __init__(self, sim, config=DEFAULT, observer=None):
        self.sim, self.config = sim, config
        self.observer = observer or ParticleObserver(sim, config=config)
        self.robot = RobotControl(sim, config)
        self.template = FlipTemplate(config)
        self.events = []
        self.retries = Counter()
        self.strokes = Counter()
        self.failure = None
        self.failure_stage = None
        self.held = False
        self.push_active = None
        self.nodes = 0
        self.observation = None
        self.input_accepted = False
        self.result = {"success": False, "scope": "single_flat_same_size", "events": self.events}

    def read(self):
        observations = self.observer.observe()
        ordered = top_first(observations)
        if len(observations) != 1:
            raise Refusal("multi_parcel_out_of_scope", f"top_first={[item.name for item in ordered]}")
        if not ordered:
            raise Refusal("no_visible_parcel")
        self.observation = ordered[0]
        return self.observation

    def reachable(self, observation):
        config = self.config
        return (
            config.left_center_x[0] <= observation.center[0] <= config.left_center_x[1]
            and config.left_center_y[0] <= observation.center[1] <= config.left_center_y[1]
        )

    def dispatch(self, node):
        self.nodes += 1
        start = self.sim.time
        if self.nodes > self.config.max_nodes:
            self.failure = "node_budget"
            self.result.update(failure=self.failure, failure_stage=node)
            self.events.append(
                {
                    "node": node,
                    "status": "abort",
                    "start": float(start),
                    "end": float(start),
                    "failure": self.failure,
                    "observation": None,
                }
            )
            return {"status": "abort"}
        self.sim._teaching_progress = {"phase": node, "phase_index": self.nodes, "phase_step": 0}
        try:
            status = yield from getattr(self, node)()
        except (Refusal, ObservationError) as error:
            self.failure = error.reason if isinstance(error, Refusal) else str(error)
            self.failure_stage = node
            status = "retry"
            if node == "retry" or self.failure in {"collision", "base_moved", "invalid_particles", "tracking_error"}:
                status = "abort"
            self.result["detail"] = str(error)
        event = {
            "node": node,
            "status": status,
            "start": float(start),
            "end": float(self.sim.time),
            "failure": self.failure if status in {"retry", "abort"} else None,
            "detail": self.result.get("detail") if status in {"retry", "abort"} else None,
            "observation": self.observation.record() if self.observation is not None else None,
        }
        self.events.append(event)
        self.result.update(
            failure=self.failure,
            failure_stage=self.failure_stage if self.failure else None,
            retries=dict(self.retries),
            strokes=dict(self.strokes),
        )
        print(f"policy {node}: {status}; {event['failure'] or ''}", flush=True)
        return {"status": status}

    def observe(self):
        observation = self.read()
        failure = observation.flat_failure(self.config)
        if failure:
            raise Refusal(failure)
        if observation.belt_coverage >= self.config.belt_coverage:
            return "belt"
        if (
            self.input_accepted
            and observation.face_up > self.config.face_threshold
            and observation.center[0] < self.config.left_center_x[0]
        ):
            yield from self.robot.park("right")
            return "belt"
        if not self.config.input_center_y[0] <= observation.center[1] <= self.config.input_center_y[1]:
            raise Refusal("outside_workspace_y")
        if not self.config.input_center_x[0] <= observation.center[0] <= self.config.input_center_x[1]:
            raise Refusal("outside_workspace_x")
        self.input_accepted = True
        if not self.reachable(observation):
            return "transfer"
        if observation.face_up > self.config.face_threshold and not self.push_reachable(observation):
            return "transfer"
        yield from self.robot.park("right")
        return "belt" if observation.face_up > self.config.face_threshold else "grasp"

    def push_reachable(self, observation):
        """Preview the actual left pushing contact, with the other hand parked."""
        joints = {side: values.copy() for side, values in self.robot.joints.items()}
        joints["right"] = np.asarray(self.robot.home["home"]["right"])
        grip = self.robot.grip.copy()
        contact = push_contact(observation)
        above = contact.copy()
        above[2] = self.config.push_approach_z
        position, rotation = self.robot.pose("left")
        position[2] = max(position[2], 1.09)
        try:
            for index, (target, orientation) in enumerate(
                [
                    (position, rotation),
                    (above, DOWN),
                    (contact, DOWN),
                    (contact - [self.config.push_step + 0.035, 0, 0], DOWN),
                ]
            ):
                if index >= 2:
                    grip["left"] = 1.0
                values = self.robot.planner.path("left", target, orientation, joints, grip, self.robot.hz)
                joints["left"] = values[-1]
            return True
        except Refusal as error:
            if error.reason not in {"unreachable", "ik_discontinuity", "collision"}:
                raise
            self.result["handoff_preview"] = str(error)
            return False

    def transfer_orientation(self, above, contact, target):
        """Choose the smallest forward tilt with a clear complete push path."""
        attempts = []
        retreat = target.copy()
        retreat[0] = max(retreat[0], 0.25)
        retreat[2] = max(retreat[2], 1.09)
        for degrees in self.config.right_push_tilts:
            # World +x rotation tips the downward fingers toward +y, bringing
            # the wrist back toward the arm while keeping the contact fixed.
            rotation = Rotation.from_euler("x", degrees, degrees=True) * DOWN
            joints = {side: values.copy() for side, values in self.robot.joints.items()}
            grip = self.robot.grip.copy()
            try:
                for index, point in enumerate((above, contact, target, retreat)):
                    if index == 1:
                        for closure in np.linspace(grip["right"], 1.0, max(2, round(0.8 * self.robot.hz))):
                            grip["right"] = closure
                            self.robot.planner.check(joints, grip)
                    values = self.robot.planner.path("right", point, rotation, joints, grip, self.robot.hz)
                    joints["right"] = values[-1]
            except Refusal as error:
                if error.reason not in {"unreachable", "ik_discontinuity", "collision"}:
                    raise
                attempts.append({"tilt_degrees": degrees, "failure": error.reason, "detail": str(error)})
                continue
            self.result.setdefault("right_push_plans", []).append(
                {"contact": contact.tolist(), "tilt_degrees": degrees, "rejected": attempts}
            )
            return rotation
        self.result.setdefault("right_push_plans", []).append(
            {"contact": contact.tolist(), "tilt_degrees": None, "rejected": attempts}
        )
        raise Refusal(attempts[-1]["failure"], f"right push: no feasible tilt; {attempts}")

    def push(self, side, stage):
        observation = self.read()
        failure = observation.flat_failure(self.config)
        if failure:
            raise Refusal(failure)
        self.before_push = observation
        self.strokes[stage] += 1
        limit = self.config.max_transfer_strokes if side == "right" else self.config.max_belt_strokes
        if self.strokes[stage] > limit:
            raise Refusal("push_budget")
        contact = push_contact(observation)
        above = contact.copy()
        above[2] = self.config.push_approach_z
        target = contact.copy()
        target[0] -= self.config.push_step + 0.035
        rotation = DOWN
        current, _ = self.robot.pose(side)
        continuing = (
            getattr(self, "push_active", None) == side
            and abs(current[1] - contact[1]) < 0.025
            and abs(current[2] - contact[2]) < 0.015
        )
        if not continuing:
            yield from self.robot.retreat(side)
            if side == "right":
                rotation = self.transfer_orientation(above, contact, target)
            yield from self.robot.move(side, above, rotation)
            yield from self.robot.set_grip(side, 1.0)
            yield from self.robot.move(side, contact, rotation)
        yield from self.robot.move(side, target, rotation)
        self.push_active = side if stage == "belt" else None
        if stage != "belt":
            yield from self.robot.retreat(side)
        yield from self.robot.wait(0.15)

    def transfer(self):
        yield from self.push("right", "transfer")
        return "check"

    def verify_transfer(self):
        observation = self.read()
        moved = self.before_push.center[0] - observation.center[0]
        if moved < self.config.minimum_push:
            raise Refusal("no_contact", f"x progress {moved:.5f} m")
        if False:
            yield
        return "observe"

    def grasp(self):
        observation = self.read()
        if observation.face_up > self.config.face_threshold:
            return "skip"
        failure = observation.flat_failure(self.config)
        if failure:
            raise Refusal(failure)
        self.grasp_target, rotation, self.grasp_frame = self.template.grasp_pose(observation)
        if np.min(np.linalg.norm(observation.corners - observation.grasp, axis=1)) < 0.08:
            raise Refusal("corner_clearance")
        yield from self.robot.set_grip("left", 0.0)
        if self.robot.jaw_gap("left") < observation.grasp_thickness + 0.012:
            raise Refusal("jaw_clearance")
        above = self.grasp_target.copy()
        above[2] = 1.09
        yield from self.robot.move("left", above, rotation)
        observation = self.read()
        self.grasp_target, rotation, self.grasp_frame = self.template.grasp_pose(observation)
        preview_joints = {side: values.copy() for side, values in self.robot.joints.items()}
        preview_grip = self.robot.grip.copy()
        for index, (target, orientation) in enumerate(
            [(self.grasp_target, rotation)]
            + [
                self.template.target(index, self.grasp_target, self.grasp_frame, self.config.flip_clearance)
                for index in (1, 2)
            ]
        ):
            if index:
                preview_grip["left"] = self.config.pinch
            values = self.robot.planner.path("left", target, orientation, preview_joints, preview_grip, self.robot.hz)
            preview_joints["left"] = values[-1]
        self.before_grasp = observation
        self.patch_indices = observation.grasp_indices.copy()
        yield from self.robot.move("left", self.grasp_target, rotation)
        yield from self.robot.set_grip("left", self.config.pinch, 1.2)
        target, orientation = self.template.target(1, self.grasp_target, self.grasp_frame)
        yield from self.robot.move("left", target, orientation)
        yield from self.robot.wait(0.25)
        return "check"

    def patch(self):
        return self.observer.track_patch(self.observation.name, self.patch_indices)

    def verify_lift(self):
        observation = self.read()
        height = observation.center[2] - self.before_grasp.center[2]
        if height < self.config.lift_minimum or observation.lower[2] < self.config.table_z + 0.020:
            raise Refusal("empty_grasp", f"lift={height:.4f}, bottom={observation.lower[2]:.4f}")
        position, rotation = self.robot.pose("left")
        self.patch_local = rotation.inv().apply(self.patch() - position)
        self.held = True
        yield from self.robot.wait(0.25)
        self.check_slip()
        return "flip"

    def check_slip(self):
        observation = self.read()
        position, rotation = self.robot.pose("left")
        expected = position + rotation.apply(self.patch_local)
        drift = np.linalg.norm(self.patch() - expected)
        if drift > self.config.slip_distance or observation.center[2] < self.config.table_z + 0.045:
            raise Refusal("slip", f"patch drift={drift:.4f}")
        if observation.normal_disagreement > 0.35 or observation.strain > 0.8:
            raise Refusal("grasp_deformation")

    def flip(self):
        self.check_slip()
        target, rotation = self.template.target(1, self.grasp_target, self.grasp_frame, self.config.flip_clearance)
        yield from self.robot.move("left", target, rotation, self.check_slip)
        observation = self.read()
        sag = max(0.0, self.config.table_z + 0.035 - observation.lower[2])
        if sag > 0.04:
            raise Refusal("flip_clearance", f"required extra lift={sag:.4f}")
        target, rotation = self.template.target(
            2, self.grasp_target, self.grasp_frame, self.config.flip_clearance + sag
        )
        yield from self.robot.move("left", target, rotation, self.check_slip)
        self.check_slip()
        target, rotation = self.template.target(2, self.grasp_target, self.grasp_frame)
        yield from self.robot.move("left", target, rotation, self.check_slip)
        yield from self.robot.wait(0.30)
        target, rotation = self.template.target(3, self.grasp_target, self.grasp_frame)
        yield from self.robot.move("left", target, rotation, self.check_slip)
        yield from self.robot.set_grip("left", 0.0)
        self.held = False
        yield from self.robot.retreat("left")
        yield from self.robot.wait(1.5)
        return "check"

    def verify_flip(self):
        observation = self.read()
        if observation.face_up < self.config.face_threshold:
            raise Refusal("wrong_face", f"normal_z={observation.face_up:.4f}")
        failure = observation.flat_failure(self.config)
        if failure:
            raise Refusal(failure)
        if False:
            yield
        return "belt"

    def belt(self):
        observation = self.read()
        if observation.face_up < self.config.face_threshold:
            raise Refusal("wrong_face")
        if observation.belt_coverage < self.config.belt_coverage:
            yield from self.push("left", "belt")
        return "check"

    def verify_belt(self):
        observation = self.read()
        if observation.belt_coverage < self.config.belt_coverage:
            if self.before_push.center[0] - observation.center[0] < self.config.minimum_push:
                raise Refusal("no_contact")
            return "continue"
        yield from self.robot.retreat("left")
        previous = self.read()
        for _ in range(3):
            yield from self.robot.wait(self.config.stable_seconds / 3)
            current = self.read()
            speed = np.linalg.norm(current.center - previous.center) / (current.timestamp - previous.timestamp)
            if (
                current.belt_coverage < self.config.belt_coverage
                or current.face_up < self.config.face_threshold
                or current.flat_failure(self.config)
                or speed > self.config.stable_speed
                or not self.config.table_z + 0.01 < current.center[2] < self.config.table_z + 0.04
            ):
                raise Refusal("unstable_belt")
            previous = current
        self.result.update(success=True, failure=None, final=current.record())
        return "success"

    def retry(self):
        self.push_active = None
        reason = self.failure
        stage = self.failure_stage
        terminal = {
            "multi_parcel_out_of_scope",
            "outside_workspace_x",
            "outside_workspace_y",
            "side_standing",
            "severe_deformation",
            "outside_surface",
            "push_budget",
            "no_visible_parcel",
        }
        if reason in terminal or self.retries[stage] >= self.config.max_retries:
            return "abort"
        self.retries[stage] += 1
        if self.held:
            position, rotation = self.robot.pose("left")
            observation = self.read()
            position[2] -= max(0, observation.lower[2] - self.config.table_z - 0.025)
            yield from self.robot.move("left", position, rotation)
        yield from self.robot.set_grip("left", 0.0)
        self.held = False
        for side in self.robot.sides:
            yield from self.robot.retreat(side)
        yield from self.robot.wait(0.8)
        self.failure = None
        return "observe"


class LiveConnector:
    def __init__(self):
        from gap_core.tools import ToolRegistry

        self.requests = queue.Queue()
        self.closed = threading.Event()
        self.tool_registry = ToolRegistry()
        self.tool_registry.register_callable(
            "parcel.node", self.node, summary="Run one parcel feedback stage on the simulation thread"
        )

    def node(self, stage: str):
        request = SimpleNamespace(name=stage, done=threading.Event(), value=None, error=None)
        self.requests.put(request)
        deadline = time.monotonic() + 3600
        while not request.done.wait(0.1):
            if self.closed.is_set():
                raise RuntimeError("Policy cancelled")
            if time.monotonic() > deadline:
                raise TimeoutError(f"Policy node timed out: {stage}")
        if request.error is not None:
            raise request.error
        return request.value


def run_graph(sim, config=DEFAULT, observer=None, trace_dir=None):
    from gap import execute

    policy = FeedbackPolicy(sim, config, observer)
    trace_dir = Path(trace_dir) if trace_dir else ROOT / "run" / "graph_trace" / str(time.time_ns())
    policy.result["trace_dir"] = str(trace_dir.relative_to(ROOT))
    connector = LiveConnector()
    outcome = {}

    def execute_graph():
        try:
            outcome["result"] = execute(
                ROOT / "graph",
                connector=connector,
                skills=[],
                trace_dir=trace_dir,
                max_node_workers=1,
            )
        except Exception as error:
            outcome["error"] = error

    thread = threading.Thread(target=execute_graph, daemon=True, name="parcel-policy")
    thread.start()
    try:
        while thread.is_alive():
            try:
                request = connector.requests.get(timeout=0.02)
            except queue.Empty:
                yield
                continue
            try:
                request.value = yield from policy.dispatch(request.name)
            except Exception as error:
                request.error = error
            finally:
                request.done.set()
            yield
    finally:
        connector.closed.set()
        thread.join(timeout=1.0)
    result = outcome.get("result")
    error = outcome.get("error") or getattr(result, "error", None)
    policy.result["graph_success"] = bool(getattr(result, "success", False))
    policy.result["success"] = bool(policy.result["success"] and policy.result["graph_success"])
    if error:
        policy.result["graph_error"] = repr(error)
    return policy.result


def routine(sim):
    result = yield from run_graph(sim)
    (ROOT / "run").mkdir(exist_ok=True)
    (ROOT / "run" / "last_result.json").write_text(json.dumps(result, indent=2) + "\n")
    if not result["success"]:
        raise RuntimeError(f"Policy did not complete: {result.get('failure') or result.get('graph_error')}")
    return result
