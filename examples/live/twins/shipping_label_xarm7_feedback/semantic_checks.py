"""Exercise actual policy failure branches without claiming physical success."""

import json
import sys
from collections import Counter
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from config import DEFAULT
from observation import estimate
from planning import Refusal
from policy import FeedbackPolicy
from validation import synthetic


def drain(generator):
    while True:
        try:
            next(generator)
        except StopIteration as complete:
            return complete.value


class Robot:
    sides = ("left", "right")

    def wait(self, seconds):
        yield

    def retreat(self, side):
        yield

    def set_grip(self, side, closure):
        yield

    def pose(self, side):
        return np.array([0, 0, 1.0]), Rotation.identity()


policy = object.__new__(FeedbackPolicy)
policy.config = DEFAULT
policy.robot = Robot()
policy.held = False
policy.retries = Counter()
policy.observation = estimate(synthetic())
policy.before_grasp = policy.observation
policy.before_push = policy.observation
policy.read = lambda: policy.observation
checks = []
for method, expected in [
    ("verify_transfer", "no_contact"),
    ("verify_lift", "empty_grasp"),
    ("verify_flip", "wrong_face"),
]:
    try:
        drain(getattr(policy, method)())
    except Refusal as error:
        assert error.reason == expected, (method, error)
        checks.append(expected)
    else:
        raise AssertionError(method)


class GraspRefusal(Robot):
    def set_grip(self, side, closure):
        raise Refusal("jaw_clearance")
        yield


policy.robot = GraspRefusal()
policy.template = SimpleNamespace(
    grasp_pose=lambda observation: (observation.grasp.copy(), Rotation.identity(), Rotation.identity())
)
for _attempt in range(2):
    try:
        drain(policy.grasp())
    except Refusal as error:
        assert error.reason == "jaw_clearance"
    else:
        raise AssertionError("Expected grasp refusal")
assert callable(policy.grasp)
checks.append("grasp_callable_after_failed_attempt")
policy.robot = Robot()
policy.patch_local = np.zeros(3)
policy.patch = lambda: np.array([0.1, 0, 1])
try:
    policy.check_slip()
except Refusal as error:
    assert error.reason == "slip"
    checks.append("slip_material_patch")
else:
    raise AssertionError("Unnoticed slip")
for attempt in range(DEFAULT.max_retries + 1):
    policy.failure, policy.failure_stage = "empty_grasp", "verify_lift"
    status = drain(policy.retry())
    assert status == ("observe" if attempt < DEFAULT.max_retries else "abort")
assert policy.retries["verify_lift"] == DEFAULT.max_retries
checks.append("retry_budget_not_reset")
policy.failure, policy.failure_stage = "severe_deformation", "observe"
assert drain(policy.retry()) == "abort"
checks.append("terminal_deformation")
policy.observation = estimate(synthetic(face_up=True))
assert drain(policy.grasp()) == "skip"
checks.append("actual_grasp_skip")
policy.events = []
policy.result = {}
policy.nodes = DEFAULT.max_nodes
policy.sim = SimpleNamespace(time=0)
assert drain(policy.dispatch("observe")) == {"status": "abort"}
checks.append("node_budget")
result = {"kind": "policy_methods_with_stub_robot_no_physics", "passed": len(checks), "checks": checks}
(ROOT / "run" / "semantic_checks.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2))
