import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from config import DEFAULT, Config
from planning import Planner, Refusal, push_contact
from policy import FeedbackPolicy

DEFAULT = globals().get("CONFIG_OVERRIDE", DEFAULT)
Config = type(DEFAULT)

root = Path(__file__).resolve().parent.parent
home = json.loads((root / 'assets/motion.json').read_text())
rows = json.loads((root / 'evidence/acceptance.json').read_text())['rows']
checks = []
for row in rows:
    if row['case']['name'] not in ('yaw_15_back', 'yaw_-15_front'):
        continue
    event = next(e for e in row['events'] if e['node'] == 'transfer' and e['status'] == 'retry')
    obs = SimpleNamespace(**{key: np.asarray(event['observation'][key]) for key in ('center', 'corners')})
    contact = push_contact(obs)
    above = contact.copy()
    above[2] = DEFAULT.push_approach_z
    target = contact - [DEFAULT.push_step + 0.035, 0, 0]
    policy = object.__new__(FeedbackPolicy)
    policy.config, policy.result = DEFAULT, {}
    joints = {side: np.array(values) for side, values in home['home'].items()}
    grip = dict(home['home_grip'])
    policy.robot = SimpleNamespace(planner=Planner(), joints=joints, grip=grip, hz=60)
    rotation = policy.transfer_orientation(above, contact, target)
    plan = policy.result['right_push_plans'][-1]
    assert plan['tilt_degrees'] == 5.0, plan
    assert plan['rejected'][0]['failure'] == 'unreachable', plan
    assert rotation.apply([0, 0, 1])[1] > 0
    assert all(np.array_equal(joints[side], home['home'][side]) for side in joints)
    assert grip == home['home_grip']
    checks.append(row['case']['name'] + '_five_degree_fallback_without_mutation')

class BlockedPlanner:
    def path(self, *args):
        raise Refusal('collision', 'blocked candidate')

policy.robot.planner = BlockedPlanner()
try:
    policy.transfer_orientation(above, contact, target)
except Refusal as error:
    assert error.reason == 'collision'
else:
    raise AssertionError('Collision was accepted')
plan = policy.result['right_push_plans'][-1]
assert plan['tilt_degrees'] is None and len(plan['rejected']) == len(DEFAULT.right_push_tilts)
checks.append('all_candidates_blocked_refuses')
for values in ((), (-5,), (50,), (5, 0), (0, 5, 5), (float('nan'),)):
    try:
        Config(right_push_tilts=values)
    except ValueError:
        continue
    raise AssertionError(values)
checks.append('invalid_tilt_candidates_rejected')
result = {'kind': 'real_kinematic_paths_and_failure_contracts_no_contact_physics', 'passed': len(checks), 'checks': checks}
(root / 'run/push_tilt_checks_45.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
