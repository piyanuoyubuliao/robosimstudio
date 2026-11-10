"""Local contract checks and physical sweeps, kept with this twin."""

import hashlib
import json
from collections import Counter
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from config import DEFAULT
from observation import CameraObserver, ObservationError, estimate, top_first
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent


def synthetic(center=(0.08, 0.1, 0.803), yaw=0, face_up=False):
    x, y = np.meshgrid(np.linspace(-0.12, 0.12, 13), np.linspace(-0.16, 0.16, 17))
    surface = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    points = np.concatenate((surface - [0, 0, 0.015], surface + np.array([0, 0, 0.015])))
    rotation = Rotation.from_euler("z", yaw, degrees=True) * Rotation.from_euler(
        "x", 0 if face_up else 180, degrees=True
    )
    return rotation.apply(points) + center


def contract_checks():
    from gap import execute
    from gap_core.tools import ToolRegistry

    checks = []
    for yaw in (-179, -91, -45, -15, 0, 15, 45, 89, 179):
        for front in (False, True):
            observation = estimate(synthetic(yaw=yaw, face_up=front))
            assert abs(observation.face_up - (1 if front else -1)) < 1e-8
            assert observation.flat_failure() is None
            assert np.linalg.norm(observation.center - [0.08, 0.1, 0.803]) < 1e-10
            expected = (np.deg2rad(yaw) + np.pi / 2) % np.pi - np.pi / 2
            assert abs(observation.yaw - expected) < 1e-8
            assert np.min(np.linalg.norm(observation.corners - observation.grasp, axis=1)) > 0.15
            checks.append(f"observe_yaw_{yaw}_front_{front}")
    lower = estimate(synthetic(), name="lower")
    upper = estimate(synthetic(center=(0.09, 0.1, 0.84)), name="upper")
    assert [item.name for item in top_first([lower, upper])] == ["upper"]
    checks.append("top_first")
    try:
        estimate(np.full((442, 3), np.nan))
    except ObservationError:
        checks.append("reject_nonfinite")
    else:
        raise AssertionError("Nonfinite particles accepted")
    try:
        CameraObserver(lambda: [lower], lambda: 1.0).observe()
    except ObservationError:
        checks.append("reject_stale_camera")
    else:
        raise AssertionError("Stale camera frame accepted")
    routes = {
        "front_skip": {"observe": ["belt"], "belt": ["check"], "verify_belt": ["success"]},
        "back_flip": {
            "observe": ["transfer", "grasp"],
            "transfer": ["check"],
            "verify_transfer": ["observe"],
            "grasp": ["check"],
            "verify_lift": ["flip"],
            "flip": ["check"],
            "verify_flip": ["belt"],
            "belt": ["check", "check"],
            "verify_belt": ["continue", "success"],
        },
        "empty_retry": {
            "observe": ["grasp", "grasp"],
            "grasp": ["check", "check"],
            "verify_lift": ["retry", "retry"],
            "retry": ["observe", "abort"],
        },
        "collision_abort": {"observe": ["transfer"], "transfer": ["abort"]},
    }
    for name, route in routes.items():
        seen = []

        def node(stage, seen=seen, route=route):
            seen.append(stage)
            return {"status": route[stage].pop(0)}

        registry = ToolRegistry()
        registry.register_callable("parcel.node", node, summary="Contract test stage")
        result = execute(
            ROOT / "graph",
            connector=SimpleNamespace(tool_registry=registry),
            skills=[],
            trace_dir=ROOT / "run" / "contracts" / name,
            max_node_workers=1,
        )
        assert result.error is None or result.exit_status == "failure", result.error
        assert result.success == (name in {"front_skip", "back_flip"})
        assert all(not remaining for remaining in route.values())
        if name == "front_skip":
            assert "grasp" not in seen and "flip" not in seen
        checks.append(name)
    result = {"passed": len(checks), "checks": checks, "kind": "synthetic_geometry_and_real_GaP_routing_only"}
    (ROOT / "run" / "contracts.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def cases(seed=20261110):
    rng = np.random.default_rng(seed)
    values = [
        {"name": "reference_back", "center": [0.08, 0.10], "yaw": 0.0, "face_up": False},
        {"name": "reference_front", "center": [0.08, 0.10], "yaw": 0.0, "face_up": True},
    ]
    for yaw in (-15, 15, -30, 30, -60, 60):
        for front in (False, True):
            values.append(
                {
                    "name": f"yaw_{yaw}_{'front' if front else 'back'}",
                    "center": [float(rng.uniform(0.04, 0.12)), float(rng.uniform(0.07, 0.16))],
                    "yaw": float(yaw),
                    "face_up": front,
                }
            )
    values += [
        {"name": "left_edge", "center": [-0.10, 0.08], "yaw": -10, "face_up": False},
        {"name": "workspace_reject", "center": [0.10, 0.28], "yaw": 0, "face_up": False},
    ]
    return values


def sweep(selected=None, filename="sweep.json", config=DEFAULT, page_sim=None):
    """Invoke through ctl eval, so builds and steps stay on the page's GPU."""
    from cell import make_scene
    from policy import run_graph

    chosen = cases() if selected is None else selected
    rows = []
    destination = ROOT / "run" / filename
    destination.parent.mkdir(exist_ok=True)
    manifest = {
        "config": asdict(config),
        "cases": chosen,
        "sha256": {
            str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in [*ROOT.glob("*.py"), ROOT / "graph/workflow.json"]
        },
    }
    destination.with_suffix(".source.json").write_text(json.dumps(manifest, indent=2) + "\n")
    for case in chosen:
        print(f"SWEEP {case['name']}", flush=True)
        scene = make_scene(center=case["center"], yaw=case["yaw"], face_up=case["face_up"], config=config)
        simulation = scene.build(control_mode="joint_position")
        try:
            trace_dir = destination.with_suffix("") / case["name"]
            generator = run_graph(simulation, config, trace_dir=trace_dir)
            batches = 0
            while True:
                try:
                    # Batch work between page redraws; occasionally hold the page's
                    # separate scene so live's idle watchdog remains satisfied.
                    for _ in range(24):
                        next(generator)
                except StopIteration as completed:
                    result = completed.value
                    break
                batches += 1
                if page_sim is not None and batches % 30 == 0:
                    page_sim.step()
                yield
            result["case"] = case
            if result.get("graph_error") and not result.get("failure"):
                result["failure"] = "graph_error"
            from PIL import Image

            image_file = destination.with_suffix("") / case["name"] / "final.png"
            try:
                Image.fromarray(simulation.render("top")["rgb"]).save(image_file)
                result["final_image"] = str(image_file.relative_to(ROOT))
            except Exception as error:
                result["image_error"] = repr(error)
        except Exception as error:
            result = {"case": case, "success": False, "infrastructure_error": repr(error), "events": []}
        finally:
            simulation.close()
        rows.append(result)
        destination.write_text(json.dumps(summarize(rows), indent=2) + "\n")
        print(f"SWEEP RESULT {case['name']} {result['success']} {result.get('failure')}", flush=True)
        del simulation, scene
    return summarize(rows)


def summarize(rows):
    stages = {}
    for stage in (
        "observe",
        "transfer",
        "verify_transfer",
        "grasp",
        "verify_lift",
        "flip",
        "verify_flip",
        "belt",
        "verify_belt",
    ):
        relevant = [row for row in rows if any(event["node"] == stage for event in row["events"])]
        succeeded = sum(
            any(
                event["node"] == stage and event["status"] not in {"retry", "abort", "continue"}
                for event in row["events"]
            )
            for row in relevant
        )
        stages[stage] = {"episodes_reached": len(relevant), "episodes_with_successful_stage": succeeded}
    return {
        "kind": "physical_particle_simulation",
        "episodes": len(rows),
        "successes": sum(row["success"] for row in rows),
        "stages": stages,
        "failures": dict(Counter(row.get("failure") or "infrastructure" for row in rows if not row["success"])),
        "rows": rows,
    }


if __name__ == "__main__":
    print(json.dumps(contract_checks(), indent=2))
