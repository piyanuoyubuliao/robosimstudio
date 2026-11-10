"""Archive completed physical episodes and report whole-stage outcomes."""

import argparse
import hashlib
import json
from collections import Counter
from pathlib import Path
from shutil import copy2

ROOT = Path(__file__).resolve().parent


def report(files):
    destination = ROOT / "evidence"
    destination.mkdir(exist_ok=True)
    rows, sources = [], []
    seen = set()
    for filename in files:
        source = ROOT / filename
        document = json.loads(source.read_text())
        manifest_file = source.with_suffix(".source.json")
        manifest = json.loads(manifest_file.read_text())
        changed = [
            name
            for name, digest in manifest["sha256"].items()
            if hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != digest
        ]
        if changed:
            raise ValueError(f"Source changed since {filename}: {changed}")
        copy2(source, destination / source.name)
        copy2(manifest_file, destination / manifest_file.name)
        sources.append(str(source.relative_to(ROOT)))
        for row in document["rows"]:
            name = row["case"]["name"]
            if name in seen:
                raise ValueError(f"Duplicate episode: {name}")
            seen.add(name)
            rows.append(row)
            target = destination / name
            target.mkdir(exist_ok=True)
            trace = ROOT / row["trace_dir"]
            for artifact in ("final.png", "dag_trace.json", "workflow.json"):
                if (trace / artifact).exists():
                    copy2(trace / artifact, target / artifact)

    def reached(row, node):
        return any(event["node"] == node for event in row["events"])

    def completed(row, node, statuses):
        return any(event["node"] == node and event["status"] in statuses for event in row["events"])

    stages = {}
    for label, entry, check, statuses in (
        ("right_handoff", "transfer", "observe", {"grasp", "belt"}),
        ("grasp_and_lift", "grasp", "verify_lift", {"flip"}),
        ("flip_and_face", "flip", "verify_flip", {"belt"}),
        ("push_and_stable_belt", "belt", "verify_belt", {"success"}),
    ):
        relevant = [row for row in rows if reached(row, entry)]
        successes = sum(completed(row, check, statuses) for row in relevant)
        stages[label] = {"entered": len(relevant), "completed": successes}
    result = {
        "kind": "completed_physical_episodes",
        "sources": sources,
        "source_hashes_verified": True,
        "episodes": len(rows),
        "successes": sum(row["success"] for row in rows),
        "stages": stages,
        "failures": dict(Counter(row.get("failure", "infrastructure") for row in rows if not row["success"])),
        "cases": [
            {
                **row["case"],
                "success": row["success"],
                "failure": row.get("failure"),
                "failure_stage": row.get("failure_stage"),
                "retries": row.get("retries", {}),
                "sim_seconds": row["events"][-1]["end"],
                "face_up_final": row.get("final", {}).get("face_up"),
                "belt_coverage_final": row.get("final", {}).get("belt_coverage"),
            }
            for row in rows
        ],
    }
    (destination / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
    for name in ("contracts.json", "semantic_checks.json"):
        copy2(ROOT / "run" / name, destination / name)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", nargs="+", help="Completed sweep JSON files relative to this twin")
    print(json.dumps(report(parser.parse_args().files), indent=2))
