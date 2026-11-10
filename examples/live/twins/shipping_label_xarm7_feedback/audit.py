"""Run the unchanged policy in batches between live redraws for physics audits."""

import json

from policy import ROOT, run_graph


def contact_audit(sim):
    generator = run_graph(sim)
    try:
        while True:
            for _ in range(24):
                try:
                    next(generator)
                except StopIteration as result:
                    (ROOT / "run" / "contact_audit_result.json").write_text(json.dumps(result.value, indent=2) + "\n")
                    if not result.value["success"]:
                        raise RuntimeError(f"Policy did not complete: {result.value.get('failure')}") from None
                    return result.value
            yield
    finally:
        generator.close()
