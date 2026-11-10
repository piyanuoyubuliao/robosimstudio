import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
command = [sys.executable, "-m", "robosimstudio_real2sim_agent", "--run-dir", str(ROOT / "run")]
subprocess.run(command + ["up", "--renderer", "rtx"], cwd=ROOT, check=True)
subprocess.run(command + ["pause"], cwd=ROOT, check=True)
subprocess.run(command + ["eval", "from live_controls import install; install(session); sim.reset(); session.on_reset({}, {})"], cwd=ROOT, check=True)
print("Open the printed URL. Run routine for the right-arm transfer, left-arm flip, and conveyor placement; Ready resets the cell.")
