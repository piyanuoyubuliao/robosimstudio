from pathlib import Path

from cell import make_scene
from policy import routine

ROOT = Path(__file__).resolve().parent
scene = make_scene()
routines = {"Feedback sorting": routine}
