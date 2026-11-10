import json
from dataclasses import replace
from pathlib import Path

from conveyor import start_conveyor, stop_conveyor
from label_appearance import MAILER_CELLS, install
from live_controls import install as install_controls
from live_controls import ready
from motion import routine
from robot_setup import robot_spec

import robosimstudio as rss
from robosimstudio.core.render.finish import Finish
from robosimstudio.core.render.room import LAB

install()


ROOT = Path(__file__).resolve().parent
TABLE_Z = 0.78
MAILER_SIZE = (0.24, 0.32)
MAILER_THICKNESS = 0.030
MOTION = json.loads((ROOT / "assets" / "motion.json").read_text())
physics = replace(
    rss.GENERIC,
    coupling=rss.coupler("one_way", contact_forces=True),
    collision=rss.CollisionSpec(full_surface_contact=True, force_sdf=True, rigid_gap=0.0005, rigid_contact_max=8192),
).with_backend(
    "cloth",
    rss.backend(
        "vbd",
        iterations=40,
        friction_epsilon=0.001,
        rigid_body_contact_buffer_size=256,
        rigid_soft_contact_use_log_barrier=True,
        particle_rest_shape_contact_exclusion_radius=1.5 * MAILER_THICKNESS,
    ),
)
scene = rss.Scene(
    cell="void",
    name="Right transfer push · left corner flip · shared-pillar xArm7",
    physics=physics,
    render=replace(LAB, scenery="", key_intensity=0.45, ambient=(0.8, 0.8, 0.8)),
)
scene.add(rss.robot(robot_spec(home=MOTION["home"], grip=MOTION.get("home_grip", {"left": 1, "right": 0}))))
metal = Finish(roughness=0.38, metallic=0.7)
steel = (0.57, 0.59, 0.60)


def fixed(name, half, at, color=steel, finish=metal, **kwargs):
    if "quat" in kwargs:
        at = (*at, *kwargs.pop("quat"))
    scene.add(rss.slab(name=name, half=half, at=at, color=color, finish=finish, **kwargs))


# The desk left edge stays at the original working line while its right edge follows the shifted ramp.
fixed("operation_desktop", (0.525, 0.4, 0.025), (0.125, 0.15, TABLE_Z - 0.025), friction=0.50)
fixed(
    "conveyor_belt",
    (0.6, 0.4, 0.018),
    (-1.0, 0.15, TABLE_Z - 0.018),
    color=(0.025, 0.22, 0.14),
    finish=Finish(roughness=0.8),
    friction=0.65,
    entity="shipping_receiving_conveyor",
)
for y in (-0.28, 0.58):
    fixed(f"conveyor_side_{y}", (0.63, 0.025, 0.055), (-1, y, 0.705))
for x in (-1.55, -0.45):
    fixed(f"conveyor_roller_{x}", (0.025, 0.4, 0.025), (x, 0.15, 0.72), color=(0.17, 0.18, 0.19))
for x in (-1.45, -0.50, -0.33, 0.33):
    for y in (-0.18, 0.48):
        fixed(f"workstation_leg_{x}_{y}", (0.024, 0.024, 0.36), (x, y, 0.36))
RAMP_X = 0.975
fixed("right_feed_ramp", (0.394, 0.4, 0.012), (RAMP_X, 0.15, 0.890), quat=(0, -0.1564345, 0, 0.9876883), friction=0.35)
for y in (-0.255, 0.555):
    fixed(f"ramp_guard_{y}", (0.394, 0.012, 0.07), (RAMP_X, y, 0.95), quat=(0, -0.1564345, 0, 0.9876883))
fixed("ramp_backstop", (0.015, 0.4, 0.18), (RAMP_X + 0.375, 0.15, 1.16))
for y in (-0.18, 0.48):
    fixed(f"ramp_leg_{y}", (0.025, 0.025, 0.49), (RAMP_X + 0.325, y, 0.49))

fixed("shared_pillar_foot", (0.36, 0.29, 0.04), (0.15, -0.76, 0.04), color=(0.15, 0.16, 0.17))
fixed("shared_pillar", (0.14, 0.115, 0.91), (0.15, -0.76, 0.99), color=(0.045, 0.05, 0.055))
for x in (-0.095, 0.095):
    fixed(f"shared_linear_rail_{x}", (0.012, 0.015, 0.81), (0.15 + x, -0.625, 0.99))
for index in range(45):
    fixed(
        f"pillar_bellows_{index:02d}",
        (0.146, 0.014, 0.009),
        (0.15, -0.63, 0.16 + index * 0.038),
        color=(0.07, 0.075, 0.08),
        finish=Finish(roughness=0.7),
    )
fixed("column_top_motor", (0.11, 0.10, 0.065), (0.15, -0.76, 1.965), color=(0.13, 0.14, 0.15))
fixed("control_box", (0.13, 0.075, 0.065), (0.31, -0.41, 0.63), color=(0.78, 0.80, 0.81))
scene.add(
    rss.post(name="emergency_stop", radius=0.025, half_height=0.012, at=(0.31, -0.41, 0.707), color=(0.7, 0.025, 0.02))
)
fixed("floor", (3, 3, 0.02), (0, 0, -0.025), color=(0.28, 0.29, 0.30), finish=Finish(roughness=0.95))

material = rss.ClothMaterialSpec(
    shell="slab",
    thickness=MAILER_THICKNESS,
    areal_density=2.0,
    tri_ke=10000,
    tri_ka=10000,
    tri_kd=0.02,
    edge_ke=0.002,
    edge_kd=0.001,
    core_shear_modulus=12000,
    core_lame_lambda=60000,
    core_damping=0.01,
    particle_radius=0.001,
    friction=0.65,
)
scene.add(
    rss.sheet(
        name="shipping_mailer",
        size=MAILER_SIZE,
        cells=MAILER_CELLS,
        at=(-0.04, 0.26, 0.803),
        quat=(1, 0, 0, 0),
        material=material,
        settle_frames=180,
        settle_substeps=16,
        color=(1, 1, 1),
        finish=Finish(roughness=0.50, texture=str(ROOT / "assets" / "mailer_faces.png"), uv_scale=(1, 1)),
    )
)
scene.add(
    rss.CameraSpec(name="overview", width=1280, height=850, eye=(-2.6, 2.7, 2.7), look_at=(-0.12, 0, 0.90), fov_y=46)
)
scene.add(
    rss.CameraSpec(name="robot_view", width=1200, height=720, eye=(0, -2.5, 2.4), look_at=(-0.15, 0.15, 0.78), fov_y=55)
)
scene.add(
    rss.CameraSpec(
        name="top", width=1200, height=760, eye=(-0.25, 0.15, 3.4), look_at=(-0.25, 0.15, 0.78), up=(0, 1, 0), fov_y=63
    )
)
scene.add(
    rss.CameraSpec(
        name="photo",
        width=1080,
        height=1404,
        fx=1536.35,
        fy=1536.35,
        cx=540,
        cy=702,
        eye=(-2.0315, 0.3381, 2.3927),
        look_at=(-1.1865, 0.2233, 1.8704),
        up=(0.5324, 0.0889, 0.8418),
    )
)
scene.add(
    rss.CameraSpec(
        name="manipulation", width=1280, height=850, eye=(-1.15, 1.0, 1.75), look_at=(-0.08, 0.0, 1.0), fov_y=55
    )
)
routines = {"Ready": ready, "routine": routine, "Conveyor left": start_conveyor, "Conveyor stop": stop_conveyor}
