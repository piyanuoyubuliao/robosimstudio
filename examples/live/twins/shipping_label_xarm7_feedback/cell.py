import json
from dataclasses import replace
from pathlib import Path

import numpy as np
from config import DEFAULT
from label_appearance import install
from robot_setup import robot_spec
from scipy.spatial.transform import Rotation

import robosimstudio as rss
from robosimstudio.core.render.finish import Finish
from robosimstudio.core.render.room import LAB

ROOT = Path(__file__).resolve().parent


def make_scene(center=(0.08, 0.10), yaw=0.0, face_up=False, config=DEFAULT):
    install()
    motion = json.loads((ROOT / "assets" / "motion.json").read_text())
    physics = replace(
        rss.GENERIC,
        coupling=rss.coupler("one_way", contact_forces=True),
        collision=rss.CollisionSpec(
            full_surface_contact=True, force_sdf=True, rigid_gap=0.0005, rigid_contact_max=8192
        ),
    ).with_backend(
        "cloth",
        rss.backend(
            "vbd",
            iterations=40,
            friction_epsilon=0.001,
            rigid_body_contact_buffer_size=256,
            rigid_soft_contact_use_log_barrier=True,
            particle_rest_shape_contact_exclusion_radius=1.5 * config.thickness,
        ),
    )
    scene = rss.Scene(
        cell="void",
        name="xArm7 parcel feedback policy · single flat parcel",
        physics=physics,
        render=replace(LAB, scenery="", key_intensity=0.45, ambient=(0.8, 0.8, 0.8)),
    )
    scene.add(rss.robot(robot_spec(home=motion["home"], grip=motion["home_grip"])))
    scene.add(
        rss.slab(
            name="operation_desktop", half=(0.525, 0.4, 0.025), at=(0.125, 0.15, config.table_z - 0.025), friction=0.50
        )
    )
    scene.add(
        rss.slab(
            name="conveyor_belt",
            half=(0.6, 0.4, 0.018),
            at=(-1.0, 0.15, config.table_z - 0.018),
            friction=0.65,
            color=(0.025, 0.22, 0.14),
        )
    )
    scene.add(
        rss.slab(name="shared_pillar", half=(0.14, 0.115, 0.91), at=(0.15, -0.76, 0.99), color=(0.045, 0.05, 0.055))
    )
    material = rss.ClothMaterialSpec(
        shell="slab",
        thickness=config.thickness,
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
    rotation = Rotation.from_euler("z", yaw, degrees=True) * Rotation.from_euler(
        "x", 0 if face_up else 180, degrees=True
    )
    origin = np.array([*center, config.table_z + config.thickness / 2 + 0.008])
    origin -= rotation.apply([config.size[0] / 2, config.size[1] / 2, 0])
    scene.add(
        rss.sheet(
            name="shipping_mailer",
            size=config.size,
            cells=config.cells,
            at=tuple(origin),
            quat=tuple(rotation.as_quat()),
            material=material,
            settle_frames=180,
            settle_substeps=16,
            color=(1, 1, 1),
            finish=Finish(roughness=0.50, texture=str(ROOT / "assets" / "mailer_faces.png")),
        )
    )
    scene.add(
        rss.CameraSpec(name="overview", width=960, height=640, eye=(-2.6, 2.7, 2.7), look_at=(-0.12, 0, 0.90), fov_y=46)
    )
    scene.add(
        rss.CameraSpec(
            name="top",
            width=960,
            height=640,
            eye=(-0.25, 0.15, 3.4),
            look_at=(-0.25, 0.15, 0.78),
            up=(0, 1, 0),
            fov_y=63,
        )
    )
    return scene
