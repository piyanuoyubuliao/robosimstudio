"""A driven receiving belt with contact surface velocity and circulating paint marks."""

from dataclasses import replace

import numpy as np
import warp as wp

from robosimstudio.domains.static.entity import EntityStatic
from robosimstudio.domains.static.welded import weld_to_world
from robosimstudio.scene.physics.couplers import SolverOneWay
from robosimstudio.scene.registry import register_entity

BELT_SPEED = -0.08
MARK_COUNT = 12


@wp.kernel
def drive_contacts(
    count: wp.array[int],
    shapes: wp.array[int],
    normals: wp.array[wp.vec3],
    belt_shape: int,
    speed: wp.array[float],
    velocities: wp.array[wp.vec3],
):
    contact = wp.tid()
    if contact < count[0] and shapes[contact] == belt_shape:
        if normals[contact][2] > 0.5:
            velocities[contact] = wp.vec3(speed[0], 0.0, 0.0)


@wp.kernel
def advance_belt(offset: wp.array[float], speed: wp.array[float], length: float, dt: float):
    distance = offset[0] + speed[0] * dt
    offset[0] = distance - wp.floor(distance / length) * length


@wp.kernel
def draw_marks(
    bodies: wp.array[int],
    offset: wp.array[float],
    center: wp.vec3,
    half: wp.vec3,
    poses: wp.array[wp.transform],
):
    mark = wp.tid()
    length = 2.0 * half[0]
    distance = (float(mark) + 0.5) * length / float(bodies.shape[0]) + offset[0]
    distance = distance - wp.floor(distance / length) * length
    position = wp.vec3(center[0] - half[0] + distance, center[1], center[2] + half[2] + 0.0001)
    poses[bodies[mark]] = wp.transform(position, wp.quat_identity())


class BeltCollision:
    def __init__(self, collision, belt):
        self.collision = collision
        self.belt = belt

    def __getattr__(self, name):
        return getattr(self.collision, name)

    def collide(self, state, contacts):
        self.collision.collide(state, contacts)
        wp.launch(
            drive_contacts,
            dim=contacts.soft_contact_max,
            inputs=[
                contacts.soft_contact_count,
                contacts.soft_contact_shape,
                contacts.soft_contact_normal,
                self.belt.belt_shape,
                self.belt.speed,
            ],
            outputs=[contacts.soft_contact_body_vel],
            device=self.belt.model.device,
        )


@register_entity("shipping_receiving_conveyor")
class EntityConveyor(EntityStatic):
    world_aware = False

    def build(self, builder, episode):
        super().build(builder, episode)
        self.belt_shape = self.range.shapes[0]
        self.center = tuple(self._placed(self.xform_now)[:3])
        self.half = self.spec.shape.half_extents
        self.mark_bodies = []
        self.mark_shapes = []
        config = builder.ShapeConfig(has_shape_collision=False, has_particle_collision=False, density=0.0)
        for index in range(MARK_COUNT):
            position = (
                self.center[0] - self.half[0] + (index + 0.5) * 2.0 * self.half[0] / MARK_COUNT,
                self.center[1],
                self.center[2] + self.half[2] + 0.0001,
            )
            body, _ = weld_to_world(
                builder, wp.transform(position, wp.quat_identity()), label=f"{self.name}_mark_{index:02d}"
            )
            self.mark_bodies.append(body)
            self.mark_shapes.append(
                builder.add_shape_box(
                    body=body,
                    hx=0.002,
                    hy=self.half[1] - 0.015,
                    hz=0.0001,
                    cfg=config,
                    label=f"{self.name}_paint_{index:02d}",
                )
            )

    def apply_display(self, builder):
        super().apply_display(builder)
        for shape in self.mark_shapes:
            builder.shape_color[shape] = (0.07, 0.36, 0.23)

    def handle(self, model):
        return replace(super().handle(model), body=-1)

    def bind(self, model):
        super().bind(model)
        self.speed = wp.array([BELT_SPEED], dtype=float, device=model.device)
        self.offset = wp.zeros(1, dtype=float, device=model.device)
        self.marks = wp.array(self.mark_bodies, dtype=int, device=model.device)

    def contribute_hooks(self, hooks):
        hooks.add("post_solve", self.advance, name="conveyor:circulate")

    def advance(self, state, dt):
        wp.launch(
            advance_belt,
            dim=1,
            inputs=[self.offset, self.speed, 2.0 * self.half[0], dt],
            device=self.model.device,
        )
        self.draw(state)

    def draw(self, state):
        wp.launch(
            draw_marks,
            dim=MARK_COUNT,
            inputs=[self.marks, self.offset, wp.vec3(*self.center), wp.vec3(*self.half)],
            outputs=[state.body_q],
            device=self.model.device,
        )

    def on_scene_ready(self, pipeline):
        if not isinstance(pipeline.solver, SolverOneWay):
            raise TypeError("The receiving belt requires this twin's one-way rigid/cloth coupling")
        pipeline.solver.collision = BeltCollision(pipeline.solver.collision, self)

    def reset(self, pipeline, episode, *, world=0):
        self.offset.zero_()
        for state in (pipeline.state_0, pipeline.state_1):
            self.draw(state)

    def set_speed(self, speed):
        if not np.isfinite(speed):
            raise ValueError("Belt speed must be finite")
        self.speed.fill_(float(speed))


def start_conveyor(sim, session):
    sim["conveyor_belt"].entity.set_speed(BELT_SPEED)
    session.set_paused(False)
    yield


def stop_conveyor(sim, session):
    sim["conveyor_belt"].entity.set_speed(0.0)
    yield
