from dataclasses import dataclass


@dataclass(frozen=True)
class Config:
    size: tuple = (0.24, 0.32)
    cells: tuple = (12, 16)
    thickness: float = 0.030
    table_z: float = 0.78
    max_retries: int = 2
    max_transfer_strokes: int = 12
    max_belt_strokes: int = 24
    max_nodes: int = 160
    push_approach_z: float = 1.00
    right_push_tilts: tuple = (0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0)
    push_step: float = 0.045
    minimum_push: float = 0.003
    left_center_x: tuple = (-0.12, 0.015)
    left_center_y: tuple = (0.04, 0.19)
    input_center_x: tuple = (-0.12, 0.18)
    input_center_y: tuple = (0.01, 0.25)
    face_threshold: float = 0.80
    flat_residual: float = 0.018
    max_strain: float = 0.22
    max_normal_disagreement: float = 0.15
    grasp_inset: float = 0.025
    grasp_height_offset: float = 0.0
    pinch: float = 0.95
    lift_minimum: float = 0.045
    slip_distance: float = 0.035
    flip_clearance: float = 0.050
    collision_margin: float = 0.002
    max_joint_speed: float = 0.35
    max_tcp_speed: float = 0.06
    max_angular_speed: float = 0.30
    belt_coverage: float = 0.98
    stable_seconds: float = 0.6
    stable_speed: float = 0.015

    def __post_init__(self):
        if self.max_retries < 0 or self.max_nodes < 1 or self.push_step <= 0:
            raise ValueError("Invalid policy budget")
        if not self.right_push_tilts or any(not 0 <= angle <= 45 for angle in self.right_push_tilts):
            raise ValueError("Right push tilts must be between 0 and 45 degrees")
        if tuple(sorted(set(self.right_push_tilts))) != self.right_push_tilts:
            raise ValueError("Right push tilts must increase without duplicates")


DEFAULT = Config()
