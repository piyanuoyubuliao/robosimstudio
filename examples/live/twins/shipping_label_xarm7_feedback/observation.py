from dataclasses import asdict, dataclass
from typing import Protocol

import numpy as np
from config import DEFAULT


class ObservationError(ValueError):
    pass


def unit(vector):
    length = np.linalg.norm(vector)
    if not np.isfinite(length) or length < 1e-8:
        raise ObservationError("degenerate_geometry")
    return vector / length


@dataclass
class ParcelObservation:
    name: str
    timestamp: float
    center: np.ndarray
    corners: np.ndarray
    lower: np.ndarray
    upper: np.ndarray
    short_axis: np.ndarray
    long_axis: np.ndarray
    normal: np.ndarray
    yaw: float
    face_up: float
    residual: float
    strain: float
    normal_disagreement: float
    grasp: np.ndarray
    grasp_tangent: np.ndarray
    grasp_normal: np.ndarray
    grasp_thickness: float
    grasp_indices: np.ndarray
    belt_coverage: float
    surface_coverage: float
    confidence: float = 1.0

    def record(self):
        return {key: value.tolist() if isinstance(value, np.ndarray) else value for key, value in asdict(self).items()}

    def flat_failure(self, config=DEFAULT):
        if self.confidence < 0.9:
            return "uncertain_observation"
        if abs(self.face_up) < config.face_threshold:
            return "side_standing"
        if (
            self.residual > config.flat_residual
            or self.strain > config.max_strain
            or self.normal_disagreement > config.max_normal_disagreement
        ):
            return "severe_deformation"
        if self.surface_coverage < 0.99:
            return "outside_surface"
        return None


class Observer(Protocol):
    def observe(self) -> list[ParcelObservation]: ...


def estimate(points, timestamp=0.0, name="shipping_mailer", config=DEFAULT):
    points = np.asarray(points, dtype=float)
    columns, rows = config.cells
    count = (columns + 1) * (rows + 1)
    if points.shape != (2 * count, 3) or not np.isfinite(points).all():
        raise ObservationError("invalid_particles")
    mid = (points[:count] + points[count:]) / 2
    grid = mid.reshape(rows + 1, columns + 1, 3)
    pairs = points[count:] - points[:count]
    normal = unit(pairs.mean(axis=0))
    local_normals = pairs / np.maximum(np.linalg.norm(pairs, axis=1, keepdims=True), 1e-9)
    disagreement = float(np.mean(local_normals @ normal < 0.7))
    short_axis = unit((grid[:, -1] - grid[:, 0]).mean(axis=0))
    long_axis = unit((grid[-1] - grid[0]).mean(axis=0))
    yaw = float((np.arctan2(short_axis[1], short_axis[0]) + np.pi / 2) % np.pi - np.pi / 2)
    center = mid.mean(axis=0)
    _, _, basis = np.linalg.svd(mid - center, full_matrices=False)
    residual = float(np.sqrt(np.mean(((mid - center) @ basis[-1]) ** 2)))
    lengths = [
        np.linalg.norm(np.diff(grid, axis=1), axis=2) / (config.size[0] / columns),
        np.linalg.norm(np.diff(grid, axis=0), axis=2) / (config.size[1] / rows),
    ]
    strain = float(max(np.quantile(np.abs(length - 1), 0.95) for length in lengths))
    edge_column = 0 if grid[rows // 2, 0, 0] < grid[rows // 2, -1, 0] else columns
    edge_rows = np.arange(rows // 2 - 1, rows // 2 + 2)
    edge_indices = edge_rows * (columns + 1) + edge_column
    grasp = mid[edge_indices].mean(axis=0)
    tangent = unit(mid[edge_indices[-1]] - mid[edge_indices[0]])
    patch_normal = unit(pairs[edge_indices].mean(axis=0))
    corners = grid[[0, 0, rows, rows], [0, columns, columns, 0]]
    lateral = (points[:, 1] >= -0.25) & (points[:, 1] <= 0.55)
    belt = (points[:, 0] >= -1.60) & (points[:, 0] <= -0.40)
    surface = (points[:, 0] >= -1.60) & (points[:, 0] <= 0.65)
    return ParcelObservation(
        name,
        float(timestamp),
        center,
        corners,
        points.min(axis=0),
        points.max(axis=0),
        short_axis,
        long_axis,
        normal,
        yaw,
        float(normal[2]),
        residual,
        strain,
        disagreement,
        grasp,
        tangent,
        patch_normal,
        float(np.max(np.linalg.norm(pairs[edge_indices], axis=1))),
        edge_indices,
        float(np.mean(belt & lateral)),
        float(np.mean(surface & lateral)),
    )


class ParticleObserver:
    def __init__(self, sim, names=("shipping_mailer",), config=DEFAULT):
        self.sim, self.names, self.config = sim, names, config

    def observe(self):
        self.sim.sync()
        return [estimate(self.sim[name].particle_q, self.sim.time, name, self.config) for name in self.names]

    def track_patch(self, name, indices):
        points = np.asarray(self.sim[name].particle_q)
        count = len(points) // 2
        return ((points[:count] + points[count:]) / 2)[indices].mean(axis=0)


class CameraObserver:
    def __init__(self, estimate_frame, clock, max_age=0.25, track_patch=None):
        self.estimate_frame, self.clock, self.max_age = estimate_frame, clock, max_age
        self.patch_estimator = track_patch

    def observe(self):
        observations = self.estimate_frame()
        for observation in observations:
            age = self.clock() - observation.timestamp
            if age < 0 or age > self.max_age or observation.confidence < 0.9:
                raise ObservationError("stale_or_uncertain_camera")
        return observations

    def track_patch(self, name, indices):
        if self.patch_estimator is None:
            raise ObservationError("camera_patch_tracking_required")
        point, timestamp, confidence = self.patch_estimator(name, indices)
        age = self.clock() - timestamp
        point = np.asarray(point, dtype=float)
        if age < 0 or age > self.max_age or confidence < 0.9 or point.shape != (3,) or not np.isfinite(point).all():
            raise ObservationError("stale_or_uncertain_patch")
        return point


def top_first(observations):
    blocked = set()
    for lower in observations:
        for upper in observations:
            overlap = np.minimum(lower.upper[:2], upper.upper[:2]) - np.maximum(lower.lower[:2], upper.lower[:2])
            if lower.name != upper.name and np.all(overlap > 0.005) and upper.center[2] > lower.center[2] + 0.01:
                blocked.add(lower.name)
    return sorted(
        (item for item in observations if item.name not in blocked), key=lambda item: (-item.upper[2], item.name)
    )
