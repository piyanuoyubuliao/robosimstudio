# Model provenance

The two xArm7s use MuJoCo Menagerie's `ufactory_xarm7/xarm7.xml` and original
STL meshes at revision `98a70c569c4c1b6da467a20012f766ee1f8dc521`.
The source model, README, license and hashes remain in `assets/ufactory_xarm7/`.
The repository has no shipped xArm7 embodiment, so this twin imports it.

`prepare_assets.py` creates two uniquely named instances, rotates their base
axes horizontally outward, and mounts them side by side on one pillar.
Each shoulder has a vertical carriage; the shared column is scene geometry.
Original arm geometry, inertias, joint limits, gripper-pad colliders and
linkage equalities are retained. Vendor affine joint drives are converted
to equivalent position drives for the toolkit's joint-target interface.

| Joints | Position gain | Damping | Torque limit |
| --- | ---: | ---: | ---: |
| 1–2 | 1500 | 150 | ±50 Nm |
| 3–5 | 1000 | 100 | ±30 Nm |
| 6–7 | 800 | 80 | ±20 Nm |

The original gripper tendon is represented by two symmetric driver servos
(`kp=50`, `kv=5`, ±25 Nm each), with the original four-bar equalities and
pads. This controller adaptation is not an independent hardware calibration.
Gravity compensation is enabled. Intra-arm collision is disabled; cross-arm
and environment collision are enabled.

The two adjacent lift axes each have 500 mm travel, gain 20,000 N/m,
damping 2,000 Ns/m and ±1,500 N force limits. These are assumed values.
Visual mesh copies occupy the render group; unchanged original meshes
remain the colliders. No fingertip collider is added.

The printed label is cropped from the supplied video's 15-second frame.
The pale blue-grey packaging and creases are procedural. A two-face atlas
places the print on the slab's local +z face and blank packaging on -z.
The RTX viewer hook only supplies UVs to existing, simulated vertices.
The photo is retained as a reference; its pixels are not placed on geometry.
