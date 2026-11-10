# Right-push / left-flip shipping-label variant

This twin uses the shared-pillar dual xArm7 cell from `shipping_label_xarm7`.
The right arm pushes the parcel toward the left hand; the left hand lifts,
flips and lowers it, then pushes it onto the stationary conveyor. The parcel
starts on the desktop. Upstream feeding is not automated.

## Final motion and result

The left flip follows the six achieved poses in
`assets/teaching/20261110T035457_916925Z.json`. Its SHA-256 is
`87e61334863821ebb9d732b0f0e0bd215f4fca7a54f4be908bb54445bf61b903`.
`assets/motion.json` records that source identity. A later teaching does not
silently replace the generated plan. Teaching controls and the user's
`assets/teaching/latest.json` remain available as described in `README.md`.

`FLIP_CLEARANCE = 0.050` adds the requested 50 mm lift after the taught rise
and raises the rotation endpoint by the same amount. A separate descent
returns 50 mm to the original table-contact target before holding, dragging,
opening and departing. The taught wrist orientations are retained.
`PINCH_CLOSURE = 0.95` scales the taught fully closed gripper command as part
of the contact correction below.

The plan contains 39 phases totaling 70.83 simulated seconds. Smoothstep
joint interpolation is retimed against joint, TCP translation and rotation
limits. `run/taught_plan_validation.json` records passing endpoint, joint
limit and speed checks. Maximum planned rates are 0.240 rad/s per joint,
0.0600 m/s translation and 0.300 rad/s rotation. The raised endpoints agree
with model FK within 0.001 mm and 0.0002 degrees; these are planning errors,
not measured physical tracking accuracy.

The final independent replay completes the flip and conveyor placement,
label-up with 100% conveyor coverage and center z = 0.802497 m. Its retained
artifacts are:

- `run/contact_fixed_full_process_video.mp4`: the complete process from
  reset, 2,126 H.264 frames, 1280 x 850 at 30 fps, 70.8667 seconds.
- `run/contact_fixed_lift_rotate_closeup_slow.mp4`: seconds 33-52 of that
  take, cropped around the lift and rotation and played at half speed.
- `run/contact_fixed_full_process_video.json`: recording metadata.
- `run/contact_fixed_full_process_result.json`: phase readings and final
  success state.
- `run/contact_fixed_full_process_summary.json`: phase contact and
  deformation statistics, including actual pad gaps.
- `run/contact_fixed_full_process_geometry_check.json`: triangle-box
  intersection results at 0.05 mm tolerance.
- `run/contact_fixed_full_process_physics.json`: the final written replay's
  physics checks.

## Contact failure and correction

The earlier 37-39 second lift failure was accompanied by actual pad
intersections, rather than being established as insufficient friction.
The baseline check measured inward depth up to 1.995 mm in a 4 mm pad.
The original finite quadratic contact penalty allowed excessive penetration.
The one-way MuJoCo/VBD coupling also does not return deformable reactions
to the rotary gripper, so the fully closed target kept reducing the actual
pad gap from about 7.7 mm to about 2 mm during lift.

The correction enables `rigid_soft_contact_use_log_barrier=True`, increases
VBD iterations from 20 to 40, and sets the rest-neighbor exclusion radius
to 1.5 times the slab thickness (45 mm). The 0.95 pinch command retains
about 6.0-6.9 mm actual pad gap during raised hold, rotation and descent.
Finger colliders, robot gains and torque limits are unchanged. Parcel
friction remains 0.65; its 240 x 320 x 30 mm dimensions, mass, elastic
parameters and mesh are unchanged. No attachment, weld, particle-position
control or other scripted parcel motion is used.

Two corrected full replays completed the task. The final replay's retained
geometry check finds no pad intersections above 0.05 mm after 33.60 seconds,
including the previous 37-39 second failure window, rotation and descent.
At 32.65-33.60 seconds, one triangle still has inward residual up to
0.3643 mm. Local internal tetrahedron inversions also remain during the
pinch. The observed lift/rotation failure is corrected in these replays;
globally intersection-free and volume-preserving simulation is not proven.

## Physics evidence and limitations

The final independent replay rebuilt the saved scene and ran the routine
through an observational wrapper with `./ctl physics verify_full --only written`.
The wrapper observed geometry and contact forces without controlling the
parcel or changing the motion. The retained report reads:

```text
PASS  contact  nothing ties a thing to the robot, no motor on a thing's joints, nothing welded
NOTE  robot    the local dual_xarm7.xml servo gains and limits are not checked
PASS  model    gravity 9.81 m/s²; the handled things' densities are ones objects have
PASS  written  finished; moved shipping_mailer
PASS  grip     no finger force on a rigid thing (a deformable's is not read)
```

Only the written replay was run for the final correction. Frozen, open,
as-built and heavier-object counterfactuals were not rerun. The generic grip
line does not measure deformable grasp forces. The soft-side contact reaction
is not an actual actuator force under the one-way coupling.

The local vendor xArm7 model and controller adaptation are documented in
`ASSETS.md`; their gains and lift-axis limits are assumptions, not hardware
calibrations. The cloth-slab material and scene dimensions remain approximate.
The retained videos use GL rendering. The room, lighting and cameras were
not newly calibrated against the reference, and no path-traced appearance
match, statistical repeatability or real-hardware force margin is claimed.

## Parcel surface texture

The existing `assets/mailer_faces.png` atlas now appears on the browser's
deforming parcel mesh and in GL snapshots and recordings, as well as RTX.
The printed atlas half is assigned to the 384 triangles of the front face;
the 384 back-face triangles and 112 rim triangles use the unprinted half.
The browser draws the parcel's actual slab surface without cloth pile.
The UV coordinates are fixed to particle indices, so the texture follows
deformation and does not change faces when the parcel flips. Ready retains
the original label-down placement.

`run/mailer_front_gl.png` and `run/mailer_back_gl.png` show the two faces
of the reset parcel with other geometry hidden only in a separate preview
renderer. The front shows the shipping label; the back shows the plastic
grain, creases and border. `run/mailer_surface.glb` is the textured surface
export used by the browser. `run/texture_validation.json` records the face
counts, all 2,640 GL texture coordinates, successful GLB encoding, and exact
equality of simulation particle positions before and after verification.
No browser client was connected during verification, so this checks the
browser mesh export rather than a browser screenshot.

`run/mailer_scene_gl.png` and `run/mailer_scene_rtx.png` show the complete
reset scene through the unchanged manipulation camera. Both display the
textured unprinted face on top, consistent with the label-down start.
The live scene rebuilt successfully and is left paused. Ruff lint and
format checks pass for `label_appearance.py` and the verification script;
`scene.py` still has its pre-existing unused `install_controls` import.

The surface and label reuse the existing assets. The plastic creases are
procedural colour detail rather than measured relief, and the label remains
the low-resolution crop from the supplied reference. No new dimensions,
material parameters, camera calibration or motion changes were introduced.
The previous physics evidence and limitations still apply; the full motion
and physics counterfactuals were not rerun for this appearance change.

## Retained files

The scene, motion generation, teaching controls, robot model and provenance,
textures, reference stills, and user teaching are retained. Intermediate
videos, screenshots, raw geometry/segmentation dumps, trial diagnostics,
scratch validation scripts, duplicate results, old backups and caches were
removed during cleanup. The five compact validation JSON files and final
recording metadata listed above remain; the original baseline measurements
are summarized here, but their raw artifacts are no longer retained.
The live server's log and control files remain in `run/`.
