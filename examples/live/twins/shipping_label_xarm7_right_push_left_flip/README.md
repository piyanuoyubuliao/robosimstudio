# Shared-pillar xArm7 parcel cell — right push / left flip

The robot faces +y. Its left (-x) is the green receiving conveyor; the
stainless desktop is in the middle; its right (+x) is the inclined feed ramp.
Two horizontally mounted xArm7 shoulders share one vertical pillar, with
adjacent lift carriages. The compliant parcel is 240 × 320 × 30 mm.

This is a separate variant of `shipping_label_xarm7`; that original twin is
left unchanged.  The geometry, robot asset, material, cameras, and references
are copied from it.  Only the parcel workflow is reworked:

1. the right arm uses the original straight pusher cadence to transfer the
   parcel toward the left arm;
2. the left arm follows the six taught poses with 50 mm extra lift and
   rotation clearance, then lowers 50 mm to the taught table-contact pose
   before dragging, opening, and departing;
3. the left arm uses the same pusher cadence to place the flipped parcel on the
   conveyor.

From this directory:

```bash
./launch.py
```

Open the URL printed by `launch.py`. **Ready** resets the scene with a
label-down parcel on the desktop. **routine** runs the right transfer, left
flip, and conveyor placement. See `REPORT.md` for the measured outcome,
assumptions, and remaining limitations.

Useful commands from this folder:

```bash
./ctl play Ready --wait
./ctl play routine --record shipping_label_corrected --camera overview --wait
./ctl snapshot --camera robot_view
```

`scene.py` defines geometry and material. `prepare_assets.py` assembles the
vendor arms and parcel texture atlas; `plan_motion.py` generates reachable
joint waypoints, using `taught_motion.py` for the demonstrated flip.
`motion.py` controls robot joints and measures the two
physical parcel faces. `label_appearance.py` maps `assets/mailer_faces.png`
onto the existing deformable mesh in the browser, GL recordings and RTX.
The front carries the reference shipping label; the back and rim carry only
the plastic surface texture. The mapping follows the particles through the
flip, without adding a separate label mesh or changing collision or
simulation state. Ready still starts label-down, so the label becomes visible
after the left arm flips the parcel. `ref/` retains the supplied motion
reference stills.

The routine has 39 phases and lasts approximately 71 simulated seconds.
The flip uses the achieved joint positions and TCP orientations from
`assets/teaching/20261110T035457_916925Z.json`, the completed six-point teaching
provided as `latest.json`. The source filename and SHA-256 are embedded in
`assets/motion.json`; a later teaching does not silently replace this plan.
`FLIP_CLEARANCE = 0.050` in `taught_motion.py` adds 50 mm in world z after the
original taught lift and raises the rotation target by the same amount,
while preserving the taught orientations. Rotation and
the final 50 mm descent are separate phases; the descent returns to the
original table-contact target before the contact hold and drag.

The taught joint paths use smoothstep interpolation and are retimed to at most
0.35 rad/s per joint, 0.06 m/s TCP translation, and 0.30 rad/s TCP rotation.
Open approach, closing, contact holds, release and settling are separate
phases. Capture timestamp gaps are not used as movement durations. The
original right transfer remains; the final left push tracks the new parcel
position and uses a straight stroke to a reachable conveyor placement.

Regenerate the plan with `python plan_motion.py` in this directory. Reset or
click Ready before Run routine. The six poses describe a motion for this
scene's reset parcel placement, not an arbitrary parcel position.

The contact correction enables VBD's rigid-soft log barrier, uses 40 solver
iterations and the slab's 45 mm rest-neighbor exclusion, and limits the taught
pinch command to `PINCH_CLOSURE = 0.95`. The original fully closed target kept
closing during lift because the one-way coupler cannot stall rotary fingers
against the parcel. The corrected pinch retains about 6 mm between the actual
pad faces during rotation. Parcel material, friction, robot gains, finger
colliders, and the 50 mm clearance trajectory are unchanged.

Two corrected replays complete the flip and conveyor placement.
`run/contact_fixed_full_process_video.mp4` records the full 70.87-second
process from reset, and `run/contact_fixed_lift_rotate_closeup_slow.mp4`
shows the lift and rotation at half speed.
`REPORT.md` distinguishes the successful outcome from remaining early grasp
contact residuals and local tetrahedron inversions.

Intermediate recordings, trial diagnostics, scratch scripts, screenshots,
old backups and Python caches have been removed. `run/` retains the two final
videos, the full video's recording metadata, five final validation JSON files,
and the live server's log and control files. The validation files are
`contact_fixed_full_process_result.json`,
`contact_fixed_full_process_physics.json`,
`contact_fixed_full_process_summary.json`,
`contact_fixed_full_process_geometry_check.json`, and
`taught_plan_validation.json`. Running the routine or recording again creates
new outputs in this directory.

All scene helpers, robot assets, textures, reference stills and the original
teaching are retained. The existing textures are ready to use; regenerating
them with `prepare_assets.py` requires the original
`ref/first_cycle/t015.00.jpg`, which was already absent from this folder.

The receiving belt runs left (world -x) at 0.08 m/s whenever simulation is
stepping. Its circulating green stripes show the surface direction. The
**Conveyor left** button starts the belt and resumes simulation;
**Conveyor stop** stops the belt without pausing the robot. **Ready** still
resets and pauses the whole scene. `BELT_SPEED` in `conveyor.py` sets the
signed surface speed. The belt carries the deformable parcel through contact
friction. The task starts with one parcel already at the work area; upstream
feeding is not automated.

## Teaching replacement motion

Open **Live → 轨迹示教** on the page started by `launch.py`:

1. Run the routine, then click **停止并接管** at the point to correct. This
   terminates the routine without resetting the scene. Physics keeps running
   so the robot follows the hand's translation and rotation gizmos.
   Each arm holds its captured joint targets until its gizmo is moved. Manual
   motion uses incremental IK with joint limits and bounded joint changes;
   moving one hand does not retarget the other hand.
2. Move either hand and adjust its **Grip** controls. Enter a meaningful
   **关键点名称**, then click **记录关键点** once the hand reaches the desired pose.
   Each point records both arms, including achieved TCP poses, joint angles,
   gripper commands and openings, and the gizmo targets separately.
3. Click **结束示教**. The scene pauses and the arm controls are suspended.
   Give `assets/teaching/latest.json` to the agent to rewrite the motion from
   the takeover onward. Reset and run the revised routine from the beginning.

There is no continuation of the interrupted routine. Running it again is
disabled until Reset or Ready. The takeover state includes the interrupted
phase, its step, and the parcel state. Poses use world coordinates in metres,
joint angles use radians, and quaternions use xyzw order.

Every point is saved immediately to a dated JSON file in `assets/teaching/`
and to `latest.json`. Reset and scene rebuilds preserve those files and mark
an unfinished teaching session as interrupted. The teaching controls survive
scene rebuilds; when starting with bare `./ctl up`, click Ready or Run routine
once to install them.
