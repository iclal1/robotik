# Phase 3D validation — 2026-11-22

## Root cause and visualization change

The baseline detector sent `Marker.DELETEALL` in every marker array, then
recreated cube and text markers with IDs based on each frame's detection
order. In a seven-second live sample, all 97 marker arrays contained
`DELETEALL`. The marker lifetime was 0.5 s. This delete/recreate behavior is
the confirmed source of unnecessary marker flashing; an ordering change
could also change marker IDs. RViz and the detector both used simulation time,
the RViz fixed frame was `map`, and the marker topic QoS matched. No
message-filter timestamp warning appeared in this launch log, and RViz
reported Global Status OK. No sensor timestamp was replaced or fabricated.

Markers now use IDs derived from odom-frame track IDs and receive ADD updates
without DELETEALL. Fresh confirmed markers are colored and labeled `LIVE`.
An interrupted track retains its last odom position for at most 1.2 s with
reduced opacity and a `LAST` or `PARTIAL` label. This is a visual memory, not
a current 3D detection or safety input. Expiration sends one DELETE for the
cube and one for its label. A 1.8 s lifetime also clears markers if the node
stops unexpectedly. Text is shorter and offset from the cube in the top-down
view. Displayed range uses a short adaptive exponential filter; large
changes pass through faster than small pixel jitter.

## Quantitative checks

| Measurement | Before | After |
|---|---:|---:|
| Marker arrays in stationary seven-second sample | 97 | 94 |
| DELETEALL actions | 97 | 0 |
| Stable cube/text IDs in sample | Index-based 0/1 | Track-based 38/39 |
| Marker update frequency | 13.7 Hz | About 13.4 Hz |
| Mean processing latency | 20.95 ms | 21.05 ms |
| Maximum measured latency | Not recorded | 24.89 ms |
| Static cube position span | 0 m | 0 m |
| Distinct static distance labels | 1 | 1 |
| Five-pose color precision / recall | 9/9 / 9/9 | 9/9 / 9/9 |
| Valid 3D position error, four observations | 0.0205 m mean | 0.0205 m mean |

The five Gazebo poses included blue at the image edge, blue frontal, blue and
green oblique, all three colors simultaneously, and red/green oblique. Green
and red partial/range-uncertain observations remained 2D only. Clipped blue
scores changed from 0 to 0.54–0.70 on three recorded views, using visible
silhouette evidence; these scores are heuristic, not calibrated probabilities.
The red far-range estimates remained invalid and were not promoted to 3D.
The colored target-pad image still produced zero candidates. The valid 3D
position errors by view were approximately 3.1 cm, 1.4 cm, 1.3 cm, and
2.5 cm in both versions. Thus no localization-error gain was measured.

In the live loss/recovery trial, the same marker IDs 36/37 survived a short
interruption. The long interruption produced exactly two DELETE actions;
recovery created IDs 38/39. During the loss, `unknown_or_unobserved` states
were observed and every sampled clearance message kept `safe_to_move: false`.
The underlying motion block `cube_clearance_not_validated` remains in place.

RViz was launched with the existing perception configuration. A screenshot
shows the raw and annotated cameras, map, LiDAR, robot and labeled blue cube,
with Global Status OK. The launch log contained no `Message Filter dropping`
or old-TF warning. RViz emitted a GLSL map-shader link warning at startup;
the map remained visible and Global Status was OK. It was not correlated with
marker deletion. Camera, LiDAR, odometry and limited Joint State rates
measured about 15.18, 9.99, 29.36 and 50.84 Hz respectively.

## Limits

Five static viewpoints and one target-only view cannot establish 95% precision
or recall under representative lighting and motion. The partial occlusion
fixture is an actual Gazebo camera image with a simulated opaque foreground
mask; it tests sudden silhouette loss, not every wall-occlusion geometry.
The stationary scene had no position or distance jitter to reduce. The EMA's
moving-robot benefit and marker behavior during a paused simulation remain
unverified. A prior `Message Filter` warning was not reproduced, so there was
no confirmed timestamp fault to correct. The OpenGL startup warning remains;
its effect on other graphics drivers is unverified.

The dedicated pre-Phase-3D backups are in
`.perception_backups/20261122_phase3d/`. During backup preparation, source
copies in the older `.perception_backups/20261122/` directory were
inadvertently overwritten. That older directory retains the original
`cube_geometry.cpython-310.pyc` bytecode, and the archived baseline reports
remain available. The production source and the new Phase-3D backups are
intact.

## Reproduce

With one running competition simulation and no autonomous movement:

```bash
cd /home/ik/robotics_competition_ws
python3 -m unittest discover -s src/competition_bringup/test -p 'test_*.py'
python3 src/competition_bringup/tools/capture_cube_angles.py --output reports/perception_validation/phase3d_repeat
python3 src/competition_bringup/tools/validate_marker_lifecycle.py --output reports/perception_validation/phase3d_repeat_lifecycle.json
python3 src/competition_bringup/tools/evaluate_cube_angles.py
```

The test-only pose scripts restore the robot's initial pose and never send
velocity commands. They do not edit the official world or saved maps.
