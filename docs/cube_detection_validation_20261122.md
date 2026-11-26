# Cube detection validation, 2026-11-22

The detector uses native 640 x 480 HSV silhouettes, a measured lower edge,
ground-plane ray intersection, top-height reprojection, and odom-frame track
association. The confidence field is a **heuristic evidence score, not a
calibrated probability**. The position uncertainty is a local sensitivity
estimate with a 3 cm shape/contact allowance, not a statistically calibrated
standard deviation. Gazebo poses are used only by the separate validation
tool; the production node does not subscribe to them.

## Measured results

| Check | Baseline | Improved |
|---|---:|---:|
| Blue at official start: confidence | 0.654 | 0.897 |
| Blue at official start: position error vs Gazebo | 0.029 m | 0.031 m |
| Blue at official start: reported position uncertainty | 0.127 m | 0.059 m |
| Green at yaw 1.0 rad: valid 3D pose | No | Yes |
| Green at yaw 1.0 rad: position error vs Gazebo | Not reported | 0.004 m |
| Red at yaw 1.0 rad: 2D confidence | 0.366 (range withheld) | 0.945 (range withheld) |
| Red at yaw 1.0 rad: tentative range error vs Gazebo | 0.052 m in earlier pose | 0.031 m |
| Color detection precision / recall on archived images | 9/9, 9/9 | 9/9, 9/9 |
| Processing latency | 18.36 ms single live status | 20.47 ms mean, 25.87 ms max over 99 statuses |
| Processed camera frames | About 5 Hz by 0.2 s gate | About 14.6 Hz in 5 s live sample |

The archived benchmark contains nine Gazebo camera images: near red and green,
five blue views including two clipped views, a distant red/green view, and a
colored target-only view. It has nine true cube color appearances and no
false labels in either version. These observations are too few and too similar
to establish 95% precision or recall under representative competition
conditions. Clipped blue views remain 2D observations with no 3D range. The
distant red range remains invalid because its position sensitivity is about
0.4 m; its 2D confidence is 0.945 and its tentative range is 2.751 m versus
2.720 m Gazebo truth. The target-only image yielded zero candidates.

The measured blue position error increased by about 1.5 mm, while the green
view gained a valid accurate pose. The lower reported blue uncertainty follows
explicit smaller pixel, height and pitch perturbations; it is not evidence of
a similarly large reduction in true error. Additional poses, lighting and
motion trials are needed to calibrate covariance and verify the 95% targets.

On the separate near centered Gazebo images, the baseline and improved
**geometry evidence** scores were red 0.773 to 0.904, green 0.700 to 0.912,
and blue 0.672 to 0.902. Corresponding reported uncertainty scales were
0.139 to 0.062 m, 0.125 to 0.057 m, and 0.120 to 0.056 m. These are offline
measurements with the same camera calibration, not ground-truth error checks.

## Reproduce

```bash
cd /home/ik/robotics_competition_ws
python3 -m unittest discover -s src/competition_bringup/test -p 'test_*.py'
python3 src/competition_bringup/tools/benchmark_cube_images.py
colcon build --packages-select competition_bringup --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch competition_bringup competition_exploration.launch.py gui:=true
```

With Gazebo running, compare a current detection to ground truth using
`python3 src/competition_bringup/tools/validate_cube_perception.py --label run
--output reports/perception_validation/run.jsonl`. This tool is independent of
the production detector.
