# Phase 2 sensor, TF, and system validation

Phase 2 uses the existing Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress 6.18.0 arena and robot. The installed ROS bridge is `ros-humble-ros-gz-bridge 0.244.23`, linked to Ignition Transport 11 and Messages 8. No official geometry or physical robot parameters changed. No SLAM, Nav2 controller, color detector, or autonomous mission was added.

## Initial findings and engineering decisions

The Phase 1 Gazebo topics were `/clock` (`ignition.msgs.Clock`), `/scan` (`LaserScan`), `/camera/image` (`Image`), `/camera/camera_info` (`CameraInfo`), `/odom` (`Odometry`), `/joint_states` (`Model`), `/model/pusher_robot/tf` (`Pose_V`), and `/model/pusher_robot/cmd_vel` (`Twist`). Their names and types were checked with `ign topic -i`. ROS camera and scan headers initially used Gazebo scoped frame names outside the URDF TF tree. DiffDrive kept the last nonzero command when a publisher quit. Joint states ran at the 1 kHz physics rate. A first ground-truth turn revealed roughly 5 cm XY odometry error because Gazebo integrates at the axle, 0.05 m behind `base_footprint`.

The bridge now uses three explicit one-way YAML configurations. Separate LiDAR and camera bridge nodes override ROS frame IDs without changing Gazebo sensor poses. A single guard owns the Gazebo command path. An odometry corrector uses only wheel odometry and the known axle offset; it is the only `odom → base_footprint` publisher. Gazebo world poses are read only in `scripts/phase2_test_support.py` and its engineering tests. The Fortress server wrapper bounds shutdown if the GUI renderer stalls; the robot state publisher runs in a separate process session to prevent duplicate terminal interrupts. Headless and GUI runs were restarted successfully after these changes.

## Node and topic architecture

| ROS interface | Type | Gazebo source or destination | Direction and owner | ROS QoS / rate measured |
|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | `/clock`, `ignition.msgs.Clock` | GZ → ROS, core bridge | reliable, queue 1; 1000 Hz |
| `/scan` | `sensor_msgs/msg/LaserScan` | `/scan`, `ignition.msgs.LaserScan` | GZ → ROS, LiDAR bridge | best effort, queue 5; 10 Hz |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | `/camera/image`, `ignition.msgs.Image` | GZ → ROS, camera bridge, lazy subscriber | reliable, queue 2; 15 Hz while subscribed |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | `/camera/camera_info`, `ignition.msgs.CameraInfo` | GZ → ROS, camera bridge | reliable, queue 5; about 15 Hz |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | — | ROS → command guard | reliable, depth 1; controller chosen later |
| `/model/pusher_robot/cmd_vel` | `geometry_msgs/msg/Twist` | same name, `ignition.msgs.Twist` | guard → core bridge → GZ; internal interface | reliable, queue 1; guard sends at 30 Hz |
| `/wheel_odom_raw` | `nav_msgs/msg/Odometry` | `/odom`, `ignition.msgs.Odometry` | GZ → ROS, core bridge; internal interface | reliable, queue 10; about 29.4 Hz |
| `/odom` | `nav_msgs/msg/Odometry` | — | odometry corrector → ROS | reliable, depth 10; about 29.4 Hz |
| `/joint_states` | `sensor_msgs/msg/JointState` | `/joint_states`, `ignition.msgs.Model` | GZ → ROS, core bridge | reliable, queue 10; 1000 Hz |
| `/tf` | `tf2_msgs/msg/TFMessage` | — | odometry corrector and robot state publisher | dynamic; roughly 30 and 50 Hz components |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | — | robot state publisher | reliable, transient local |

No topic is bridged in both directions. The bridge does not publish odometry TF. `/cmd_vel` has one production subscriber, and the internal drive topic has one production publisher. The camera image bridge is lazy with a 100 ms subscriber check. Its ROS publisher is reliable because this Humble RViz Camera display requested reliable images; the shallow queue limits backlog. LiDAR uses best effort so a slow viewer cannot hold scan processing. Bridge source timestamps are preserved. Robot state publisher and RViz use simulation time. The guard intentionally uses a steady wall clock for its safety timeout; the odometry corrector copies simulation timestamps from incoming odometry and never calls its local clock. Both therefore avoid the unnecessary 1 kHz `/clock` subscription, which reduced their observed CPU use.

## TF tree and coordinate conventions

```text
map                         future SLAM/localization parent; absent in Phase 2
└── odom                     locally continuous wheel odometry
    └── base_footprint        corrected chassis-center ground projection
        └── base_link         z = 0.14 m
            ├── left_wheel_link
            ├── right_wheel_link
            ├── caster_swivel_link
            │   └── caster_link
            ├── pusher_link
            ├── lidar_link   z = 0.25 m above ground
            └── camera_link  x = 0.205 m, z = 0.195 m, 0.20 rad down
                └── camera_optical_frame
```

The diagram shows the intended future `map` parent; **no `map → odom` transform exists or is published now**. The base and LiDAR frames use X forward, Y left, Z up. The camera optical frame uses Z forward, X right, Y down as required by [REP 103](https://reps.openrobotics.org/rep-0103/); `odom` is the continuous local frame described in [REP 105](https://reps.openrobotics.org/rep-0105/). The live optical +Z vector in `base_link` measured `(0.9801, 0, -0.1987)`, confirming the 0.20 rad downward pitch. Wheel transforms and the caster chain resolved through TF. Both scan and camera timestamps resolved from `odom` after startup, and the two dynamic `/tf` publishers had distinct child frames. RViz emitted brief first-second cache warnings before the odometry transform arrived; it showed no persistent transform failure.

Gazebo odometry represents the axle motion. With yaw `θ`, the corrector shifts it to the chassis-center ground projection using `x_base = x_axle + 0.05(cos θ − 1)` and `y_base = y_axle + 0.05 sin θ`; it also adds the corresponding `0.05 ω` lateral velocity at the base origin. This uses wheel data and the known robot geometry only. The published planar pose covariance starts at σx=σy=0.03 m and σyaw=0.05 rad; those conservative constants are initial engineering estimates, not a calibrated long-run drift model.

## LiDAR measurements

The scanning plane remained exactly **0.25 m** above ground. The scan delivered 720 ranges from −π to +π radians, about 0.008739 rad between samples, with 0.08–8.0 m limits, at 10.00 Hz in simulation time. At the official start all 720 ranges were finite; the nearest return was the southern wall at about 0.55 m. No near-zero cluster from the chassis or pusher appeared. A live eight-second check measured median scan age relative to `/clock` near 0.003–0.004 simulated seconds.

The upright blue cube started 0.825 m from the robot center at a 14.0° bearing; the scan range toward it was **2.056 m**, from the static environment. At 0.406 m before contact, the scan still reported **1.611 m** in that direction. At the later contact pose, with the cube center 0.331 m away after physical motion, it reported **1.465 m** while camera pixels still showed the cube. The green and red cubes likewise produced wall-range returns when directly in camera view. This is experimental evidence that upright cubes below the scan plane are not normally mapped by LiDAR. The engineering camera route also collected scans while moving. Gazebo reports zero `scan_time` and `time_increment`, consistent with its instantaneous synthetic sweep; scan timestamps, coverage, and frame transforms were valid.

## Camera measurements

The camera produced `rgb8` images of **640 × 480**, 921,600 bytes each, at about 15.0 Hz. CameraInfo used `plumb_bob`, zero distortion, and `fx=fy≈381.361`, `cx=320`, `cy=240` pixels. Those intrinsics match the 80° horizontal field of view. In an eight-second subscription, 119 image stamps matched CameraInfo stamps. Both messages used `camera_optical_frame`. The median image age relative to `/clock` was about 0.005–0.007 simulated seconds. The illumination made red, green, and blue surfaces visually distinct in raw frames; the pusher did not obscure them.

The engineering script counted saturated color pixels in a central 440 × 360 image region to measure visibility. These counts are **test measurements**, not an autonomous detector:

| Observation | Cube center distance | Bearing | Matching color pixels |
|---|---:|---:|---:|
| Blue at official start | 0.825 m | +14.0° | 20,193 |
| Blue after alignment | 0.781 m | +2.4° | 19,069 |
| Blue at angled view | 0.784 m | −17.6° | 18,430 |
| Blue during approach | 0.585 m | +4.4° | 50,294 |
| Blue immediately before contact | 0.406 m | +6.5° | 92,040 |
| Blue during contact | 0.331 m, cube moved | +8.1° | 106,469 |
| Green centered | 0.803 m | +2.4° | 17,497 |
| Red centered | 0.866 m | −1.0° | 13,997 |

Raw sample frames: [blue at start](camera_blue_initial.png), [blue before contact](camera_blue_precontact.png), [blue at contact](camera_blue_contact.png), [green](camera_green_centered.png), and [red](camera_red_centered.png). Near contact the cube fills much of the image, so a later detector should use approach observations to estimate its full outline and use close images for color confirmation.

## Odometry, limits, and stop response

Engineering tests used `/cmd_vel` for control and read Gazebo world poses only for independent comparison. Straight 0.35 m/s and reverse 0.25 m/s trials had less than 0.1 mm reported displacement difference at printed precision. The low-speed 0.08 m/s trial also agreed at that precision. A combined 0.20 m/s, 0.40 rad/s arc had 0.0055 m position and 0.014 rad yaw error. Two 0.30 m/s stop-and-start runs had about 0.0038–0.0039 m position error. Basic 0.80 rad/s turns had about 0.008–0.011 m corrected XY error, versus about 0.05 m before the axle correction.

Oversized 2.0 m/s and 3.0 rad/s requests were clamped by the guard and the DiffDrive plugin. In sustained open-path tests, the peak odometry velocities reached exactly **0.80 m/s** and **1.50 rad/s**; no overspeed was observed. At maximum linear speed, a corridor run had 0.0021 m final position error and the physical robot traveled about **0.484 m after input stopped** while the 0.30 s watchdog and the configured 1.0 m/s² deceleration acted. The watchdog's observed input-to-zero-command delay was **0.313 wall seconds**. A nonfinite command was rejected. Repeated maximum-speed rotation had about 0.028–0.031 m XY error and up to 0.043 rad yaw error, showing that wheel odometry still drifts under fast turns. The configured acceleration caps remain ±1.0 m/s² and ±2.0 rad/s²; they provided stable motion and reached both competition speed limits in the available corridor.

The full collision footprint audit gave X from **−0.2000 to +0.2375 m**, maximum width **0.3350 m**, and maximum turning diameter **0.5322 m** around the ground-projected chassis center. The swiveling caster stays within that envelope. The robot again crossed both official openings through the public guarded command path; its Gazebo world pose ended at approximately `(1.290, −0.324)` beyond wall B, while the green cube stayed at its initial pose. A fresh centered pusher run moved the blue cube **0.316 m** forward; final cube yaw was **0.0055 rad**, with the cube upright at `z≈0.09`.

## Simulation performance and reproducibility

Rates and real-time factors below come from **simulated timestamps divided by wall-clock time**, not assumed scheduler rates. CPU percentages are measured over six wall seconds and are percentages of one core; totals may exceed 100% on a multicore host. RSS is resident memory.

| Mode | Clock-derived real-time factor | Gazebo server CPU / RSS | GUI CPU / RSS | RViz2 CPU / RSS | Other notable process CPU |
|---|---:|---:|---:|---:|---|
| Phase 1 baseline headless | 0.999 | about 57% / 300 MiB | — | — | single bridge about 11%; robot state publisher about 5% |
| Final headless | 1.000 | 51.3% / 296.1 MiB | — | — | core bridge 10.6%; LiDAR 4.5%; camera 4.3%; corrector 1.3%; guard 0.8% |
| Final GUI + RViz2 | 0.997 | 73.5% / 300.9 MiB | 149.2% / 408.4 MiB | 21.6% / 186.3 MiB | core bridge 17.0%; LiDAR 6.6%; camera 8.5% |

The first baseline CPU figures were a `ps` process snapshot; final CPU/RSS figures used `scripts/phase2_performance_check.py` over six seconds, so they are not identical measurement methods. The final GUI view and RViz displays were visually inspected: [Gazebo arena](phase2_gazebo_final.png), [RViz robot, scan, and camera](phase2_rviz_final.png). GUI display and camera image remained responsive. The GUI server now shuts down in under a second in the tested Ctrl-C path through `fortress_server.py`.

The automated `scripts/phase2_integration_check.py` built all three packages, parsed Xacro/URDF and SDF, passed the independent geometry audit, launched Fortress twice headlessly, confirmed the exact start pose `(-2.4, -1.4, 0)`, checked live sensors and stamped TF, exercised command saturation and timeout, and observed clean shutdown with no critical log errors in either restart. The final two runs measured a clock-derived real-time factor of 0.998.

## Run and repeat the checks

```bash
cd ~/robotics_competition_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch competition_bringup competition.launch.py gui:=false rviz:=false
```

Desktop launch: `ros2 launch competition_bringup competition.launch.py gui:=true rviz:=true`. To open RViz2 separately against a running simulation:

```bash
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix --share competition_bringup)/rviz/competition.rviz" --ros-args -p use_sim_time:=true
```

Stop a foreground launch with Ctrl-C. Run this all-in-one headless check **when no simulation is already running**:

```bash
python3 scripts/phase2_integration_check.py
```

For individual checks, start a fresh headless launch in another terminal, then run `python3 scripts/phase2_system_check.py`, `python3 scripts/phase2_safety_check.py`, or `python3 scripts/phase2_performance_check.py`. Run `python3 scripts/phase2_motion_validation.py --case basic`, `--case max_linear`, and `--case max_angular` from separate fresh launches. Likewise use `python3 scripts/phase2_camera_lidar_check.py --case blue_approach` or `--case colors` from separate fresh launches. These test scripts can command motion and alter cube poses through physical contact; restart before a different case.

## Remaining limitations and Phase 3 readiness

The installed Fortress JointStatePublisher sends `/joint_states` at **1000 Hz**. Its current online API lists an `<update_rate>` option, but this installed binary ignored it in a live test, so the ineffective tag was removed. Despite the redundant joint traffic, both final modes kept near-real-time performance. A later custom Gazebo publisher is justified only if profiling with SLAM/Nav2 shows a bottleneck. Odometry covariance is an initial conservative estimate and should be calibrated over longer routes. Fast repeated turns still drift, so SLAM localization remains necessary. RViz can log a brief startup message-filter drop before the first odometry transform; stamped TF lookups pass after startup and the displays recover. No map transform, SLAM, Nav2 navigation, autonomous perception, or mission behavior is present.

**Phase 3 can begin with a validated `odom → base_footprint → lidar_link` chain, valid scans, and synchronized camera data.** Its first work should integrate SLAM Toolbox and compare localization against the separate engineering truth source. The official mission must never subscribe to that truth source.

## Files changed in Phase 2

Modified: `README.md`, `scripts/audit_geometry.py`, `scripts/phase1_corridor_check.py`, `scripts/phase1_motion_smoke.py`, `scripts/phase1_pusher_check.py`, `src/competition_bringup/CMakeLists.txt`, `src/competition_bringup/package.xml`, and `src/competition_bringup/launch/competition.launch.py`.

Created: `src/competition_bringup/config/core_bridges.yaml`, `src/competition_bringup/config/lidar_bridge.yaml`, `src/competition_bringup/config/camera_bridges.yaml`, `src/competition_bringup/scripts/cmd_vel_guard.py`, `src/competition_bringup/scripts/odometry_corrector.py`, `src/competition_bringup/scripts/fortress_server.py`, `src/competition_bringup/rviz/competition.rviz`, `scripts/phase2_camera_lidar_check.py`, `scripts/phase2_integration_check.py`, `scripts/phase2_motion_validation.py`, `scripts/phase2_performance_check.py`, `scripts/phase2_safety_check.py`, `scripts/phase2_system_check.py`, `scripts/phase2_test_support.py`, and `docs/phase2_validation.md`.

Created evidence images: `docs/camera_blue_centered.png`, `docs/camera_blue_contact.png`, `docs/camera_blue_initial.png`, `docs/camera_blue_precontact.png`, `docs/camera_green_centered.png`, `docs/camera_red_centered.png`, `docs/phase2_gazebo_final.png`, and `docs/phase2_rviz_final.png`. The final Xacro and world geometry files remained as in Phase 1.

## External references and code attribution

The workspace robot, arena, guard, corrector, bridge YAML, launch, and test scripts are original local implementation. External runtime packages are ROS 2 Humble, `ros_gz_bridge`, RViz2, and Gazebo Fortress. The implementation follows [REP 103](https://reps.openrobotics.org/rep-0103/) and [REP 105](https://reps.openrobotics.org/rep-0105/). The bridge configuration and frame override behavior were checked against the [ros_gz_bridge documentation](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md) and the installed bridge headers/parameters. The joint rate check was compared with the [Gazebo 6 JointStatePublisher API](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1JointStatePublisher.html). No prebuilt robot, arena, or external project code was copied.
