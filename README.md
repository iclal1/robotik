# Autonomous Cube Pushing Robot — ROS 2 Robotics Competition

Work in progress for a university robotics competition. This project develops a simulated differential-drive robot intended to navigate an arena and push three colored cubes into matching target zones within an eight-minute autonomous run. Autonomous cube delivery and the full timed run have **not** been demonstrated.

## Environment and technology

The workspace uses Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress (`ign gazebo`), SLAM Toolbox, Nav2, AMCL, RViz2, Python 3, OpenCV, NumPy, Xacro and `ros_gz` bridges. The 6 × 4 m arena has two interior wall openings, three 0.18 m colored cubes and matching visual target zones. The robot has a 0.40 × 0.30 m chassis, a front passive pusher, an RGB camera and a LiDAR. The LiDAR plane sits above the cubes, so camera-based cube awareness is part of collision safety.

## Current status

| Capability | Status | Evidence |
| --- | --- | --- |
| Gazebo arena, robot, camera, LiDAR and ROS interfaces | Verified | [Phase 2](docs/phase2_validation.md) |
| Saved SLAM map and map server | Verified; 97.07% known grid | [Phase 5B](reports/phase5b_final_status.md) |
| AMCL initialization and dynamic localization | Verified in bounded trials | [Phases 5C–5E-2](reports/phase5e2_sensor_freshness_navigation_validation.md) |
| Nav2 path generation and bounded open-area motion | Verified; one 0.335 m trial succeeded | [Phase 5F](reports/phase5f_extended_navigation_validation.md) |
| Camera cube detection, tracking and costmap obstacles | Verified in diagnostic scenarios | [Phase 5G](reports/phase5g_cube_tracking_costmap_validation.md) |
| Oriented robot-footprint path checking | Verified in planning-only tests | [Phase 5H](reports/phase5h_narrow_passage_validation.md) |
| Passage B crossing | Two-direction planning validation only; physical trial pending | [Phase 5H](reports/phase5h_narrow_passage_validation.md) |
| Passage A crossing | Clearance remains unresolved under measured yaw uncertainty | [Phase 5H](reports/phase5h_narrow_passage_validation.md) |
| Autonomous cube approach, pushing and three-cube timed run | Pending | [Roadmap](docs/ROADMAP_TR.md) |

The Phase 5F bounded navigation trial traveled approximately 0.335 m in Gazebo, returned Nav2 `SUCCEEDED`, and had about 0.0936 m relative ground-truth goal error; no cube moved. This does not validate full-arena or passage traversal. Phase 5H's 76 automated tests passed, but its passage checks were planning-only. The robot's current position to passage A's approach did not pass the required clearance check. See [known issues](docs/KNOWN_ISSUES.md).

## Packages and layout

| Path | Purpose |
| --- | --- |
| `src/competition_robot_description` | Xacro robot geometry, links and sensors |
| `src/competition_simulation` | Competition SDF arena |
| `src/competition_bringup` | Launch, bridges, localization, navigation, camera perception, safety, RViz, tools and tests |
| `maps/competition_mapping/20261125T103259Z_572d1928` | Original saved map YAML and PGM |
| `reports/` | Phase reports and local experimental evidence |
| `scripts/` | Earlier geometry and simulation validation tools |

## Quick start

Install dependencies from the package manifests using ROS dependency resolution on a prepared Ubuntu 22.04 / Humble / Fortress machine. See [setup](docs/SETUP_TR.md) for verification and the current map portability step. The checked-in default Nav2 map metadata still points to the original machine's absolute PGM path. On another checkout, select the separate [portable Nav2 metadata example](maps/competition_mapping/20261125T103259Z_572d1928/nav2_portable_example.yaml) as `map:=`. Do not edit the original saved map.

```bash
cd ~/robotics_competition_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Run **one** launch mode at a time:

```bash
# Simulation and ROS sensor interfaces
ros2 launch competition_bringup competition.launch.py gui:=false rviz:=true

# Alternatively, online mapping mode
ros2 launch competition_bringup competition_mapping.launch.py gui:=false rviz:=true

# Alternatively, saved-map Nav2, RViz and planning with motion disabled
ros2 launch competition_bringup competition_nav2.launch.py gui:=false rviz:=true enable_motion:=false map:=$PWD/maps/competition_mapping/20261125T103259Z_572d1928/map.yaml
```

Run the non-motion automated suite after sourcing the workspace:

```bash
python3 -m pytest -q src/competition_bringup/test
```

`enable_motion:=false` is the Nav2 launch default. Keep it disabled for planning and diagnostics. A successful `ComputePathToPose` result alone does not prove the full robot can follow a path safely. Motion-enabled trials require fresh localization and cube tracks, map and live-costmap clearance, footprint and controller checks, a working emergency stop, and a bounded trial plan. Do not start duplicate Gazebo or Nav2 instances.

## Documentation

- [Turkish project status](docs/PROJECT_STATUS_TR.md)
- [Turkish roadmap](docs/ROADMAP_TR.md)
- [Turkish setup guide](docs/SETUP_TR.md)
- [Turkish run guide](docs/RUN_GUIDE_TR.md)
- [Architecture](docs/ARCHITECTURE.md)
- [Known issues](docs/KNOWN_ISSUES.md)
- [Team workflow](docs/TEAM_GUIDE_TR.md)
- [GitHub checklist](docs/GITHUB_CHECKLIST.md)
- [Repository preparation report](reports/github_repository_preparation.md)

The next work is controlled passage-B validation, passage-A clearance resolution, camera-guided cube approach, pushing, and finally a timed three-cube run. These are [planned phases](docs/ROADMAP_TR.md), not completed functions.
