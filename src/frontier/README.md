# AeroSLAM Frontier Modules

This directory hosts state-of-the-art third-party research systems
(LiDAR-inertial odometry, onboard planning, autonomous exploration). The code is
**not vendored into this repository** on purpose: several upstreams carry
GPL-2.0/GPL-3.0 licenses or move quickly. Instead the revisions are pinned in
`thirdparty/aeroslam.repos`.

## Fetch

```bash
# from the workspace root
vcs import src/frontier < thirdparty/aeroslam.repos
# nested submodules are required (FAST_LIO bundles ikd-Tree):
for d in src/frontier/*/; do git -C "$d" submodule update --init --recursive; done

# or simply:
./scripts/setup.sh --with-frontier
```

| Module | Role | License |
|---|---|---|
| `FAST_LIO` | LiDAR-inertial odometry & mapping (ikd-Tree) | GPL-2.0 |
| `livox_ros_driver` | Livox LiDAR driver + `CustomMsg` definitions | MIT |
| `livox_laser_simulation` | Gazebo Classic simulation of Livox Mid-40/100/360 | MIT |
| `ego-planner` | Gradient-based onboard local planning (EGO-Planner) | GPL-3.0 |
| `FUEL` | Fast UAV exploration in unknown environments | GPL-3.0 |
| `octomap_mapping` | 3D occupancy mapping | BSD-3-Clause |

## Compatibility patches

`thirdparty/patches/<module>/` carries minimal patches applied automatically by
`scripts/setup.sh --with-frontier`:

| Patch | Why |
|---|---|
| `livox_laser_simulation/gazebo11-compat.patch` | upstream `main` hardcodes Gazebo 7 includes/`ignition/math4`; patch makes it build against Gazebo Classic 11 (ignition-math6) with C++17 |
| `FAST_LIO/build-order.patch` | `fastlio_mapping` does not depend on its generated `fast_lio/Pose6D` headers; parallel builds race. Patch adds the standard catkin `add_dependencies(...)` |

## Status

- [x] Manifest pinned (`thirdparty/aeroslam.repos`)
- [x] Buildable in the workspace (FAST_LIO + livox driver + livox simulation)
- [ ] Sensing bringup: Livox Mid-360 mount + Gazebo plugin on the
      hector_quadrotor URDF (`aeroslam_uav_sensing` package)
- [ ] Control bridge: `quadrotor_msgs/PositionCommand` → hector command topics
- [ ] Exploration bringup with depth front-end (RealSense/Astra)
- [ ] Tutorials + validation scenarios

## Notes

- Build the full frontier set only after `./scripts/setup.sh` (needs PCL,
  OpenCV, DDS-less ROS 1 deps).
- Some modules (FUEL, ego-planner) expect `quadrotor_msgs` from the
  `uav_simulator` bundled with their own repositories; keep those upstream
  copies instead of re-declaring messages in AeroSLAM.
