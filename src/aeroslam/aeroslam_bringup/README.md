# aeroslam_bringup

Launch files, URDF and configuration for AeroSLAM scenarios.

## Scenarios

| Launch | Runs |
|---|---|
| `uav_livox_fastlio.launch` | Gazebo world, hector_quadrotor with a simulated Livox Mid-360, IMU relay, FAST-LIO odometry/mapping, RViz |
| `uav_livox_sensing.launch` | the same sensing chain without the SLAM backend |
| `uav_ego_planner.launch` | sensing + LIO + EGO-Planner + `PositionCommand` bridge (2D Nav Goal on `/move_base_simple/goal`) |

EGO-Planner flight (headless):

```bash
roslaunch aeroslam_bringup uav_ego_planner.launch gui:=false headless:=true rviz:=false
rosservice call /enable_motors true                    # enable the motors
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 5.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}"
```

The vehicle must be above the inflated ground cells before a goal is sent
(e.g. lift to ~1 m via `command/pose`, as in the benchmark), otherwise the FSM
reports the start as being inside an obstacle.

Both require the frontier modules:

```bash
./scripts/setup.sh --with-frontier   # fetch + patch + build FAST-LIO and Livox
./scripts/build.sh
roslaunch aeroslam_bringup uav_livox_fastlio.launch
```

Headless:

```bash
roslaunch aeroslam_bringup uav_livox_fastlio.launch gui:=false headless:=true rviz:=false
```

## Data flow

```
hector QuadrotorHardwareSim ──raw_imu──▶ topic_tools/relay ──▶ /livox/imu
livox_laser_simulation plugin ──/livox/lidar_sim (PointCloud)──▶ aeroslam_livox_bridge
                                   └──▶ /livox/lidar (CustomMsg) ──▶ FAST-LIO ──▶ /Odometry, /cloud_registered
```

- The Gazebo plugin publishes `sensor_msgs/PointCloud`; FAST-LIO consumes
  `livox_ros_driver/CustomMsg`. `aeroslam_livox_bridge` performs the conversion
  (`offset_time` is zero — no per-point deskew in simulation).
- The LiDAR is mounted at `base_link + 10 cm` (`livox_frame`); the FAST-LIO
  extrinsic is identity (`config/fastlio_livox_sim.yaml`).

## Tuning

- `samples` in `urdf/quadrotor_with_livox.urdf.xacro` trades ray count for
  simulation speed (default 8000 rays/scan at 10 Hz).
- `det_range` / `<range><max>` set the sensing range (default 70 m).
