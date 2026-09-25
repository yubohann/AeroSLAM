# aeroslam_livox_bridge

`sensor_msgs/PointCloud` → `livox_ros_driver/CustomMsg` converter.

The Gazebo Livox sensor simulation (`livox_laser_simulation`, frontier module)
publishes `sensor_msgs/PointCloud`. FAST-LIO consumes
`livox_ros_driver/CustomMsg`. This node bridges the two.

| Parameter | Default | Meaning |
|---|---|---|
| `input_topic` | `/livox/lidar_sim` | simulated LiDAR cloud |
| `output_topic` | `/livox/lidar` | CustomMsg for FAST-LIO |
| `frame_id` | `livox_frame` | output frame (input frame is the sensor name) |
| `reflectivity` | `100` | constant reflectivity (the simulator has no intensity) |
| `scan_period` | `0.0` | spread point timestamps linearly over a scan for de-skew (0 disables; the A/B in `docs/technical-report.md` shows 0 is better in this simulator) |

Build note: the executable is only built when `livox_ros_driver` is present
(`./scripts/setup.sh --with-frontier`); otherwise the package still configures
and the node is skipped.
