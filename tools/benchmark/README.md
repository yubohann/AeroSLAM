# AeroSLAM Benchmark

Reproducible measurements on the simulation scenarios. All runs are headless and
CPU-only; timestamps use `/use_sim_time` (Gazebo clock).

## LiDAR-inertial odometry (`run_lio_benchmark.py`)

Launches `aeroslam_bringup uav_livox_fastlio.launch` headless, commands a
four-waypoint flight through the hector position controller, records FAST-LIO
`/Odometry` and Gazebo `/ground_truth/state`, pairs the streams by timestamp and
reports:

| Metric | Meaning |
|---|---|
| `odom_rate_hz` | FAST-LIO output rate |
| `odom_path_length_m` / `truth_path_length_m` | accumulated path (LIO / ground truth) |
| `position_rmse_m` | offset-aligned drift RMSE |
| `aligned_rmse_m` | RMSE after 2D rigid (Procrustes) alignment |
| `heading_offset_deg` | frame heading offset recovered by the alignment |
| `final_drift_m` / `max_drift_m` | terminal / peak drift |

```bash
source /opt/ros/noetic/setup.bash
source <workspace>/devel/setup.bash
python3 tools/benchmark/run_lio_benchmark.py --duration 45 --runs 3
```

Reference run (`results/ab_no_deskew.json`, 45 s flight, de-skew off):
**position RMSE 0.161 m, aligned RMSE 0.078 m, final drift 0.136 m**; a repeat
run (`results/run_01.json`) agrees (0.159 m / 0.069 m / 0.138 m).
De-skew A/B: linear per-point timestamps (`--scan-period 0.1`) degrade accuracy
in the simulator (RMSE 0.354 m), so they are disabled by default.

## Ground navigation (`run_ground_benchmark.py`)

Launches `sim_env config.launch` headless, initialises AMCL at the spawn pose,
picks/uses a goal and measures success / time / path length / final distance.
`pick_goal.py` selects a reachable free cell from the occupancy map.

```bash
python3 tools/benchmark/pick_goal.py \
  --map-yaml src/ground/sim_env/maps/test2/test2.yaml --min-dist 3 --max-dist 5
python3 tools/benchmark/run_ground_benchmark.py --goal-x 0.78 --goal-y 3.93
```

Reference run (`results/ground_02.json`): **success in 14.0 s**, path 3.68 m,
final distance to goal 0.438 m (A\* + RDP + safety corridor + min-snap global,
HLP+MPC+corridor local, AMCL localisation).

## Exploration

A baseline frontier explorer is provided for the UAV stack; see
`docs/tutorials/exploration.md` and
`tools/exploration/frontier_goal_publisher.py`.
