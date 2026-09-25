# Tutorial: autonomous exploration on the AeroSLAM UAV stack

This tutorial walks from a manual goal to a scripted frontier-based explorer.
Everything runs headless on CPU; no GPU is required.

## 0. Prerequisites

```bash
./scripts/setup.sh --with-frontier
./scripts/build.sh
source devel/setup.bash
```

## 1. Bring up sensing, odometry and planning

```bash
roslaunch aeroslam_bringup uav_ego_planner.launch gui:=false headless:=true rviz:=false
rosservice call /enable_motors true
```

The chain is: simulated Livox → `aeroslam_livox_bridge` → FAST-LIO
(`/Odometry`, `/cloud_registered`) → EGO-Planner → `position_command_bridge`
→ hector `command/pose`.

## 2. Manual goal (2D Nav Goal)

Lift the vehicle above the inflated ground cells (e.g. to 1 m) and send a goal:

```bash
rostopic pub -r 10 /command/pose geometry_msgs/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"

rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 5.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}}"
```

In RViz (or with a joystick) the same goal can be placed with the *2D Nav Goal*
tool. Verified outcome: EGO-Planner streams commands at 100 Hz and the vehicle
reaches the goal (5 m goal reached at x = 4.93 m in ground truth).

## 3. Scripted tour

Send a sequence of goals with small shells; EGO-Planner plans each leg:

```bash
for p in "5 0" "5 5" "0 5" "0 0"; do
  set -- $p
  rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
    "{header: {frame_id: world}, pose: {position: {x: $1, y: $2, z: 1.5}, orientation: {w: 1.0}}}"
  sleep 25
done
```

## 4. Frontier-based exploration (baseline)

`tools/exploration/frontier_goal_publisher.py` rasterises `/cloud_registered`
into a 2D occupancy grid, marks the travelled path as free, and periodically
publishes the nearest frontier (a free cell bordering never-observed space) as
a goal:

```bash
# with the stack from section 1 running
python3 tools/exploration/frontier_goal_publisher.py --cooldown 15 --max-goals 10
```

Key parameters:

| Parameter | Meaning |
|---|---|
| `--range` / `--resolution` | local grid extent / cell size (20 m / 0.25 m) |
| `--min-height` / `--max-height` | z band treated as obstacles (ignores the ground) |
| `--occupancy-threshold` | points per cell to mark occupied |
| `--min-goal-distance` | ignore frontiers too close to the current position |
| `--cooldown` | seconds between goals |
| `--max-goals` | stop after N goals |

Baseline explorer; richer policies (information gain, TARE/FUEL-style) plug in behind the same goal interface.

## 5. Notes and pitfalls

- **Start above the ground cells**: EGO-Planner refuses to plan when the start
  falls into the inflated ground layer — lift to ~1 m first.
- **Motors**: nothing moves until `rosservice call /enable_motors true`.
- **Goal frame**: goals are in the FAST-LIO world frame (`camera_init`); RViz
  goals from the *2D Nav Goal* tool use the fixed frame of the RViz config.
- **Deskew**: configurable via `livox_scan_period` (default off).
  scan (`~scan_period`), which is an approximation; see the technical report for
  the measured drift.
