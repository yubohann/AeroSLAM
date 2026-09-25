#!/usr/bin/env bash
# Launch a ground-robot navigation simulation (Gazebo + RViz + move_base).
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"
aeroslam_source_ros
source "${AEROSLAM_ROOT}/devel/setup.bash"

WORLD=test2
GUI=true
HEADLESS=false
PLANNER=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --world)   WORLD="$2"; shift ;;
    --map)     WORLD="$2"; shift ;;  # convenience alias: world and map usually match
    --headless) GUI=false; HEADLESS=true ;;
    --planner) PLANNER="$2"; shift ;;
    -h|--help)
      cat <<EOF
Usage: $0 [--world test2|warehouse] [--headless] [--planner a_star|hybrid_astar|sunshine]
  Default world: test2 (roslaunch sim_env main.launch).
  Other worlds/maps: uses sim_env/config.launch with world:=<world> map:=<world>.
EOF
      exit 0 ;;
    *) aeroslam_die "unknown option: $1" ;;
  esac
  shift
done

if [[ "${WORLD}" == "test2" ]]; then
  aeroslam_log "sim_env main.launch (test2)…"
  exec roslaunch sim_env main.launch gui:="${GUI}" headless:="${HEADLESS}" \
    ${PLANNER:+global_planner:="${PLANNER}"}
else
  aeroslam_log "sim_env config.launch world=${WORLD} map=${WORLD}…"
  exec roslaunch sim_env config.launch world:="${WORLD}" map:="${WORLD}" \
    gui:="${GUI}" headless:="${HEADLESS}" \
    ${PLANNER:+global_planner:="${PLANNER}"}
fi
