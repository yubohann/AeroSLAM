#!/usr/bin/env bash
# Launch an aerial simulation demo.
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"
aeroslam_source_ros
source "${AEROSLAM_ROOT}/devel/setup.bash"

DEMO="${1:-empty}"
shift || true

case "${DEMO}" in
  empty)
    aeroslam_log "hector_quadrotor_gazebo: empty world"
    exec roslaunch hector_quadrotor_gazebo quadrotor_empty_world.launch "$@"
    ;;
  indoor-slam)
    aeroslam_log "hector_quadrotor_demo: indoor SLAM (willow garage + hector_mapping)"
    exec roslaunch hector_quadrotor_demo indoor_slam_gazebo.launch "$@"
    ;;
  outdoor-flight)
    aeroslam_log "hector_quadrotor_demo: outdoor flight (rolling landscape)"
    exec roslaunch hector_quadrotor_demo outdoor_flight_gazebo.launch "$@"
    ;;
  teleop)
    aeroslam_log "Quadrotor teleop (xbox; velocity mode)"
    exec roslaunch hector_quadrotor_teleop xbox_controller.launch "$@"
    ;;
  *)
    cat <<EOF
Usage: $0 [empty|indoor-slam|outdoor-flight|teleop] [roslaunch args…]
  empty            Gazebo empty world + quadrotor (default)
  indoor-slam      indoor SLAM demo (hector_mapping + geotiff)
  outdoor-flight   outdoor flight demo
  teleop           joystick teleop launch (needs /dev/input/jsX)
Examples:
  $0 empty gui:=false headless:=true
  $0 indoor-slam
EOF
    exit 1 ;;
esac
