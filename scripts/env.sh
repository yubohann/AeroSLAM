#!/usr/bin/env bash
# AeroSLAM common environment helpers. Source this from other scripts:
#   source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

AEROSLAM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export AEROSLAM_ROOT

AEROSLAM_ROS_DISTRO="${AEROSLAM_ROS_DISTRO:-noetic}"

aeroslam_source_ros() {
  local setup="/opt/ros/${AEROSLAM_ROS_DISTRO}/setup.bash"
  if [[ ! -f "${setup}" ]]; then
    echo "[AeroSLAM] ERROR: ${setup} not found." >&2
    echo "[AeroSLAM] Install ROS 1 ${AEROSLAM_ROS_DISTRO} (Ubuntu 20.04) or run: ./scripts/setup.sh" >&2
    return 1
  fi
  # shellcheck disable=SC1090
  source "${setup}"
}

aeroslam_packages_in() {
  # Print the <name> of every package.xml under the given directory.
  local dir="$1"
  find "${dir}" -name package.xml -print0 2>/dev/null \
    | xargs -0 -r grep -ohP '(?<=<name>)[^<]+' 2>/dev/null | sort -u
}

aeroslam_log()  { printf '\033[1;36m[AeroSLAM]\033[0m %s\n' "$*"; }
aeroslam_warn() { printf '\033[1;33m[AeroSLAM]\033[0m %s\n' "$*" >&2; }
aeroslam_die()  { printf '\033[1;31m[AeroSLAM]\033[0m %s\n' "$*" >&2; exit 1; }
