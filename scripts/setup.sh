#!/usr/bin/env bash
# Install system, ROS and Python dependencies for AeroSLAM.
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

WITH_FRONTIER=0
for arg in "$@"; do
  case "${arg}" in
    --with-frontier) WITH_FRONTIER=1 ;;
    -h|--help)
      echo "Usage: $0 [--with-frontier]"
      exit 0 ;;
    *) aeroslam_die "unknown option: ${arg}" ;;
  esac
done

aeroslam_log "System packages (Ubuntu 20.04)…"
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential cmake git curl wget \
  python3-pip python3-dev python3-setuptools python3-yaml \
  libeigen3-dev libboost-all-dev libyaml-cpp-dev \
  libgoogle-glog-dev \
  libpcl-dev libopencv-dev

aeroslam_log "Navigation stack & tooling (costmap_2d, move_base, amcl, cmake_modules, joy)…"
sudo apt-get install -y --no-install-recommends \
  ros-noetic-navigation ros-noetic-cmake-modules ros-noetic-joy

aeroslam_log "OSQP (QP solver used by the MPC / minimum-snap optimizers)…"
sudo apt-get install -y --no-install-recommends \
  ros-noetic-osqp ros-noetic-osqp-vendor

aeroslam_log "ROS Noetic dependencies via rosdep…"
if ! command -v rosdep >/dev/null 2>&1; then
  sudo apt-get install -y python3-rosdep
fi
if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths "${AEROSLAM_ROOT}/src" --ignore-src -r -y --rosdistro "${AEROSLAM_ROS_DISTRO}"

if [[ "${WITH_FRONTIER}" -eq 1 ]]; then
  if ! command -v vcs >/dev/null 2>&1; then
    aeroslam_log "Installing vcstool (frontier third-party fetch)…"
    pip3 install --user vcstool
  fi
  mkdir -p "${AEROSLAM_ROOT}/src/frontier"
  aeroslam_log "Fetching frontier modules (see thirdparty/aeroslam.repos)…"
  ( cd "${AEROSLAM_ROOT}" && vcs import src/frontier < thirdparty/aeroslam.repos )
  aeroslam_log "Initializing nested submodules (e.g. FAST_LIO/ikd-Tree)…"
  for d in "${AEROSLAM_ROOT}"/src/frontier/*/; do
    if [[ -e "${d}.git" ]]; then
      git -C "${d}" submodule update --init --recursive || aeroslam_warn "submodule init failed for ${d}"
    fi
  done
  aeroslam_log "Applying AeroSLAM compatibility patches…"
  for patch in "${AEROSLAM_ROOT}"/thirdparty/patches/*/*.patch; do
    [[ -e "${patch}" ]] || continue
    module="$(basename "$(dirname "${patch}")")"
    if [[ -d "${AEROSLAM_ROOT}/src/frontier/${module}" ]]; then
      git -C "${AEROSLAM_ROOT}/src/frontier/${module}" apply "${patch}" \
        || aeroslam_warn "patch failed (already applied?): ${patch}"
    fi
  done
fi

aeroslam_log "Done. Next: ./scripts/build.sh"
