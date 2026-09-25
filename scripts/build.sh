#!/usr/bin/env bash
# Build the AeroSLAM workspace with catkin_make.
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"
aeroslam_source_ros

BUILD_TYPE=Release
JOBS="$(nproc)"
UAV_ONLY=0
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --debug)    BUILD_TYPE=Debug ;;
    --uav-only) UAV_ONLY=1 ;;
    -j)         JOBS="$2"; shift ;;
    -j*)        JOBS="${1#-j}" ;;
    -h|--help)
      cat <<EOF
Usage: $0 [--debug] [--uav-only] [-j N] [-- <extra catkin_make args>]
  --debug      build with CMAKE_BUILD_TYPE=Debug
  --uav-only   blacklist ground-stack packages (fast UAV/common build)
EOF
      exit 0 ;;
    --)         shift; EXTRA_ARGS=("$@"); break ;;
    *)          EXTRA_ARGS+=("$1") ;;
  esac
  shift
done

cd "${AEROSLAM_ROOT}"

CMAKE_ARGS=(-DCMAKE_BUILD_TYPE="${BUILD_TYPE}")

if [[ "${UAV_ONLY}" -eq 1 ]]; then
  GROUND_PKGS="$(aeroslam_packages_in "${AEROSLAM_ROOT}/src/ground" | paste -sd';')"
  CMAKE_ARGS+=(-DCATKIN_BLACKLIST_PACKAGES="${GROUND_PKGS}")
  aeroslam_log "UAV/common only. Blacklisted ground packages: ${GROUND_PKGS}"
fi

aeroslam_log "catkin_make (${BUILD_TYPE}, -j${JOBS})…"
catkin_make -j"${JOBS}" "${CMAKE_ARGS[@]}" "${EXTRA_ARGS[@]}"

aeroslam_log "Build finished. Source the workspace with:"
aeroslam_log "  source ${AEROSLAM_ROOT}/devel/setup.bash"
