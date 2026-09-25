#!/usr/bin/env bash
# Run the workspace test suite (catkin/gtest + python).
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"
aeroslam_source_ros

cd "${AEROSLAM_ROOT}"

if [[ $# -gt 0 ]]; then
  aeroslam_log "Running tests for packages: $*"
  catkin_make run_tests "$@" 
else
  aeroslam_log "Building with CATKIN_ENABLE_TESTING and running all tests…"
  catkin_make -DCATKIN_ENABLE_TESTING=ON
  catkin_make run_tests
fi

catkin_test_results build/test_results --verbose || {
  aeroslam_warn "Some tests failed (see build/test_results)."
  exit 1
}
aeroslam_log "All tests passed."
