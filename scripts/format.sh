#!/usr/bin/env bash
# Apply .clang-format to C++ sources (check mode: --check).
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh"

MODE="--in-place"
if [[ "${1:-}" == "--check" ]]; then MODE="--dry-run --Werror"; fi

command -v clang-format >/dev/null 2>&1 || aeroslam_die "clang-format not found (apt install clang-format)."

mapfile -d '' FILES < <(find "${AEROSLAM_ROOT}/src" \
  -path "*/frontier/*" -prune -o \
  \( -name '*.h' -o -name '*.hpp' -o -name '*.cpp' -o -name '*.cc' \) -print0)

aeroslam_log "clang-format ${MODE} on ${#FILES[@]} files…"
clang-format "${MODE}" "${FILES[@]}"
