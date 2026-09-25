#!/usr/bin/env bash
set -eo pipefail
repo_dir="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
source /opt/ros/melodic/setup.bash
mkdir -p "$repo_dir/output"
catkin_make --directory "$repo_dir" --source "$repo_dir/src" \
  --build "$repo_dir/output/build" \
  -DCATKIN_DEVEL_PREFIX="$repo_dir/output/devel" \
  -DCMAKE_INSTALL_PREFIX="$repo_dir/output/install" \
  -j"${SMARTCAR_BUILD_JOBS:-2}" -l"${SMARTCAR_BUILD_JOBS:-2}" "$@"
