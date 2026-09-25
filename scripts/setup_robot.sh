#!/usr/bin/env bash
# Source this file; it does not launch any robot nodes.
_smartcar_script="$(readlink -f "${BASH_SOURCE[0]}")"
export SMARTCAR_REPO="$(cd "$(dirname "$_smartcar_script")/.." && pwd)"
export PYTHONDONTWRITEBYTECODE=1
unset _smartcar_script
if [[ ! -f "$SMARTCAR_REPO/output/devel/setup.bash" ]]; then
  echo "尚未构建工作空间，请先执行 $SMARTCAR_REPO/scripts/build.sh" >&2
  return 1 2>/dev/null || exit 1
fi
source "$SMARTCAR_REPO/output/devel/setup.bash"
