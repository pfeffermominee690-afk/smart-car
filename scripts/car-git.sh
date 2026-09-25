#!/usr/bin/env bash
# Never stages or commits files automatically.
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_dir"

usage() {
  echo "用法: ./scripts/car-git.sh [status|pull|push]"
  echo "status 查看分支和改动；pull 快进更新；push 推送已提交的开发分支。"
}

action="${1:-status}"
if [[ "$action" == "--help" || "$action" == "-h" ]]; then
  usage
  exit 0
fi
if [[ "$action" != "status" && "$action" != "pull" && "$action" != "push" ]]; then
  usage >&2
  exit 2
fi
if [[ "$action" == "status" ]]; then
  git status --short --branch
  exit 0
fi

branch="$(git symbolic-ref --quiet --short HEAD)" || {
  echo "当前处于 detached HEAD，请先创建或切换开发分支。" >&2
  exit 1
}
if [[ "$action" == "push" && "$branch" == "main" ]]; then
  echo "拒绝直接推送 main：请创建功能分支并通过 PR 合并。" >&2
  exit 1
fi
if [[ -n "$(git status --porcelain)" ]]; then
  echo "存在未提交改动，请检查并明确提交后再同步。" >&2
  git status --short >&2
  exit 1
fi
if [[ "$action" == "pull" ]]; then
  git pull --ff-only
else
  git push -u origin "$branch"
  echo "已推送分支 $branch；请到 GitHub 创建或查看对应 PR。"
fi
