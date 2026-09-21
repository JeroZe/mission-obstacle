#!/usr/bin/env bash
# 把当前目录（mid360_obstacle_stop 的 ROS 2 workspace）推到 GitHub。
#
#   用法：  bash push_to_github.sh
#   前提：  git 可用；对 https://github.com/JeroZe/mission-obstacle.git 有写权限
#
# 说明：
#   * 仓库根目录 = 这个 workspace，包在 mid360_obstacle_stop/ 下。
#     在 Ubuntu 上建议 clone 到 ~/mid360-obstacle/src/mission-obstacle，
#     这样包路径是 src/mission-obstacle/mid360_obstacle_stop，和你现在的布局一致。
#   * build/ install/ log/ *.zip 已在 .gitignore 里，不会提交。

set -euo pipefail

REMOTE_URL="https://github.com/JeroZe/mission-obstacle.git"
BRANCH="main"

cd "$(dirname "${BASH_SOURCE[0]}")"

if ! git --version >/dev/null 2>&1; then
  echo "错误：git 不可用。macOS 上通常是 Xcode 许可协议没同意，先执行：" >&2
  echo "    sudo xcodebuild -license accept" >&2
  exit 1
fi

if [ ! -d .git ]; then
  echo "==> git init ($BRANCH)"
  git init -b "$BRANCH"
fi

if ! git config user.name >/dev/null 2>&1 || ! git config user.email >/dev/null 2>&1; then
  echo "错误：git 提交身份未配置，先执行：" >&2
  echo "    git config --global user.name  \"你的名字\"" >&2
  echo "    git config --global user.email \"你的邮箱\"" >&2
  exit 1
fi

if ! git remote get-url origin >/dev/null 2>&1; then
  echo "==> git remote add origin $REMOTE_URL"
  git remote add origin "$REMOTE_URL"
fi

echo "==> 暂存并提交"
git add -A
if git diff --cached --quiet; then
  echo "    没有新改动，跳过提交"
else
  git commit -m "feat: mid360_obstacle_stop - MID360 障碍物触发 PX4 Mission 暂停"
fi

echo "==> 检查远端状态"
if git ls-remote --exit-code --heads origin "$BRANCH" >/dev/null 2>&1; then
  echo "    远端已有 ${BRANCH} 分支，先 rebase 再推送"
  git fetch origin "$BRANCH"
  git rebase "origin/$BRANCH"
else
  echo "    远端还没有 ${BRANCH} 分支（空仓库）"
fi

echo "==> push"
git push -u origin "$BRANCH"

echo
echo "完成。提交内容："
git log --oneline -3
git ls-tree -r --name-only HEAD
