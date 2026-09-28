#!/usr/bin/env bash
# 把当前 workspace（仓库根）提交并推送到所有已配置的远端。
#
#   用法：  bash scripts/push.sh                       # 用默认提交信息
#           bash scripts/push.sh "feat: xxx"           # 指定提交信息
#   前提：  git 可用；对远端仓库有写权限
#
# 远端：
#   origin  https://github.com/JeroZe/mission-obstacle.git   主仓库
#   gitee   https://gitee.com/JeroZe/mission-obstacle.git     国内镜像（可选）
#
#   任一远端缺失会被跳过，任一远端失败不影响其它远端 —— 镜像的意义就是冗余。
#
# 说明：
#   仓库根 = colcon workspace 根，只跟踪 src/ 下的源码；
#   build/ install/ log/ bags/ *.bag *.db3 和 src/px4_msgs/ 都在 .gitignore 里，不会提交。

set -uo pipefail

BRANCH="main"
MESSAGE="${1:-chore: sync workspace}"
REMOTE_NAMES=(origin gitee)

cd "$(dirname "${BASH_SOURCE[0]}")/.."

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

# 收集实际存在的远端
REMOTES=()
for name in "${REMOTE_NAMES[@]}"; do
  if git remote get-url "$name" >/dev/null 2>&1; then
    REMOTES+=("$name")
  fi
done

if [ "${#REMOTES[@]}" -eq 0 ]; then
  echo "错误：没有配置任何远端，先执行：" >&2
  echo "    git remote add origin https://github.com/JeroZe/mission-obstacle.git" >&2
  echo "    git remote add gitee  https://gitee.com/JeroZe/mission-obstacle.git" >&2
  exit 1
fi

echo "==> 暂存并提交"
git add -A
if git diff --cached --quiet; then
  echo "    没有新改动，跳过提交"
else
  git commit -m "$MESSAGE"
fi

failures=0
for name in "${REMOTES[@]}"; do
  echo "==> push $name ($(git remote get-url "$name"))"

  # 远端已有该分支时先 rebase，避免 non-fast-forward
  if git ls-remote --exit-code --heads "$name" "$BRANCH" >/dev/null 2>&1; then
    if git fetch "$name" "$BRANCH" >/dev/null 2>&1; then
      git rebase "FETCH_HEAD" || echo "    警告：rebase 失败，跳过 rebase 直接尝试推送" >&2
    fi
  fi

  if git push "$name" "$BRANCH"; then
    echo "    $name 完成"
  else
    echo "    $name 失败" >&2
    failures=$((failures + 1))
  fi
done

echo
echo "==> 提交内容"
git log --oneline -3

if [ "$failures" -gt 0 ]; then
  echo
  echo "有 $failures 个远端推送失败，检查网络/权限后重试。" >&2
  exit 1
fi
