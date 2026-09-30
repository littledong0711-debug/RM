#!/usr/bin/env bash
# 可从任意目录调用；不自动安装依赖，不连接游戏串口。
set -eo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'ROS 2 Humble not found. See README.md.'
  exit 2
fi
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --return-code-on-test-failure
colcon test-result --verbose
