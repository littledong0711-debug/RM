#!/usr/bin/env bash
# 唯一运行版本：balanced，参数统一读取params.yaml。难度只用于日志标签。
set -eo pipefail
test_port="${1:-}"
test_level="${2:-}"
test_profile="${3:-balanced}"
if [[ "$test_port" =~ ^[0-9]+$ ]]; then
  test_port="/dev/pts/$test_port"
fi
if [[ ! "$test_port" =~ ^/dev/pts/[0-9]+$ ]] ||
  [[ "$test_level" != middle && "$test_level" != large && "$test_level" != super ]]; then
  echo 'Usage: bash run_test.sh PORT middle|large|super [balanced]'
  exit 2
fi
# 兼容旧命令中的balanced；历史实验档已移除。
if [[ "$test_profile" != balanced ]]; then
  echo 'Only the latest balanced profile is retained.'
  exit 2
fi
if [[ ! -c "$test_port" ]]; then
  echo "Serial port missing: $test_port. Check the game's latest Slave port."
  exit 2
fi
if pgrep -x armor_aim_node >/dev/null; then
  echo 'An aim node is already running. Press Ctrl+C in its terminal before retrying.'
  exit 2
fi
test_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$test_root"
if [[ ! -f install/setup.bash ]]; then
  echo 'Build first: bash scripts/build_and_test.sh'
  exit 2
fi
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p diagnostics/repeated_tests
test_log="diagnostics/repeated_tests/${test_level}_${test_profile}_$(date +%Y%m%d_%H%M%S)_pts${test_port##*/}_$$.log"
echo "Level label: $test_level (select the matching difficulty in the game)."
echo "Port: $test_port | Log: $test_log"
echo 'Press Ctrl+C after EVERY round, even if the next port is unchanged.'
{
  echo "level=$test_level profile=$test_profile port=$test_port started=$(date -Is)"
  echo 'Settings: src/armor_aim/config/params.yaml (latest balanced)'
  sha256sum install/armor_aim/lib/armor_aim/armor_aim_node src/armor_aim/config/params.yaml
  ros2 run armor_aim armor_aim_node --ros-args \
    --params-file src/armor_aim/config/params.yaml -p "serial_port:=$test_port"
} 2>&1 | tee "$test_log"
