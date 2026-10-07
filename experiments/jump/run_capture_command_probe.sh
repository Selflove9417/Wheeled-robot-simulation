#!/usr/bin/env bash
# One private actual-command publication/native-state timing probe; no default promotion.
set -eo pipefail
task_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$task_workspace"
source /opt/ros/iron/setup.bash
source "$task_workspace/install/setup.bash"
candidate_binary="$task_workspace/build_native_allocator/bbot_balance_controller/bbot_landing_repair_controller"
if [[ ! -x "$candidate_binary" ]]; then
  echo 'Build the independent bbot_landing_repair_controller target in build_native_allocator first.' >&2
  exit 1
fi
observer_build="$task_workspace/build_native_command_observer/bbot_bringup"
if [[ ! -f "$observer_build/libbbot_native_command_observer.so" ]]; then
  echo 'Build the independent bbot_native_command_observer target first.' >&2
  exit 1
fi
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$observer_build:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$observer_build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}"
export IGN_PARTITION="${IGN_PARTITION:-bbot_capture_command_probe}"
export IGN_IP="${IGN_IP:-127.0.0.1}"
export PYTHONDONTWRITEBYTECODE=1
exec python3 -u src/bbot_balance_controller/scripts/run_flat_ground_jump_trial.py \
  --jump-controller-executable bbot_landing_repair_controller \
  --record-native-wrench --native-command-observation \
  --wheel-actuation effort --wheel-effort-gain 1.0 \
  --thrust-support-allocator --native-state-diagnostics --trials 1 --jumps-per-session 1 \
  --real-time-factor 1.0 --jump-height 0.20 --post-balance-observe-duration 12 \
  --headless "$@"
