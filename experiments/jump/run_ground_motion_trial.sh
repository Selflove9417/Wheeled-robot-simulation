#!/usr/bin/env bash
set -eo pipefail
task_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$task_workspace"
source "$task_workspace/setup_env.sh"
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$task_workspace/build_native_command_observer/bbot_bringup:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$task_workspace/build_native_command_observer/bbot_bringup:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-104}"
export IGN_PARTITION="${IGN_PARTITION:-bbot_ground_motion_stage2}"
export IGN_IP="${IGN_IP:-127.0.0.1}"
export PYTHONDONTWRITEBYTECODE=1
exec python3 -u "$task_workspace/experiments/jump/run_ground_motion_trial.py" "$@"
