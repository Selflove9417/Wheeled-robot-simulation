#!/usr/bin/env bash
# Display the restored complete-jump baseline and stop after two jumps.
set -eo pipefail
task_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$task_workspace"
source /opt/ros/iron/setup.bash
source "$task_workspace/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONUNBUFFERED=1
output_dir="$task_workspace/src/bbot_balance_controller/src/data_logs/flat_jump_trials/restored_demo_$(date +%Y%m%d_%H%M%S)"
exec python3 src/bbot_balance_controller/scripts/run_flat_ground_jump_trial.py \
  --trials 1 --jumps-per-session 2 --real-time-factor 1.0 \
  --post-balance-observe-duration 12 --gui --output-dir "$output_dir" "$@"
