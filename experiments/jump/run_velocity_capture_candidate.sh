#!/usr/bin/env bash
# Private velocity candidate at the README's normal speed; never a default.
set -eo pipefail
task_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$task_workspace/setup_env.sh"
observer_build="$task_workspace/build_native_command_observer/bbot_bringup"
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$observer_build:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$observer_build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
exec ros2 launch "$task_workspace/experiments/jump/velocity_capture.launch.py" \
    controller_type:=jump_velocity jump_height:=0.25 \
    world:=native_command_observation_world.sdf gazebo_world_name:=flat_jump_world "$@"
