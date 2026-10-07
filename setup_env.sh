#!/usr/bin/env bash
# Source this file from any directory to load this checkout's ROS environment.
bbot_setup_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/iron/setup.bash
source "$bbot_setup_workspace/install/setup.bash"

export ROS_HOME="$bbot_setup_workspace/.ros"
export ROS_LOG_DIR="$bbot_setup_workspace/.ros/log"

bbot_setup_opt="$bbot_setup_workspace/opt_ros/opt/ros/iron"
if [[ -d "$bbot_setup_opt" ]]; then
  export AMENT_PREFIX_PATH="$bbot_setup_opt:${AMENT_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$bbot_setup_opt/lib:${LD_LIBRARY_PATH:-}"
  export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$bbot_setup_opt/lib:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}"
  export GZ_SIM_SYSTEM_PLUGIN_PATH="$bbot_setup_opt/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
fi
bbot_setup_bringup_lib="$bbot_setup_workspace/install/bbot_bringup/lib"
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$bbot_setup_bringup_lib:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$bbot_setup_bringup_lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export IGN_GAZEBO_RESOURCE_PATH="$bbot_setup_workspace/src:$bbot_setup_workspace/src/bbot_bringup/worlds:$bbot_setup_workspace/install/bbot_description/share:$bbot_setup_workspace/install/bbot_bringup/share:${IGN_GAZEBO_RESOURCE_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$bbot_setup_workspace/src:$bbot_setup_workspace/src/bbot_bringup/worlds:$bbot_setup_workspace/install/bbot_description/share:$bbot_setup_workspace/install/bbot_bringup/share:${GZ_SIM_RESOURCE_PATH:-}"
unset bbot_setup_opt bbot_setup_bringup_lib bbot_setup_workspace
