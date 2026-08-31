#!/usr/bin/env bash
source /opt/ros/iron/setup.bash
source /home/admin/bbot_ws_new/install/setup.bash

export ROS_HOME=/home/admin/bbot_ws_new/.ros
export ROS_LOG_DIR=/home/admin/bbot_ws_new/.ros/log

export AMENT_PREFIX_PATH=/home/admin/bbot_ws_new/opt_ros/opt/ros/iron:${AMENT_PREFIX_PATH}
export LD_LIBRARY_PATH=/home/admin/bbot_ws_new/opt_ros/opt/ros/iron/lib:${LD_LIBRARY_PATH}
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/home/admin/bbot_ws_new/opt_ros/opt/ros/iron/lib:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/admin/bbot_ws_new/opt_ros/opt/ros/iron/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}
export IGN_GAZEBO_RESOURCE_PATH=/home/admin/bbot_ws_new/src:/home/admin/bbot_ws_new/src/bbot_bringup/worlds:/home/admin/bbot_ws_new/install/bbot_description/share:/home/admin/bbot_ws_new/install/bbot_bringup/share:${IGN_GAZEBO_RESOURCE_PATH}
export GZ_SIM_RESOURCE_PATH=/home/admin/bbot_ws_new/src:/home/admin/bbot_ws_new/src/bbot_bringup/worlds:/home/admin/bbot_ws_new/install/bbot_description/share:/home/admin/bbot_ws_new/install/bbot_bringup/share:${GZ_SIM_RESOURCE_PATH}
