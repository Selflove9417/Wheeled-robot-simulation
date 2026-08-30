cd ~/bbot_ws
colcon build --packages-select bbot_rc_receiver 编译
source ~/bbot_ws/install/setup.bash
ros2 run bbot_rc_receiver rc_node 运行节点 出现Error 13则执行sudo usermod -aG dialout $USER并重启
开一新终端执行 ros2 topic list
ros2 topic echo /rc_input

