#!/usr/bin/env bash
# 启动导航栈 - 建图模式 (SLAM, slam_toolbox + point_lio)
# 前提: 先运行 01_start_sim.sh 且仿真世界内点击了"启动"按钮
# 用法: ./02_start_nav_slam.sh [world]
set -e

source /opt/ros/humble/setup.bash
source /home/labaicai/Desktop/RM_PB_SIMULATION/install/setup.bash

WORLD="${1:-rmuc_2025}"

echo ">>> 启动建图模式 (SLAM), world=$WORLD"
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:="$WORLD" \
  slam:=True

# 保存地图: 另开终端执行
#   ros2 run nav2_map_server map_saver_cli -f <地图名> --ros-args -r __ns:=/red_standard_robot1