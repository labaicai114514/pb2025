#!/usr/bin/env bash
# 启动导航栈 - 导航模式 (point_lio 先验点云 + small_gicp 重定位 + NAV2)
# 前提: 先运行 01_start_sim.sh
# 用法: ./03_start_nav_localize.sh [world]
set -e

source /opt/ros/humble/setup.bash
source /home/labaicai/Desktop/RM_PB_SIMULATION/install/setup.bash

WORLD="${1:-rmuc_2025}"

echo ">>> 启动导航模式 (重定位 + NAV2), world=$WORLD"
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:="$WORLD" \
  slam:=False

# 在 RViz 中使用 "Nav2 Goal" 插件发布目标点
# 若遇到容器加载卡住, 可加 use_composition:=False 尝试