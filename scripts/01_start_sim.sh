#!/usr/bin/env bash
# 启动 Gazebo(Ignition) 仿真世界 (rmu_gazebo_simulator)
# 用法: ./01_start_sim.sh [world]   world: rmuc_2025(默认) | rmul_2024 | rmuc_2024 | rmul_2025
set -e

source /opt/ros/humble/setup.bash
source /home/labaicai/Desktop/RM_PB_SIMULATION/install/setup.bash

WORLD="${1:-rmuc_2025}"

# 修改仿真世界（默认 rmuc_2025 无需改）
# 如需切换世界: 编辑 src/rmu_gazebo_simulator/rmu_gazebo_simulator/config/gz_world.yaml 中的 world 字段

# 注意: 本机为双显卡 (NVIDIA + Intel 核显)，必须强制 glvnd 使用 NVIDIA EGL 厂商，
# 否则 gz 的 GPU 传感器 (mid360/rplidar/相机) 因选错设备而无法出数据
echo ">>> 启动仿真世界: $WORLD (Gazebo GUI, NVIDIA EGL)"
echo ">>> 世界默认暂停, 将自动解除 (等价于点击 Gazebo 左下角播放按钮)"

# 后台"保姆"进程: 世界加载完成后延迟 30 秒再 pause 解除 (unpause)
# 注意: 过早 unpause 会打断 spawn/performer 初始化导致服务器崩溃, 必须等稳定后再解除
(
  for i in $(seq 1 60); do
    ign topic -l 2>/dev/null | grep -q "/world/default/stats" && break
    sleep 2
  done
  sleep 30
  ign service -s /world/default/control --reqtype ignition.msgs.WorldControl \
      --reptype ignition.msgs.Boolean --timeout 3000 --req 'pause: false' > /dev/null 2>&1
) &

__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py gz_args:='-r'