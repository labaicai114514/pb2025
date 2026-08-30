#!/usr/bin/env bash
# =====================================================================
# 一键启动: 仿真世界 + 导航栈
# 用法:
#   ./00_start_all.sh                 # 导航模式 (定位+NAV2, 默认 rmuc_2025)
#   ./00_start_all.sh slam            # 建图模式 (SLAM)
#   ./00_start_all.sh nav rmul_2024   # 指定场地导航
#
# 行为:
#   - 自动打开一个 gnome-terminal 跑仿真 (Gazebo GUI)
#   - 当前终端等待仿真就绪 (odometry 主题出现) 后启动导航栈
#   - Ctrl+C 退出导航时自动关闭仿真窗口
# =====================================================================
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-nav}"
WORLD="${2:-rmuc_2025}"

case "$MODE" in
  nav)  NAV_SCRIPT="03_start_nav_localize.sh" ;;
  slam) NAV_SCRIPT="02_start_nav_slam.sh" ;;
  *) echo "用法: $0 [nav|slam] [world], 例: $0 nav rmul_2024"; exit 1 ;;
esac

# ---------- 前置检查 ----------
if [ ! -d "$WS_DIR/install" ]; then
  echo "[错误] 未找到 $WS_DIR/install —— 请先编译: colcon build --symlink-install"
  exit 1
fi
if pgrep -f "[i]gn gazebo" > /dev/null 2>&1; then
  echo "[提示] 检测到已有仿真进程, 先清理旧进程..."
  pkill -9 -f "[i]gn gazebo" 2>/dev/null || true
  pkill -9 -f "[r]os2 launch rmu_gazebo" 2>/dev/null || true
  # 清除 FastDDS 残留 (含粘性信号量, 否则容器组件加载会卡住)
  rm -f /dev/shm/sem.fastrtps* /dev/shm/fastrtps* 2>/dev/null || true
  sleep 2
fi

# ---------- 1. 打开仿真窗口 (gnome-terminal) ----------
echo ">>> [1/3] 打开仿真窗口 (Gazebo GUI, world=$WORLD)"
gnome-terminal --window --geometry=120x32 --title="RM仿真 Gazebo($WORLD)" -- \
  bash -c "cd $WS_DIR && bash scripts/01_start_sim.sh $WORLD; exec bash" 2>/dev/null \
  || { echo "[错误] gnome-terminal 启动失败"; exit 1; }

# ---------- 2. 等待仿真就绪 ----------
echo ">>> [2/3] 等待仿真世界加载 (约 30~90 秒)..."
READY=0
for i in $(seq 1 60); do
  if ign topic -l 2>/dev/null | grep -q "/red_standard_robot1/odometry"; then
    # 世界就绪后延迟 30 秒再 unpause (过早会打断 spawn/performer 初始化导致服务器崩溃)
    sleep 30
    ign service -s /world/default/control --reqtype ignition.msgs.WorldControl \
        --reptype ignition.msgs.Boolean --timeout 3000 --req 'pause: false' > /dev/null 2>&1 || true
    # 确认仿真时间在走 (stats 的 sim_time > 0)
    if timeout 3 ign topic -e -t /world/default/stats -m ignition.msgs.WorldStatistics -n 1 2>/dev/null \
        | grep -qE "sim_time|iterations"; then
      READY=1
      break
    else
      echo "    (仿真时间未流动, 等待 15 秒后重试...)"; sleep 15
    fi
  fi
  sleep 3
done
if [ $READY -ne 1 ]; then
  echo "[错误] 仿真世界 180 秒未就绪, 请检查仿真窗口输出。"
  echo "        (常见: 显卡渲染问题 / gnome-terminal 未弹出)"
  pkill -f "[0]1_start_sim.sh" 2>/dev/null || true
  exit 1
fi
echo ">>> 仿真就绪 ✓ (odometry 已发布)"
sleep 5   # 等 TF/桥接稳定

# ---------- 3. 前台启动导航栈 (Ctrl+C 退出) ----------
echo ">>> [3/3] 启动导航栈: $NAV_SCRIPT (mode=$MODE, world=$WORLD)"
echo "    ▸ 建图模式: RViz 定位后手动操控建图, 完成后 map_saver 保存"
echo "    ▸ 导航模式: RViz 中选 [Nav2 Goal] 点击目标点"
echo "    ▸ 按 Ctrl+C 退出导航并自动关闭仿真窗口"
trap 'echo; echo ">>> 清理: 关闭仿真窗口..."; pkill -f "[0]1_start_sim.sh" 2>/dev/null || true; pkill -f "[r]os2 launch rmu_gazebo" 2>/dev/null || true' EXIT

bash "$WS_DIR/scripts/$NAV_SCRIPT" "$WORLD"