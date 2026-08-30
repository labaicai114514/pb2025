#!/usr/bin/env bash
# =====================================================================
# 一键启动: 仿真世界 + MINCO 模式导航栈
# (MincoPlanner 替代全局规划 / MincoMpcController 替代局部控制 / ROGMap 进程内建图)
#
# 用法:
#   ./start_minco_sim.sh                      # 默认 rmuc_2025, 自动开仿真 + 自动导航测试
#   ./start_minco_sim.sh rmul_2025            # 指定场地
#   START_SIM=0 ./start_minco_sim.sh          # 仿真已在运行, 只起导航栈
#   AUTO_TEST=0 ./start_minco_sim.sh          # 只做诊断, 不自动发导航目标
#   AUTO_INIT=0 ./start_minco_sim.sh          # 不自动设置初始位姿 (手动在 RViz 点 2D Pose Estimate)
#   GOAL_X=1.5 GOAL_Y=2.0 ./start_minco_sim.sh    # 指定测试目标点 (map 系)
#   GOAL_DIST=3.0 ./start_minco_sim.sh        # 相对当前位姿向前 3m (默认 2m)
#
# 感知链路 (Hop1~6, 任一断都会导致下游 0Hz):
#   Hop1 livox/lidar   gz GPU 传感器 -> ros_gz_bridge
#   Hop2 velodyne_points       ign_sim_pointcloud_tool 转换
#   Hop3 aft_mapped_to_init    Point-LIO (IMU 初始化约 100 帧, 可能 1~2 分钟)
#   Hop4 registered_scan       loam_interface 转发
#   Hop5 odometry              sensor_scan_generation (需 lidar_odometry+registered_scan 同步)
#   Hop6 TF odom->base_footprint  sensor_scan_generation 发布
#   之后: 自动初始位姿 (真值 - 发射点 -> map 系) -> small_gicp -> map->odom TF
#
# 行为:
#   - 自动打开 gnome-terminal 跑仿真 (Gazebo GUI, world 可指定)
#   - 启动前置清理: FastDDS /dev/shm 残留无条件清除 (强杀进程残留锁文件会引发
#     RTPS_TRANSPORT_SHM open_and_lock_file failed, 导致新进程无法收发话题)
#   - 等待仿真就绪后启动 MINCO 导航栈, 按 Hop1~6 逐级等待/定位感知链路断点
#   - 链路通后自动发布初始位姿 (真值-发射点, 与 RViz 点击等效), 等 map->odom
#   - 诊断 -> 自动 navigate_to_pose 闭环测试 -> 汇总 (任一 FAIL 退出码非 0)
#   - Ctrl+C 退出时自动关闭仿真窗口
# =====================================================================
# 注意: ROS setup 脚本访问未设默认值的变量, 必须在 set -u 之前 source
# =====================================================================
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NS="red_standard_robot1"
WORLD="${1:-rmuc_2025}"
START_SIM="${START_SIM:-1}"
AUTO_TEST="${AUTO_TEST:-1}"
AUTO_INIT="${AUTO_INIT:-1}"
GOAL_DIST="${GOAL_DIST:-2.0}"
GOAL_X="${GOAL_X:-}"
GOAL_Y="${GOAL_Y:-}"
GZ_WORLD_YAML="$WS_DIR/src/rmu_gazebo_simulator/rmu_gazebo_simulator/config/gz_world.yaml"

source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

set -euo pipefail

# --- RMW 选择: 装有 CycloneDDS 时默认启用 (本机 FastDDS SHM 端口锁随机失败) ---
# 注意: RMW 按进程启动时生效, 必须在任何 ros2/桥接进程启动前 export;
#       仿真(gnome-terminal 子进程)与导航栈都会继承, 全栈保持一致。
#       覆盖: RMW_IMPL=fastrtps ./start_minco_sim.sh  (或直接 export RMW_IMPLEMENTATION=...)
if [ -z "${RMW_IMPLEMENTATION:-}" ]; then
  if [ "${RMW_IMPL:-rmw_cyclonedds_cpp}" = "rmw_cyclonedds_cpp" ] && \
     [ -f /opt/ros/humble/lib/librmw_cyclonedds_cpp.so ]; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    echo ">>> RMW: CycloneDDS (librmw_cyclonedds_cpp 已安装)"
  fi
fi

PASS=0
FAIL=0
NAV_PID=""
GOAL_LOG=""
FAILED_HOPS=()

check() { # $1=名称  $2=0/1 (0=OK 1=FAIL)
  if [ "$2" -eq 0 ]; then PASS=$((PASS+1)); echo "    [OK]   $1"
  else                  FAIL=$((FAIL+1)); echo "    [FAIL] $1"; fi
}

# 检查话题频率: $1=topic  $2=最低Hz  $3=采样秒(默认10)
# 注意: ros2 topic hz 不支持 --qos-reliability (Humble 内部固定 sensor_data QoS),
#       加该参数会导致 argparse 报错而测不到任何数据 (实测误报 0Hz)
topic_rate() {
  local out rate
  out="$(timeout "${3:-10}" ros2 topic hz "$1" --window 10 2>/dev/null \
        | grep -oE "average rate: [0-9.]+" | tail -1 || true)"
  rate="${out##*: }"
  if [ -n "$rate" ] && awk -v r="$rate" -v m="$2" 'BEGIN{exit !(r>=m)}'; then
    check "$1 >= ${2}Hz (实测 ${rate}Hz)" 0
  else
    check "$1 >= ${2}Hz (实测 ${rate:-0}Hz)" 1
  fi
}

# 话题是否有数据 (0=有)
topic_alive() { # $1=topic $2=超时秒(默认5)
  [ -n "$(timeout "${2:-5}" ros2 topic echo "$1" --once --field header.frame_id \
           --qos-reliability best_effort 2>/dev/null || true)" ]
}

# 检查 TF 是否存在 (消息级, 不用 tf2_echo): $1=child_frame  $2=最小sec(排除 epoch 0 戳)
# 背景: 全部 TF 以仿真时间戳发布, tf2_echo 的 buffer/时钟语义在本环境下永远查不到
#       (实测三次运行 Hop6/map->odom 均误报), 直接读 /tf 消息文本最可靠
tf_direct() {
  local out sec
  out="$(timeout 8 ros2 topic echo /tf --once --qos-reliability best_effort 2>/dev/null || true)"
  [ -z "$out" ] && return 1
  sec="$(printf '%s\n' "$out" | awk \
    '/^[[:space:]]+sec: /{v=$2} /child_frame_id: '"$1"'$/{print v; exit}')"
  [ -z "$sec" ] && return 1
  awk -v s="$sec" -v m="$2" 'BEGIN{exit !(s>=m)}'
}

check_tf_direct() { # $1=child_frame  $2=最小sec  $3=显示名
  if tf_direct "$1" "$2"; then check "$3" 0
  else                       check "$3" 1; fi
}

# 检查话题 frame_id: $1=topic $2=期望frame
check_topic_frame() {
  local out
  out="$(timeout 5 ros2 topic echo "$1" --once --field header.frame_id --qos-reliability best_effort 2>/dev/null || true)"
  if [ "$out" = "$2" ]; then check "$1 frame_id == $2" 0
  else                       check "$1 frame_id == $2 (实测: ${out:-无数据})" 1; fi
}

# 轮询等待条件: 返回 0 表示已就绪
wait_cond() { # $1=描述  $2=重试次数  $3=每次间隔秒  $4=条件命令(求值)
  local desc="$1" tries="$2" gap="$3" cond="$4" i
  echo "    --- 等待 $desc (最多 $((tries*gap)) 秒)..."
  for i in $(seq 1 "$tries"); do
    if eval "$cond"; then return 0; fi
    sleep "$gap"
  done
  return 1
}

cleanup() {
  echo; echo ">>> 清理: 关闭仿真窗口..."
  [ -n "$GOAL_LOG" ] && rm -f "$GOAL_LOG" 2>/dev/null || true
  [ -n "$NAV_PID" ] && kill "$NAV_PID" 2>/dev/null || true
  pkill -f "[0]1_start_sim.sh" 2>/dev/null || true
  pkill -f "[r]os2 launch rmu_gazebo" 2>/dev/null || true
}
trap cleanup EXIT

# ---------- 前置检查与清理 ----------
if [ ! -d "$WS_DIR/install" ]; then
  echo "[错误] 未找到 $WS_DIR/install —— 请先编译: colcon build --symlink-install"
  exit 1
fi
if [ "$START_SIM" = "1" ]; then
  echo ">>> 前置清理: 停止残留仿真进程 + 清除 FastDDS /dev/shm 残留锁文件"
  pkill -9 -f "[i]gn gazebo" 2>/dev/null || true
  pkill -9 -f "[r]os2 launch rmu_gazebo" 2>/dev/null || true
  pkill -9 -f "[r]os2 launch pb_minco_sim_integration" 2>/dev/null || true
  # 无条件清除: 被强杀进程不会释放 /dev/shm 锁 (sem.fastrtps*), 残留会导致
  # 新进程 RTPS_TRANSPORT_SHM open_and_lock_file failed, 话题收发静默失败
  rm -f /dev/shm/sem.fastrtps* /dev/shm/fastrtps* 2>/dev/null || true
  sleep 2
fi

# ---------- 1. 打开仿真窗口 (gnome-terminal) ----------
if [ "$START_SIM" = "1" ]; then
  echo ">>> [1/4] 打开仿真窗口 (Gazebo GUI, world=$WORLD)"
  gnome-terminal --window --geometry=120x32 --title="RM仿真 Gazebo($WORLD)" -- \
    bash -c "cd $WS_DIR && bash scripts/01_start_sim.sh $WORLD; exec bash" 2>/dev/null \
    || { echo "[错误] gnome-terminal 启动失败"; exit 1; }
else
  echo ">>> [1/4] START_SIM=0: 跳过仿真启动, 假设仿真已在运行"
fi

# ---------- 2. 等待仿真就绪 ----------
if [ "$START_SIM" = "1" ]; then
  echo ">>> [2/4] 等待仿真世界加载 (约 30~90 秒)..."
  READY=0
  for i in $(seq 1 60); do
    if ign topic -l 2>/dev/null | grep -q "/$NS/odometry"; then
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
fi

# ---------- 3. 启动 MINCO 导航栈 (后台) ----------
echo ">>> [3/4] 启动 MINCO 导航栈 (MincoPlanner + MincoMpcController + ROGMap, world=$WORLD)"
ros2 launch pb_minco_sim_integration minco_sim_nav.launch.py \
  world:="$WORLD" use_composition:=False use_rviz:=True slam:=False &
NAV_PID=$!

# 等待导航栈就绪: navigate_to_pose action 出现 (生命周期全部 ACTIVE)
echo ">>> 等待导航栈就绪 (/$NS/navigate_to_pose action)..."
NAV_READY=0
for i in $(seq 1 30); do
  if ros2 action list 2>/dev/null | grep -q "/$NS/navigate_to_pose"; then
    NAV_READY=1
    break
  fi
  sleep 3
done
if [ $NAV_READY -ne 1 ]; then
  echo "[警告] 90 秒内未见到 /$NS/navigate_to_pose action —— 后续诊断会如实报告"
fi

# ---------- 3.5 地图 (map_server) 健康门 ----------
# 实测: 启动期 FastDDS 服务调用失败会让 lifecycle_manager_localization 中止 bringup
# (map_server/get_state async_send_request failed -> Aborting bringup), map_server 停在
# 未配置, /map 不发布 -> global_costmap 静态层空 -> PRIORMAP 全局搜索无图、RViz 无地图。
# 传输恢复后重试生命周期通常可成功, 故在此自动补齐并校验 /map 有数据。
echo ">>> 检查地图 (map_server) 健康..."
if wait_cond "map_server 节点出现" 15 3 "ros2 node list 2>/dev/null | grep -q '/$NS/map_server'"; then
  rst="?"
  for i in 1 2 3 4 5; do
    rst="$(ros2 lifecycle get "/$NS/map_server" 2>&1 || true)"
    case "$rst" in
      *active* )      break ;;
      *inactive*|*configured*) ros2 lifecycle set "/$NS/map_server" activate > /dev/null 2>&1 || true ;;
      *unconfigured*) ros2 lifecycle set "/$NS/map_server" configure  > /dev/null 2>&1 || true
                      ros2 lifecycle set "/$NS/map_server" activate   > /dev/null 2>&1 || true ;;
    esac
    sleep 3
  done
  if wait_cond "地图话题 /$NS/map 发布" 8 3 "topic_alive '/$NS/map' 3"; then
    check "map_server 地图已加载 (/map 有数据)" 0
  else
    check "map_server 地图已加载 (/map 无数据, 状态=${rst:-?})" 1
  fi
else
  check "map_server 节点存在" 1
fi

# ---------- 4. 感知链路逐级定位 + 初始位姿 + 诊断 + 闭环测试 ----------
echo ">>> [4/4] 感知链路检查 (Hop1~6) + 诊断 + 闭环测试"

# hop_check: $1=编号 $2=描述 $3=话题 $4=等待总秒数
hop_check() {
  local tries=$(( $4 / 5 )); [ "$tries" -lt 1 ] && tries=1
  echo "    [Hop$1] $2 ($3), 等待最多 $4 秒..."
  if wait_cond "$2 ($3) 有数据" "$tries" 5 "topic_alive '$3' 4"; then
    check "Hop$1 $2 ($3) 有数据" 0
  else
    check "Hop$1 $2 ($3) 无数据" 1
    FAILED_HOPS+=("$1")
  fi
}

# 4a. Hop1~2: 原始点云链路 (gz 传感器 + 桥接 + 点云转换)
hop_check 1 "gz 点云桥接"        "/$NS/livox/lidar" 60
hop_check 2 "点云转换(velodyne)" "/$NS/velodyne_points" 60

# 4b. Hop3: Point-LIO (IMU 初始化约 100 帧, 仿真负载下可能 1~2 分钟甚至更久)
hop_check 3 "Point-LIO 里程计"   "/$NS/aft_mapped_to_init" 300

# 4c. Hop4: loam_interface 转发
hop_check 4 "loam 注册点云"      "/$NS/registered_scan" 60

# 4d. Hop5: sensor_scan_generation (需 lidar_odometry + registered_scan 同步)
hop_check 5 "底盘里程计 odometry" "/$NS/odometry" 60

# 4e. Hop6: odom -> base_footprint TF (sensor_scan_generation 发布)
if wait_cond "TF odom->base_footprint" 10 3 \
     'tf_direct base_footprint 1'; then
  check "Hop6 TF odom -> base_footprint" 0
else
  check "Hop6 TF odom -> base_footprint" 1
  FAILED_HOPS+=("6")
fi

# 4f. 自动设置初始位姿: 真值(世界系) - 发射点(gz_world.yaml) -> map 系, 发布到 /initialpose
#     等价于 RViz 点击 2D Pose Estimate; small_gicp 回调还需 odom->gimbal_yaw TF (已有 Hop6+URDF)
if [ "$AUTO_INIT" = "1" ]; then
  echo "    --- 自动设置初始位姿 (真值 - 发射点 -> map 系, 发布到 /$NS/initialpose) ---"
  INIT_OUT="$(WORLD="$WORLD" NS="$NS" GZ_WORLD_YAML="$GZ_WORLD_YAML" python3 - <<'PY' || true
import math, os, sys, time
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

world = os.environ["WORLD"]
ns = os.environ["NS"]
with open(os.environ["GZ_WORLD_YAML"]) as f:
    cfg = yaml.safe_load(f)
spawn = None
for r in (cfg.get("robots") or {}).get(world, []) or []:
    if r.get("name") == "red_standard_robot1":
        spawn = r
        break
if not spawn:
    print("ERR_SPAWN"); sys.exit(3)
sx, sy = float(spawn["x_pose"]), float(spawn["y_pose"])
syaw = float(spawn.get("yaw", 0.0))

rclpy.init()
node = Node("auto_init_pose")
msgs = []
node.create_subscription(Odometry, "/%s/chassis_odometry_gt" % ns, lambda m: msgs.append(m), 1)
deadline = time.time() + 8.0
while time.time() < deadline and not msgs:
    rclpy.spin_once(node, timeout_sec=0.5)
if not msgs:
    print("ERR_GT"); rclpy.shutdown(); sys.exit(2)
m = msgs[0]
gx, gy = m.pose.pose.position.x, m.pose.pose.position.y
q = m.pose.pose.orientation
gyaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

# map = R(-yaw_spawn) * (gt - spawn);  rmuc_2025 中 spawn yaw=0
c, s = math.cos(-syaw), math.sin(-syaw)
mx = c * (gx - sx) - s * (gy - sy)
my = s * (gx - sx) + c * (gy - sy)
myaw = gyaw - syaw
mz, mw = math.sin(myaw / 2.0), math.cos(myaw / 2.0)

pub = node.create_publisher(PoseWithCovarianceStamped, "/%s/initialpose" % ns, 10)
for _ in range(3):
    p = PoseWithCovarianceStamped()
    p.header.frame_id = "map"
    p.pose.pose.position.x = mx
    p.pose.pose.position.y = my
    p.pose.pose.orientation.z = mz
    p.pose.pose.orientation.w = mw
    pub.publish(p)
    rclpy.spin_once(node, timeout_sec=0.2)
    time.sleep(0.3)
rclpy.shutdown()
print("OK %.3f %.3f yaw=%.3f" % (mx, my, myaw))
PY
)"
  case "$INIT_OUT" in
    OK*)
      check "自动初始位姿发布 ($INIT_OUT)" 0 ;;
    ERR_GT*)
      check "自动初始位姿 (读取真值失败)" 1
      echo "    ▸ 请在 RViz 手动点 2D Pose Estimate" ;;
    ERR_SPAWN*)
      check "自动初始位姿 (gz_world.yaml 解析失败)" 1
      echo "    ▸ 请在 RViz 手动点 2D Pose Estimate" ;;
    *)
      check "自动初始位姿发布 (python 异常)" 1
      echo "    ▸ 详情见上方输出; 可手动在 RViz 点 2D Pose Estimate" ;;
  esac
else
  echo "    (AUTO_INIT=0: 请在 RViz 手动点 2D Pose Estimate 设置初始位姿)"
fi

# 4g. 等待 map->odom TF 可用 (small_gicp 的 TF 需新鲜时间戳; 1970 戳会被 tf2 缓存丢弃)
wait_cond "map->odom TF (初始位姿 + GICP 收敛)" 30 3 \
     'tf_direct odom 1' \
  || echo "[警告] map->odom TF 90 秒内未出现 —— 若 AUTO_INIT=0 请在 RViz 用 2D Pose Estimate 设置初始位姿"

# 4h. 预目标诊断: 传感器 / 里程计 / TF
echo "    --- 预目标诊断: 传感器 / 里程计 / TF ---"
check_tf_direct "odom" 1 "TF map -> odom (小GICP, 新鲜戳)"
check_tf_direct "base_footprint" 1 "TF odom -> base_footprint"
check_tf_direct "gimbal_yaw_fake" 1 "TF gimbal_yaw -> gimbal_yaw_fake"
topic_rate "/$NS/registered_scan" 1
topic_rate "/$NS/lidar_odometry" 1
topic_rate "/$NS/odometry" 1

# 4i. 自动导航闭环测试
if [ "$AUTO_TEST" = "1" ]; then
  # 目标点: GOAL_X/GOAL_Y 优先; 否则取 TF map->gimbal_yaw_fake 当前位姿向前 GOAL_DIST
  if [ -n "$GOAL_X" ] && [ -n "$GOAL_Y" ]; then
    POS_OUT="$GOAL_X $GOAL_Y 0.0 1.0"
    echo "    --- 使用指定目标点 ($GOAL_X, $GOAL_Y) ---"
  else
    echo "    --- 计算当前位姿 (rclpy TF buffer: map->gimbal_yaw_fake 等) 前方 ${GOAL_DIST}m 目标点 ---"
    POS_OUT="$(GOAL_DIST="$GOAL_DIST" python3 - <<'PY'
import math, os, sys, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

dist = float(os.environ["GOAL_DIST"])
rclpy.init()
node = Node("goal_compute",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
buf = Buffer(node.get_clock())
TransformListener(buf, node)
for _ in range(15):          # 给 listener 时间收到 /tf 与 /tf_static
    rclpy.spin_once(node, timeout_sec=0.2)
    time.sleep(0.05)

def lookup(frame):
    for _ in range(8):
        try:
            return buf.lookup_transform("map", frame, Time())   # Time()=latest
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.2)
            time.sleep(0.1)
    return None

for frame in ("gimbal_yaw_fake", "gimbal_yaw", "base_footprint"):
    t = lookup(frame)
    if t is None:
        continue
    x = t.transform.translation.x
    y = t.transform.translation.y
    q = t.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    gx, gy = x + dist * math.cos(yaw), y + dist * math.sin(yaw)
    print(f"{gx:.3f} {gy:.3f} {math.sin(yaw/2):.6f} {math.cos(yaw/2):.6f}")
    rclpy.shutdown()
    raise SystemExit(0)
print("ERR_NO_TF")
rclpy.shutdown()
raise SystemExit(1)
PY
)" || true
    if [[ "$POS_OUT" == ERR* ]] || [ -z "$POS_OUT" ]; then
      POS_OUT=""
      echo "[警告] 无法获取当前位姿 (TF 链未就绪), 跳过自动导航测试"
    fi
  fi

  if [ -n "$POS_OUT" ]; then
    read -r GX GY GZ GW <<< "$POS_OUT"
    GOAL_LOG="$(mktemp)"
    echo "    --- 发送测试目标: navigate_to_pose 到 ($GX, $GY) [frame=map, 最长 180 秒] ---"
    timeout 180 ros2 action send_goal "/$NS/navigate_to_pose" \
        nav2_msgs/action/NavigateToPose \
        "{pose: {header: {frame_id: 'map'}, pose: {position: {x: $GX, y: $GY}, orientation: {z: $GZ, w: $GW}}}}" \
        --feedback > "$GOAL_LOG" 2>&1 || true
    if grep -q "status: SUCCEEDED" "$GOAL_LOG"; then
      check "导航目标 ($GX, $GY) 闭环 SUCCEEDED" 0
    elif grep -q "status: ABORTED" "$GOAL_LOG"; then
      check "导航目标 ($GX, $GY) 闭环 ABORTED" 1
    elif grep -q "REJECTED" "$GOAL_LOG"; then
      check "导航目标 ($GX, $GY) 被 REJECTED" 1
    else
      check "导航目标 ($GX, $GY) 未完成/超时 (日志: $GOAL_LOG)" 1
      GOAL_LOG=""   # 保留日志供排查, 不在 cleanup 删除
    fi

    # 首次全局规划含 costmap 初始化, 先等 /opt_path 首帧再测频率, 避免误报
    wait_cond "MINCO 首次发布 /opt_path" 20 3 \
         '[ -n "$(timeout 3 ros2 topic echo /opt_path --once --field header.frame_id --qos-reliability best_effort 2>/dev/null || true)" ]' \
      || echo "[警告] /opt_path 60 秒内未发布 —— MINCO 全局规划可能未启动或目标被拒"

    echo "    --- 后目标诊断: MINCO 规划 / MPC / 命令链 ---"
    topic_rate "/opt_path" 1 12
    check_topic_frame "/opt_path" "map"
    topic_rate "/mpc_predict_path" 1 12
    topic_rate "/$NS/cmd_vel_controller" 5 12
    topic_rate "/$NS/cmd_vel" 5 12
  fi
else
  echo "    (AUTO_TEST=0: 跳过自动导航测试)"
  echo "    手动测试命令参考:"
  echo "      ros2 action send_goal /$NS/navigate_to_pose nav2_msgs/action/NavigateToPose \\"
  echo "        \"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}}\" --feedback"
fi

# ---------- 汇总 ----------
echo
echo "==================== 汇总 ===================="
echo "  通过: $PASS   失败: $FAIL"
if [ "$FAIL" -gt 0 ]; then
  echo "  结论: 存在失败项, 按断点定位:"
  for h in "${FAILED_HOPS[@]}"; do
    case "$h" in
      1) echo "    ▸ Hop1 死: gz 侧无 livox/lidar —— 检查仿真是否暂停、GPU 传感器(EGL 变量)、"
         echo "      ros_gz_bridge 是否启动; 命令: ign topic -l | grep livox; ros2 topic hz /$NS/livox/lidar" ;;
      2) echo "    ▸ Hop2 死: velodyne_points 无数据 —— 检查 ign_sim_pointcloud_tool 进程与 QoS:"
         echo "      ros2 topic info -v /$NS/livox/lidar  (对比 发布/订阅 QoS)" ;;
      3) echo "    ▸ Hop3 死: Point-LIO 未出 aft_mapped_to_init —— IMU 初始化需约 100 帧, 仿真慢时可能数分钟;"
         echo "      向上翻终端找 [point_lio] 日志 (IMU init 进度/报错); 命令: ros2 topic hz /$NS/velodyne_points" ;;
      4) echo "    ▸ Hop4 死: registered_scan 无数据 —— loam_interface 问题: 检查 QoS 与 TF:"
         echo "      ros2 topic info -v /$NS/aft_mapped_to_init; ros2 topic echo /tf --once | grep front_mid360" ;;
      5) echo "    ▸ Hop5 死: odometry 无数据 —— sensor_scan_generation 需 lidar_odometry+registered_scan 同步:"
         echo "      ros2 topic hz /$NS/lidar_odometry; ros2 topic hz /$NS/registered_scan" ;;
      6) echo "    ▸ Hop6 死: TF odom->base_footprint 缺 —— sensor_scan_generation 未发布 TF; 联系 Hop5" ;;
    esac
  done
  echo "  若 Hop1~6 全通过但仍有 FAIL: 检查 map->odom TF 与初始位姿 (AUTO_INIT 或 RViz 手动),"
  echo "  以及导航栈日志 ([planner_server]/[controller_server] 是否报错)。"
  echo "  地图未加载 / 话题偶发无数据 / small_gicp 一直 No accumulated points:"
  echo "    多为启动期 DDS(FastDDS SHM) 通信退化, 脚本已自动重试 map_server 生命周期; 仍失败时手工:"
  echo "      ros2 lifecycle set /$NS/map_server configure"
  echo "      ros2 lifecycle set /$NS/map_server activate"
  echo "    升级方案: sudo apt install ros-humble-rmw-cyclonedds-cpp 后,"
  echo "      RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ./start_minco_sim.sh 重启 (全栈自动继承)"
  echo "  TF 手工检查请用: ros2 topic echo /tf --once  (tf2_echo 在仿真时间戳下不可靠)"
  echo "  FastDDS SHM 反复报错时可用: export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
else
  echo "  结论: 全部通过 —— 感知链路 (Hop1~6) 正常, map->odom TF 正常,"
  echo "         MINCO 全局规划与 MPC 闭环 (目标点 -> /opt_path -> cmd_vel) 正常"
fi
echo "=============================================="

echo ">>> 导航栈保持运行中 (RViz 可继续手动发目标点), 按 Ctrl+C 退出并关闭仿真窗口"
wait "$NAV_PID"   # Ctrl+C 退出; trap 负责清理

if [ "$FAIL" -gt 0 ]; then exit 1; fi