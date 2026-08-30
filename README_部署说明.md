# RM 哨兵导航仿真工作空间部署说明

> 基于 [SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav) + [rmu_gazebo_simulator](https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator) 的 ROS2 Humble 工作空间

## 1. 工作空间结构

```
RM_PB_SIMULATION/
├── src/
│   ├── pb2025_sentry_nav/              # 哨兵导航栈（含 7 个子模块/包）
│   │   ├── pb2025_nav_bringup/         #   ● 启动文件/地图/参数/RViz 配置（最重要的包）
│   │   ├── point_lio/                  #   ● 激光惯性里程计（mid360，点云配准 SLAM）
│   │   ├── small_gicp_relocalization/  #   ● 基于先验点云的重定位
│   │   ├── loam_interface/             #   ● 雷达系 lidar_odom → 底盘系 odom 变换
│   │   ├── sensor_scan_generation/     #   ● odom 系点云 → 雷达系 + 发布 odom→chassis TF
│   │   ├── terrain_analysis(_ext)/     #   ● 地形分析：障碍物高度写入点云 intensity
│   │   ├── fake_vel_transform/         #   ● 云台自旋补偿的虚拟速度参考系
│   │   ├── pb_omni_pid_pursuit_controller/  # ● 全向底盘 PID 路径跟踪控制器
│   │   ├── pb_nav2_plugins/            #   ● 自定义 NAV2 插件（代价图层/行为）
│   │   ├── pb_teleop_twist_joy/        #   ● 手柄控制
│   │   ├── ign_sim_pointcloud_tool/    #   ● 仿真点云补 time field 工具
│   │   ├── pointcloud_to_laserscan/    #   ● 地形图转二维激光（仅 SLAM 模式）
│   │   └── livox_ros_driver2/          #   ● Livox 雷达驱动（实车用，仿真不用）
│   ├── rmu_gazebo_simulator/           # 仿真器本体（世界模型+机器人+裁判系统）
│   ├── rmoss_core/ rmoss_gazebo/       # RMOSS 机器人仿真中间件
│   ├── rmoss_interfaces/               # 自定义消息
│   ├── rmoss_gz_resources/             # 场地/机器人资源
│   ├── pb2025_robot_description/       # 机器人 URDF 描述
│   └── sdformat_tools/
├── build/  install/  log/              # colcon 产物
├── scripts/                            # 一键启动脚本
└── docs/                               # 研究文档
```

## 2. 快速使用

```bash
# 终端 1: 启动仿真世界（弹出 Gazebo 窗口，自动运行）
./scripts/01_start_sim.sh            # 默认 rmuc_2025 场地

# 终端 2: 建图模式（新场地建栅格图）
./scripts/02_start_nav_slam.sh
#   保存地图: ros2 run nav2_map_server map_saver_cli -f my_map --ros-args -r __ns:=/red_standard_robot1

# 或 导航模式（已有先验点云/地图, 用 RViz 的 Nav2 Goal 发目标点）
./scripts/03_start_nav_localize.sh
```

## 3. 重要已知事项（部署时发现）

### 3.1 显卡驱动（✅ 已修复，2026-08-26）
本机双显卡（NVIDIA RTX 5060 + Intel 核显）。之前 NVIDIA 只有内核模块没有用户态 GL 库，导致 GPU 传感器全部无数据。
- 已安装 NVIDIA **580.173.02** 完整驱动（`nvidia-smi` 正常，`glxinfo` 显示 RTX 5060）
- **启动仿真时必须指定 NVIDIA EGL 厂商**（双显卡机器 EGL 会选错设备）：
  ```bash
  __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json ros2 launch ...
  ```
  该环境变量已写入 `scripts/01_start_sim.sh`。
- 验证：仿真启动后 `ros2 topic echo /red_standard_robot1/livox/lidar --once` 应显示 32×1875 点云；若无数据先点 Gazebo 左下角「启动」按钮。

### 3.2 上游仓库 bug 修复
- `rmu_gazebo_simulator/scripts/referee_system/simple_competition_1v1.py` 和 `player_web/main_vision.py` 缺 shebang → 已补 `#!/usr/bin/env python3`（不补则 launch 报 Exec format error）。

### 3.3 世界暂停与 unpause 时序（★ 重点）
Gazebo 世界启动后默认**暂停**（`-r` 参数在 gui 模式下不生效），需手动点击窗口底部左侧的**播放按钮**或调用 WorldControl 服务解除暂停（`ign service -s /world/default/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --req 'pause: false'`）。

**⚠️ 过早 unpause 会导致服务器崩溃**：世界处于暂停时 spawn/LevelManager performer 初始化尚未完成，立即 unpause 会触发场景重复注册，日志报 `Another item already exists ... armor_0::light_bar_visual` 后 server 退出（现象：世界能加载、传感器创建成功，但进程很快"finished cleanly"）。
- **安全做法**：世界就绪后**延迟 ≥30 秒**再 unpause（或像手动点按钮那样等待 spawn 完成）
- 脚本 `01_start_sim.sh` / `00_start_all.sh` 已内置"延迟 30 秒自动 unpause"，无需手动点按钮

### 3.4 多进程残留问题
反复 `kill -9` launch 会留下孤儿节点进程，并导致 FastDDS 共享内存锁残留：
- `open_and_lock_file failed` → **命名信号量残留**（`/dev/shm/sem.fastrtps_*` 是粘性的，kill -9 后不会自动释放，必须手动删除！否则容器组件加载只加载 1~3 个就卡住、`/map` 无发布者、RViz 看不见地图）
- 清理命令:
  ```bash
  rm -f /dev/shm/sem.fastrtps* /dev/shm/fastrtps*
  ```
- **RViz 地图加载不出来排查顺序**：
  1. `ros2 topic info /red_standard_robot1/map` → Publisher count 应为 1
  2. Publisher=0 → 清理 `/dev/shm/sem.fastrtps*` 后重启导航栈
  3. 仍卡 → 用 `use_composition:=False`（非组合模式，独立进程绕开容器加载机制）
- 长期方案: 重启系统一次，并避免中途硬杀 launch

### 3.5 仿真世界切换
编辑 `src/rmu_gazebo_simulator/rmu_gazebo_simulator/config/gz_world.yaml` 的 `world` 字段
（rmul_2024 / rmuc_2024 / rmul_2025 / rmuc_2025），同时 `--world` 参数保持同名。

## 4. 实车迁移要点（后续研究）

| 项目 | 仿真 | 实车 |
|---|---|---|
| 雷达数据 | gz 点云 → ign_sim_pointcloud_tool 补 time | livox_ros_driver2 驱动 mid360 |
| 里程计 | point_lio（仿真云） | point_lio（真实云，装 prior 点云） |
| 重定位 | small_gicp 读仿真先验点云 | small_gicp 读实车建图点云 |
| TF | 仿真器发布机器人 TF | `use_robot_state_pub:=True` 或独立机器人包 |
| 时间 | use_sim_time:=True | use_sim_time:=False |
| 启动 | rm_navigation_simulation_launch.py | rm_navigation_reality_launch.py（world 指定实际地图名） |

更多细节见 [docs/架构分析.md](docs/架构分析.md)