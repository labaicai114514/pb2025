# PBSimulation Mode A integration

Simulation-only adapter for vendored navi_minco_bit; the original source tree is unchanged.

Mode A keeps Gazebo, Point-LIO, loam_interface, small_gicp_relocalization, map server, behavior tree and velocity conversion. ROGMap replaces dynamic 3-D obstacle mapping. MincoPlanner replaces the Nav2 global planner and owns local cropping plus MINCO optimization. MincoMpcController replaces the controller plugin. Costmaps remain compatibility objects: global_costmap supplies the static PRIORMAP search surface and local_costmap keeps Nav2 behavior interfaces without dynamic voxel layers.

ROGMap is constructed inside MincoPlanner; no separate ROGMap node is started. The vendored controller is qpOASES MPC, not MPPI.

Start after build and sourcing:

    ros2 launch pb_minco_sim_integration minco_sim_nav.launch.py namespace:=red_standard_robot1 world:=rmuc_2025 use_composition:=False use_rviz:=True slam:=False

Interfaces: Point-LIO publishes /red_standard_robot1/aft_mapped_to_init and /red_standard_robot1/cloud_registered; loam publishes /red_standard_robot1/registered_scan and /red_standard_robot1/lidar_odometry; ROGMap uses odom, global planning uses map, MINCO publishes /opt_path, and the command chain is cmd_vel_controller -> cmd_vel_nav2_result -> /red_standard_robot1/cmd_vel.

The YAML explicitly supplies ROGMap legacy/defaulted keys (ESDF, PCD, frontier, inflation, virtual heights and debug). PCD loading and legacy ESDF are disabled; MINCO uses the projection-backed 2-D field. Prior-map fusion is disabled because PBSimulation map_server/global_costmap is the static source.

Static checks:

    python3 -c 'import ast; ast.parse(open("src/pb_minco_sim_integration/launch/minco_sim_nav.launch.py").read())'
    python3 -c 'import yaml; yaml.safe_load(open("src/pb_minco_sim_integration/config/minco_sim_nav2_params.yaml"))'

Runtime acceptance: verify non-zero scan/odom rates, /opt_path frame map, MPC and final command rates, map -> odom TF, then an RViz goal covering global planning, obstacle response, replanning and recovery. This wrapper is single-robot because vendor topics such as /opt_path are absolute.

## Vendored 源码适配 (与上游的差异)

`navi_minco_bit_vendor/minco_planner/src/minco_core/minco_planner.cpp`: 原代码用
`rclcpp::Clock().now()` (固定墙钟) 给 /opt_path、/backup_path、escape、createPlan
响应以及 FSM 时间轴 (nowSeconds/start_WT/轨迹过期判断) 盖时间戳。真机 (use_sim_time
=false) 下墙钟正确; 仿真下全栈是仿真时钟, 墙钟戳导致 MPC 的 map->odom 变换失败
("Transform data too old", 控制器误判到达、机器人不动)。已全部替换为
`node_.lock()->now()` (节点时钟): 仿真=仿真时间, 真机=墙钟, 行为等价。
`global_path_searcher.cpp` 的 4 处墙钟 (中间 Path 对象, 纯几何用途, 对外发布均重新
盖戳) 保持原样。此改动为唯一的 vendored 源码偏离 (其余对齐上游)。

## 启动脚本与闭环验证 (start_minco_sim.sh)

`scripts/start_minco_sim.sh` 一键完成: 开仿真 -> 等就绪/unpause -> 启动本 wrapper -> 感知链路
逐级定位 (Hop1~6) -> 自动设置初始位姿 -> 诊断 -> 自动导航闭环测试。

    ./scripts/start_minco_sim.sh                # 默认 rmuc_2025; 自动开仿真 + 自动测试
    START_SIM=0 ./scripts/start_minco_sim.sh    # 仿真已在运行, 只起导航栈
    AUTO_TEST=0 ./scripts/start_minco_sim.sh    # 只诊断不导航
    AUTO_INIT=0 ./scripts/start_minco_sim.sh    # 不自动设初始位姿 (RViz 手动 2D Pose Estimate)
    GOAL_X=1.5 GOAL_Y=2.0 ./scripts/start_minco_sim.sh  # 指定测试目标点 (map 系)
    GOAL_DIST=3.0 ./scripts/start_minco_sim.sh  # 相对当前位姿向前 3m (默认 2m)

感知链路与断点定位 (Hop1~6, 任一断则下游全部 0Hz):

- Hop1 `/red_standard_robot1/livox/lidar` — gz GPU 传感器 -> ros_gz_bridge
- Hop2 `/red_standard_robot1/velodyne_points` — ign_sim_pointcloud_tool 转换
- Hop3 `/red_standard_robot1/aft_mapped_to_init` — Point-LIO (IMU 初始化约 100 帧,
  仿真负载下可能 1~2 分钟甚至更久, 这是常见"假死"点)
- Hop4 `/red_standard_robot1/registered_scan` + `lidar_odometry` — loam_interface
- Hop5 `/red_standard_robot1/odometry` — sensor_scan_generation (需 lidar_odometry
  与 registered_scan 同步), 注意它不是 Gazebo 发布
- Hop6 TF `odom -> base_footprint` — sensor_scan_generation 发布
- 之后: 自动初始位姿 -> small_gicp -> TF `map -> odom`

初始位姿: gz 真值 `/red_standard_robot1/chassis_odometry_gt` (世界系) 减去
`gz_world.yaml` 中的发射点 (rmuc_2025: x=3.4, y=9.5, yaw=0) 即得 map 系位姿 (map
原点 ≈ 世界原点, 与 RViz 点击等效, 残差在 GICP 收敛范围内), 经 rclpy 单进程
发布到 `/<ns>/initialpose`。注意不能直接用世界系真值当初始位姿 (会偏出先验图)。

重要约束: planner 插件实例名必须注册为 `GridBased` (见 config 中 planner_server.
GridBased 块) —— pb2025 的 BT XML (navigate_to_pose_w_replanning_and_recovery.xml)
里 ComputePathToPose 硬编码 `planner_id="GridBased"`, 实例名若为 MincoPlanner 会报
"planner GridBased is not a valid planner" 且所有导航目标必然失败。插件类型仍是
`minco_planner/MincoPlanner`。

其他要点: 启动前置无条件清除 FastDDS `/dev/shm` 残留 (强杀进程遗留锁文件会引发
RTPS_TRANSPORT_SHM open_and_lock_file failed, 话题收发静默失败)。所有 TF 都以仿真
时间戳发布, 手工查 TF 请用 `ros2 topic echo /tf --once` (tf2_echo 的 buffer/时钟
语义在本环境下不可靠, 会误报 TF 不存在); `ros2 topic hz` 不支持
`--qos-reliability` 参数 (会 argparse 报错导致测不到数据)。启动脚本内部同样基于
/tf 消息级探测与 rclpy TF buffer (use_sim_time), 不再使用 tf2_echo。

脚本含 map_server 健康门: 启动期 DDS 服务调用失败会令 lifecycle_manager_localization
中止 bringup (地图不加载), 脚本会探测到并自动重试 configure/activate, 再校验 /map
有数据。仍失败 (多为 FastDDS SHM 通信退化) 时手工:

    ros2 lifecycle set /red_standard_robot1/map_server configure
    ros2 lifecycle set /red_standard_robot1/map_server activate

或换 CycloneDDS 全栈重启: sudo apt install ros-humble-rmw-cyclonedds-cpp
(脚本会自动检测并启用, 无需手动 export; 可用 RMW_IMPL=fastrtps 强制回退 FastDDS)。
注意 RMW 按进程启动生效: 切换后必须完全重启 (脚本前置清理会杀旧进程), 否则新旧进程
RMW 不一致无法互相通信, 且全栈 (仿真桥接+导航) 需在同一 RMW 下。

RViz: wrapper 默认加载集成视图 `rviz/nav2_default_view_minco.rviz` (基线视图 + MINCO
/opt_path_vis/astar/backup/candidate/control_points/recover_goal、ROGMap
occupied/inflated/field/layer_value/map_bound、MPC predict/real 共 14 个显示项,
含 transien_local 与 best_effort 的 QoS 匹配), 可用 rviz_config_file:= 覆盖。

内置诊断项 (每项 OK/FAIL 汇总, 任一 FAIL 退出码非 0):

- 链路 Hop1~6 各话题是否有数据 + 失败 Hop 的修复提示
- TF `map -> odom` (初始位姿 + GICP 收敛后), TF `odom -> base_footprint`
- 话题频率: `/<ns>/registered_scan`, `/<ns>/lidar_odometry`,
  `/<ns>/odometry` (sensor_scan_generation 融合输出)
- 测试目标下发后: 等 `/opt_path` 首帧 (首次全局规划含 costmap 初始化, 10~30s),
  然后 `/opt_path` 频率且 `header.frame_id == map` (MincoPlanner 输出),
  `/mpc_predict_path` 频率 (MPC 工作证据), `/<ns>/cmd_vel_controller` (MPC 原始输出),
  `/<ns>/cmd_vel` (经 smoother + fake_vel_transform 的最终底盘命令)
- 闭环结果: `/red_standard_robot1/navigate_to_pose` action
  (SUCCEEDED / ABORTED / REJECTED / 超时)
