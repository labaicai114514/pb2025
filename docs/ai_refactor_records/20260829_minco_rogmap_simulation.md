# MINCO/ROGMap 仿真集成改造记录

## User Intent

将 navi_minco_bit 中需要测试的 ROGMap、MINCO planner 和控制器迁移到 PBSimulation，采用模式 A；激光雷达、里程计、重定位和全局路径地图沿用 PBSimulation 配置；不修改原项目源码。

## Scope

新增 vendor 副本、仿真集成 launch、参数文件、说明文档和本记录。

## Out of Scope

不改原项目源码，不重构 PBSimulation 既有传感器、重定位、行为树和底盘链路，不实现 MPPI（源代码实际提供的是 qpOASES MPC），不做多机器人适配，不执行未经授权的构建或运行。

## Explorer Findings

检查了 PBSimulation 的仿真导航 launch、bringup/localization/navigation launch、simulation Nav2 YAML，以及 vendor 的 ROGMap Config/ROS2 wrapper、MincoPlanner、MincoMpcController、plugin XML 和 CMake/package 文件。

### Active logic path

Planner plugin 在 configure() 内创建 ROGMapROS，通过 MapRegistry 和 MapQueryInterface 提供查询；同一 planner 进程内的搜索、MINCO、corridor 和 safety 共享该查询。ControllerServer 加载 minco_controller::MincoMpcController。

### Data flow

Gazebo → pointcloud converter → Point-LIO → loam_interface → registered_scan/lidar_odometry → planner-owned ROGMap → PRIORMAP Nav2 global search + MINCO → qpOASES MPC → velocity_smoother/fake_vel_transform → Gazebo。

### Risk notes

vendor controller 对 /aft_mapped_to_init、/opt_path 和诊断 topic 使用绝对名；因此当前 wrapper 仅保证单机器人测试。ROGMap frame 为 odom，全局静态搜索 frame 为 map，必须依赖现有 map -> odom TF。Nav2 plugin ABI 和 qpOASES 依赖仍需构建后确认。

## Modifier Changes

### Files changed

- src/navi_minco_bit_vendor/ros_interfaces/**
- src/navi_minco_bit_vendor/rog_map/**
- src/navi_minco_bit_vendor/minco_planner/**
- src/navi_minco_bit_vendor/minco_controller/**
- src/pb_minco_sim_integration/launch/minco_sim_nav.launch.py
- src/pb_minco_sim_integration/config/minco_sim_nav2_params.yaml
- src/pb_minco_sim_integration/README.md

### Key changes

vendor 目录由原目录复制而来；新增 wrapper 选择 Mode A 参数并复用 PBSimulation bringup；YAML 注册 MincoPlanner/MincoMpcController，关闭 Nav2 动态 voxel 层，显式配置 ROGMap 默认参数并关闭 PCD/旧 ESDF/先验地图加载。

### Behavior preserved

Gazebo、激光点云转换、Point-LIO、loam、small_gicp 重定位、静态 map_server、Nav2 行为树、速度平滑和 fake_vel_transform 保持原有启动链路。

### Behavior intentionally adjusted

Nav2 global planner 改为 MincoPlanner；原局部控制器改为 MincoMpcController；动态障碍地图职责从 costmap voxel/intensity 层移交 ROGMap；全局 costmap 保留为静态 PRIORMAP 搜索接口。原用户修改 point_lio.publish.path_en: True 保留。

### Notes

当前“MPPI”称呼按源码事实更正为 MPC。ROGMap 不是独立节点。

## Auditor Review

### Checks performed

- [x] 关键路径 grep 检查
- [x] vendor 与原项目对应目录 diff -rq 比较
- [x] plugin XML 与 YAML 插件名核对
- [x] launch Python AST 解析
- [x] YAML 解析
- [x] workspace XML/package.xml 解析
- [x] ROGMap 参数读取与 YAML 显式键核对
- [x] projection/decay/mask 约束静态核对
- [x] 构建和启动测试（用户已授权）

### Issues found

未发现静态语法或目录复制差异。剩余风险是构建时依赖/ABI、运行时 TF、QoS 和绝对 topic 在实际 ROS graph 中的确认。

### Final result

PASS（构建与插件初始化）；运行闭环待在允许 DDS UDP 的环境中完成。

## Verification Log (2026-08-30)

### Build

命令：`source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select ros_interfaces rog_map minco_planner minco_controller pb_minco_sim_integration --continue-on-error`

结果：5 个目标包全部 Finished。ROGMap、MincoPlanner、MincoMpcController 的 stderr 仅包含 vendor/第三方代码 warning。全量 workspace 构建另受 `small_gicp` FetchContent 访问 GitHub 的网络限制影响。

### Launch smoke test

使用 `ROS_LOG_DIR=/tmp/pb_minco_smoke_log2` 启动 `pb_minco_sim_integration/minco_sim_nav.launch.py`，参数为 `namespace=red_standard_robot1`, `world=rmuc_2025`, `use_composition=False`, `use_rviz=False`, `slam=False`。日志确认：

- `MincoMpcController` 成功加载为 `FollowPath` 控制器；
- `MincoPlanner` 成功加载为全局规划器；
- ROGMap 成功初始化，`frame_id=odom`、`cloud=registered_scan`、`odom=lidar_odometry`；
- `planner_mode=PRIORMAP`、`global_search=Nav2Costmap`、`dynamic_query=FrameAwareRogQuery`。

首次 smoke test 发现 `cloud_filter.box_sizes.*` 空数组会导致参数解析失败，已在集成 YAML 中补齐与单框配置一致的数组并重建通过。

### Environment limitation

沙箱内各节点启动后报告 `Error creating socket: Operation not permitted`，因此无法可靠执行 `ros2 topic hz/echo`、TF 查询或发送导航目标；该限制来自运行环境，不是插件初始化错误。
