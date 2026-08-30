# HERO_2026_Sentry_NAV 开源分析 与 rm_slam 改进路线

> 分析日期：2026-08-27
> 分析对象：`/home/labaicai/RM2026/HERO_2026_Sentry_NAV`（git 单 commit `d96408c`）
> 背景：为改进我方哨兵导航方案（rm_slam，`/home/labaicai/RM2026/sentry_develop/src/rm_slam`）而进行的开源调研

---

## 1. 总览

**来源**：哈尔滨工业大学（威海）HERO 竞技机器人实验室 25/26 赛季导航组开源，
作者：刘谨搏、赵家康、冯永康。致谢 GCOPTER/DDR-Opt、陈立憨（北极熊 LihanChen2004）等——与北极熊/rm_slam 属同一技术生态。

**一句话定位**：这不是一套"导航定位栈"，而是一套**"会打仗的哨兵导航系统"**——
在 NAV2 之上加了赛场行为树决策层，在 NAV2 之下加了 ROG-Map 式感知后端与工业级 MPC/MINCO 控制链。

**分层架构**：

```
┌─ decision/   赛场行为树决策层（自主决策）★ rm_slam 最大空白
├─ perception/ 双雷达融合 + 点云去畸变 + 雷达/视觉融合
├─ planning/   dog_map(ROG-Map后端) → fast_layer(代价直投)
│              A*(SmacHybrid) → MINCO平滑 → acados MPC(100Hz) ★ 控制上限
├─ interfaces/ 导航内部消息 + 串口协议消息（裁判系统全套）
└─ scripts/    云台雷达外参标定工具 ★ 实车立即可用
```

**主数据流**：
`双雷达 → hero_lidar_scan(去畸变) → dog_map(log-odds 占据) → rog_map/inf_occ → fast_layer(代价图直投)`
`SmacPlannerHybrid(A*) → pb_minco/MincoSmoother(MINCO 多项式 + ESDF, 发布 MincoTrajectory) → hero_mpc_controller(acados MPC 100Hz 全状态跟踪) → velocity_smoother → 底盘`

---

## 2. 模块深度分析

### 2.1 decision/bt —— 赛场行为树决策层（借鉴价值 ★★★★★）

**rm_slam 现状**：无决策层。`navigation_data_processer`（94 行）只是 150Hz 转发器——
把 cmd_vel + dip_angle 打包发串口。哨兵没有自主决策能力。

**HERO 做法**（BehaviorTree.CPP 4.x 官方源码随仓库提交 + 15 个自定义 BT 节点 + 48 个赛场 XML）：

```
ReactiveFallback（优先级从高到低）
├─ 比赛结束 → 回家
└─ Stage4_Combat
   ├─ P1 生存: 血量<200 → 回补给点守护 → 加血后继续
   ├─ P2 战斗: 能量机关触发 → 打能量；前哨站存活 → 打前哨
   └─ P3 巡航: 巡点轮转 + GuardWithAttack（驻守追击）
```

**值得抄的工程细节**：

| 节点/机制 | 文件 | 要点 |
|---|---|---|
| `DoubleCheckValueNode` | `bt_node.hpp` | 双阈值滞回器（如血量 200 触发 / 350 退出 + 上锁标志），防决策在临界值反复横跳 |
| `GuardWithAttackNode` | `guard_with_attack_node.hpp` | 驻守时以敌人为圆心、engage_distance 为半径，水波纹搜索（8 圈 × 0.4m）代价地图上最近可行点追击；无敌人回锚点 |
| `NavNode` | `cruiser_node.hpp` | 直接是 `navigate_to_pose` Action 客户端——与任何 NAV2 栈天然兼容 |
| `PublishNavOutputNode` | `attack_rune_nodes.hpp` | 向 `/carrot_pose` 发 `NavOutput`（mode：0 导航/1 前哨/2 小能量/3 大能量/4 关闭），导航意图广播给云台/自瞄 |
| `RuneStateMachineNode` | `attack_rune_nodes.hpp` | 能量机关状态机：StatefulActionNode，20s 超时失败，被打断安全取消 |
| `hp_spin_watcher` | `refree_subscriber_node.cpp` | 10Hz 血量变化率监测，2s 掉血 >30% 发布 hp_danger_status |
| `BlackboardUpdater` | 同上 | 20Hz 线程拉裁判/雷达/触发点数据写黑板，主线程专注树执行（10Hz tick） |

**对 rm_slam 的启示**：决策层整体移植思路（不必抄代码，接口不同）。我方 sentry_develop 已有
`rm_behavior_tree`（北极熊），HERO 的赛场 XML + 节点设计是"行为树如何打成哨兵战术"的最佳教科书。

### 2.2 hero_mpc_controller —— acados 动力学 MPC（借鉴价值 ★★★★）

- **插件**：nav2 控制器插件 `hero_mpc_controller::HeroMpcController`，`controller_frequency: 100.0`
- **模型**：六状态全向双积分器 `[px, py, ψ, vx, vy, ω]`，控制 `[ax, ay, α]`（质量/惯量归一化）
- **求解**：acados SQP-RTI + PARTIAL_CONDENSING_HPIPM + GAUSS_NEWTON，N=40 / Tf=2.0s / dt=0.05s，每步 1 次 RTI 迭代（实时性）
- **参考轨迹**：不走 nav2 path 接口，订阅 `/smoother_server/minco_polynomial_trajectory`，按当前时间采样含加速度的参考点（微分平坦前馈）

**"工业级改造"亮点**（代码注释原话，实车血泪经验）：

1. **状态截断（蓄力机制）**：MPC 初始速度不用带噪声的里程计速度，改用上一帧 MPC 的最优预测指令——防串扰（屏蔽抖动）、防死锁（积分蓄能逃逸）
2. **指令前瞻抽取**：`get_next_state()` 实际取第 3 步预测（≈0.15s）的速度下发，比直接用 u_0 稳
3. **carrot_pose 发布**：预测点发布到 `/carrot_pose`（SensorDataQoS），供决策/自瞄使用
4. **轨迹超时保护**：参考轨迹过期输出零速

**对 rm_slam 的启示**：移植成本高（外部 acados 依赖 + 代码生成）。**务实建议**：先取"状态截断"与
"指令前瞻"两个思想（不依赖 acados，可移植进任何控制器，包括 OmniPidPursuit）。

### 2.3 pb_minco_smoother —— MINCO 多项式轨迹平滑（借鉴价值 ★★★★，低成本高收益）

- **MINCO** = Minimum Control：浙大 Zhepei Wang gcopter 系。本包用 `MINCO_S3NU`：S=3 五次多项式、最小化 Jerk 能量，2D 轨迹类 `Trajectory<5, 2>`（源码在 `include/gcopter/`，MIT License）
- **插件**：nav2 标准 smoother 插件 `pb_minco/MincoSmoother` —— **smoother_server 挂上即用，BT 加一个 `SmoothPath` 节点**
- **两阶段优化**（自带 L-BFGS）：Stage1 放宽避障/可行性权重快速定型（8000 iters）→ Stage2 严格权重最终优化
- **代价**：`J = w_s·Jerk + w_obs·ESDF避障 + w_feas·速度²/加速度² + w_time·ΣT`
- **ESDF**：OpenCV 从 nav2 Master Costmap **按需快照**（ROI 外扩 `roi_margin=4.0`）计算，之后全内存操作
- **热启动**：上一帧轨迹最近投影点（含加速度，S=3 热启动）作起点边界，超 `trajectory_continuity_threshold` 退化为零速启动
- **零动态内存分配**：梯度缓冲区 initialize() 预分配，costFunction 不 malloc
- **输出**：平滑路径 + `MincoTrajectory` 多项式系数话题（供 MPC 吃）

**使用侧参数**（bringup 实值）：`weight_smooth:1.0 / weight_obstacle:7500.0 / weight_feasibility:100.0 / weight_time:25.0 / weight_mean_time:100.0 / max_iterations:8000 / integral_resolution:32 / max_vel:2.2 / max_acc:3.0 / safe_distance:0.3 / resample_time_resolution:0.65 / rotation_penalty_weight:0.1`

**对 rm_slam 的启示**：**最值得先移植的一块**。rm_slam 现在 A* 锯齿路径直接给 OmniPidPursuit；
挂 MINCO 平滑器（标准 smoother_server 接口）即可显著改善路径质量，不动控制器、风险最小。

### 2.4 dog_map + fast_layer —— ROG-Map 式点云感知后端（借鉴价值 ★★★）

**架构思想：把费时分析从代价地图里剥离到异步后端，图层退化成"零开销直投"。**

```
dog_map（异步节点，非 nav2 插件）:
  3D 体素 log-odds 占据栅格（XY 分辨率 0.025、z 0.01~0.03、半图 10m）
  + Amanatides-Woo 3D-DDA 射线步进清空
  + 时间遗忘机制（FORGET_FACTOR -30，动态物体自动消退）
  + 双雷达 log-odds 分设备（LOG_OCC_HIT_mid360:34 / odin1:16 / FREE:-20，THR_OCC:30）
  + StaticFixMap：先验 pgm 静态地图 + 当前观测 2D 射线修正 → rog_map/fix
  发布：rog_map/inf_occ（占据）、rog_map/ground（地面）、rog_map/fix（静态修正）
      ↓
fast_layer（代价图层插件，~120 行）:
  订阅 rog_map/inf_occ → 最新 TF（TimePointZero 不等时延）→ worldToMap
  → 10 帧缓冲 → updateCosts() 批量 setCost(LETHAL_OBSTACLE) → 清空
```

**对 rm_slam 的启示**：rm_slam 现用 linefit 地面分割 → pointcloud_to_laserscan → obstacle_layer/STVL
流派；dog_map+fast_layer 是**下一代动态障碍感知路线**（log-odds 鲁棒性 + 动态遗忘），中期值得架构升级参考。

### 2.5 perception —— 双雷达与视觉融合（借鉴价值 ★★★）

| 包 | 作用 | 要点 |
|---|---|---|
| `hero_lidar_scan` | 点云去畸变 + 稠密全景 | 按每点时间戳 + 高频里程计（100-500Hz，best_effort）SLERP 插值去畸变；2.5ms 位姿桶 + TBB 并行；双线程模型（回调只入队，workerLoop 计算） |
| `lidar_merge` | 双雷达融合 | message_filters ApproximateTime 同步，6-DOF 外参变换到 Odin1 基准，按 offset_time 归并排序 |
| `nav_cv_bridge` | 雷达+视觉融合 | 视觉自瞄目标（/tracker/autoaim）与雷达目标（/tracker/radar）双端融合：5.5m/4.5m 滞回切换 + 1s 双超时删除 + 强切回退；**工程豁免区**（id=="2" 且 HSV 红色区 → 发 MaskID 防打工程） |
| `zenoh_bridge.json5` | 双机通信 | 导航机 ↔ 自瞄机网线直连：pub `/carrot_pose`,`/MaskID`,`/gimbal_decision`；sub `/tracker/autoaim`；PTP 时间戳硬同步 |

**对 rm_slam 的启示**：sentry_develop 已有 `nn_auto_aim`（自瞄）——`nav_cv_bridge` 的双端融合 +
豁免区设计是接实车自瞄的现成模板；zenoh 是双机（导航机/自瞄机）架构的事实标准。

### 2.6 scripts/lidar_extrinsic_calibration.py —— 云台雷达标定（借鉴价值 ★★★★★，立即可用）

**原理**：底盘静止、云台转 1-3 圈，录 FAST-LIVO `/aft_mapped_to_init` bag，
最小二乘拟合圆形轨迹 → 输出：

- **旋转半径 R**（喂 LIVMapper `t_lidar_joint` 的 x 分量）
- **安装倾角 Pitch**（喂 `angle_y` 参数）
- **拟合质量 RMSE**（<5mm 优秀）

**验证**：标定后轨迹收敛成静止点。
**依赖**：rosbags / numpy / scipy / matplotlib（无导航耦合）。
**用法**：`python3 lidar_extrinsic_calibration.py /path/to/bag -t /custom/odom_topic [-o result.png] [--no-plot]`

**对 rm_slam 的启示**：**全仓库最"零门槛"的宝贝**——rm_slam 实车雷达同样装在云台上，
雷达外参没标定则建图/定位全歪。此脚本抄过来即可用。

---

## 3. 与 rm_slam 差距矩阵

| 能力 | rm_slam 现状 | HERO 方案 | 借鉴成本 |
|---|---|---|---|
| 自主决策 | ❌ 无（只有转发器 navigation_data_processer） | 行为树 + 48 个赛场 XML | 中（参考 rm_behavior_tree 融合） |
| 轨迹平滑 | ❌ 无 / SimpleSmoother | MINCO 多项式 + ESDF | **低（nav2 标准插件接口）** |
| 控制器 | OmniPidPursuit（自研全向 PID） | 100Hz acados MPC | 高（需 acados 依赖） |
| 动态障碍后端 | linefit 地面分割 + obstacle_layer/STVL | ROG-Map log-odds + 直投层 | 中（架构升级） |
| 云台雷达标定 | ❌ 无 | 圆轨迹拟合脚本 | **极低（抄脚本）** |
| 双机通信/自瞄融合 | ❌（未接自瞄） | zenoh + PTP + nav_cv_bridge | 中 |
| 裁判/串口协议 | 有 navigation_interfaces（简） | serial_interfaces 全套（Refree/Judgment 3.8KB） | 按需 |

---

## 4. 推荐借鉴路线图（按性价比排序）

| 优先级 | 借鉴项 | 收益 | 动作 |
|---|---|---|---|
| 🥇 立即 | `lidar_extrinsic_calibration.py` 云台标定 | 实车定位精度 | 抄脚本 → 实车标一次 |
| 🥇 立即 | MINCO 平滑器挂进 rm_slam | 路径质量质变 | smoother_server 挂 `pb_minco/MincoSmoother` + BT 加 SmoothPath（标准接口，风险最小） |
| 🥈 短期 | 决策层行为树（HERO XML + rm_behavior_tree 融合） | 哨兵从"会走"变"会打" | 研读 `decision/bt/`，重写成本地接口 |
| 🥈 短期 | MPC 的"状态截断/指令前瞻"思想进 OmniPidPursuit | 控制稳定性 | 读 `hero_mpc_controller.cpp` 注释，取思想不取依赖 |
| 🥉 中期 | dog_map + fast_layer 架构升级感知 | 动态障碍鲁棒性 | 参考 `occ_map`（ROG-Map）重写 |
| 🥉 中期 | zenoh 双机 + nav_cv_bridge | 接自瞄/哨兵协同 | 结合我方 nn_auto_aim |

---

## 5. 坑与警告（开源版不完整处）

1. **直接编译会失败**：
   - `thread_node.hpp:23` 引用缺失的 `feasibility_calculate_node.hpp`（git ls-files 亦无）
   - BT main 注册的 `PointInQuadrilateralCondition` 定义缺失
   - CMakeLists 硬编码 include `/home/dji/projects/hero2025_sentinel_ws/...`
2. **分支代码未编译**：`lidar_merge` 默认编译旧版单雷达 `src/lidars.cpp`（`#ifdef TWO_LIDARS` 分支的 `merge.cpp` 与 `cfg_odin.yaml` 未纳入构建）
3. **硬编码路径**：`nav_cv_bridge` 地图路径写死 `/home/dji/hero2026_-sentry/...`；bringup `params/` 目录缺失（launch 引用与 install 不一致）
4. **重依赖**：acados（第三方编译 + 代码生成）、BehaviorTree.CPP 4.x 源码随仓库提交（体积大）
5. **接口不兼容**：`interfaces/*.msg`（MincoTrajectory、NavOutput、TrackerOutput 等）与 rm_slam 的 `navigation_interfaces` 不兼容，移植消息层需改字段
6. **典型赛场配置**：48 个 XML 是 RMUC 赛场实测树（含点位坐标），需按自队战术重写

---

## 6. 关键文件路径速查

| 组件 | 路径 |
|---|---|
| BT 入口 | `decision/bt/src/refree_subscriber_node.cpp` |
| BT 自定义节点 | `decision/bt/include/{bt_node,behavior_node,cruiser_node,attack_rune_nodes,guard_with_attack_node}.hpp` |
| 赛场 XML 库 | `decision/bt/config/xml/`（48 个） |
| MPC 控制器 | `planning/hero_mpc_controller/src/hero_mpc_controller.cpp`（状态截断在步骤 3） |
| MPC 模型生成 | `planning/hero_mpc_controller/model/omnidirectional_dynamic_tracking.py` |
| MINCO 平滑 | `planning/pb_minco_smoother/src/minco_smoother.cpp`（gcopter 在 `include/gcopter/`） |
| 点云后端 | `planning/dog_map/src/{occ_map.cpp,node.cpp}` |
| 直投图层 | `planning/fast_layer/src/fast_Layer.cpp` |
| bringup | `planning/hero2025_nav_bringup/launch/bringup_launch.py` + `config/nav2_params.yaml` |
| 去畸变 | `perception/hero_lidar_scan/src/hero_lidar_scan_node.cpp` |
| 双雷达融合 | `perception/lidar_merge/src/{lidars.cpp,merge.cpp}` |
| 雷达/视觉桥 | `perception/nav_cv_bridge/src/autoaim_tracker.cpp` |
| 双机通信 | `perception/zenoh_bridge.json5` |
| 云台标定 | `scripts/lidar_extrinsic_calibration.py` + `scripts/README_calibration.md` |
| 自审文档 | `docs/full_stack_analysis.md`（1025 行团队完整自审报告，含 MPC/MINCO/DogMap 算法详解与 10 条改进建议） |

> **一句话总结**：HERO 开源把"哨兵导航栈"从"会定位会走"提升到"会决策、会打能量机关、会驻守追击、
> 会用 MPC 飙到 100Hz 的工业级控制"。对 rm_slam 最现实的三步走：
> **先抄标定脚本（当天见效）→ 挂 MINCO 平滑（一周见效）→ 研读决策层（决定哨兵的灵魂）**。