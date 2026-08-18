# Plan B 交接文档（机器狗多层探索/巡检）

> **注意：此文档为旧版交接。2026-08-17 之后请优先查看
> [`CHECKPOINT_2026-08-17.md`](CHECKPOINT_2026-08-17.md)。**

> 日期：2026-08-12
> 状态：**核心架构已实现**，可靠演示 = 手动航点 + 建图；自主方案（RACER 分层探索）存在未解问题
> 架构：RACER（决策/规划） + SCAN（执行/建图） + dog_bridge（桥接） + task_node（任务）

---

## 1. 项目目标

机器狗（GO2，四足）在**多层楼建筑**（`map.pcd`，5 层，L0~L4）中做**目标巡检/探索建图**。
- 巡检：到达指定目标点（不最大化覆盖 → 无绕圈）
- 探索：逐层建图，经坡道/楼梯在楼层间移动（z 变化）
- 关键约束：**不能"飞"**（不能穿未建图的楼板），要沿真实坡道/楼梯爬

地图：`/home/sigma/dog_racer/SCAN-Planner-Ros2/map.pcd`（5 层建筑）
- 西侧 x=-5.5 坡道：L0→L1（z 0.1→1.55）
- 东侧 x≈9 通道：L1→L2→L3（楼梯井）
- 楼板高度：L0=0.1, L1=1.5, L2=3.1, L3=4.5, L4=6.0（狗身体 = 楼板+0.4）

---

## 2. 架构分层

```
[任务层] task_node（dog_bridge 包）
  目标清单 yaml / 分层探索编排（覆盖率/时间触发爬楼）
        ↓ /plan_b/target 或 /plan_b/climb_path
[决策层] RACER exploration_node
  goal 模式：planTrajToView → 目标（A* + B样条）
  探索模式：前沿（z 可锁到 explore_height）
        ↓ /planning/bspline_1
[桥接层] dog_bridge_node
  平层：bspline → /initial_path（z 钳 body_height）
  多层：bspline → /initial_path（z 透传 3D）+ 转 scan_planner_msgs → /planning/bspline
  手动航点：publish_path=false 时只中继 cloud/odom（RACER 建图）
        ↓ 3D 路径 / 3D Bspline
[执行层] open_loop_controller（SCAN 包，已改）
  订阅 /planning/bspline（RACER/SCAN 轨迹）精确跟随（z 精确）
  或订阅 /initial_path（手动航点）直接跟随（enable_path_follow）
[建图] pcl_render_node（激光渲染）+ cloud_accumulator（累积地图，RViz 显示）
```

---

## 3. 各阶段做了什么 + 验证结果

### Step 1：平层目标巡检（goal 模式）✅ 已验证
- RACER goal 模式：`/plan_b/target` → `planTrajToView` 直接规划到目标（不走前沿）
- task_node 逐个发目标，到达检测（odom 3D 距离 <0.8m）→ 下一目标
- **验证**：3 目标 `(3,2)→(-3,2)→(0,-3)` 47s 完成，0 崩溃，0 绕圈
- 运行：`ros2 launch dog_bridge plan_b.launch.py goal_mode:=true`

### Step 2：多层目标巡检（分段爬楼）✅ 已验证
- multi_floor 参数：放开 body_height 钳制、box 扩到整栋楼、到达检测 3D
- open_loop_controller 精确跟随 3D B样条（替代 closed_loop+仿真，后者 z 固定）
- **验证**：7 目标（爬西坡 3 跳→L1 横穿 3 跳→L2）38s 完成，0 崩溃
- 运行：`ros2 launch dog_bridge plan_b.launch.py goal_mode:=true multi_floor:=true`
- 配置：`config/targets.yaml`（分段 7 目标）

### Step 3：分层探索（用户提出）⚠️ 部分实现，有问题
- 逻辑：**手动发布楼层高度 → RACER 锁 z 探索建图 → 一层探索完毕 → 手动引导爬楼到下一层 → 重复**
- 已实现：RACER `interrupt_goal`（探索中可被打断）、`explore_height_` 锁探索 z、task_node 分层编排（覆盖率/时间触发）、`/plan_b/floor_height` 话题
- **验证**：3 层跑通（L0→L1→L2，ALL 3 FLOORS EXPLORED），0 崩溃
- **问题**：探索 z 锁 OK（不飞），但**爬楼（goal 模式）的 A\* 会穿未建图楼板**（L1→L2 单跳飞到 z~6）
- 配置：`config/layered_targets.yaml`

### 最终可靠演示：手动航点 + 建图 ✅ 已验证（推荐）
- 手写 3D 航点路线（`config/reference_path_multifloor.yaml`），open_loop 直接跟随（z 精确，不飞）
- cloud_accumulator 累积激光点云成地图，RViz 显示增长
- **验证**：西坡→L1→东侧→L2，z 0.5→3.5 精确，0 穿地板，0 崩溃
- 运行：`ros2 launch dog_bridge scan_manual.launch.py rviz:=true`
- 演示视频：`/tmp/dog_multifloor_map.mp4`

---

## 4. 碰到的问题 + 根因 + 状态

| # | 问题 | 根因 | 状态 |
|---|------|------|------|
| 1 | RACER 收到目标即段错误 | `BsplineOptimizer::optimize` NLopt 5ms 内 0 次求值 → `best_variable_` 空 → 拷贝读空指针 | ✅ 已修（兜底保留输入控制点） |
| 2 | 首个目标丢失 | task_node 发目标时 RACER 还在 INIT | ✅ 已修（WAIT_TRIGGER 检查挂起目标） |
| 3 | 远目标 "No path" 卡死 | 目标在未建图区，乐观规划额度用完 | ✅ 已修（goal 模式始终乐观 A*） |
| 4 | 完成后死循环发垃圾目标 | task_node 列表耗尽后 OOB | ✅ 已修（done_ 标志） |
| 5 | **狗"平地起飞"/穿地板** | **乐观 A\* 把未建图楼板当自由空间**，竖直路径穿楼 | ⚠️ **核心未解问题**（见下） |
| 6 | RACER 间歇 malloc 堆损坏 | 大 box 初始化/线程竞态 | ⚠️ 未修（加 respawn 兜底） |
| 7 | SCAN 局部规划无法爬坡 | rebound A* 在坡道失败（A-star error 刷屏） | ⚠️ 绕开（多层用 open_loop 直接跟） |
| 8 | task_node 时间源崩溃 | `rclcpp::Time()` 默认 RCL_ROS_TIME vs `now()` RCL_SYSTEM_TIME | ✅ 已修（用 now() 初始化时间成员） |
| 9 | open_loop 误跟随 bridge 的 /initial_path | open_loop 新路径跟随把 RACER 轨迹当手动航点（z 双加） | ✅ 已修（enable_path_follow 参数门控） |
| 10 | RACER 启动 malloc 崩溃（加 z 带/竖直约束后） | frontier z 带/竖直约束触发线程竞态（疑似） | ⚠️ 已回退这两个改动 |

### 核心未解问题 #5：乐观 A\* 穿楼板

**现象**：RACER 的 A\*（goal 模式爬楼时）把未建图的楼板/上层当自由空间，规划竖直路径，狗"飞"穿楼层。

**尝试过的修复**：
1. **竖直穿越需已知空间**约束（A* 邻居扩展：dz≠0 要求已知自由格）→ 能防飞，但**加了之后 RACER 变确定性 malloc 崩溃**（疑似线程竞态被触发），回退了
2. **frontier z 带**（探索只检测本层前沿）→ 探索更有效，但同样伴随 malloc 崩溃，回退
3. **floor_height 锁探索 z**（`explore_height_`）→ 探索期不飞（有效！），但爬楼（goal 模式）仍穿
4. **分段爬楼**（把 L1→L2 单跳拆成多跳）→ 缓解但不根治（A* 仍会下穿）

**当前结论**：让 RACER **自主规划爬楼**不可靠。可靠做法是**手动航点**（open_loop 直接跟随，不经过 A*）。

**建议**：若要 RACER 自主爬楼，需修 RACER 的 malloc 竞态（很难），或想别的约束方式。否则用手动航点爬楼 + RACER 只做探索建图。

---

## 5. 代码改动清单

### RACER-ROS2（/home/sigma/dog_racer/RACER-ROS2）
| 文件 | 改动 |
|------|------|
| `exploration_manager/src/fast_exploration_fsm.cpp/.h` | goal 模式（target 订阅/goalCallback/到达检测/resume）；interrupt_goal（探索中可被打断爬楼） |
| `exploration_manager/src/fast_exploration_manager.cpp/.h` | multi_floor/explore_height_（锁探索 z）/goal_plan_/floor_height 订阅 |
| `bspline_opt/src/bspline_optimizer.cpp` | NLopt 无解兜底 |
| `path_searching/src/astar.cpp` | （竖直约束已回退，当前无改动） |
| `plan_env/src/map_ros.cpp/.h` | `/map_ros/coverage_floor`（per-floor 覆盖率） |

### SCAN-Planner-Ros2（/home/sigma/dog_racer/SCAN-Planner-Ros2）
| 文件 | 改动 |
|------|------|
| `plan_manage/src/open_loop_controller.cpp` | **手动航点直接跟随**（enable_path_follow + /initial_path 路径跟随，z 加 body_height）；原 3D B样条跟随保留 |
| `local_sensing/src/pointcloud_render_node.cpp` | SIGFPE 修复（空云守卫） |

### dog_explor（/home/sigma/dog_explor）
| 文件 | 改动 |
|------|------|
| `src/dog_bridge_node.cpp` | goal/multi_floor/publish_path 参数；NaN 守卫；bspline 转发 open_loop；bspline 超时重触发 |
| `src/task_node.cpp` | goal 模式 + 分层模式（覆盖率/时间触发爬楼 + floor_height 发布） |
| `src/cloud_accumulator.cpp` | **新增**：累积激光点云成地图（RViz 显示建图） |
| `launch/plan_b.launch.py` | goal_mode/multi_floor/layered 参数；open_loop 条件节点；地图选择 |
| `launch/scan_manual.launch.py` | **新增**：手动航点 demo（SCAN 栈 + 手动路径 + 累积地图） |
| `launch/plan_b.rviz` | 加 Accumulated Map 显示 |
| `config/targets.yaml` | Step2 分段 7 目标 |
| `config/layered_targets.yaml` | Step3 分层配置（floor_height/climb_groups） |
| `config/reference_path_multifloor.yaml` | **新增**：手动 3D 航点路线 |

---

## 6. 如何运行（3 个 demo）

```bash
# 环境
source /opt/ros/humble/setup.bash
source ~/dog_explor/install/setup.sh
source ~/dog_racer/RACER-ROS2/install/setup.sh
source ~/dog_racer/SCAN-Planner-Ros2/install/setup.sh

# 1. 平层目标巡检（goal 模式）
ros2 launch dog_bridge plan_b.launch.py goal_mode:=true

# 2. 多层目标巡检（分段爬楼）
ros2 launch dog_bridge plan_b.launch.py goal_mode:=true multi_floor:=true

# 3. 手动航点 + 建图（最可靠，推荐演示）
ros2 launch dog_bridge scan_manual.launch.py rviz:=true

# 4. 分层探索（有爬楼穿板问题，不推荐演示）
ros2 launch dog_bridge plan_b.launch.py multi_floor:=true layered:=true
```

---

## 7. 遗留问题 / 下一步

1. **RACER 自主爬楼穿楼板**（#5）：建议用手动航点爬楼 + RACER 只做探索建图，或修 RACER 竞态
2. **RACER 间歇 malloc 崩溃**（#6）：已加 respawn，但重启丢地图；彻底修需抓竞态
3. **SCAN 局部规划无法爬坡**（#7）：多层被旁路（open_loop 直接跟），如需 SCAN 主动避障需另修
4. **巡检语义**："到达即算"（多角度观测未实现）
5. **3 层以上**：东侧 x≈9 通道 L1→L2 已通，L2→L3+ 未验证（手动航点可扩展）
6. **手动航点路线**：`reference_path_multifloor.yaml` 目前到 L2，可加更多层航点

---

## 8. 关键文件位置

- 设计文档：`docs/multifloor_target_inspection.md`
- 本交接：`docs/PlanB_handover.md`
- 演示视频：`/tmp/dog_multifloor_map.mp4`
- 日志：`/tmp/plan_b_*.log`（各阶段运行日志）
