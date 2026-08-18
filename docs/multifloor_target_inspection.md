# 多楼层目标巡检（Plan B 实现状态）

> 状态：**Step 1/2/3 核心已实现并验证**（2026-08-11/12）
> 验证：分段式 7 目标多层巡检 38s 完成，0 崩溃；**分层探索 3 层完成，0 崩溃**

## 目标

机器狗在**多层楼建筑**（SCAN `map.pcd`，5 层）中对**指定目标点**进行巡检（到达 → 检查）。系统在楼层间通过**坡道**移动（z 变化），按目标清单访问各层目标。不做覆盖探索 → 无绕圈。

## 分层架构（已实现）

```
[任务层] task_node（dog_bridge 包）
  读 config/targets.yaml 目标清单 [{floor,x,y,z,yaw}, ...]
  逐个发 /plan_b/target，收到 /plan_b/reached 后发下一个
        ↓ PoseStamped 目标
[决策层] RACER exploration_node（goal 模式）
  goalCallback：/plan_b/target → next_pos
  callExplorationPlanner：goal_active_ 时 planTrajToView(→目标)（A* + B样条，不选前沿）
  到达检测：odom 3D 距离 < 0.8m → /plan_b/reached → WAIT_TRIGGER
  multi_floor：放开 body_height 钳制，A* 自由 z
        ↓ /planning/bspline_1（3D）
[桥接层] dog_bridge_node
  平层：bspline → /initial_path（z 钳到 body_height）
  多层：bspline → /initial_path（z 透传 3D）+ 转 scan_planner_msgs Bspline → /planning/bspline
        ↓ 3D 路径 / 3D Bspline
[执行层] open_loop_controller（SCAN 包）
  订阅 /planning/bspline，评估 3D B 样条，发布带 z 的 body_pose
  （替代 closed_loop + go2_kinematic_sim：后者只积分 x/y，无法爬坡）
```

## 关键改动清单

### RACER-ROS2
| 文件 | 改动 |
|------|------|
| `exploration_manager/src/fast_exploration_fsm.cpp/.h` | goal 模式：`/plan_b/target` 订阅、`goalCallback`、`callExplorationPlanner` goal 分支、EXEC_TRAJ 3D 到达检测（odom）、WAIT_TRIGGER 挂起目标兜底、triggerCallback 屏蔽 |
| `exploration_manager/src/fast_exploration_manager.cpp/.h` | `multi_floor_` 参数（放开 body_height 钳制）；goal 模式始终乐观 A*（目标在未建图区可达） |
| `bspline_opt/src/bspline_optimizer.cpp` | NLopt 无解兜底（`best_variable_` 空 → 保留输入控制点，防空指针段错误） |

### SCAN-Planner-Ros2
| 文件 | 改动 |
|------|------|
| `local_sensing/src/pointcloud_render_node.cpp` | **SIGFPE 修复**：`transformPointCloud` 前守卫空 `local_map_filled`（空云 `point_count/width` = `0/0` 除零崩溃） |
| `plan_manage/src/open_loop_controller.cpp` | （原版，未改）3D B 样条跟随，z 精确 |

### dog_bridge（dog_explor 工程）
| 文件 | 改动 |
|------|------|
| `src/task_node.cpp` | 目标巡检驱动：yaml 目标清单、楼层标签、逐个发布、到达推进、超时跳过、完成后停止 |
| `src/dog_bridge_node.cpp` | goal 模式禁用探索触发/清扫；multi_floor 透传 3D z；转发 RACER Bspline → open_loop |
| `launch/plan_b.launch.py` | `goal_mode`/`multi_floor` 参数；map.pcd 选择；open_loop_controller 条件节点；RACER/SCAN/桥接多层参数 |
| `config/targets.yaml` | 目标清单（含 floor 标签） |

## 如何运行

```bash
source /opt/ros/humble/setup.bash
source ~/dog_explor/install/setup.sh
source ~/dog_racer/RACER-ROS2/install/setup.sh
source ~/dog_racer/SCAN-Planner-Ros2/install/setup.sh

# 平层目标巡检（默认开 RViz）
ros2 launch dog_bridge plan_b.launch.py goal_mode:=true

# 多层目标巡检（map.pcd + open_loop，默认开 RViz）
ros2 launch dog_bridge plan_b.launch.py goal_mode:=true multi_floor:=true

# 分层探索（RACER 每层探索，覆盖率/时间触发爬楼，默认开 RViz）
ros2 launch dog_bridge plan_b.launch.py multi_floor:=true layered:=true
# 分层配置：src/dog_bridge/config/layered_targets.yaml（覆盖率阈值/爬楼目标/探索时长）
```

目标点编辑：`src/dog_bridge/config/targets.yaml` 后 `colcon build --packages-select dog_bridge`。
实时发任意目标：`ros2 topic pub -r 2 /plan_b/target geometry_msgs/msg/PoseStamped "{header: {frame_id: world}, pose: {position: {x: .., y: .., z: ..}, orientation: {w: 1.0}}}"`

## 验证结果

### 平层（Step 1）
- 3 目标巡检 47s 完成：`(3,2) → (-3,2) → (0,-3)` 全部到达，0 崩溃，0 绕圈
- 桥接轨迹连续跟随（bspline → /initial_path → SCAN）

### 多层（Step 2 + Step 3）
- **分段式 7 目标**（map.pcd，L0/L1/L2）：38s 完成，0 崩溃，0 超时跳过
- **分层探索**（RACER 每层探索 + 覆盖率/时间触发爬楼）：3 层完成，0 崩溃
  - L0 探索（锁 z~0.5，90s）→ 爬西坡(x=-5.5) → L1（锁 z~1.8）
  - L1 探索 → 爬东侧(x≈9) → L2（锁 z~3.5）
  - L2 探索 → ALL 3 FLOORS EXPLORED
- 狗 z：0.5 → 1.8 → 3.5，终点在 L2
- 地图持续建（renderer 修复后覆盖率增长）

### 关键 bug 修复（本会话）
1. **RACER BsplineOptimizer 段错误**：NLopt 5ms 内 0 次求值 → 空 `best_variable_` → 空指针。兜底修复。
2. **目标丢失（INIT 竞态）**：task_node 发目标时 RACER 还在 INIT。WAIT_TRIGGER 兜底。
3. **远目标 No path 卡死**：目标在未建图区，乐观额度用完。goal 模式始终乐观。
4. **task_node 完成后死循环**：列表耗尽后 OOB 发目标。done_ 标志。
5. **pcl_render_node SIGFPE**：`transformPointCloud` 对空云 `0/0`。守卫空云。

## 遗留 / 下一步

- **导航质量**：乐观 A* 会穿未建图楼板（分段式缓解但不根治）。修法：让激光更好建出楼板（vertical FOV 已 90°，但楼板被墙遮挡时仍未知），或"先建图后目标"。
- **巡检语义**：目前"到达即算"。多角度观测（绕目标看一圈）待实现（用户确认后）。
- **东侧通道结构**：x≈9 的 L1→L2→L3 连接较复杂，分段式可过；3 层以上路线需继续验证。
- **SCAN 局部规划**：多层时被旁路（rebound A* 无法规划坡道）。若需 SCAN 主动避障，需修它的局部规划。
