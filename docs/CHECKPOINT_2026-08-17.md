# 多层探索项目检查点（2026-08-17）

> 本文件是当前最新进度存档，优先级高于 `PlanB_handover.md`。
> 当前状态：所有 ROS/RViz 测试已停止；代码和已构建产物保留。

## 1. 本轮完成内容

- 将分层探索改为确定性单块往复式扫掠，避免前沿目标在近似等价候选间反复切换。
- 扫掠模式不再启动无用的 LKH/前沿更新，也不会被“前沿已覆盖”的 3 秒重规划条件截断。
- 每次扫掠分段重规划都从实时 odom 开始，避免旧 B 样条超出有效时域后的外推状态参与下一段规划。
- 扫掠 A* 始终允许规划到尚未观测区域；普通前沿模式行为不变。
- 新楼层到达后重置扫掠索引，首个目标使用新楼层精确高度。
- 新增 `/plan_b/manual_climb` 暂停/恢复握手：爬楼期间 RACER 进入 IDLE，丢弃旧轨迹；落地后才恢复。
- SCAN open-loop 在爬楼时忽略 RACER B 样条，爬楼结束后等待从真实落点开始的新 B 样条，防止回跳旧楼层。
- 爬楼完成判定收紧为平面误差 `<0.25 m`、高度误差 `<0.08 m`；实测三次均为 `dist=0.00, dz=0.00`。
- 楼层覆盖率改为“本层落地后新产生的 3 m 激光巡检足迹”，楼层变化立即清零，消除预扫描上层导致连续跳层的问题。
- `plan_b.launch.py` 默认正式覆盖阈值设为 85%，分层模式启用 360° 激光视场，并开放覆盖阈值/保持时间/最长探索时间参数。

## 2. 已验证结果

### 正式 85% 单层与首次跨层（RViz）

日志：`/tmp/dog_explor_rviz_85_2.log`（临时目录，重启后可能丢失）

- 第一层覆盖从 9.7% 增长到 85.2%，随后达到 90.9%。
- 覆盖触发后 RACER 正确暂停，手工路线正常爬楼。
- 第一层到第二层精确落地：`dist=0.00, dz=0.00`。
- 二层高度锁定为 3.30 m，扫掠状态重置并产生二层首个目标。
- 该测试暴露了旧覆盖统计继承问题，随后已改为独立巡检足迹；修改后的 85% 全流程尚未重跑。

### 四层加速跨层回归（RViz，阈值 50%）

日志：`/tmp/dog_explor_rviz_reset_50_2.log`（临时目录，重启后可能丢失）

- 完整输出：`ALL 4 FLOORS EXPLORED (final floor coverage 52.0% reached)`。
- 三次爬楼全部精确落地，三次均记录 `dist=0.00, dz=0.00`。
- 二、三、四层落地后的初始覆盖分别约为 6.6%、6.6%、6.5%，证明覆盖率逐层独立清零。
- 四个楼层均重新执行 RACER 扫掠，没有出现落地后立即触发下一次爬楼。
- 未发现 `process has died`。

## 3. 当前改动文件

工作空间 `/home/sigma/dog_explor`：

- `src/dog_bridge/src/task_node.cpp`
- `src/dog_bridge/src/dog_bridge_node.cpp`
- `src/dog_bridge/launch/plan_b.launch.py`

参考工作区 `/home/sigma/dog_racer/RACER-ROS2`：

- `src/exploration_manager/include/exploration_manager/fast_exploration_manager.h`
- `src/exploration_manager/include/exploration_manager/fast_exploration_fsm.h`
- `src/exploration_manager/src/fast_exploration_manager.cpp`
- `src/exploration_manager/src/fast_exploration_fsm.cpp`
- `src/plan_env/include/plan_env/map_ros.h`
- `src/plan_env/src/map_ros.cpp`

参考工作区 `/home/sigma/dog_racer/SCAN-Planner-Ros2`：

- `src/planner/plan_manage/src/open_loop_controller.cpp`

以上文件的 SHA-256 已在本轮终端输出中记录；由于当前目录没有可用 Git 元数据，本检查点是主要变更索引。

## 4. 构建状态

- `plan_env`：构建、安装成功。
- `exploration_manager` / `exploration_node`：构建、安装成功。
- `scan_planner`：构建、安装成功。
- `dog_bridge/task_node`：构建成功；安装目录为 symlink-install，可直接使用当前源码和构建产物。
- 编译仅有既存的 signed/unsigned、unused variable 等警告，无新增编译错误。

## 5. 下一次恢复步骤

所有后续测试必须按用户要求启用 RViz：`rviz:=true`。

```bash
source /opt/ros/humble/setup.bash
source /home/sigma/dog_racer/RACER-ROS2/install/setup.bash
source /home/sigma/dog_racer/SCAN-Planner-Ros2/install/setup.bash
source /home/sigma/dog_explor/install/setup.bash

ros2 launch dog_bridge plan_b.launch.py \
  multi_floor:=true layered:=true rviz:=true \
  coverage_threshold:=85.0 coverage_hold_sec:=5.0 explore_time_sec:=360.0
```

正式验收重点：

1. 用新的独立覆盖统计完整跑四层 85%（本轮只完成了 50% 四层回归）。
2. 每层确认覆盖从约 6%~9% 开始，并达到至少 85%。
3. 观察长 B 样条是否仍存在不必要的弯绕；状态机已修复为每段从 odom 重规划，但仿真轨迹仍可能受优化器形状影响。
4. 四层结束后，当前 `task_node` 只停止任务编排，RACER 仍可能继续执行已有扫掠轨迹；如需要完成即停车，应新增最终暂停消息。
5. 仿真通过后再上真机校准 `floor_coverage_radius`、爬楼速度和落地容差。

## 6. 停止状态

2026-08-17 最后一轮 RViz/ROS launch 已通过 Ctrl-C 正常停止。检查时未发现残留的 `rviz2`、`exploration_node`、`task_node` 或 `open_loop_controller` 进程。
