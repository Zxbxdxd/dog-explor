# dog-explor

基于 ROS 2 Humble 的机器狗多层楼自主探索与建图编排工作空间。

系统使用 RACER 完成每层的覆盖探索与避障规划；达到本层覆盖率后，RACER 先规划机器狗从探索终点到楼梯入口，随后由预设直线坡道轨迹完成爬楼。落地后锁定新楼层高度并重新启动该层探索，循环至四层全部完成。

## 主要功能

- 四层独立 RACER 往复式覆盖探索
- 逐条激光射线覆盖统计，考虑墙体遮挡
- 占据栅格与柱体避障
- 轨迹执行到约 75% 时提前重规划，并继承当前速度和加速度
- RACER 避障规划至楼梯入口
- 预设直线坡道轨迹爬楼，避免三维规划导致悬空飞行
- 多层累计点云和 RViz 可视化

## 依赖

- Ubuntu 22.04
- ROS 2 Humble
- `/home/sigma/dog_racer/RACER-ROS2`
- `/home/sigma/dog_racer/SCAN-Planner-Ros2`

RACER 与 SCAN 工作空间包含本项目配套修改，详细文件索引见 [`docs/CHECKPOINT_2026-08-17.md`](docs/CHECKPOINT_2026-08-17.md)。

## 构建

```bash
source /opt/ros/humble/setup.bash
source /home/sigma/dog_racer/RACER-ROS2/install/setup.bash
source /home/sigma/dog_racer/SCAN-Planner-Ros2/install/setup.bash

cd /home/sigma/dog_explor
colcon build --symlink-install
source install/setup.bash
```

## 正式运行

所有测试和演示均启用 RViz。正式覆盖率阈值为 85%，必须连续保持 5 秒才允许切换楼层：

```bash
source /opt/ros/humble/setup.bash
source /home/sigma/dog_racer/RACER-ROS2/install/setup.bash
source /home/sigma/dog_racer/SCAN-Planner-Ros2/install/setup.bash
source /home/sigma/dog_explor/install/setup.bash

ros2 launch dog_bridge plan_b.launch.py \
  rviz:=true coverage_metrics:=true \
  multi_floor:=true layered:=true \
  coverage_threshold:=85.0 coverage_hold_sec:=5.0 \
  explore_time_sec:=360.0
```

`coverage_threshold:=20.0` 仅用于加速跨层回归，不得用于正式演示或验收。

## 目录

- `src/dog_bridge/src/`：桥接、任务状态机和累计点云节点
- `src/dog_bridge/launch/`：一体化启动与 RViz 配置
- `src/dog_bridge/config/`：多层目标、覆盖路径和爬楼轨迹
- `docs/`：设计、检查点和验证记录
