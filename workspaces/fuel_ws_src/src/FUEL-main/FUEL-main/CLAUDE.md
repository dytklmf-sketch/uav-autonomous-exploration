# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 语言与规则

- 必须使用中文与用户沟通。
- 在本仓库中编写或修改代码时，新增的代码必须带有中文注释；注释应简洁，重点说明关键逻辑、状态机转换、参数含义或易错点。
- 本仓库的 `CLAUDE.md` 以中文为主；后续在本仓库中工作时，优先使用中文回复。
- 这是一个 ROS catkin 工作区下的研究型无人机规划/探索项目，修改时优先沿用现有参数驱动、launch 组装、FSM 调度的实现方式，不要随意引入新的框架层。
- 本仓库存在较多通过 launch/XML 参数切换行为的模块；排查问题时先看 launch 文件里的参数和 remap，再看 C++ 实现。
- `uav_simulator/Utils/*/build`、`devel` 等目录下有已生成产物痕迹；搜索代码时优先看源文件，不要把生成文件当作真实实现入口。
- `fuel_planner/exploration_manager/src/fast_exploration_manager.cpp` 和 `fuel_planner/plan_env/src/map_ros.cpp` 含有中文调试输出与额外可视化逻辑；改动相关功能时不要误删这些项目内已有的诊断代码，除非任务明确要求清理。

## 构建与运行命令

本仓库是 catkin 工作区源码目录中的一个工程，通常在工作区根目录执行以下命令，而不是在当前目录直接编译。

### 安装依赖

README 提到的关键依赖：

```bash
sudo apt-get install libarmadillo-dev
```

还需要手动安装 `nlopt v2.7.1`（见根目录 `README.md`）。

### 编译整个工作区

```bash
cd ..
catkin_make
```

如果当前目录就是工作区根目录的 `src/FUEL-main/FUEL-main`，则应先回到其上一级 catkin 根目录再执行。

### 仅编译某个包

```bash
cd ..
catkin_make --pkg plan_manage
catkin_make --pkg exploration_manager
catkin_make --pkg px4ctrl
```

### 加载环境

```bash
source devel/setup.bash
```

### 运行典型仿真 / 可视化

FUEL 探索演示（README 推荐流程）：

```bash
source devel/setup.bash
roslaunch exploration_manager rviz.launch
```

新终端中运行：

```bash
source devel/setup.bash
roslaunch exploration_manager exploration.launch
```

单机重规划演示：

```bash
source devel/setup.bash
roslaunch plan_manage rviz.launch
roslaunch plan_manage kino_replan.launch
roslaunch plan_manage topo_replan.launch
```

真机/实机相关入口（集成 RealSense + PX4Ctrl）：

```bash
source devel/setup.bash
roslaunch plan_manage topo_replan.launch
```

### 地图生成相关

创建交互式 PCD 地图：

```bash
source devel/setup.bash
rosrun map_generator click_map
rosrun map_generator map_recorder ~/
```

### 测试

仓库里明确启用 `catkin_add_gtest` 的包只发现 `uav_simulator/Utils/uav_utils`。

运行该包测试：

```bash
cd ..
catkin_make --pkg uav_utils run_tests
```

运行单个 gtest 用例：

```bash
source devel/setup.bash
./devel/lib/uav_utils/uav_utils-test --gtest_filter=TestName.*
```

## 高层架构

## 1. 仓库主线

这个仓库可以看成三层：

1. `fuel_planner/`：规划与探索核心。
2. `uav_simulator/`：仿真、地图生成、传感器/控制相关支撑。
3. `px4ctrl/` + `quadrotor_msgs/`：飞控控制链路与自定义消息。

其中 FUEL 的“探索”是在 Fast-Planner 的局部/全局轨迹规划能力之上扩展出来的，不是完全独立的一套系统。

## 2. `fuel_planner/` 内部职责分层

### `plan_env`

地图层。核心职责是：

- 接收深度图或点云 + 位姿；
- 通过 `MapROS` 做 ROS 订阅/同步与可视化发布；
- 通过 `SDFMap` 维护占据栅格与 ESDF；
- 通过 `EDTEnvironment` 对上层规划器提供距离场查询。

阅读入口：

- `fuel_planner/plan_env/src/map_ros.cpp`
- `fuel_planner/plan_env/src/sdf_map.cpp`
- `fuel_planner/plan_env/src/edt_environment.cpp`

重要理解：规划模块几乎都不直接处理传感器输入，而是统一依赖 `EDTEnvironment/SDFMap` 做碰撞与距离查询。

### `path_searching`

路径搜索前端：

- `astar.cpp`：几何 A*。
- `kinodynamic_astar.cpp`：满足动力学约束的搜索。
- `topo_prm.cpp`：拓扑路径候选生成。

它们负责生成“可行路径骨架”，后续再交给轨迹表示与优化模块。

### `bspline` / `bspline_opt` / `poly_traj` / `traj_utils`

轨迹表示与优化层：

- `bspline`：非均匀 B 样条表示。
- `bspline_opt`：基于代价函数的轨迹优化。
- `poly_traj`：多项式轨迹工具。
- `traj_utils`：消息转换和 RViz 可视化。

关键点：仓库的执行轨迹主线是 B-spline，不是直接执行搜索路径。

### `plan_manage`

规划调度层，是单机规划主入口。

关键角色：

- `FastPlannerManager`：装配地图、搜索器、优化器、主动感知模块；根据参数决定启用哪些子模块。
- `KinoReplanFSM`：动力学重规划状态机。
- `TopoReplanFSM`：拓扑重规划状态机。
- `traj_server`：把规划结果转换成持续发布的控制命令。

阅读入口：

- `fuel_planner/plan_manage/src/planner_manager.cpp`
- `fuel_planner/plan_manage/src/kino_replan_fsm.cpp`
- `fuel_planner/plan_manage/src/topo_replan_fsm.cpp`

重要理解：

- `launch/kino_algorithm.xml` 通过 `planner_node/planner=1` 选择 kinodynamic replanning。
- `launch/topo_algorithm.xml` 通过 `planner_node/planner=2` 选择 topological replanning。
- 大多数行为切换不是在代码里硬编码，而是通过 launch 参数控制 `FastPlannerManager::initPlanModules()` 中启用哪些模块。

## 3. FUEL 探索链路

`exploration_manager` 是 FUEL 相对 Fast-Planner 新增的高层探索层。

核心流程：

1. `FastExplorationFSM` 等待里程计和触发信号。
2. `FastExplorationManager` 调用 `FrontierFinder` 在已更新地图内搜索 frontier。
3. 对 frontier 计算候选 viewpoint，并通过 LKH 求解全局访问顺序。
4. 对最近几个 frontier 做局部视点细化。
5. 最终仍调用 `FastPlannerManager` 生成到下一观察位姿的轨迹。
6. 轨迹经 `planning/bspline` → `traj_server` → 下游控制器执行。

阅读入口：

- `fuel_planner/exploration_manager/src/fast_exploration_fsm.cpp`
- `fuel_planner/exploration_manager/src/fast_exploration_manager.cpp`
- `fuel_planner/active_perception/src/frontier_finder.cpp`

重要理解：

- FUEL 不是“探索器直接出控制命令”，而是“探索决策 + 规划器复用”。
- frontier 搜索只在地图更新区域附近增量进行，不是每次全图扫描。
- 多个关键阈值（frontier 聚类、候选视点、安全距离、replan 时机）都在 `exploration_manager/launch/algorithm.xml`。

## 4. launch 文件如何组织系统

要理解系统运行方式，优先从 launch 看：

- `fuel_planner/exploration_manager/launch/exploration.launch`
  - FUEL 入口。
  - 组装 `exploration_manager/algorithm.xml`、`traj_server`、`waypoint_generator` 和模拟器。
- `fuel_planner/plan_manage/launch/kino_replan.launch`
  - 单机 kinodynamic 重规划演示。
- `fuel_planner/plan_manage/launch/topo_replan.launch`
  - 单机 topological 重规划 / 真机集成入口。
- `fuel_planner/plan_manage/launch/*.xml`
  - 真正承载参数：地图范围、相机内参、速度加速度、frontier 参数、优化器权重、topic remap。

排查问题时的推荐顺序：先看 `.launch`，再看被 include 的 `*.xml` 参数文件，最后看对应 C++ 节点实现。

## 5. 控制与消息链路

### `traj_server` / `planning/pos_cmd`

规划器不会直接控制 PX4。典型链路是：

- FSM/manager 产出 B-spline；
- `traj_server` 将轨迹展开为 `quadrotor_msgs/PositionCommand`；
- 下游控制器订阅 `planning/pos_cmd` 或 remap 后的 `cmd`。

### `px4ctrl`

`px4ctrl` 是实际控制层，订阅：

- 里程计；
- `quadrotor_msgs::PositionCommand`；
- IMU、RC、电池、MAVROS 状态。

然后发布 PX4 所需的 attitude/local setpoint，并提供起飞触发等能力。

阅读入口：

- `px4ctrl/src/px4ctrl_node.cpp`
- `px4ctrl/src/PX4CtrlFSM.cpp`
- `px4ctrl/src/controller.cpp`

### `quadrotor_msgs`

存放规划和控制之间共享的自定义消息，例如 `PositionCommand`。涉及轨迹下发、控制接口或 topic 类型时先看这里。

## 6. 仿真与真实传感器的差别

仓库同时支持模拟深度输入和真实相机输入，差别主要体现在 launch remap：

- 仿真时常用 `/pcl_render_node/depth`、`/pcl_render_node/cloud`、`/pcl_render_node/sensor_pose`。
- 真机集成时 `topo_replan.launch` 里改成 RealSense 深度图和 MAVROS 位姿，并同时启动 `px4ctrl`。

因此同一个规划节点在代码层通常不区分“仿真/实机”，而是依赖 launch remap 到不同上游 topic。

## 7. 地图与环境资源

- 根目录 `README.md` 说明了默认探索环境是 `.pcd` 文件。
- `uav_simulator/map_generator` 提供 `map_pub`、`click_map`、`map_recorder` 等节点用于加载或生成环境。
- 若修改探索边界，除了环境文件本身，还要同步检查 launch 中的 `box_min/max` 与 `map_size_*`。

## 8. 修改时的经验性切入点

- 轨迹生成问题：先看 `plan_manage` 的 FSM，再看 `planner_manager.cpp` 中启用了哪些搜索/优化器。
- 探索提前结束 / 找不到 frontier：优先看 `exploration_manager/launch/algorithm.xml` 里的 frontier 参数，以及 `fast_exploration_manager.cpp` 的调试输出。
- 地图可视化或深度融合异常：先看 `plan_env/src/map_ros.cpp` 的 remap、深度转换和 ESDF 更新。
- 真机控制问题：先确认 `topo_replan.launch` 中 `planning/pos_cmd -> px4ctrl` remap 和 MAVROS 话题命名空间是否一致。

## README 中值得保留的信息

- 项目依赖 Ubuntu 18.04/20.04 + ROS Melodic/Noetic（根 README）。
- 探索环境由 `.pcd` 文件表示，默认样例在 `map_generator/resource`。
- 若替换环境，不仅要改 `map_pub` 加载的 `.pcd`，还要调整探索 bounding box。
- `nlopt` 与 `libarmadillo-dev` 是明确写出的关键依赖。
