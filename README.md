
# Multi-AGV ECBS + Safe-Corridor $C^2$ B-Spline + Distributed Consensus-CBF-MPC

<p align="center">
  <b><a href="README.md">English</a> | <a href="README_zh.md">简体中文</a></b>
</p>

## 🚀 Project Overview / 项目简介（中英双语）

### 🌐 English Summary
> **Full-Stack Multi-AGV Space-Time Planning (`ECBS`), Safe-Corridor $C^2$ Trajectory Optimization (`BSpline`), and Distributed `Consensus-CBF-MPC` Control in a 3D + 2D Synchronized OpenGL Warehouse Digital Twin**:
> This repository extends [`ai-winter/python_motion_planning`](https://github.com/ai-winter/python_motion_planning) with a complete 3-stage industrial fleet planning & control pipeline and a hardware-accelerated 3D/2D synchronized warehouse digital twin (`Visualizer3D` + UV-textured AWS RoboMaker 3D Shelves/Forklifts + 8-part Neobotix `MP-400` CAD AGVs) on a **72m × 44m, 12-Rack (`Rack-1..12`), 20-Intersection, 8-Bottleneck-Hazard (`H1..H8`), 8-AGV Multi-Stage (`Start -> Pickup -> Delivery`) Industrial Warehouse**:
> 1. **Kinematic Turn-Penalized Two-Level Focal ECBS (`src/python_motion_planning/path_planner/graph_search/ecbs.py`) + Safe-Corridor Cubic $C^2$ B-Spline (`src/python_motion_planning/traj_optimizer/curve_generator/point_based/bspline.py`)** (*ECAI 2014 / IJCAI 2016*): Reduces low-level space-time state expansions by **84.0%** (`8,127 -> 1,302`), speeds up planning by **3.0x** (`102.9 ms -> 34.4 ms`), cuts 90° turns by **43.8%** (`48 -> 27`), and converts discrete grid detours around `H1..H8` into $C^2$-continuous smooth S-curves inside `TYPES.FREE` safe corridors.
> 2. **Distributed Multi-Agent Velocity Consensus + Discrete-Time Control Barrier Function MPC (`src/python_motion_planning/controller/path_tracker/mpc.py`)** (*Proc. IEEE 2007 / IEEE TAC 2017*): Fills the upstream `MPC (Not migrated)` and `3D Controller Simulator (Not implemented)` gaps, achieving **0 collisions** (`min_dist = 1.16m > 2R = 0.88m`) and **100% (8/8) multi-stage mission completion** (`0.27m` mean terminal error) across 8 differential-drive AGVs, 8 S-curve aisle bottlenecks, and 20 intersections where `PurePursuit` (`4` collisions), `RPP` (`2` collisions), and `DWA` (`0.0%` completion, trapped in local minima at narrow bottlenecks) all fail.
> 3. **Reproduce in one command**: `python scripts/run_multi_agv_ecbs_consensus_demo.py`

### 🇨🇳 中文简介
> **工业级多 AGV 集群“全局时空规划（`ECBS`）$\rightarrow$ 安全走廊 $C^2$ B 样条平滑（`BSpline`）$\rightarrow$ 分布式速度一致性与控制屏障预测控制（`Consensus-CBF-MPC`）”全栈架构与 3D/2D 实时联动仓储数字孪生**：
> 本项目基于开源运动规划框架 [`ai-winter/python_motion_planning`](https://github.com/ai-winter/python_motion_planning) 进行深度扩展，面向 **72m × 44m、12 座货架岛（`Rack-1..12`）、20 个正交交叉路口、8 处巷道半幅占道瓶颈（`H1..H8`）、8 台差速驱动 Neobotix `MP-400` AGV 两阶段拣选与出库（`Start -> Pickup -> Delivery`）** 复合压力场景，构建了完整的“规划—平滑—控制—数字孪生”闭环系统：
> 1. **运动学转向惩罚两级焦点搜索（`ECBS`）+ 安全走廊三次 B 样条 $C^2$ 平滑（`BSpline`）**：引入冲突规避表（CAT）引导的两级焦点搜索（$w=1.08$）与可采纳转弯感知启发函数，相比标准精确 `CBS` 将**底层搜索状态数减少 84.0%（`8,127 -> 1,302`）、规划速度提升 3.0 倍（`102.9 ms -> 34.4 ms`）、车队 90° 急转弯削减 43.8%（`48 -> 27`）**；并通过 `TYPES.FREE` 安全走廊约束下的 Clamped 三次 B 样条生成曲率连续（$C^2$）的 S 型平滑换道轨迹。
> 2. **分布式多智能体速度一致性 + 离散时间控制屏障函数预测控制（`Consensus-CBF-MPC`）**：补齐了原仓库缺失的 `MPC` 控制器与 `3D 控制器仿真器`（集成 AWS RoboMaker 全贴图 3D 货架/叉车、Neobotix `MP-400` 8 部件 CAD 模型及右下角 `1:1` 帧级同步 2D `ESDF` 雷达监控图），在 `PurePursuit`（4 次碰撞）、`RPP`（2 次碰撞）与 `DWA`（窄道局部极小值死锁 0% 完成）均失效的复合瓶颈场景下，实现 **全程 0 碰撞（最小车距 `1.16m > 2R = 0.88m`）、100%（8/8）任务达成、终点平均跟踪误差仅 `0.27m`**。
> 3. **一键复现命令**：`python scripts/run_multi_agv_ecbs_consensus_demo.py` | **完整中文说明文档**：[`README_zh.md`](README_zh.md)

![3D + 2D Synchronized Multi-AGV Warehouse Digital Twin (48 3D Shelves + 8 MP-400 AGVs + Live 2D ESDF Radar)](assets/multi_agv_consensus_mpc_3d.gif)
![3D + 2D Synchronized Warehouse Digital Twin Keyframe Snapshot](assets/multi_agv_consensus_mpc_3d_snapshot.png)
![3-Stage Progressive Pipeline: Standard CBS vs. Focal ECBS + Cubic BSpline vs. 3D Space-Time Corridors](assets/ecbs_vs_cbs_warehouse.png)
![2D Multi-AGV ECBS + BSpline + Consensus-CBF-MPC Simulation](assets/multi_agv_consensus_mpc_2d.gif)
![Quantitative Ablation Dashboard](assets/multi_agv_ablation_dashboard.png)

`Python Motion Planning` repository provides the implementations of common `Motion planning` algorithms, including path planners on N-D grid, controllers for path-tracking, trajectory optimizers, a visualizer based on matplotlib and a toy physical simulator to test controllers.

`Motion planning` plans the state sequence of the robot without conflict between the start and goal. 

`Motion planning` mainly includes `Path planning` and `Trajectory planning`.

* `Path Planning`: It's based on path constraints (such as obstacles), planning the optimal path sequence for the robot to travel without conflict between the start and goal.
* `Trajectory planning`: It plans the motion state to approach the global path based on kinematics, dynamics constraints and path sequence.

The theory analysis can be found at [motion-planning](https://blog.csdn.net/frigidwinter/category_11410243.html).

We also provide [ROS C++](https://github.com/ai-winter/ros_motion_planning) version and [Matlab](https://github.com/ai-winter/matlab_motion_planning) version.

**Your stars and forks are welcome!**

# Quick Start

## Overview
The source file structure is shown below

```
python_motion_planning
├─common
|   ├─env
|   |   ├─map
|   |   ├─robot
|   |   └─world
|   ├─utils
|   └─visualizer
├─controller
|   └─path_tracker
├─path_planner
|   ├─graph_search
|   ├─sample_search
|   └─hybrid_search
└─traj_optimizer
    └─curve_generator
```

## Install
*(Optional)* The code was tested in python=3.10, though other similar versions should also work. We recommend using `conda` to install the dependencies.

```shell
conda create -n pmp python=3.10
conda activate pmp
```

To install the repository, please run the following command in shell.

```shell
pip install python-motion-planning
```

## Run

Please refer to the Tutorials part of [online documentation](https://ai-winter.github.io/python_motion_planning/).

# Demos
## Path Planner
### Graph Search
|Planner|2D Grid|3D Grid
|-------|-------|-------
**Dijkstra**|![dijkstra_2d.svg](assets/dijkstra_2d.svg)|![dijkstra_3d.png](assets/dijkstra_3d.png)
**GBFS**|![gbfs_2d.svg](assets/gbfs_2d.svg)|![gbfs_3d.png](assets/gbfs_3d.png)
**A\***|![a_star_2d.svg](assets/a_star_2d.svg)|![a_star_3d.png](assets/a_star_3d.png)
**JPS**|![jps_2d.svg](assets/jps_2d.svg)|![jps_3d.png](assets/jps_3d.png)
**Theta\***|![theta_star_2d.svg](assets/theta_star_2d.svg)|![theta_star_3d.png](assets/theta_star_3d.png)
**Lazy Theta\***|![lazy_theta_star_2d.svg](assets/lazy_theta_star_2d.svg)|![lazy_theta_star_3d.png](assets/lazy_theta_star_3d.png)
**CBS / ECBS (Multi-AGV)**|![ecbs_vs_cbs_warehouse.png](assets/ecbs_vs_cbs_warehouse.png)|Not implemented
**D\***|Implemented in [V1.1.1](https://github.com/ai-winter/python_motion_planning/tree/v1.1.1), not migrated|Not implemented
**LPA\***|Implemented in [V1.1.1](https://github.com/ai-winter/python_motion_planning/tree/v1.1.1), not migrated|Not implemented
**D\* Lite**|Implemented in [V1.1.1](https://github.com/ai-winter/python_motion_planning/tree/v1.1.1), not migrated|Not implemented
**Anya**|Not implemented|Not implemented

### Sample Search
|Planner|2D Grid|3D Grid
|-------|-------|-------
**RRT**|![rrt_2d.svg](assets/rrt_2d.svg)|![rrt_3d.png](assets/rrt_3d.png)
**RRT\***|![rrt_star_2d.svg](assets/rrt_star_2d.svg)|![rrt_star_3d.png](assets/rrt_star_3d.png)
**RRT-Connect**|![rrt_connect_2d.svg](assets/rrt_connect_2d.svg)|![rrt_connect_3d.png](assets/rrt_connect_3d.png)
**Informed RRT**|Implemented in [V1.1.1](https://github.com/ai-winter/python_motion_planning/tree/v1.1.1), not migrated|Not implemented
**PRM**|Not implemented|Not implemented

### Evolutionary Search
|Planner|2D Grid|3D Grid
|-------|-------|-------
| **ACO** |Implemented in [V1.1.1](https://github.com/ai-winter/python_motion_planning/tree/v1.1.1), not migrated|Not implemented
| **GA**  |Implemented in [V1.1.1](https://github.com/ai-winter/python_motion_planning/tree/v1.1.1), not migrated|Not implemented
| **PSO**  |Implemented in [V1.1.1](https://github.com/ai-winter/python_motion_planning/tree/v1.1.1), not migrated|Not implemented

### Hybrid Search
|Planner|2D Grid|3D Grid
|-------|-------|-------
**Voronoi Planner**|![voronoi_planner_2d.svg](assets/voronoi_planner_2d.svg)|![voronoi_planner_3d.png](assets/voronoi_planner_3d.png)

## Controller

We provide a toy simulator with simple physical simulation to test controllers (path-trakcers). The toy simulator supports multi-agents/multi-robots. The available robots include `CircularRobot` (Omnidirectional) and `DiffDriveRobot` (Only support moving forward and backward). Currently only 2D simulator is provided. 3D simulator has not been implemented.

In the following demos, the blue robot 1 is the `CircularRobot`, and the orange robot 2 is the `DiffDriveRobot`.

|Controller|2D|3D
|-------|-------|-------
|**Path Trakcer**|![path_tracker_2d.gif](assets/path_tracker_2d.gif)|Not implemented
| **Pure Pursuit**     |![pure_pursuit_2d.gif](assets/pure_pursuit_2d.gif)|Not implemented
| **PID**     |![pid_2d.gif](assets/pid_2d.gif)|Not implemented
| **APF**     |![apf_2d.gif](assets/apf_2d.gif)|Not implemented
| **DWA**     |![dwa_2d.gif](assets/dwa_2d.gif)|Not implemented
| **RPP**     |![rpp_2d.gif](assets/rpp_2d.gif)|Not implemented
| **LQR**     |Implemented in [V1.1.1](https://github.com/ai-winter/python_motion_planning/tree/v1.1.1), not migrated|Not implemented
| **MPC / Consensus-CBF-MPC** |![multi_agv_consensus_mpc_2d.gif](assets/multi_agv_consensus_mpc_2d.gif)|![multi_agv_consensus_mpc_3d.gif](assets/multi_agv_consensus_mpc_3d.gif)
| **MPPI**    |Not implemented|Not implemented
| **TEB**     |Not implemented|Not implemented
| **Lattice** |Not implemented|Not implemented
| **DQN**    |Not implemented|Not implemented
| **DDPG**    |Implemented in [V1.0](https://github.com/ai-winter/python_motion_planning/tree/v1.0), not migrated|Not implemented

## Trajectory Optimizer
### Curve Generator
#### Point-based

|Generator|2D|3D|
| ------- | -------------------------------------------------------- | -------------------------------------------------------- 
| **Cubic Spline** |![cubic_spline_2d.svg](assets/cubic_spline_2d.svg)|Not implemented
| **BSpline** |![bspline_2d.svg](assets/bspline_2d.svg)|Not implemented

#### Pose-based
|Generator|2D|3D|
| ------- | -------------------------------------------------------- | -------------------------------------------------------- 
| **Polynomia** | ![polynomial_2d.svg](assets/polynomial_2d.svg)|Not implemented
| **Bezier** |![bezier_2d.svg](assets/bezier_2d.svg)|Not implemented
| **Dubins** |![dubins_2d.svg](assets/dubins_2d.svg)|Not implemented
| **Reeds-Shepp** |![reeds_shepp_2d.svg](assets/reeds_shepp_2d.svg)|Not implemented

# Future Works

* N-D controllers (path-trackers).

* Path planning for robotic arms.

* Path planning on topological map.

* Sample search with Dubins or Reeds-Shepp curves.

* Application on ROS2.

* Application in mainstream robot simulation environments (e.g. Gazebo, Carla, Airsim, PyBullet, MuJoCo, Issac Sim).

* More mainstream motion planning algorithms.

* Performance optimization.

Contributors are welcome! For trivial modification, please directly contribute to `dev` branch. For big modification, please [contact](#contact) us before you contribute.

# Contact

Long-term maintainers:

* [@omigeft](https://github.com/omigeft) (Wu Maojia)
* [@ai-winter](https://github.com/ai-winter) (Yang Haodong)

You can contact us via the information provided on our profile.
