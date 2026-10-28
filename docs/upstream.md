# Upstream Framework Reference (`python_motion_planning` v2.1)

This repository is built upon the open-source single-robot motion planning library [`ai-winter/python_motion_planning`](https://github.com/ai-winter/python_motion_planning) (v2.1, GPL-3.0 License), originally authored and maintained by **Haodong Yang** and **Maojia Wu**.

## Upstream Modules Retained as Infrastructure

| Package | Upstream Components | Role in This Repository |
| :--- | :--- | :--- |
| `common/env/map/grid.py` | `Grid`, obstacle inflation, Euclidean Signed Distance Field (`esdf`) | 2D grid map representation and static obstacle clearance lookup |
| `common/env/robot/` | `BaseRobot`, `DiffDriveRobot`, `OmniRobot` | Differential-drive kinematic & acceleration-bounded robot state integration |
| `common/env/world/` | `BaseWorld`, `ToySimulator` | Continuous-time step integration (`dt = 0.1 s`), velocity damping, and boundary contact |
| `common/visualizer/` | `Visualizer2D`, `Visualizer3D` | Matplotlib 2D rendering and PyVista/VTK 3D mesh rendering |
| `path_planner/graph_search/` | `AStar`, `Dijkstra`, `ThetaStar`, `JPS`, `DStarLite` | Single-agent graph search baselines |
| `path_planner/sample_search/` | `RRT`, `RRTConnect`, `RRTStar` | Single-agent sampling-based planners |
| `traj_optimizer/curve_generator/` | `Polynomial`, `Bezier`, `CubicSpline`, `BSpline`, `Dubins`, `ReedsShepp` | Parametric curve generators (with `BSpline._approximation` knot generation fixed and `SafeCorridorBSplineOptimizer` added in this fork) |
| `controller/path_tracker/` | `PurePursuit`, `RPP`, `PID`, `DWA`, `APF` | Single-agent path-tracking baselines used in our controller ablation |

## Upstream Tutorials & Standalone Examples

The original single-robot notebook tutorials and standalone algorithm scripts remain available under:
- [`tutorials/`](../tutorials/) — Interactive Jupyter notebooks for single-robot `Grid`, `AStar`, `BSpline`, and `PurePursuit` / `DWA`.
- [`examples/`](../examples/) — Standalone 2D and 3D single-robot planning and tracking scripts.

## Upstream Citation

```bibtex
@misc{python_motion_planning,
  author       = {Yang, Haodong and Wu, Maojia},
  title        = {Python Motion Planning},
  year         = {2025},
  publisher    = {GitHub},
  journal      = {GitHub repository},
  howpublished = {\url{https://github.com/ai-winter/python_motion_planning}}
}
```
