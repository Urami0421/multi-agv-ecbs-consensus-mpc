# Changelog

All notable changes to this project are documented in this file.

## [0.2.0] - 2026-10-28

### Added
- **Space-Time Planning (`path_planner/graph_search/ecbs.py`)**:
  - Full two-level `OPEN` + `FOCAL` priority queues with lazy deletion and dynamic threshold expansion (`f <= w * f_min` at low level, `cost <= w * min_lb` at high level), strictly preserving the bounded suboptimality guarantee `SOC <= w * LB`.
  - $O(T \cdot N)$ spatial-hash conflict detection (`detect_conflicts`) replacing $O(T \cdot N^2)$ pairwise scanning.
  - Public utility methods `detect_conflicts`, `count_turns`, and `path_cost`, plus optional `record_expanded` toggle.
- **Safe-Corridor Trajectory Optimization (`traj_optimizer/curve_generator/point_based/bspline.py`)**:
  - `SafeCorridorBSplineOptimizer`: axis-aligned Safe Flight Corridor (SFC) box inflation in `TYPES.FREE` + `osqp` Quadratic Program minimizing jerk, acceleration, and reference deviation under hard corridor box constraints, with dense curve-level collision verification.
  - Added `"control_polygon"` evaluation mode to `BSpline`.
- **Multi-Agent Cooperative Control (`controller/path_tracker/mpc.py`)**:
  - `cbf_qp_filter`: Distributed Control Barrier Function Quadratic Program (CBF-QP) safety filter via Near-Identity Diffeomorphism (NID) and `osqp` (Wang et al., IEEE T-RO 2017), with safe-stop fallback on infeasible constraint sets.
  - `MPC`: Condensed $2N$-variable Linearized Error-State Receding-Horizon MPC solved via `osqp` with primal warm-starting (Kühne et al., 2004) and true piecewise-linear Frenet projection `compute_frenet_state`.
  - `ActionDependencyGraph` (`ADG`): Time-synchronized schedule execution extracting Type-2 inter-agent precedence dependencies from discrete `ECBS` plans (Hönig et al., IEEE RA-L 2019).
  - `ORCAController`: Multi-agent Reciprocal Velocity Obstacles (ORCA) half-plane QP baseline solved via `osqp` (van den Berg et al., 2011).
- **Standardized Benchmarks & Modular Scripts (`scripts/`, `results/`)**:
  - Externalized 8-AGV two-stage warehouse scenario configuration in `scripts/configs/warehouse_8agv.yaml`.
  - Standardized MovingAI `.map`/`.scen` benchmark suite (`warehouse-10-20-10-2-1`, `random-32-32-20`, `room-32-32-4`) in `scripts/maps/`.
  - Modular benchmark and rendering entrypoints: `scripts/benchmark_mapf.py`, `scripts/benchmark_control.py`, `scripts/render_2d.py`, `scripts/render_3d.py`.
  - Reproducible CSV outputs in `results/`: `mapf_ablation.csv`, `mapf_param_sweep.csv`, `movingai_benchmark.csv`, `control_ablation.csv`, `perturbation_robustness.csv`.
- **Continuous Integration & Packaging**:
  - GitHub Actions workflow `.github/workflows/ci.yml` running `ruff` and `pytest` across Python 3.9, 3.10, and 3.11.
  - Layered core vs. optional dependencies (`[viz]`, `[sim]`, `[rrt]`, `[dev]`, `[all]`) in `pyproject.toml`.
  - Expanded `tests/test_ecbs_and_consensus_mpc.py` to 12 unit tests covering vertex/edge conflicts, multi-stage waypoints, `min_separation`, unsolvable termination, suboptimality bounds, turn reduction, CBF-QP invariance, Linear-MPC convergence, SFC B-Spline safety, and ADG delay propagation.

### Fixed
- Fixed upstream `BSpline._approximation` knot vector length mismatch (`n + k + 1` instead of `h + k + 1`) by implementing Piegl & Tiller (1997) Eq. 9.68–9.69 parameter averaging and least-squares SVD solve (`np.linalg.lstsq`).
- Removed hardcoded `planning_time_ms` for `Independent_AStar`; all planning times are now measured with `time.perf_counter()` over 10 repetitions (reporting median and P90).
- Unified low-level A* tie-breaking (`g_sign = -1.0`) across `CBS` (`w = 1.0`) and `ECBS` (`w > 1.0`).

## [0.1.0] - 2026-10-27

### Added
- Initial multi-AGV warehouse planning and control extension on top of `python_motion_planning` v2.1.
