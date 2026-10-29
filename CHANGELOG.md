# Changelog

All notable changes to this project are documented in this file.

## [0.2.2] - 2026-10-29

### Added
- **Lateral Safe-Corridor Linear Constraints in `MPC` (`controller/path_tracker/mpc.py`)**:
  - Added `obstacle_surface_distance` and `MPC.corridor_half_widths` (1-Lipschitz certified obstacle-free lateral half-widths along the reference trajectory) and integrated soft-slacked linear corridor constraints (`-d_k - s_k <= n_k^T e_{p,k} <= d_k + s_k, s_k >= 0`) into a pre-allocated $3N$-variable, $5N$-constraint OSQP workspace (`use_corridor=True`).
  - Added 11th controller variant `ConsensusMPC_Corridor` (`ADG + Corridor-Linear-MPC + Velocity-Level CBF-QP`) in `scripts/benchmark_control.py`, reducing nominal makespan to `26.6 s` and achieving `100.0%` (`20/20` trials) full completion under 60s randomized stall perturbations.
  - Added unit test `test_linear_mpc_corridor_constraint_limits_lateral_excursion` (13 total tests in `tests/test_ecbs_and_consensus_mpc.py`).
- **60.0s Full-Horizon Perturbation Evaluation, Finish-Time Quantiles & 95% Wilson CIs (`scripts/benchmark_control.py`, `results/perturbation_robustness.csv`)**:
  - Extended randomized stall perturbation simulations to `60.0 s` (`600` steps) while reporting completion at both the `33.5 s` nominal cutoff and `60.0 s` full horizon, along with 95% Wilson score confidence intervals (`wilson_ci95`), per-agent finish-time distributions (`median / P90 / max`), and obstacle contact metrics (`obstacle_contact_events`, `min_obstacle_clearance`).
- **Explicit `collision_free` Verification Flag in `SafeCorridorBSplineOptimizer` (`traj_optimizer/curve_generator/point_based/bspline.py`)**:
  - Added `SafeCorridorBSplineOptimizer.find_violations` with arc-length segment densification (`<= 0.25 * resolution`) so sparse curve segments cannot jump across obstacle cells; `optimize()` now returns `collision_free`, `num_violations`, and `repair_iters` in `info` and requires `collision_free == True` for `info["success"]`.

### Changed & Fixed
- **Per-Controller OSQP Workspaces (`QP2DWorkspace` in `controller/path_tracker/mpc.py`)**:
  - Replaced module-level global `_CBF_WS` and `_ORCA_WS` singletons with per-controller `QP2DWorkspace` instances (`self._cbf_ws`, `self._orca_ws`), eliminating import-time side effects and cross-agent warm-start state sharing.
- **Unbiased Zero-Turn-Penalty Low-Level State Space (`path_planner/graph_search/ecbs.py`)**:
  - Fixed `ECBS._focal_low_level_astar` so `nxt_dir` stays `-1` whenever `self.turn_penalty <= 0.0`, eliminating redundant heading-direction expansions in `CBS (λ=0)` (`2,188 -> 1,890` low-level expansions) and `ECBS (λ=0)` (`650 -> 619`). Regenerated `results/mapf_ablation.csv`, `results/mapf_param_sweep.csv`, and `results/synthetic_grid_benchmark.csv`.
- **Precision of Safety Guarantees & Documentation (`README.md`, `README_zh.md`, `mpc.py`)**:
  - Clarified `cbf_qp_filter` as a **velocity-level** distributed CBF-QP safety filter and documented unmodeled bounded-acceleration tracking (`±3.5 m/s²`), discrete-time integration (`dt = 0.1 s`), and longitudinal-dominated avoidance authority under small lookahead offset (`l = 0.04 m`, $l \cdot \omega_{\max} \approx 0.13\text{ m/s}$).

## [0.2.1] - 2026-10-28

### Added
- **Component-Isolation Controller Ablation & 20-Seed Randomized Perturbation Suite (`scripts/benchmark_control.py`)**:
  - Expanded the continuous controller benchmark to 10 variants (`PurePursuit`, `RPP`, `DWA`, `ORCA`, `ADG_ORCA`, `ADG_PurePursuit`, `ADG_PurePursuit_CBF`, `ADG_LinearMPC`, `LinearMPC_CBF`, `ConsensusMPC`) to isolate the contributions of `ADG`, `Linear-MPC`, and `CBF-QP`.
  - Replaced the 5 fixed stall scenarios with 20 genuinely randomized trials (`seed = 1000..1019`) sampling random stalled AGVs, random start step $t_s \in [35, 110]$, and random duration $\Delta t \in [2.0, 3.5]\text{ s}$.
  - Separated OSQP internal C solve time (`osqp_mean_us / osqp_p95_us`) from total step wall time (`wall_mean_us / wall_p95_us`) and renamed `deadlock_count` to `incomplete_count`.

### Changed & Fixed
- **Velocity-Level Distributed `cbf_qp_filter` & Pure Rollout `MPC` (`controller/path_tracker/mpc.py`)**:
  - Enforced exact linear unicycle actuation bounds ($0 \le [\cos\theta, \sin\theta] u \le v_{\max}$ and $-\omega_{\max} \le [-\sin\theta/l, \cos\theta/l] u \le \omega_{\max}$) directly inside the `cbf_qp_filter` and `ORCAController` QPs, removing all post-QP angular-velocity blending and heuristic lane skipping.
  - Added full $2l$ lookahead offset compensation to $d_{\text{safe,eff}} = r_i + r_j + 2l + d_{\text{margin}}$ and computed neighbor velocity $\hat{u}_j$ at the neighbor's lookahead point.
  - Replaced per-step `osqp.OSQP().setup()` recreation in `MPC`, `cbf_qp_filter`, and `ORCAController` with persistent OSQP workspaces updated via `prob.update(...)`.
  - Removed the 28-step time-difference cutoff in `ActionDependencyGraph` so precedence edges hold across arbitrary stall durations.
  - Replaced single-pass B-Spline corridor repair in `SafeCorridorBSplineOptimizer` with an 8-iteration dense curve verification and control-point pull-back loop.
- **Synthetic Grid Benchmark Naming & Full Table (`scripts/maps/`, `scripts/benchmark_mapf.py`)**:
  - Renamed self-generated benchmark maps in `scripts/maps/` to `synthetic_warehouse_48x32`, `synthetic_random_32x32_20`, and `synthetic_room_32x32_4` (`results/synthetic_grid_benchmark.csv`), and included all 9 map/scale pairs (18 rows) in `README.md` and `README_zh.md`.
  - Unified `Independent A*` in `scripts/benchmark_mapf.py` to use the same low-level A* search engine (`ECBS._focal_low_level_astar`) as `CBS` and `ECBS`.

## [0.2.0] - 2026-10-28

### Added
- **Space-Time Planning (`path_planner/graph_search/ecbs.py`)**:
  - Full two-level `OPEN` + `FOCAL` priority queues with lazy deletion and dynamic threshold expansion (`f <= w * f_min` at low level, `cost <= w * min_lb` at high level), strictly preserving the bounded suboptimality guarantee `SOC <= w * LB`.
  - $O(T \cdot N)$ spatial-hash conflict detection (`detect_conflicts`) replacing $O(T \cdot N^2)$ pairwise scanning.
  - Public utility methods `detect_conflicts`, `count_turns`, and `path_cost`, plus optional `record_expanded` toggle.
- **Safe-Corridor Trajectory Optimization (`traj_optimizer/curve_generator/point_based/bspline.py`)**:
  - `SafeCorridorBSplineOptimizer`: axis-aligned Safe Flight Corridor (SFC) box inflation in `TYPES.FREE` + `osqp` Quadratic Program minimizing jerk, acceleration, and reference deviation under hard corridor box constraints.
  - Added `"control_polygon"` evaluation mode to `BSpline`.
- **Multi-Agent Cooperative Control (`controller/path_tracker/mpc.py`)**:
  - `cbf_qp_filter`, `MPC`, `ActionDependencyGraph`, `ConsensusMPC`, and `ORCAController`.
- **Continuous Integration & Packaging**:
  - GitHub Actions workflow `.github/workflows/ci.yml` running `ruff` and `pytest` across Python 3.9, 3.10, and 3.11.
  - Layered core vs. optional dependencies (`[viz]`, `[sim]`, `[rrt]`, `[dev]`, `[all]`) in `pyproject.toml`.
  - Expanded `tests/test_ecbs_and_consensus_mpc.py` to 12 unit tests.

### Fixed
- Fixed upstream `BSpline._approximation` knot vector length mismatch (`n + k + 1` instead of `h + k + 1`) by implementing Piegl & Tiller (1997) Eq. 9.68–9.69 parameter averaging and least-squares SVD solve (`np.linalg.lstsq`).
- Removed hardcoded `planning_time_ms` for `Independent_AStar`; all planning times are measured with `time.perf_counter()` over 10 repetitions (reporting median and P90).

## [0.1.0] - 2026-10-27

### Added
- Initial multi-AGV warehouse planning and control extension on top of `python_motion_planning` v2.1.
