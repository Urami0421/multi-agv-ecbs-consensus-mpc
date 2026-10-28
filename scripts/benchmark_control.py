"""
Multi-AGV Continuous Control Benchmark, Component-Isolation Ablation & Perturbation Robustness Suite:
  1. Evaluates 10 controller variants on the 8-AGV two-stage (`Start -> Pickup -> Delivery`) warehouse:
     - `PurePursuit` (single-agent baseline)
     - `RPP` (Regulated Pure Pursuit single-agent baseline)
     - `DWA` (Dynamic Window Approach single-agent baseline)
     - `ORCA` (Non-Holonomic Lookahead ORCA QP via OSQP, Alonso-Mora et al., 2013)
     - `ADG_ORCA` (`ADG` + Non-Holonomic Lookahead `ORCA`)
     - `ADG_PurePursuit` (`ADG` + `PurePursuit`)
     - `ADG_PurePursuit_CBF` (`ADG` + `PurePursuit` + strict `CBF-QP` filter)
     - `ADG_LinearMPC` (`ADG` + `Linear-MPC` without `CBF-QP`)
     - `LinearMPC_CBF` (`Linear-MPC` + `CBF-QP` without `ADG`)
     - `ConsensusMPC` (`ADG` + `Linear-MPC` + strict `CBF-QP`, full proposed method)
  2. Tracks true current-position pairwise collisions, strict sequential two-stage completion
     (`Start -> Pickup -> Delivery`), makespan, mean speed, stop time, incomplete agents,
     OSQP internal C solve time (us), and total step wall time (us).
  3. Runs 20 genuinely randomized trials (random stalled AGVs, random start step, random 2.0-3.5s
     duration, plus Gaussian velocity noise).
  4. Saves `results/control_ablation.csv`, `results/perturbation_robustness.csv`, and renders
     `assets/multi_agv_ablation_dashboard.png`.
"""
from __future__ import annotations

import csv
import math
from pathlib import Path
import random
import sys
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))
if str(ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts"))

from python_motion_planning.common import Grid, DiffDriveRobot, ToySimulator, FrameTransformer
from python_motion_planning.controller import (
    PurePursuit,
    RPP,
    DWA,
    ORCAController,
    ActionDependencyGraph,
    ConsensusMPC,
    cbf_qp_filter,
    unicycle_to_lookahead_point,
    unicycle_vel_to_single_integrator,
    single_integrator_to_unicycle_vel,
)
from benchmark_mapf import (
    load_warehouse_config,
    build_warehouse_grid,
    get_fleet_mission,
    optimize_corridor_bspline_paths,
    run_planning_ablation,
)


class ADGPurePursuit(PurePursuit):
    """PurePursuit augmented with Arc-Length Action Dependency Graph (`ADG`) hold constraints."""

    def __init__(
        self,
        *args,
        self_id: str = "1",
        adg: Optional[ActionDependencyGraph] = None,
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.self_id = str(self_id)
        self.adg = adg
        self._path_arr = np.asarray(self.path, dtype=float)
        if len(self._path_arr) > 1:
            diffs = np.diff(self._path_arr[:, :2], axis=0)
            self._path_s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(diffs, axis=1))])
        else:
            self._path_s = np.zeros(len(self._path_arr), dtype=float)
        self._progress_s = 0.0

    def get_action(self, obs: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        pose, vel, pos, orient, _, _ = self.get_pose_velocity(obs)
        if len(self._path_arr) > 0:
            mask = (self._path_s >= self._progress_s - 1.5) & (self._path_s <= self._progress_s + 8.0)
            cands = np.where(mask)[0]
            if len(cands) == 0:
                cands = np.arange(len(self._path_arr))
            idx = int(cands[int(np.argmin(np.linalg.norm(self._path_arr[cands, :2] - pos[:2], axis=1)))])
            self._progress_s = max(self._progress_s, float(self._path_s[idx]))

        if self.adg is not None:
            self.adg.update_progress(self.self_id, self._progress_s)
            s_cap = self.adg.max_allowed_s(self.self_id)
            if s_cap - self._progress_s <= 0.25:
                vel_robot = FrameTransformer.vel_world_to_robot(self.dim, vel, orient)
                desired_vel = np.zeros(3, dtype=float)
                return self._get_desired_action(desired_vel, vel_robot, orient), pose
        return super().get_action(obs)


class ADGPurePursuitCBF(ADGPurePursuit):
    """ADG + PurePursuit + Strict Distributed CBF-QP safety filter (`cbf_qp_filter`)."""

    def __init__(
        self,
        *args,
        peer_robots: Optional[Dict[str, DiffDriveRobot]] = None,
        priority: int = 1,
        priorities: Optional[Dict[str, int]] = None,
        sensing_radius: float = 3.0,
        safe_margin: float = 0.02,
        cbf_gamma: float = 2.2,
        lookahead_l: float = 0.04,
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.peer_robots = peer_robots if peer_robots is not None else {}
        self.priority = int(priority)
        self.priorities = priorities if priorities is not None else {}
        self.sensing_radius = float(sensing_radius)
        self.safe_margin = float(safe_margin)
        self.cbf_gamma = float(cbf_gamma)
        self.lookahead_l = float(lookahead_l)
        self.solve_times_us: List[float] = []
        self.wall_times_us: List[float] = []

    def _get_desired_vel(self, target_pose: np.ndarray, cur_pose: np.ndarray) -> np.ndarray:
        nom_vel = super()._get_desired_vel(target_pose, cur_pose)
        v_nom = max(0.0, float(nom_vel[0]))
        w_nom = float(nom_vel[2])
        if not self.peer_robots:
            return np.array([v_nom, 0.0, w_nom], dtype=float)

        theta_i = float(cur_pose[2])
        p_i = unicycle_to_lookahead_point(cur_pose, self.lookahead_l)
        u_nom = unicycle_vel_to_single_integrator(theta_i, v_nom, w_nom, self.lookahead_l)
        r_i = getattr(self.robot_model, "radius", 0.44)
        d_safe_eff = 2.0 * r_i + 2.0 * self.lookahead_l + self.safe_margin

        neighbors: List[Tuple[np.ndarray, np.ndarray]] = []
        weights: List[float] = []
        for peer_id, peer in self.peer_robots.items():
            if str(peer_id) == self.self_id:
                continue
            pos_j = peer.pos[:2]
            if float(np.linalg.norm(pos_j - cur_pose[:2])) > self.sensing_radius:
                continue
            r_j = getattr(peer, "radius", 0.44)
            theta_j = float(peer.orient[0]) if len(peer.orient) > 0 else 0.0
            heading_j = np.array([math.cos(theta_j), math.sin(theta_j)], dtype=float)
            p_j = unicycle_to_lookahead_point(np.array([pos_j[0], pos_j[1], theta_j]), self.lookahead_l)
            v_j_body = float(np.dot(peer.lin_vel[:2], heading_j))
            w_j_body = float(peer.ang_vel[0]) if len(peer.ang_vel) > 0 else 0.0
            u_j = unicycle_vel_to_single_integrator(theta_j, v_j_body, w_j_body, self.lookahead_l)
            d_safe_eff = max(d_safe_eff, r_i + r_j + 2.0 * self.lookahead_l + self.safe_margin)
            peer_prio = self.priorities.get(str(peer_id), 1)
            is_lower = (self.priority < peer_prio) or (
                self.priority == peer_prio and str(self.self_id) > str(peer_id)
            )
            weights.append(0.35 if is_lower else 0.65)
            neighbors.append((p_j, u_j))

        if not neighbors:
            return np.array([v_nom, 0.0, w_nom], dtype=float)

        u_safe, info = cbf_qp_filter(
            p_i=p_i,
            u_nom=u_nom,
            neighbors=neighbors,
            d_safe=d_safe_eff,
            gamma=self.cbf_gamma,
            u_max=self.max_lin_speed,
            weights=weights,
            theta=theta_i,
            lookahead_l=self.lookahead_l,
            omega_max=self.max_ang_speed,
        )
        self.solve_times_us.append(info["solve_time_us"])
        self.wall_times_us.append(info["wall_time_us"])
        v_safe, w_safe = single_integrator_to_unicycle_vel(
            theta_i,
            u_safe,
            lookahead_l=self.lookahead_l,
            v_max=self.max_lin_speed,
            omega_max=self.max_ang_speed,
            allow_reverse=False,
        )
        return np.array([v_safe, 0.0, w_safe], dtype=float)


_CACHED_WAREHOUSE: Optional[Tuple[Dict[str, Any], Grid, Any, Any, Any]] = None


def _get_cached_warehouse() -> Tuple[Dict[str, Any], Grid, Any, Any, Any]:
    global _CACHED_WAREHOUSE
    if _CACHED_WAREHOUSE is None:
        cfg = load_warehouse_config()
        grid = build_warehouse_grid(inflate_radius=1, cfg=cfg)
        _ = grid.esdf
        _, cfg_waypoints, cfg_goals, _, agv_colors = get_fleet_mission(cfg)
        _CACHED_WAREHOUSE = (cfg, grid, cfg_waypoints, cfg_goals, agv_colors)
    return _CACHED_WAREHOUSE


def simulate_controller(
    controller_name: str,
    world_paths: Dict[str, List[Tuple[float, float, float]]],
    priorities: Dict[str, int],
    ecbs_paths: Optional[Dict[str, List[Tuple[int, int]]]] = None,
    waypoints: Optional[Dict[str, List[Tuple[int, int]]]] = None,
    goals: Optional[Dict[str, Tuple[int, int]]] = None,
    steps: int = 305,
    robot_radius: float = 0.44,
    seed: int = 0,
    noise: float = 0.001,
    stall_windows: Optional[Dict[str, Tuple[int, int]]] = None,
) -> Dict[str, Any]:
    """
    Run an 8-AGV continuous differential-drive simulation in `ToySimulator` and measure:
      - True current-position and pre-bounce pairwise collision events (`d < 2 * robot_radius`)
      - Strict sequential two-stage completion (`Start -> Pickup -> Delivery`)
      - Makespan (s), mean speed (m/s), stop time (s), incomplete agents, and OSQP solve time (us).
    """
    random.seed(seed)
    np.random.seed(seed)
    cfg, grid, cfg_waypoints, cfg_goals, agv_colors = _get_cached_warehouse()
    waypoints = waypoints if waypoints is not None else cfg_waypoints
    goals = goals if goals is not None else cfg_goals

    env = ToySimulator(
        dim=2,
        dt=float(cfg["simulation"]["dt"]),
        obstacle_grid=grid,
        friction=0.015,
        restitution=0.3,
        noise=noise,
        robot_collisions=True,
        boundary_collisions=True,
    )

    robots: Dict[str, DiffDriveRobot] = {}
    for rid, p_world in world_paths.items():
        start_pose = np.array(p_world[0], dtype=float)
        robots[rid] = DiffDriveRobot(
            dim=2,
            radius=robot_radius,
            pose=start_pose,
            vel=np.zeros(3),
            max_lin_speed=2.8,
            max_ang_speed=3.14,
            action_min=np.array([-3.5, 0.0, -6.28]),
            action_max=np.array([3.5, 0.0, 6.28]),
            color=agv_colors[rid],
            text=rid,
        )
        env.add_robot(rid, robots[rid])

    adg: Optional[ActionDependencyGraph] = None
    if ecbs_paths is not None and controller_name in (
        "ADG_ORCA",
        "ADG_PurePursuit",
        "ADG_PurePursuit_CBF",
        "ADG_LinearMPC",
        "ConsensusMPC",
    ):
        adg = ActionDependencyGraph(grid, ecbs_paths, world_paths)

    controllers: Dict[str, Any] = {}
    for rid, robot in robots.items():
        obs_space, act_space = env.build_robot_spaces(robot)
        path_w = world_paths[rid]
        if controller_name == "PurePursuit":
            controllers[rid] = PurePursuit(
                obs_space,
                act_space,
                env.dt,
                path_w,
                robot_model=robot,
                obstacle_grid=grid,
                max_lin_speed=2.6,
                max_ang_speed=3.14,
                lookahead_distance=2.4,
                goal_dist_tol=0.70,
            )
        elif controller_name == "RPP":
            controllers[rid] = RPP(
                obs_space,
                act_space,
                env.dt,
                path_w,
                robot_model=robot,
                obstacle_grid=grid,
                max_lin_speed=2.6,
                max_ang_speed=3.14,
                lookahead_distance=2.4,
                d_prox=1.2,
                goal_dist_tol=0.70,
            )
        elif controller_name == "DWA":
            controllers[rid] = DWA(
                obs_space,
                act_space,
                env.dt,
                path_w,
                robot_model=robot,
                obstacle_grid=grid,
                max_lin_speed=2.6,
                max_ang_speed=3.14,
                lookahead_distance=2.6,
                vel_reso=np.array([0.4, 0.4, np.deg2rad(25)]),
                goal_dist_tol=0.70,
            )
        elif controller_name in ("ORCA", "ADG_ORCA"):
            controllers[rid] = ORCAController(
                obs_space,
                act_space,
                env.dt,
                path_w,
                robot_model=robot,
                obstacle_grid=grid,
                max_lin_speed=2.6,
                max_ang_speed=3.14,
                lookahead_distance=2.4,
                goal_dist_tol=0.70,
                self_id=rid,
                peer_robots=robots,
                adg=adg if controller_name == "ADG_ORCA" else None,
                time_horizon=2.0,
                safe_margin=0.02,
                lookahead_l=0.04,
            )
        elif controller_name == "ADG_PurePursuit":
            controllers[rid] = ADGPurePursuit(
                obs_space,
                act_space,
                env.dt,
                path_w,
                robot_model=robot,
                obstacle_grid=grid,
                max_lin_speed=2.6,
                max_ang_speed=3.14,
                lookahead_distance=2.4,
                goal_dist_tol=0.70,
                self_id=rid,
                adg=adg,
            )
        elif controller_name == "ADG_PurePursuit_CBF":
            controllers[rid] = ADGPurePursuitCBF(
                obs_space,
                act_space,
                env.dt,
                path_w,
                robot_model=robot,
                obstacle_grid=grid,
                max_lin_speed=2.6,
                max_ang_speed=3.14,
                lookahead_distance=2.4,
                goal_dist_tol=0.70,
                self_id=rid,
                adg=adg,
                peer_robots=robots,
                priority=priorities[rid],
                priorities=priorities,
                sensing_radius=3.0,
                safe_margin=0.02,
                cbf_gamma=2.2,
                lookahead_l=0.04,
            )
        elif controller_name in ("ADG_LinearMPC", "LinearMPC_CBF", "ConsensusMPC"):
            controllers[rid] = ConsensusMPC(
                obs_space,
                act_space,
                env.dt,
                path_w,
                robot_model=robot,
                obstacle_grid=grid,
                max_lin_speed=2.6,
                max_ang_speed=3.14,
                lookahead_distance=2.4,
                goal_dist_tol=0.70,
                self_id=rid,
                peer_robots=robots,
                priority=priorities[rid],
                priorities=priorities,
                adg=adg if controller_name in ("ADG_LinearMPC", "ConsensusMPC") else None,
                use_cbf=(controller_name in ("LinearMPC_CBF", "ConsensusMPC")),
                sensing_radius=3.0,
                safe_margin=0.02,
                cbf_gamma=2.2,
                lookahead_l=0.04,
            )
        else:
            raise ValueError(f"Unknown controller: {controller_name}")

    env.reset()
    min_pair_dist_history: List[float] = []
    speed_history: Dict[str, List[float]] = {rid: [] for rid in robots}
    pose_history: Dict[str, List[np.ndarray]] = {rid: [] for rid in robots}
    collision_events = 0
    in_collision_pairs = set()

    rids = sorted(robots.keys())
    contact_limit = 2.0 * robot_radius + 0.02  # 0.90m physical contact envelope
    pickup_xy = {rid: np.asarray(grid.map_to_world(waypoints[rid][0]), dtype=float) for rid in rids}
    delivery_xy = {rid: np.asarray(grid.map_to_world(goals[rid]), dtype=float) for rid in rids}
    picked_up: Dict[str, bool] = {rid: False for rid in rids}
    completed: Dict[str, bool] = {rid: False for rid in rids}
    finish_step: Dict[str, int] = {rid: steps for rid in rids}
    stop_steps: Dict[str, int] = {rid: 0 for rid in rids}

    for step_idx in range(steps):
        actions = {}
        for rid, robot in robots.items():
            pos_cur = robot.pos[:2]
            spd = float(np.linalg.norm(robot.lin_vel[:2]))
            pose_history[rid].append(robot.pose.copy())
            speed_history[rid].append(spd)

            # Sequential Stage 1 -> Stage 2 verification
            if not picked_up[rid] and float(np.linalg.norm(pos_cur - pickup_xy[rid])) <= 1.65:
                picked_up[rid] = True
            if picked_up[rid] and not completed[rid]:
                if float(np.linalg.norm(pos_cur - delivery_xy[rid])) <= 0.95:
                    completed[rid] = True
                    finish_step[rid] = step_idx

            if not completed[rid] and spd < 0.08:
                stop_steps[rid] += 1

            ob = robot.get_observation(env)
            act, _ = controllers[rid].get_action(ob)

            # Optional external delay / actuator stall injection
            if stall_windows is not None and rid in stall_windows:
                t_s, t_e = stall_windows[rid]
                if t_s <= step_idx <= t_e:
                    robot.vel[:] = 0.0
                    act = np.zeros_like(act)
            actions[rid] = act

        # Check both current true center distance AND next-step kinematic distance before physics bounce
        step_min_dist = float("inf")
        for i in range(len(rids)):
            for j in range(i + 1, len(rids)):
                ra, rb = robots[rids[i]], robots[rids[j]]
                d_cur = float(np.linalg.norm(ra.pos[:2] - rb.pos[:2]))
                pa_nxt = ra.pos[:2] + ra.lin_vel[:2] * env.dt
                pb_nxt = rb.pos[:2] + rb.lin_vel[:2] * env.dt
                d_nxt = float(np.linalg.norm(pa_nxt - pb_nxt))
                d_ab = min(d_cur, d_nxt)
                step_min_dist = min(step_min_dist, d_ab)
                pair_key = (rids[i], rids[j])
                if d_ab < contact_limit:
                    if pair_key not in in_collision_pairs:
                        collision_events += 1
                        in_collision_pairs.add(pair_key)
                else:
                    in_collision_pairs.discard(pair_key)

        min_pair_dist_history.append(step_min_dist)
        env.step(actions)

    nav_errors = []
    traj_lengths = []
    active_speeds = []
    solve_times_all: List[float] = []
    wall_times_all: List[float] = []
    incomplete_count = 0

    for rid, robot in robots.items():
        poses = np.array(pose_history[rid])
        diffs = np.diff(poses[:, :2], axis=0)
        length = float(np.sum(np.linalg.norm(diffs, axis=1)))
        goal_xy = delivery_xy[rid]
        nav_err = float(np.linalg.norm(poses[-1, :2] - goal_xy))
        if picked_up[rid] and nav_err <= 0.90:
            completed[rid] = True
        if not completed[rid]:
            incomplete_count += 1
        nav_errors.append(nav_err)
        traj_lengths.append(length)
        f_idx = max(1, finish_step[rid])
        active_speeds.extend(speed_history[rid][:f_idx])
        ctrl_obj = controllers[rid]
        if hasattr(ctrl_obj, "solve_times_us"):
            solve_times_all.extend(getattr(ctrl_obj, "solve_times_us"))
        if hasattr(ctrl_obj, "wall_times_us"):
            wall_times_all.extend(getattr(ctrl_obj, "wall_times_us"))

    makespan_s = float(max(finish_step.values()) * env.dt)
    total_stop_time_s = float(sum(stop_steps.values()) * env.dt)
    osqp_mean_us = float(np.mean(solve_times_all)) if solve_times_all else 0.0
    osqp_p95_us = float(np.percentile(solve_times_all, 95)) if solve_times_all else 0.0
    wall_mean_us = float(np.mean(wall_times_all)) if wall_times_all else 0.0
    wall_p95_us = float(np.percentile(wall_times_all, 95)) if wall_times_all else 0.0

    return {
        "controller": controller_name,
        "collision_events": collision_events,
        "min_inter_robot_dist": round(float(min(min_pair_dist_history)), 3),
        "pickup_rate": round(float(np.mean(list(picked_up.values()))), 3),
        "two_stage_success_rate": round(float(np.mean(list(completed.values()))), 3),
        "success_rate": round(float(np.mean(list(completed.values()))), 3),
        "incomplete_count": incomplete_count,
        "makespan_s": round(makespan_s, 2),
        "mean_speed_mps": round(float(np.mean(active_speeds)) if active_speeds else 0.0, 3),
        "total_stop_time_s": round(total_stop_time_s, 2),
        "mean_nav_error": round(float(np.mean(nav_errors)), 3),
        "mean_traj_length": round(float(np.mean(traj_lengths)), 2),
        "osqp_mean_us": round(osqp_mean_us, 1),
        "osqp_p95_us": round(osqp_p95_us, 1),
        "wall_mean_us": round(wall_mean_us, 1),
        "wall_p95_us": round(wall_p95_us, 1),
        "min_dist_series": min_pair_dist_history,
        "speed_series": speed_history,
        "pose_series": pose_history,
    }


def run_control_ablation(
    world_paths: Dict[str, List[Tuple[float, float, float]]],
    priorities: Dict[str, int],
    ecbs_paths: Optional[Dict[str, List[Tuple[int, int]]]] = None,
    steps: int = 305,
) -> Dict[str, Dict[str, Any]]:
    """Run all 10 controller variants on the 8-AGV warehouse and save `results/control_ablation.csv`."""
    method_order = (
        "PurePursuit",
        "RPP",
        "DWA",
        "ORCA",
        "ADG_ORCA",
        "ADG_PurePursuit",
        "ADG_PurePursuit_CBF",
        "ADG_LinearMPC",
        "LinearMPC_CBF",
        "ConsensusMPC",
    )
    results: Dict[str, Dict[str, Any]] = {}
    csv_rows: List[Dict[str, Any]] = []

    for ctrl_name in method_order:
        res = simulate_controller(
            ctrl_name,
            world_paths,
            priorities,
            ecbs_paths=ecbs_paths,
            steps=steps,
        )
        results[ctrl_name] = res
        csv_rows.append(
            {
                k: v
                for k, v in res.items()
                if k not in ("min_dist_series", "speed_series", "pose_series")
            }
        )

    out_csv = ROOT / "results" / "control_ablation.csv"
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(csv_rows[0].keys()))
        writer.writeheader()
        writer.writerows(csv_rows)
    return results


def run_perturbation_experiment(
    world_paths: Dict[str, List[Tuple[float, float, float]]],
    priorities: Dict[str, int],
    ecbs_paths: Dict[str, List[Tuple[int, int]]],
    num_trials: int = 20,
) -> List[Dict[str, Any]]:
    """
    Evaluate controller robustness across `num_trials` genuinely randomized seeds:
      - In each seed `1000 + trial_idx`, 2 AGVs are sampled uniformly at random without replacement
        (`rng.sample(rids, k=2)`), stalled at a random step `t_start ~ Uniform{35, ..., 110}` for a
        random duration `duration ~ Uniform{20, ..., 35}` (`2.0-3.5s`), plus Gaussian velocity noise
        (`noise=0.015`).
    """
    rids = sorted(world_paths.keys())
    stall_scenarios: List[Dict[str, Tuple[int, int]]] = []
    for trial_idx in range(num_trials):
        rng = random.Random(1000 + trial_idx)
        chosen = rng.sample(rids, k=2)
        scenario: Dict[str, Tuple[int, int]] = {}
        for rid in chosen:
            t_s = rng.randint(35, 110)
            dur = rng.randint(20, 35)
            scenario[rid] = (t_s, t_s + dur)
        stall_scenarios.append(scenario)

    methods = (
        "PurePursuit",
        "ORCA",
        "ADG_ORCA",
        "ADG_PurePursuit",
        "ADG_PurePursuit_CBF",
        "ADG_LinearMPC",
        "LinearMPC_CBF",
        "ConsensusMPC",
    )
    summary_rows: List[Dict[str, Any]] = []

    for m in methods:
        coll_list: List[int] = []
        min_d_list: List[float] = []
        succ_list: List[float] = []
        mk_list: List[float] = []
        for trial_idx, stall_map in enumerate(stall_scenarios):
            res = simulate_controller(
                m,
                world_paths,
                priorities,
                ecbs_paths=ecbs_paths,
                steps=335,
                seed=100 + trial_idx,
                noise=0.015,
                stall_windows=stall_map,
            )
            coll_list.append(int(res["collision_events"]))
            min_d_list.append(float(res["min_inter_robot_dist"]))
            succ_list.append(float(res["two_stage_success_rate"]))
            mk_list.append(float(res["makespan_s"]))

        summary_rows.append(
            {
                "controller": m,
                "trials": num_trials,
                "mean_collisions": round(float(np.mean(coll_list)), 2),
                "max_collisions": int(np.max(coll_list)),
                "min_clearance_m": round(float(np.min(min_d_list)), 3),
                "mean_two_stage_success_rate": round(float(np.mean(succ_list)), 3),
                "mean_makespan_s": round(float(np.mean(mk_list)), 2),
            }
        )

    out_csv = ROOT / "results" / "perturbation_robustness.csv"
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(summary_rows[0].keys()))
        writer.writeheader()
        writer.writerows(summary_rows)
    return summary_rows


def plot_ablation_dashboard(
    planning_summary: Dict[str, Dict[str, Any]],
    control_results: Dict[str, Dict[str, Any]],
    output_png: Path,
    robot_radius: float = 0.44,
) -> None:
    """Render the 2x2 quantitative ablation dashboard (`assets/multi_agv_ablation_dashboard.png`)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    cfg = load_warehouse_config()
    _, _, _, _, agv_colors = get_fleet_mission(cfg)

    fig, axes = plt.subplots(2, 2, figsize=(14.4, 9.0), dpi=160)
    fig.patch.set_facecolor("#ffffff")
    ax_dist, ax_ctrl, ax_plan, ax_vel = axes[0, 0], axes[0, 1], axes[1, 0], axes[1, 1]

    ctrl_colors = {
        "PurePursuit": "#94a3b8",
        "ORCA": "#8b5cf6",
        "ADG_ORCA": "#ec4899",
        "ADG_PurePursuit": "#0ea5e9",
        "ADG_PurePursuit_CBF": "#06b6d4",
        "ADG_LinearMPC": "#f59e0b",
        "LinearMPC_CBF": "#6366f1",
        "ConsensusMPC": "#10b981",
    }
    ctrl_short = {
        "PurePursuit": "PurePursuit",
        "ORCA": "NH-ORCA (QP)",
        "ADG_ORCA": "ADG + NH-ORCA",
        "ADG_PurePursuit": "ADG + PP",
        "ADG_PurePursuit_CBF": "ADG + PP + CBF",
        "ADG_LinearMPC": "ADG + LinearMPC",
        "LinearMPC_CBF": "LinearMPC + CBF",
        "ConsensusMPC": "ADG + LinearMPC + CBF-QP",
    }
    for name, res in control_results.items():
        if name not in ctrl_colors:
            continue
        t_axis = np.arange(len(res["min_dist_series"])) * 0.1
        lw = 2.6 if name == "ConsensusMPC" else 1.4
        ls = "-" if name in ("ConsensusMPC", "ADG_PurePursuit_CBF", "LinearMPC_CBF") else "--"
        label = (
            f"{ctrl_short[name]} ({res['collision_events']} Coll., min={res['min_inter_robot_dist']:.2f}m)"
        )
        ax_dist.plot(
            t_axis, res["min_dist_series"], linestyle=ls, linewidth=lw, color=ctrl_colors[name], label=label
        )

    contact_limit = 2.0 * robot_radius
    ax_dist.axhline(
        contact_limit,
        color="#dc2626",
        linestyle=":",
        linewidth=1.6,
        label=f"Physical Contact Limit (2R = {contact_limit:.2f}m)",
    )
    ax_dist.set_title(
        "(a) Minimum Inter-AGV Distance Across 8-AGV Warehouse", fontsize=10.5, fontweight="bold"
    )
    ax_dist.set_xlabel("Simulation Time (s)")
    ax_dist.set_ylabel("Min Center-to-Center Distance (m)")
    ax_dist.set_ylim(0.25, 14.0)
    ax_dist.grid(True, linestyle="--", alpha=0.4)
    ax_dist.legend(loc="upper right", fontsize=6.9)

    ctrl_names = [
        k
        for k in (
            "PurePursuit",
            "ORCA",
            "ADG_ORCA",
            "ADG_PurePursuit",
            "ADG_PurePursuit_CBF",
            "ADG_LinearMPC",
            "LinearMPC_CBF",
            "ConsensusMPC",
        )
        if k in control_results
    ]
    x = np.arange(len(ctrl_names))
    width = 0.36
    collisions = [control_results[k]["collision_events"] for k in ctrl_names]
    nav_errs = [control_results[k]["mean_nav_error"] for k in ctrl_names]

    bars1 = ax_ctrl.bar(x - width / 2, collisions, width, label="Collision Events", color="#ef4444", alpha=0.85)
    bars2 = ax_ctrl.bar(
        x + width / 2, nav_errs, width, label="Mean Terminal Error (m)", color="#2563eb", alpha=0.85
    )
    for b in bars1:
        ax_ctrl.text(
            b.get_x() + b.get_width() / 2,
            b.get_height() + 0.12,
            f"{int(b.get_height())}",
            ha="center",
            fontsize=7.5,
            fontweight="bold",
        )
    for b in bars2:
        ax_ctrl.text(
            b.get_x() + b.get_width() / 2,
            b.get_height() + 0.12,
            f"{b.get_height():.2f}m",
            ha="center",
            fontsize=7.2,
        )

    ax_ctrl.set_xticks(x)
    ax_ctrl.set_xticklabels(
        [
            "Pure\nPursuit",
            "NH-ORCA\n(QP)",
            "ADG +\nNH-ORCA",
            "ADG +\nPP",
            "ADG + PP\n+ CBF-QP",
            "ADG +\nLinearMPC",
            "LinearMPC\n+ CBF-QP",
            "ADG + LinearMPC\n+ CBF-QP",
        ][: len(ctrl_names)],
        fontsize=7.3,
    )
    ax_ctrl.set_title(
        "(b) Component-Isolation Controller Ablation: Collisions & Terminal Error",
        fontsize=10.5,
        fontweight="bold",
    )
    ax_ctrl.set_ylabel("Collision Count / Error (m)")
    max_bar_val = max(max(collisions, default=1), max(nav_errs, default=1.0))
    ax_ctrl.set_ylim(0, max_bar_val * 1.25 + 0.5)
    ax_ctrl.grid(axis="y", linestyle="--", alpha=0.4)
    ax_ctrl.legend(loc="upper left", fontsize=8.2)

    plan_keys = [k for k in ("Standard_CBS", "CBS_Turn", "ECBS_NoTurn", "Proposed_ECBS") if k in planning_summary]
    if not plan_keys:
        plan_keys = list(planning_summary.keys())
    plan_labels = [
        "CBS\n(w=1.0, λ=0)",
        "CBS + Turn\n(w=1.0, λ=0.45)",
        "ECBS\n(w=1.08, λ=0)",
        "ECBS + Turn\n(w=1.08, λ=0.45)",
    ][: len(plan_keys)]
    px = np.arange(len(plan_keys))
    exp_nodes = [planning_summary[k]["low_level_expanded"] for k in plan_keys]
    turns_list = [planning_summary[k]["total_turns"] for k in plan_keys]

    pbars1 = ax_plan.bar(px - width / 2, exp_nodes, width, label="Low-Level Expanded States", color="#0284c7")
    pbars2 = ax_plan.bar(
        px + width / 2,
        turns_list,
        width,
        label="Total 90° Turns",
        color="#f97316",
    )
    for b in pbars1:
        ax_plan.text(
            b.get_x() + b.get_width() / 2,
            b.get_height() + 55,
            f"{int(b.get_height())}",
            ha="center",
            fontsize=8.0,
            fontweight="bold",
        )
    for b in pbars2:
        ax_plan.text(
            b.get_x() + b.get_width() / 2,
            b.get_height() + 55,
            f"{int(b.get_height())} turns",
            ha="center",
            fontsize=8.0,
            fontweight="bold",
        )

    ax_plan.set_xticks(px)
    ax_plan.set_xticklabels(plan_labels, fontsize=8.4)
    ax_plan.set_title(
        "(c) 4-Group MAPF Ablation: Focal Heuristic (w) vs. Turn Penalty (λ)", fontsize=10.5, fontweight="bold"
    )
    ax_plan.set_ylabel("Expanded States / Turn Count")
    ax_plan.set_ylim(0, max(exp_nodes, default=1000) * 1.22)
    ax_plan.grid(axis="y", linestyle="--", alpha=0.4)
    ax_plan.legend(loc="upper right", fontsize=8.2)

    cmpc_speeds = control_results["ConsensusMPC"]["speed_series"]
    for rid, s_list in cmpc_speeds.items():
        t_axis = np.arange(len(s_list)) * 0.1
        ax_vel.plot(
            t_axis,
            s_list,
            linewidth=1.8,
            color=agv_colors[rid],
            label=f"AGV-{rid} (Prio {9-int(rid)})",
        )
    ax_vel.set_title(
        "(d) 8-AGV Velocity Profiles (ADG + Linear-MPC + CBF-QP)",
        fontsize=10.5,
        fontweight="bold",
    )
    ax_vel.set_xlabel("Simulation Time (s)")
    ax_vel.set_ylabel("Linear Speed ||v|| (m/s)")
    ax_vel.grid(True, linestyle="--", alpha=0.4)
    ax_vel.legend(loc="upper right", fontsize=7.5, ncol=2)

    fig.suptitle(
        "Quantitative Planning & Control Ablation on 72x44m Warehouse (8 AGVs, Two-Stage Pickup-Delivery)",
        fontsize=12.8,
        fontweight="bold",
        y=0.99,
    )
    fig.tight_layout()
    output_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_png, dpi=160)
    plt.close(fig)


def main() -> int:
    cfg = load_warehouse_config()
    grid = build_warehouse_grid(inflate_radius=1, cfg=cfg)
    _, waypoints, _, priorities, _ = get_fleet_mission(cfg)

    ecbs_paths, _, planning_summary = run_planning_ablation(repeats=10)
    world_paths, _ = optimize_corridor_bspline_paths(grid, ecbs_paths, waypoints, cfg)

    print("=== [1/2] Running 6-Method Controller Ablation on 8-AGV Warehouse ===", flush=True)
    control_results = run_control_ablation(world_paths, priorities, ecbs_paths=ecbs_paths, steps=305)
    for name, res in control_results.items():
        print(
            f"  {name:16s} | coll={res['collision_events']:2d} | "
            f"min_d={res['min_inter_robot_dist']:5.2f}m | "
            f"2stage={res['two_stage_success_rate']*100:5.1f}% | "
            f"mk={res['makespan_s']:5.1f}s | "
            f"err={res['mean_nav_error']:5.2f}m | "
            f"osqp={res['osqp_mean_us']:6.1f}us",
            flush=True,
        )

    print("\n=== [2/2] Running 20-Seed Randomized Delay & Perturbation Robustness Experiment ===", flush=True)
    pert_rows = run_perturbation_experiment(world_paths, priorities, ecbs_paths, num_trials=20)
    for r in pert_rows:
        print(
            f"  {r['controller']:16s} | mean_coll={r['mean_collisions']:4.2f} | "
            f"min_clear={r['min_clearance_m']:5.2f}m | "
            f"2stage_succ={r['mean_two_stage_success_rate']*100:5.1f}% | "
            f"makespan={r['mean_makespan_s']:5.1f}s",
            flush=True,
        )

    plot_ablation_dashboard(
        planning_summary,
        control_results,
        ROOT / "assets" / "multi_agv_ablation_dashboard.png",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
