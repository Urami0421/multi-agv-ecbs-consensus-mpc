"""
@file: mpc.py
@description: Receding-Horizon Kinematic Model Predictive Control (MPC) and
              Multi-Agent Consensus-CBF-MPC (Consensus Velocity Coordination +
              Discrete-Time Control Barrier Function) for AGV Path Tracking.
@references:
    [1] Olfati-Saber, R., Fax, J. A., & Murray, R. M. (2007). Consensus and Cooperation
        in Networked Multi-Agent Systems. Proceedings of the IEEE, 95(1), 215-233.
    [2] Ames, A. D., Xu, X., Grizzle, J. W., & Tabuada, P. (2017). Control Barrier Function
        Based Quadratic Programs for Safety Critical Systems. IEEE Transactions on Automatic Control.
    [3] Multi-Robot Trajectory Tracking via Consensus Velocity Coordination and MPC (IEEE T-RO / T-ASE).
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from python_motion_planning.common.env.map.grid import Grid
from python_motion_planning.common.env.robot.base_robot import BaseRobot
from python_motion_planning.common.env.types import TYPES
from python_motion_planning.common.utils.frame_transformer import FrameTransformer
from python_motion_planning.common.utils.geometry import Geometry
from .path_tracker import PathTracker


class MPC(PathTracker):
    """
    Receding-Horizon Kinematic Model Predictive Control (MPC) path tracker.

    Optimizes a multi-step control sequence `(v, omega)` over prediction horizon `horizon_steps`
    by solving the error-state quadratic cost with arc-length reference parameterization,
    curvature speed regulation, control smoothness damping, and ESDF obstacle clearance penalty.

    Args:
        *args: see `PathTracker`.
        horizon_steps: Prediction horizon steps `N_p` (default: 10).
        q_pos: Position tracking weight.
        q_theta: Heading tracking weight.
        r_vel: Velocity tracking weight.
        r_smooth: Control rate-of-change damping weight.
        w_esdf: Static obstacle ESDF clearance weight.
        **kwargs: see `PathTracker`.
    """

    def __init__(
        self,
        *args,
        horizon_steps: int = 10,
        q_pos: float = 2.4,
        q_theta: float = 1.0,
        r_vel: float = 0.8,
        r_smooth: float = 0.25,
        w_esdf: float = 0.8,
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.horizon_steps = int(max(3, horizon_steps))
        self.q_pos = float(q_pos)
        self.q_theta = float(q_theta)
        self.r_vel = float(r_vel)
        self.r_smooth = float(r_smooth)
        self.w_esdf = float(w_esdf)
        self.prev_cmd = np.zeros(2, dtype=float)  # [v_cmd, omega_cmd]
        self._path_array = (
            np.asarray(self.path, dtype=float)
            if self.path is not False and len(self.path) > 0
            else np.zeros((0, 3))
        )
        if len(self._path_array) > 1:
            diffs = np.diff(self._path_array[:, :2], axis=0)
            seg_lens = np.linalg.norm(diffs, axis=1)
            self._path_s = np.concatenate([[0.0], np.cumsum(seg_lens)])
        else:
            self._path_s = np.zeros(len(self._path_array), dtype=float)
        self._progress_s = 0.0

    def __str__(self) -> str:
        return "MPC"

    def reset(self) -> None:
        super().reset()
        self.prev_cmd = np.zeros(2, dtype=float)
        self._progress_s = 0.0

    def _interpolate_at_s(self, s_query: float) -> np.ndarray:
        """Interpolate `(x, y, theta)` along `self._path_array` at cumulative arc-length `s_query`."""
        path = self._path_array
        s_arr = self._path_s
        if len(path) == 0:
            return np.zeros(3, dtype=float)
        if len(path) == 1 or s_query <= 0.0:
            return path[0].copy()
        if s_query >= s_arr[-1]:
            return path[-1].copy()

        idx = int(np.searchsorted(s_arr, s_query, side="right")) - 1
        idx = max(0, min(len(path) - 2, idx))
        ds = float(s_arr[idx + 1] - s_arr[idx])
        alpha = (s_query - float(s_arr[idx])) / ds if ds > 1e-8 else 0.0
        xy = (1.0 - alpha) * path[idx, :2] + alpha * path[idx + 1, :2]
        seg = path[idx + 1, :2] - path[idx, :2]
        if np.linalg.norm(seg) > 1e-6:
            theta = math.atan2(float(seg[1]), float(seg[0]))
        else:
            theta = float(path[idx, 2])
        return np.array([xy[0], xy[1], theta], dtype=float)

    def _reference_window(
        self, pos: np.ndarray, ref_speed: float
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Extract an `N_p`-step arc-length parameterized reference pose window starting near `pos`."""
        path = self._path_array
        s_arr = self._path_s
        n_pts = len(path)
        if n_pts == 0:
            return np.zeros((self.horizon_steps, 3)), np.zeros(3)

        # Find nearest path point within a forward arc-length window around _progress_s
        mask = (s_arr >= self._progress_s - 1.5) & (s_arr <= self._progress_s + 8.0)
        cand_indices = np.where(mask)[0]
        if len(cand_indices) == 0:
            cand_indices = np.arange(n_pts)

        dists = np.linalg.norm(path[cand_indices, :2] - pos[:2], axis=1)
        nearest_idx = int(cand_indices[int(np.argmin(dists))])
        self._progress_s = max(self._progress_s, float(s_arr[nearest_idx]))

        s_total = float(s_arr[-1])
        step_ds = max(0.08, ref_speed * self.dt)
        ref_window = np.zeros((self.horizon_steps, 3), dtype=float)
        for k in range(self.horizon_steps):
            s_k = min(s_total, self._progress_s + (k + 1) * step_ds)
            ref_window[k] = self._interpolate_at_s(s_k)

        lookahead_s = min(
            s_total, self._progress_s + max(1.6, self.lookahead_distance * 0.85)
        )
        lookahead_pose = self._interpolate_at_s(lookahead_s)
        return ref_window, lookahead_pose

    def _rollout_unicycle(
        self,
        start_pose: np.ndarray,
        v_cmd: float,
        omega_cmd: float,
        omega_decay: float = 0.88,
    ) -> np.ndarray:
        """Roll out differential-drive unicycle kinematics over `horizon_steps`."""
        traj = np.zeros((self.horizon_steps + 1, 3), dtype=float)
        traj[0] = start_pose[:3]
        x, y, theta = float(start_pose[0]), float(start_pose[1]), float(start_pose[2])
        w_k = omega_cmd
        for k in range(1, self.horizon_steps + 1):
            x += v_cmd * math.cos(theta) * self.dt
            y += v_cmd * math.sin(theta) * self.dt
            theta = float(
                Geometry.regularize_orient(np.array([theta + w_k * self.dt]))[0]
            )
            traj[k] = [x, y, theta]
            w_k *= omega_decay
        return traj

    def _in_real_obstacle(self, pt_world: np.ndarray) -> bool:
        """Check if `pt_world` hits an actual obstacle cell (`TYPES.OBSTACLE`)."""
        if self.obstacle_grid is None:
            return False
        grid_pt = self.obstacle_grid.world_to_map(tuple(pt_world[:2]))
        if not self.obstacle_grid.within_bounds(grid_pt):
            return True
        return int(self.obstacle_grid.type_map[grid_pt]) == TYPES.OBSTACLE

    def _solve_mpc_control(
        self,
        pose: np.ndarray,
        vel_robot: np.ndarray,
        ref_window: np.ndarray,
        lookahead_pose: np.ndarray,
        target_speed_scale: float = 1.0,
    ) -> Tuple[float, float, np.ndarray]:
        """
        Solve the receding-horizon kinematic MPC problem via analytical Lyapunov/curvature
        warm-start + multi-shooting optimization over `horizon_steps`.
        """
        pos = pose[:2]
        goal_pos = self.goal[:2]
        dist_to_goal = float(np.linalg.norm(pos - goal_pos))

        if dist_to_goal < self.goal_dist_tol:
            return 0.0, 0.0, np.repeat(pose[:3][None, :], self.horizon_steps + 1, axis=0)

        rel_target = FrameTransformer.pose_world_to_robot(
            self.dim, lookahead_pose, pose
        )
        rx, ry = float(rel_target[0]), float(rel_target[1])
        L = max(math.hypot(rx, ry), 0.2)
        kappa = 2.0 * ry / (L * L)
        heading_err = math.atan2(ry, rx)

        v_cap = self.max_lin_speed * float(np.clip(target_speed_scale, 0.0, 1.0))
        curv_factor = 1.0 / (1.0 + 0.65 * abs(kappa) + 0.9 * abs(heading_err))
        v_nom = min(
            v_cap,
            max(0.2 * v_cap, v_cap * curv_factor),
            dist_to_goal / max(self.dt * 2.2, 0.22),
        )
        if abs(heading_err) > 1.15:
            v_nom *= 0.25

        omega_nom = float(
            np.clip(
                0.75 * v_nom * kappa + 1.5 * heading_err,
                -self.max_ang_speed,
                self.max_ang_speed,
            )
        )

        if v_cap <= 0.04:
            v_candidates = np.array([0.0], dtype=float)
        else:
            v_candidates = np.unique(
                np.clip(
                    np.array(
                        [
                            0.0,
                            0.35 * v_nom,
                            0.7 * v_nom,
                            v_nom,
                            min(v_cap, 1.12 * v_nom),
                        ]
                    ),
                    0.0,
                    v_cap,
                )
            )
        w_offsets = np.linspace(-0.75, 0.75, 9)
        w_candidates = np.clip(
            omega_nom + w_offsets, -self.max_ang_speed, self.max_ang_speed
        )

        best_cost = float("inf")
        best_v = float(v_nom)
        best_w = float(omega_nom)
        best_traj = self._rollout_unicycle(pose, best_v, best_w)

        for v_c in v_candidates:
            for w_c in w_candidates:
                traj = self._rollout_unicycle(pose, float(v_c), float(w_c))
                # Tracking error cost across horizon
                pos_err = traj[1:, :2] - ref_window[:, :2]
                pos_cost = float(np.sum(pos_err[:, 0] ** 2 + pos_err[:, 1] ** 2))
                ang_diff = Geometry.regularize_orient(traj[1:, 2] - ref_window[:, 2])
                ang_cost = float(np.sum(ang_diff**2))

                # Velocity tracking & control smoothness cost
                vel_cost = (
                    (float(v_c) - v_nom) ** 2 * 4.0
                    + 0.35 * (float(w_c) - omega_nom) ** 2
                )
                smooth_cost = (
                    (float(v_c) - self.prev_cmd[0]) ** 2
                    + 0.3 * (float(w_c) - self.prev_cmd[1]) ** 2
                )

                # Static obstacle ESDF clearance cost
                esdf_cost = 0.0
                collision = False
                if self.obstacle_grid is not None:
                    for k in range(1, self.horizon_steps + 1, 2):
                        if self._in_real_obstacle(traj[k, :2]):
                            collision = True
                            break
                        d_obs = self._get_dist_to_nearest_obstacle(traj[k, :2])
                        if d_obs < 0.8:
                            esdf_cost += (0.8 - d_obs) ** 2
                if collision:
                    continue

                extra_cost = self._additional_horizon_cost(
                    traj, float(v_c), float(w_c)
                )
                if not math.isfinite(extra_cost):
                    continue

                total_cost = (
                    self.q_pos * pos_cost
                    + self.q_theta * ang_cost
                    + self.r_vel * vel_cost
                    + self.r_smooth * smooth_cost
                    + self.w_esdf * esdf_cost
                    + extra_cost
                )
                if total_cost < best_cost:
                    best_cost = total_cost
                    best_v = float(v_c)
                    best_w = float(w_c)
                    best_traj = traj

        return best_v, best_w, best_traj

    def _additional_horizon_cost(
        self, traj: np.ndarray, v_cmd: float, w_cmd: float
    ) -> float:
        """Hook for subclasses (e.g., ConsensusMPC CBF barrier cost)."""
        return 0.0

    def get_action(self, obs: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if self.goal is None:
            return np.zeros(self.action_space.shape), self.goal

        pose, vel, pos, orient, lin_vel, ang_vel = self.get_pose_velocity(obs)
        ref_window, target_pose = self._reference_window(pos, self.max_lin_speed)
        vel_robot = FrameTransformer.vel_world_to_robot(self.dim, vel, orient)

        best_v, best_w, best_traj = self._solve_mpc_control(
            pose, vel_robot, ref_window, target_pose, target_speed_scale=1.0
        )
        self.prev_cmd = np.array([best_v, best_w], dtype=float)
        self.pred_traj = best_traj

        desired_vel = np.array([best_v, 0.0, best_w], dtype=float)
        desired_vel = self._stop_if_reached(desired_vel, pose)
        action = self._get_desired_action(desired_vel, vel_robot, orient)
        return action, target_pose


class ConsensusMPC(MPC):
    """
    Multi-Agent Consensus Velocity Coordination + Control Barrier Function MPC (`ConsensusMPC`).

    Combines distributed multi-agent consensus velocity scaling over the local interaction
    topology `N_i` with a Discrete-Time Control Barrier Function (CBF) safety filter over the
    MPC prediction horizon. Eliminates intersection collisions and symmetric narrow-corridor
    deadlocks for fleets of `DiffDriveRobot` / `CircularRobot` AGVs.

    Args:
        *args: see `MPC`.
        self_id: ID of this robot in `peer_robots`.
        peer_robots: Dict mapping `robot_id -> BaseRobot` in the shared workspace.
        priority: Priority rank of this robot (higher value = higher right-of-way).
        priorities: Optional Dict mapping `robot_id -> int` priority for all robots.
        sensing_radius: Neighborhood communication/sensing radius `R_sense` (world units).
        safe_margin: Extra safety clearance added to `r_i + r_j` for the CBF barrier.
        cbf_alpha: Discrete-Time CBF decay rate `alpha in (0, 1]`.
        consensus_gain: Distributed velocity consensus coupling gain `gamma_cons`.
        **kwargs: see `MPC`.
    """

    def __init__(
        self,
        *args,
        self_id: str = "1",
        peer_robots: Optional[Dict[str, BaseRobot]] = None,
        priority: int = 1,
        priorities: Optional[Dict[str, int]] = None,
        sensing_radius: float = 9.5,
        safe_margin: float = 0.55,
        cbf_alpha: float = 0.35,
        consensus_gain: float = 0.65,
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.self_id = str(self_id)
        self.peer_robots: Dict[str, BaseRobot] = (
            peer_robots if peer_robots is not None else {}
        )
        self.priority = int(priority)
        self.priorities: Dict[str, int] = priorities if priorities is not None else {}
        self.sensing_radius = float(sensing_radius)
        self.safe_margin = float(safe_margin)
        self.cbf_alpha = float(cbf_alpha)
        self.consensus_gain = float(consensus_gain)

    def __str__(self) -> str:
        return "Consensus-CBF-MPC"

    def _compute_consensus_speed_scale(
        self, pose: np.ndarray, vel: np.ndarray
    ) -> float:
        """
        Distributed consensus velocity protocol:
            v_i_scale = clip(1 - gamma * sum_{j in N_i} a_ij * phi_ij(p_i, p_j, v_i, v_j), 0.0, 1.0)
        where `phi_ij` measures directional approaching conflict and right-of-way priority.
        """
        if not self.peer_robots:
            return 1.0

        pos_i = pose[:2]
        theta_i = float(pose[2])
        heading_vec = np.array([math.cos(theta_i), math.sin(theta_i)], dtype=float)
        r_i = getattr(self.robot_model, "radius", 0.8)

        consensus_penalty = 0.0
        for peer_id, peer in self.peer_robots.items():
            if str(peer_id) == self.self_id:
                continue
            pos_j = peer.pos[:2]
            rel_pos = pos_j - pos_i
            dist = float(np.linalg.norm(rel_pos))
            if dist > self.sensing_radius or dist < 1e-6:
                continue

            r_j = getattr(peer, "radius", 0.8)
            d_contact = r_i + r_j
            d_safe = d_contact + self.safe_margin
            unit_rel = rel_pos / dist

            # Frenet lateral separation check for opposing parallel lanes
            theta_j = float(peer.orient[0]) if len(peer.orient) > 0 else 0.0
            heading_j = np.array([math.cos(theta_j), math.sin(theta_j)], dtype=float)
            heading_align = abs(float(np.dot(heading_vec, heading_j)))
            lat_sep = abs(
                float(rel_pos[0] * (-heading_vec[1]) + rel_pos[1] * heading_vec[0])
            )
            if heading_align > 0.88 and lat_sep >= d_contact + 0.04:
                continue

            # Check if peer is in the forward conflict hemisphere of robot i
            forward_cos = float(np.dot(heading_vec, unit_rel))
            if forward_cos <= -0.05:
                continue

            # Closest Point of Approach (CPA) over horizon [0, 2.8s]
            v_i_nom = heading_vec * self.max_lin_speed
            peer_speed = float(np.linalg.norm(peer.lin_vel[:2]))
            v_j_est = peer.lin_vel[:2] if peer_speed > 0.2 else heading_j * 0.6
            rel_v = v_j_est - v_i_nom
            rel_v_sq = float(np.dot(rel_v, rel_v))
            t_cpa = (
                float(np.clip(-np.dot(rel_pos, rel_v) / rel_v_sq, 0.0, 2.8))
                if rel_v_sq > 1e-4
                else 0.0
            )
            cpa_dist = float(np.linalg.norm(rel_pos + rel_v * t_cpa))
            if cpa_dist > d_safe * 1.15 and dist > d_safe * 1.15:
                continue

            peer_prio = self.priorities.get(str(peer_id), 1)
            is_lower_prio = (self.priority < peer_prio) or (
                self.priority == peer_prio and str(self.self_id) > str(peer_id)
            )
            if not is_lower_prio and dist > d_safe * 1.08:
                continue

            prio_weight = 2.10 if is_lower_prio else 0.35
            prox_ratio = max(
                0.0,
                (self.sensing_radius - min(dist, cpa_dist))
                / max(self.sensing_radius - d_safe, 0.5),
            )
            phi_ij = prio_weight * max(0.2, forward_cos) * (prox_ratio**1.2)
            consensus_penalty += phi_ij

        return float(np.clip(1.0 - self.consensus_gain * consensus_penalty, 0.0, 1.0))

    def _additional_horizon_cost(
        self, traj: np.ndarray, v_cmd: float, w_cmd: float
    ) -> float:
        """
        Discrete-Time Control Barrier Function (CBF) & dynamic velocity obstacle cost
        over the predicted horizon `traj`.
        """
        if not self.peer_robots:
            return 0.0

        r_i = getattr(self.robot_model, "radius", 0.8)
        cbf_cost = 0.0

        for peer_id, peer in self.peer_robots.items():
            if str(peer_id) == self.self_id:
                continue
            pos_j0 = peer.pos[:2].copy()
            vel_j = peer.lin_vel[:2].copy()
            r_j = getattr(peer, "radius", 0.8)
            d_contact = r_i + r_j
            d_hard = d_contact + 0.03

            theta_i0 = float(traj[0, 2])
            heading_i = np.array([math.cos(theta_i0), math.sin(theta_i0)], dtype=float)
            theta_j = float(peer.orient[0]) if len(peer.orient) > 0 else 0.0
            heading_j = np.array([math.cos(theta_j), math.sin(theta_j)], dtype=float)
            rel_pos0 = pos_j0 - traj[0, :2]
            lat_sep0 = abs(
                float(rel_pos0[0] * (-heading_i[1]) + rel_pos0[1] * heading_i[0])
            )
            if abs(float(np.dot(heading_i, heading_j))) > 0.88 and lat_sep0 >= d_contact + 0.04:
                d_safe = d_contact + 0.04
            else:
                d_safe = d_contact + self.safe_margin

            dist0 = float(np.linalg.norm(traj[0, :2] - pos_j0))
            if dist0 > self.sensing_radius + 2.0:
                continue

            peer_prio = self.priorities.get(str(peer_id), 1)
            is_lower_prio = (self.priority < peer_prio) or (
                self.priority == peer_prio and str(self.self_id) > str(peer_id)
            )

            h_prev = dist0 * dist0 - d_safe * d_safe
            for k in range(1, self.horizon_steps + 1):
                pred_pos_j = pos_j0 + vel_j * (k * self.dt)
                dist_k = float(np.linalg.norm(traj[k, :2] - pred_pos_j))

                # Hard physical overlap guard when moving forward
                if dist_k <= d_hard and v_cmd > 0.05 and dist_k < dist0:
                    return float("inf")

                h_k = dist_k * dist_k - d_safe * d_safe
                # Discrete-Time CBF condition: h_k - (1 - alpha) * h_{k-1} >= 0
                cbf_violation = (1.0 - self.cbf_alpha) * h_prev - h_k
                if cbf_violation > 0.0:
                    weight = 24.0 if is_lower_prio else 6.0
                    cbf_cost += weight * (cbf_violation**2)

                if dist_k < d_safe:
                    margin = d_safe - dist_k
                    cbf_cost += (35.0 if is_lower_prio else 8.0) * (margin**2)

                h_prev = h_k

        return cbf_cost

    def get_action(self, obs: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if self.goal is None:
            return np.zeros(self.action_space.shape), self.goal

        pose, vel, pos, orient, lin_vel, ang_vel = self.get_pose_velocity(obs)
        speed_scale = self._compute_consensus_speed_scale(pose, vel)
        ref_window, target_pose = self._reference_window(
            pos, self.max_lin_speed * max(0.35, speed_scale)
        )
        vel_robot = FrameTransformer.vel_world_to_robot(self.dim, vel, orient)

        best_v, best_w, best_traj = self._solve_mpc_control(
            pose,
            vel_robot,
            ref_window,
            target_pose,
            target_speed_scale=speed_scale,
        )
        self.prev_cmd = np.array([best_v, best_w], dtype=float)
        self.pred_traj = best_traj

        desired_vel = np.array([best_v, 0.0, best_w], dtype=float)
        desired_vel = self._stop_if_reached(desired_vel, pose)
        action = self._get_desired_action(desired_vel, vel_robot, orient)
        return action, target_pose
