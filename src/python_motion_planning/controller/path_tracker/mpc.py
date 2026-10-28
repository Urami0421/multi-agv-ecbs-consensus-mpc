"""
@file: mpc.py
@description: Linearized Error-State Model Predictive Control (MPC via OSQP),
              Distributed Control Barrier Function Quadratic Program (CBF-QP) Safety Filter,
              Action Dependency Graph (ADG) Schedule Synchronization, and ORCA Baseline.
@references:
    [1] Kuhne, F., Lages, W. F., & Gomes da Silva Jr, J. M. (2004). Model Predictive Control
        of a Mobile Robot Using Linearization. Proc. Mechatronics and Robotics, 525-530.
    [2] Ames, A. D., Xu, X., Grizzle, J. W., & Tabuada, P. (2017). Control Barrier Function
        Based Quadratic Programs for Safety Critical Systems. IEEE Transactions on Automatic
        Control, 62(8), 3861-3876.
    [3] Wang, L., Ames, A. D., & Egerstedt, M. (2017). Safety Barrier Certificates for
        Collisions-Free Multirobot Systems. IEEE Transactions on Robotics, 33(3), 661-674.
    [4] Honig, W., Kiesel, S., Tinka, A., Durham, J. W., & Ayanian, N. (2019). Persistent and
        Robust Execution of MAPF Schedules in Warehouses. IEEE Robotics and Automation Letters,
        4(2), 1125-1131.
    [5] van den Berg, J., Guy, S. J., Lin, M., & Manocha, D. (2011). Reciprocal n-Body
        Collision Avoidance. Robotics Research (ISRR), Springer, 3-19.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import osqp
import scipy.sparse as sp

from python_motion_planning.common.env.map.grid import Grid
from python_motion_planning.common.env.robot.base_robot import BaseRobot
from python_motion_planning.common.env.types import TYPES
from python_motion_planning.common.utils.frame_transformer import FrameTransformer
from python_motion_planning.common.utils.geometry import Geometry
from .path_tracker import PathTracker


_OSQP_V1 = int(str(getattr(osqp, "__version__", "0.6.0")).split(".")[0]) >= 1


def _osqp_solve(
    P: sp.csc_matrix,
    q: np.ndarray,
    A: sp.csc_matrix,
    lb: np.ndarray,
    ub: np.ndarray,
    eps: float = 1e-4,
    polish: bool = False,
    warm_x: Optional[np.ndarray] = None,
):
    """Version-compatible OSQP setup and solve wrapper (supports OSQP 0.6.x and 1.x)."""
    prob = osqp.OSQP()
    kwargs = {
        "P": P,
        "q": q,
        "A": A,
        "l": lb,
        "u": ub,
        "verbose": False,
        "eps_abs": eps,
        "eps_rel": eps,
    }
    if _OSQP_V1:
        kwargs["warm_starting"] = True
        kwargs["polishing"] = polish
    else:
        kwargs["warm_start"] = True
        kwargs["polish"] = polish
    prob.setup(**kwargs)
    if warm_x is not None:
        prob.warm_start(x=warm_x)
    return prob.solve(raise_error=False) if _OSQP_V1 else prob.solve()


def unicycle_to_lookahead_point(pose: np.ndarray, lookahead_l: float = 0.18) -> np.ndarray:
    """
    Near-Identity Diffeomorphism (NID): map unicycle state `(x, y, theta)` to a lookahead
    control point `p = [x + l*cos(theta), y + l*sin(theta)]`.
    """
    theta = float(pose[2])
    return np.array(
        [
            float(pose[0]) + lookahead_l * math.cos(theta),
            float(pose[1]) + lookahead_l * math.sin(theta),
        ],
        dtype=float,
    )


def unicycle_vel_to_single_integrator(
    theta: float, v: float, omega: float, lookahead_l: float = 0.18
) -> np.ndarray:
    """Map unicycle command `(v, omega)` to lookahead point velocity `p_dot = R(theta) diag(1, l) [v, omega]^T`."""
    ct, st = math.cos(theta), math.sin(theta)
    return np.array(
        [v * ct - lookahead_l * omega * st, v * st + lookahead_l * omega * ct],
        dtype=float,
    )


def single_integrator_to_unicycle_vel(
    theta: float,
    u_vec: np.ndarray,
    lookahead_l: float = 0.18,
    v_max: float = 2.6,
    omega_max: float = 3.14,
    allow_reverse: bool = False,
) -> Tuple[float, float]:
    """Inverse Near-Identity Diffeomorphism: `[v, omega]^T = diag(1, 1/l) R(theta)^T u_vec`."""
    ct, st = math.cos(theta), math.sin(theta)
    ux, uy = float(u_vec[0]), float(u_vec[1])
    v_raw = ct * ux + st * uy
    w_raw = (-st * ux + ct * uy) / max(lookahead_l, 1e-4)
    v_min = -0.35 * v_max if allow_reverse else 0.0
    v_cmd = float(np.clip(v_raw, v_min, v_max))
    w_cmd = float(np.clip(w_raw, -omega_max, omega_max))
    return v_cmd, w_cmd


def cbf_qp_filter(
    p_i: np.ndarray,
    u_nom: np.ndarray,
    neighbors: Sequence[Tuple[np.ndarray, np.ndarray]],
    d_safe: float,
    gamma: float = 1.5,
    u_max: float = 2.6,
    weights: Optional[Sequence[float]] = None,
) -> Tuple[np.ndarray, Dict[str, float]]:
    """
    Distributed Control Barrier Function Quadratic Program (CBF-QP) safety filter
    (Wang, Ames, & Egerstedt, IEEE T-RO 2017):

        min_{u in R^2}  ||u - u_nom||_2^2
        s.t.            -2 (p_i - p_j)^T u <= alpha_ij * gamma * h_ij - 2 (p_i - p_j)^T u_j_hat
                        -u_max <= u_x, u_y <= u_max

    where `h_ij = ||p_i - p_j||_2^2 - d_safe^2` and `alpha_ij` is the responsibility share
    (default `0.5` for symmetric splitting). If the strict QP is infeasible, returns `zeros(2)`
    (safe stop) rather than raising or violating bounds.
    """
    t0 = time.perf_counter()
    p_i = np.asarray(p_i, dtype=float)[:2]
    u_nom = np.asarray(u_nom, dtype=float)[:2]
    if not neighbors:
        u_clip = np.clip(u_nom, -u_max, u_max)
        return u_clip, {
            "status_val": 1.0,
            "solve_time_us": (time.perf_counter() - t0) * 1e6,
            "min_h": float("inf"),
        }

    A_rows: List[np.ndarray] = []
    lb: List[float] = []
    ub: List[float] = []
    min_h = float("inf")

    for idx, (p_j, u_j) in enumerate(neighbors):
        dp = p_i - np.asarray(p_j, dtype=float)[:2]
        u_j_vec = np.asarray(u_j, dtype=float)[:2]
        h = float(dp @ dp - d_safe * d_safe)
        min_h = min(min_h, h)
        share = float(weights[idx]) if weights is not None and idx < len(weights) else 0.5
        A_rows.append(-2.0 * dp)
        lb.append(-np.inf)
        ub.append(share * gamma * h - 2.0 * float(dp @ u_j_vec))

    A_rows.append(np.array([1.0, 0.0], dtype=float))
    lb.append(-float(u_max))
    ub.append(float(u_max))
    A_rows.append(np.array([0.0, 1.0], dtype=float))
    lb.append(-float(u_max))
    ub.append(float(u_max))

    P = sp.csc_matrix(2.0 * np.eye(2, dtype=float))
    q = -2.0 * u_nom
    A = sp.csc_matrix(np.vstack(A_rows))

    res = _osqp_solve(
        P=P,
        q=q,
        A=A,
        lb=np.asarray(lb, dtype=float),
        ub=np.asarray(ub, dtype=float),
        eps=1e-5,
        polish=True,
    )
    solve_us = (time.perf_counter() - t0) * 1e6

    if res.info.status_val in (1, 2) and res.x is not None:
        return np.clip(np.asarray(res.x, dtype=float), -u_max, u_max), {
            "status_val": float(res.info.status_val),
            "solve_time_us": solve_us,
            "min_h": min_h,
        }
    return np.zeros(2, dtype=float), {
        "status_val": float(res.info.status_val),
        "solve_time_us": solve_us,
        "min_h": min_h,
    }


@dataclass(frozen=True)
class ADGDependency:
    """Inter-agent precedence constraint: `(pred_agent, pred_leave_s) -> (succ_agent, succ_hold_s)`."""
    pred_agent: str
    pred_leave_s: float
    succ_agent: str
    succ_hold_s: float
    conflict_xy: Tuple[float, float]


class ActionDependencyGraph:
    """
    Action Dependency Graph (ADG) for time-synchronized, delay-robust execution of
    multi-agent space-time plans (Honig et al., IEEE RA-L 2019).

    Extracts the partial order of spatial cell/corridor visits from the discrete `ECBS`
    space-time trajectories and maps each precedence event onto cumulative arc-length `s`
    along each agent's continuous trajectory.

    If agent `A` visits a shared region at step `t_A` before agent `B` visits it at step `t_B > t_A`,
    then agent `B` cannot advance past `succ_hold_s` until agent `A` has advanced past `pred_leave_s`.
    """

    def __init__(
        self,
        grid: Grid,
        ecbs_paths: Dict[str, Sequence[Tuple[int, int]]],
        world_paths: Dict[str, Sequence[Tuple[float, float, float]]],
        conflict_radius: float = 1.15,
        hold_buffer_m: float = 1.55,
        leave_buffer_m: float = 1.35,
    ) -> None:
        self.agent_ids = sorted(ecbs_paths.keys())
        self.conflict_radius = float(conflict_radius)
        self.hold_buffer_m = float(hold_buffer_m)
        self.leave_buffer_m = float(leave_buffer_m)

        # Map each discrete ECBS step k of agent i to cumulative arc-length s_{i}[k] on world_paths[i]
        self.step_to_s: Dict[str, np.ndarray] = {}
        self.total_s: Dict[str, float] = {}
        self._world_xy: Dict[str, np.ndarray] = {}

        for aid in self.agent_ids:
            w_arr = np.asarray(world_paths[aid], dtype=float)[:, :2]
            if len(w_arr) > 1:
                s_vec = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(w_arr, axis=0), axis=1))])
            else:
                s_vec = np.zeros(len(w_arr), dtype=float)
            self._world_xy[aid] = w_arr
            self.total_s[aid] = float(s_vec[-1]) if len(s_vec) > 0 else 0.0

            gpath = ecbs_paths[aid]
            step_s = np.zeros(len(gpath), dtype=float)
            search_start = 0
            for k, gpt in enumerate(gpath):
                wpt = np.asarray(grid.map_to_world(gpt), dtype=float)
                # Monotonically match along world trajectory
                window_end = min(len(w_arr), search_start + max(40, len(w_arr) // max(1, len(gpath)) * 6))
                sub = w_arr[search_start:window_end]
                if len(sub) == 0:
                    step_s[k] = step_s[k - 1] if k > 0 else 0.0
                    continue
                idx_local = int(np.argmin(np.linalg.norm(sub - wpt, axis=1)))
                matched_idx = search_start + idx_local
                step_s[k] = float(s_vec[matched_idx])
                if k > 0:
                    step_s[k] = max(step_s[k], step_s[k - 1])
                search_start = matched_idx
            self.step_to_s[aid] = step_s

        # Build Type-2 inter-agent ordering dependencies
        self.dependencies_by_succ: Dict[str, List[ADGDependency]] = {
            aid: [] for aid in self.agent_ids
        }
        r_sq = self.conflict_radius ** 2
        g_world: Dict[str, np.ndarray] = {
            aid: np.asarray([grid.map_to_world(p) for p in ecbs_paths[aid]], dtype=float)
            for aid in self.agent_ids
        }

        for i_idx, aid_a in enumerate(self.agent_ids):
            pts_a = g_world[aid_a]
            s_a = self.step_to_s[aid_a]
            for j_idx in range(i_idx + 1, len(self.agent_ids)):
                aid_b = self.agent_ids[j_idx]
                pts_b = g_world[aid_b]
                s_b = self.step_to_s[aid_b]

                # Pairwise spatial proximity matrix between discrete steps
                diff = pts_a[:, None, :] - pts_b[None, :, :]
                dist_sq = np.sum(diff * diff, axis=2)
                close_pairs = np.argwhere(dist_sq <= r_sq)
                if len(close_pairs) == 0:
                    continue

                # Group contiguous conflict windows
                last_ta, last_tb = -999, -999
                for ta, tb in close_pairs:
                    ta_i, tb_i = int(ta), int(tb)
                    if abs(ta_i - tb_i) > 28:
                        continue
                    if abs(ta_i - last_ta) <= 3 and abs(tb_i - last_tb) <= 3:
                        last_ta, last_tb = ta_i, tb_i
                        continue
                    last_ta, last_tb = ta_i, tb_i

                    if ta_i < tb_i or (ta_i == tb_i and aid_a < aid_b):
                        pred_id, succ_id = aid_a, aid_b
                        pred_s_arr, succ_s_arr = s_a, s_b
                        t_pred, t_succ = ta_i, tb_i
                        c_xy = (float(pts_a[ta_i, 0]), float(pts_a[ta_i, 1]))
                    else:
                        pred_id, succ_id = aid_b, aid_a
                        pred_s_arr, succ_s_arr = s_b, s_a
                        t_pred, t_succ = tb_i, ta_i
                        c_xy = (float(pts_b[tb_i, 0]), float(pts_b[tb_i, 1]))

                    pred_leave = min(
                        self.total_s[pred_id],
                        float(pred_s_arr[t_pred]) + self.leave_buffer_m,
                    )
                    succ_hold = max(
                        0.0,
                        float(succ_s_arr[t_succ]) - self.hold_buffer_m,
                    )
                    if pred_leave <= 0.2 or succ_hold >= self.total_s[succ_id] - 0.2:
                        continue
                    self.dependencies_by_succ[succ_id].append(
                        ADGDependency(
                            pred_agent=pred_id,
                            pred_leave_s=pred_leave,
                            succ_agent=succ_id,
                            succ_hold_s=succ_hold,
                            conflict_xy=c_xy,
                        )
                    )

        for aid in self.agent_ids:
            self.dependencies_by_succ[aid].sort(key=lambda d: d.succ_hold_s)

        self.progress_s: Dict[str, float] = {aid: 0.0 for aid in self.agent_ids}

    def reset(self) -> None:
        for aid in self.agent_ids:
            self.progress_s[aid] = 0.0

    def update_progress(self, agent_id: str, s_val: float) -> None:
        if agent_id in self.progress_s:
            self.progress_s[agent_id] = max(self.progress_s[agent_id], float(s_val))

    def max_allowed_s(self, agent_id: str) -> float:
        """
        Return the maximum cumulative arc-length `s_max` that `agent_id` is currently allowed
        to advance to under the Action Dependency Graph partial order.
        """
        s_cap = self.total_s.get(agent_id, float("inf"))
        for dep in self.dependencies_by_succ.get(agent_id, ()):
            pred_s = self.progress_s.get(dep.pred_agent, 0.0)
            pred_total = self.total_s.get(dep.pred_agent, float("inf"))
            if pred_s < dep.pred_leave_s and pred_s < pred_total - 0.35:
                s_cap = min(s_cap, dep.succ_hold_s)
                break
        return s_cap


class MPC(PathTracker):
    """
    Linearized Error-State Receding-Horizon Model Predictive Control (MPC) path tracker
    solved via OSQP with warm-starting (Kuhne et al., 2004).

    At each control step:
      1. Projects the robot state onto the piecewise-linear reference trajectory to obtain true
         Frenet coordinates `(s, d_lat, e_theta)` and extracts an `N`-step reference window
         `(x_{r,k}, y_{r,k}, theta_{r,k}, v_{r,k}, omega_{r,k})`.
      2. Linearizes the discrete-time unicycle error dynamics around the reference window:
             e_{k+1} = A_k e_k + B_k u_tilde_k
      3. Solves the condensed `2N`-variable Quadratic Program (QP) via `osqp` (with primal/dual
         warm-starting across control steps) subject to control amplitude and rate-of-change bounds.

    Args:
        *args: see `PathTracker`.
        horizon_steps: Prediction horizon steps `N` (default: 8).
        q_pos: Position tracking weight.
        q_theta: Heading tracking weight.
        r_vel: Linear velocity tracking weight.
        r_omega: Angular velocity tracking weight.
        r_smooth: Control rate-of-change weight.
        **kwargs: see `PathTracker`.
    """

    def __init__(
        self,
        *args,
        horizon_steps: int = 8,
        q_pos: float = 3.2,
        q_theta: float = 1.4,
        r_vel: float = 0.65,
        r_omega: float = 0.30,
        r_smooth: float = 0.25,
        w_esdf: float = 0.8,
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.horizon_steps = int(max(3, horizon_steps))
        self.q_pos = float(q_pos)
        self.q_theta = float(q_theta)
        self.r_vel = float(r_vel)
        self.r_omega = float(r_omega)
        self.r_smooth = float(r_smooth)
        self.w_esdf = float(w_esdf)
        self.prev_cmd = np.zeros(2, dtype=float)
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

        # OSQP solver diagnostics & warm-start buffers
        self.last_solve_time_us: float = 0.0
        self.last_status: str = "uninitialized"
        self.last_iter: int = 0
        self.solve_times_us: List[float] = []
        self._warm_x: Optional[np.ndarray] = None

    def __str__(self) -> str:
        return "Linear-MPC (OSQP)"

    def reset(self) -> None:
        super().reset()
        self.prev_cmd = np.zeros(2, dtype=float)
        self._progress_s = 0.0
        self.last_solve_time_us = 0.0
        self.last_status = "reset"
        self.last_iter = 0
        self.solve_times_us.clear()
        self._warm_x = None

    def compute_frenet_state(
        self, pos: np.ndarray, theta: float
    ) -> Tuple[float, float, float]:
        """
        Compute true Frenet coordinates `(s, d_lat, e_theta)` by orthogonal projection of `pos[:2]`
        onto the active local window of `self._path_array`.
        """
        path = self._path_array
        s_arr = self._path_s
        if len(path) == 0:
            return 0.0, 0.0, 0.0
        if len(path) == 1:
            dp = pos[:2] - path[0, :2]
            th_r = float(path[0, 2])
            d_lat = float(-math.sin(th_r) * dp[0] + math.cos(th_r) * dp[1])
            e_th = float(Geometry.regularize_orient(np.array([theta - th_r]))[0])
            return 0.0, d_lat, e_th

        mask = (s_arr >= self._progress_s - 1.5) & (s_arr <= self._progress_s + 8.0)
        cand_indices = np.where(mask)[0]
        if len(cand_indices) == 0:
            cand_indices = np.arange(len(path))

        dists = np.linalg.norm(path[cand_indices, :2] - pos[:2], axis=1)
        idx = int(cand_indices[int(np.argmin(dists))])
        seg_idx = min(max(0, idx), len(path) - 2)
        p0 = path[seg_idx, :2]
        p1 = path[seg_idx + 1, :2]
        seg = p1 - p0
        seg_len = float(np.linalg.norm(seg))
        if seg_len > 1e-6:
            t_hat = seg / seg_len
            proj = float(np.clip(np.dot(pos[:2] - p0, t_hat), 0.0, seg_len))
            s_val = float(s_arr[seg_idx]) + proj
            foot = p0 + proj * t_hat
            th_r = math.atan2(float(t_hat[1]), float(t_hat[0]))
        else:
            s_val = float(s_arr[idx])
            foot = path[idx, :2]
            th_r = float(path[idx, 2])

        dp = pos[:2] - foot
        d_lat = float(-math.sin(th_r) * dp[0] + math.cos(th_r) * dp[1])
        e_th = float(Geometry.regularize_orient(np.array([theta - th_r]))[0])
        return s_val, d_lat, e_th

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
        self,
        pos: np.ndarray,
        ref_speed: float,
        s_cap: Optional[float] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Extract an `N`-step arc-length parameterized reference pose window starting near `pos`."""
        path = self._path_array
        s_arr = self._path_s
        n_pts = len(path)
        if n_pts == 0:
            return np.zeros((self.horizon_steps, 3)), np.zeros(3)

        mask = (s_arr >= self._progress_s - 1.5) & (s_arr <= self._progress_s + 8.0)
        cand_indices = np.where(mask)[0]
        if len(cand_indices) == 0:
            cand_indices = np.arange(n_pts)

        dists = np.linalg.norm(path[cand_indices, :2] - pos[:2], axis=1)
        nearest_idx = int(cand_indices[int(np.argmin(dists))])
        self._progress_s = max(self._progress_s, float(s_arr[nearest_idx]))

        s_total = float(s_arr[-1])
        if s_cap is not None:
            s_limit = min(s_total, max(self._progress_s, float(s_cap)))
        else:
            s_limit = s_total

        step_ds = max(0.06, ref_speed * self.dt)
        ref_window = np.zeros((self.horizon_steps, 3), dtype=float)
        for k in range(self.horizon_steps):
            s_k = min(s_limit, self._progress_s + (k + 1) * step_ds)
            ref_window[k] = self._interpolate_at_s(s_k)

        lookahead_s = min(
            s_limit, self._progress_s + max(1.5, self.lookahead_distance * 0.85)
        )
        lookahead_pose = self._interpolate_at_s(lookahead_s)
        return ref_window, lookahead_pose

    def _rollout_sequence(
        self, start_pose: np.ndarray, u_seq: np.ndarray
    ) -> np.ndarray:
        """Roll out differential-drive unicycle kinematics under control sequence `u_seq` (N, 2)."""
        n = len(u_seq)
        traj = np.zeros((n + 1, 3), dtype=float)
        traj[0] = start_pose[:3]
        x, y, theta = float(start_pose[0]), float(start_pose[1]), float(start_pose[2])
        for k in range(n):
            v_k = float(u_seq[k, 0])
            w_k = float(u_seq[k, 1])
            x += v_k * math.cos(theta) * self.dt
            y += v_k * math.sin(theta) * self.dt
            theta = float(
                Geometry.regularize_orient(np.array([theta + w_k * self.dt]))[0]
            )
            traj[k + 1] = [x, y, theta]
        return traj

    def _solve_linear_mpc_osqp(
        self,
        pose: np.ndarray,
        ref_window: np.ndarray,
        lookahead_pose: np.ndarray,
        target_speed_scale: float = 1.0,
    ) -> Tuple[float, float, np.ndarray]:
        """
        Solve the linearized error-state MPC Quadratic Program over horizon `N` using `osqp`:
            min_{U_tilde in R^{2N}}  sum_{k=1..N} e_k^T Q e_k + sum_{k=0..N-1} u_tilde_k^T R u_tilde_k
                                     + sum_{k=0..N-1} ||u_k - u_{k-1}||_S^2
            s.t.                     e_{k+1} = A_k e_k + B_k u_tilde_k
                                     0 <= v_{r,k} + v_tilde_k <= v_cap
                                     -w_max <= w_{r,k} + w_tilde_k <= w_max
        """
        t_start = time.perf_counter()
        pos = pose[:2]
        goal_pos = self.goal[:2]
        dist_to_goal = float(np.linalg.norm(pos - goal_pos))
        N = self.horizon_steps

        if dist_to_goal < self.goal_dist_tol or target_speed_scale <= 0.01:
            self.last_solve_time_us = (time.perf_counter() - t_start) * 1e6
            self.last_status = "at_goal_or_held"
            self.last_iter = 0
            return 0.0, 0.0, np.repeat(pose[:3][None, :], N + 1, axis=0)

        rel_target = FrameTransformer.pose_world_to_robot(
            self.dim, lookahead_pose, pose
        )
        rx, ry = float(rel_target[0]), float(rel_target[1])
        L = max(math.hypot(rx, ry), 0.2)
        kappa = 2.0 * ry / (L * L)
        heading_err = math.atan2(ry, rx)

        v_cap = self.max_lin_speed * float(np.clip(target_speed_scale, 0.0, 1.0))
        curv_factor = 1.0 / (1.0 + 0.65 * abs(kappa) + 0.85 * abs(heading_err))
        v_nom = min(
            v_cap,
            max(0.18 * v_cap, v_cap * curv_factor),
            dist_to_goal / max(self.dt * 2.0, 0.2),
        )
        if abs(heading_err) > 1.15:
            v_nom *= 0.25
        omega_nom = float(
            np.clip(
                0.75 * v_nom * kappa + 1.55 * heading_err,
                -self.max_ang_speed,
                self.max_ang_speed,
            )
        )

        # Reference control sequence u_ref (N, 2) and linearized matrices A_k, B_k
        u_ref = np.zeros((N, 2), dtype=float)
        A_list: List[np.ndarray] = []
        B_list: List[np.ndarray] = []
        dt = float(self.dt)

        for k in range(N):
            th_k = float(ref_window[k, 2])
            if k < N - 1:
                dth = float(
                    Geometry.regularize_orient(
                        np.array([ref_window[k + 1, 2] - th_k])
                    )[0]
                )
                w_rk = float(np.clip(dth / dt, -self.max_ang_speed, self.max_ang_speed))
            else:
                w_rk = 0.5 * omega_nom
            v_rk = v_nom
            u_ref[k, 0] = v_rk
            u_ref[k, 1] = w_rk

            ct, st = math.cos(th_k), math.sin(th_k)
            A_k = np.array(
                [
                    [1.0, 0.0, -v_rk * st * dt],
                    [0.0, 1.0, v_rk * ct * dt],
                    [0.0, 0.0, 1.0],
                ],
                dtype=float,
            )
            B_k = np.array(
                [
                    [ct * dt, 0.0],
                    [st * dt, 0.0],
                    [0.0, dt],
                ],
                dtype=float,
            )
            A_list.append(A_k)
            B_list.append(B_k)

        # Initial error state e_0 = x_0 - x_{r,0}
        e0 = np.array(
            [
                float(pose[0] - ref_window[0, 0]),
                float(pose[1] - ref_window[0, 1]),
                float(
                    Geometry.regularize_orient(
                        np.array([pose[2] - ref_window[0, 2]])
                    )[0]
                ),
            ],
            dtype=float,
        )

        # Condense state prediction: E = Phi * e0 + Gamma * U_tilde, where E in R^{3N}, U_tilde in R^{2N}
        Phi = np.zeros((3 * N, 3), dtype=float)
        Gamma = np.zeros((3 * N, 2 * N), dtype=float)
        A_prod = np.eye(3, dtype=float)
        for k in range(N):
            A_prod = A_list[k] @ A_prod
            Phi[3 * k : 3 * (k + 1), :] = A_prod
            G_step = np.eye(3, dtype=float)
            for j in range(k, -1, -1):
                if j < k:
                    G_step = G_step @ A_list[j + 1]
                Gamma[3 * k : 3 * (k + 1), 2 * j : 2 * (j + 1)] = G_step @ B_list[j]

        q_diag = np.tile([self.q_pos, self.q_pos, self.q_theta], N)
        q_diag[-3:] *= 1.5  # Terminal state weight
        Q_bar = np.diag(q_diag)
        R_bar = np.diag(np.tile([self.r_vel, self.r_omega], N))

        # Control rate-of-change difference matrix D_u on U = U_ref + U_tilde
        D_u = np.eye(2 * N, dtype=float)
        for k in range(1, N):
            D_u[2 * k : 2 * (k + 1), 2 * (k - 1) : 2 * k] = -np.eye(2, dtype=float)
        S_bar = np.diag(np.tile([self.r_smooth, 0.35 * self.r_smooth], N))

        u_prev_offset = np.zeros(2 * N, dtype=float)
        u_prev_offset[:2] = self.prev_cmd
        u_ref_flat = u_ref.ravel()
        d_const = D_u @ u_ref_flat - u_prev_offset

        e_free = Phi @ e0
        H = (
            Gamma.T @ Q_bar @ Gamma
            + R_bar
            + D_u.T @ S_bar @ D_u
            + 1e-6 * np.eye(2 * N)
        )
        g = Gamma.T @ (Q_bar @ e_free) + D_u.T @ (S_bar @ d_const)

        lb = np.zeros(2 * N, dtype=float)
        ub = np.zeros(2 * N, dtype=float)
        for k in range(N):
            lb[2 * k] = 0.0 - u_ref[k, 0]
            ub[2 * k] = v_cap - u_ref[k, 0]
            lb[2 * k + 1] = -self.max_ang_speed - u_ref[k, 1]
            ub[2 * k + 1] = self.max_ang_speed - u_ref[k, 1]

        P_mat = sp.csc_matrix(2.0 * H)
        q_vec = 2.0 * g
        A_mat = sp.eye(2 * N, format="csc")

        warm_x = (
            self._warm_x
            if (self._warm_x is not None and len(self._warm_x) == 2 * N)
            else None
        )
        res = _osqp_solve(
            P=P_mat,
            q=q_vec,
            A=A_mat,
            lb=lb,
            ub=ub,
            eps=1e-4,
            polish=False,
            warm_x=warm_x,
        )
        solve_us = (time.perf_counter() - t_start) * 1e6
        self.last_solve_time_us = solve_us
        self.solve_times_us.append(solve_us)
        self.last_status = str(res.info.status)
        self.last_iter = int(res.info.iter)

        if res.info.status_val in (1, 2) and res.x is not None:
            u_tilde = np.clip(np.asarray(res.x, dtype=float), lb, ub)
            self._warm_x = np.concatenate([u_tilde[2:], u_tilde[-2:]])
            u_seq = u_ref + u_tilde.reshape(N, 2)
        else:
            u_seq = u_ref.copy()

        # Blend first-step command with nonlinear lookahead curvature guidance for tight 90-deg turns
        v_qp = float(np.clip(u_seq[0, 0], 0.0, v_cap))
        w_qp = float(
            np.clip(
                0.55 * float(u_seq[0, 1]) + 0.45 * omega_nom,
                -self.max_ang_speed,
                self.max_ang_speed,
            )
        )
        u_seq[0, 0] = v_qp
        u_seq[0, 1] = w_qp

        # Static obstacle ESDF safety check along predicted rollout
        pred_traj = self._rollout_sequence(pose, u_seq)
        if self.obstacle_grid is not None:
            for k in range(1, min(5, N + 1)):
                d_obs = self._get_dist_to_nearest_obstacle(pred_traj[k, :2])
                if d_obs < 0.58:
                    # Steer away from static obstacle using ESDF gradient
                    gx, gy = pred_traj[k, 0], pred_traj[k, 1]
                    dx_e = self._get_dist_to_nearest_obstacle(
                        np.array([gx + 0.25, gy])
                    ) - self._get_dist_to_nearest_obstacle(
                        np.array([gx - 0.25, gy])
                    )
                    dy_e = self._get_dist_to_nearest_obstacle(
                        np.array([gx, gy + 0.25])
                    ) - self._get_dist_to_nearest_obstacle(
                        np.array([gx, gy - 0.25])
                    )
                    if abs(dx_e) + abs(dy_e) > 1e-4:
                        away_yaw = math.atan2(dy_e, dx_e)
                        diff_yaw = float(
                            Geometry.regularize_orient(
                                np.array([away_yaw - float(pose[2])])
                            )[0]
                        )
                        w_qp = float(
                            np.clip(
                                w_qp + 0.85 * np.sign(diff_yaw),
                                -self.max_ang_speed,
                                self.max_ang_speed,
                            )
                        )
                        v_qp *= 0.75
                        u_seq[0] = [v_qp, w_qp]
                        pred_traj = self._rollout_sequence(pose, u_seq)
                    break

        return v_qp, w_qp, pred_traj

    def get_action(self, obs: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if self.goal is None:
            return np.zeros(self.action_space.shape), self.goal

        pose, vel, pos, orient, lin_vel, ang_vel = self.get_pose_velocity(obs)
        ref_window, target_pose = self._reference_window(pos, self.max_lin_speed)
        vel_robot = FrameTransformer.vel_world_to_robot(self.dim, vel, orient)

        best_v, best_w, best_traj = self._solve_linear_mpc_osqp(
            pose, ref_window, target_pose, target_speed_scale=1.0
        )
        self.prev_cmd = np.array([best_v, best_w], dtype=float)
        self.pred_traj = best_traj

        desired_vel = np.array([best_v, 0.0, best_w], dtype=float)
        desired_vel = self._stop_if_reached(desired_vel, pose)
        action = self._get_desired_action(desired_vel, vel_robot, orient)
        return action, target_pose


class ConsensusMPC(MPC):
    """
    Multi-Agent Cooperative Trajectory Tracker combining:
      1. Optional Action Dependency Graph (`adg`) schedule synchronization (Honig et al., 2019),
      2. Priority-weighted velocity consensus + Linearized Error-State OSQP MPC (`MPC`),
      3. Distributed Control Barrier Function Quadratic Program (`cbf_qp_filter`, Wang et al., 2017)
         via Near-Identity Diffeomorphism (`lookahead_l`).
    """

    def __init__(
        self,
        *args,
        self_id: str = "1",
        peer_robots: Optional[Dict[str, BaseRobot]] = None,
        priority: int = 1,
        priorities: Optional[Dict[str, int]] = None,
        adg: Optional[ActionDependencyGraph] = None,
        sensing_radius: float = 9.5,
        safe_margin: float = 0.30,
        cbf_gamma: float = 1.6,
        lookahead_l: float = 0.20,
        consensus_gain: float = 0.65,
        cbf_alpha: Optional[float] = None,
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.self_id = str(self_id)
        self.peer_robots: Dict[str, BaseRobot] = (
            peer_robots if peer_robots is not None else {}
        )
        self.priority = int(priority)
        self.priorities: Dict[str, int] = priorities if priorities is not None else {}
        self.adg: Optional[ActionDependencyGraph] = adg
        self.sensing_radius = float(sensing_radius)
        self.safe_margin = float(safe_margin)
        self.cbf_gamma = float(cbf_alpha * 4.0 if cbf_alpha is not None else cbf_gamma)
        self.lookahead_l = float(lookahead_l)
        self.consensus_gain = float(consensus_gain)
        self.last_cbf_min_h: float = float("inf")
        self.last_cbf_status: float = 1.0

    def __str__(self) -> str:
        return "ADG+LinearMPC+CBF-QP" if self.adg is not None else "LinearMPC+CBF-QP"

    def _compute_consensus_speed_scale(
        self, pose: np.ndarray, vel: np.ndarray, s_cap: Optional[float] = None
    ) -> float:
        """
        Priority-weighted distributed velocity consensus protocol combined with optional
        ADG schedule hold limit `s_cap`.
        """
        scale = 1.0
        if s_cap is not None:
            rem_s = float(s_cap - self._progress_s)
            if rem_s <= 0.15:
                return 0.0
            if rem_s < 1.8:
                scale = min(scale, max(0.12, rem_s / 1.8))

        if not self.peer_robots:
            return scale

        pos_i = pose[:2]
        theta_i = float(pose[2])
        heading_vec = np.array([math.cos(theta_i), math.sin(theta_i)], dtype=float)
        r_i = getattr(self.robot_model, "radius", 0.44)

        consensus_penalty = 0.0
        for peer_id, peer in self.peer_robots.items():
            if str(peer_id) == self.self_id:
                continue
            pos_j = peer.pos[:2]
            rel_pos = pos_j - pos_i
            dist = float(np.linalg.norm(rel_pos))
            if dist > self.sensing_radius or dist < 1e-6:
                continue

            r_j = getattr(peer, "radius", 0.44)
            d_contact = r_i + r_j
            d_safe = d_contact + self.safe_margin
            unit_rel = rel_pos / dist

            # Body-frame lateral separation check for opposing parallel lanes
            theta_j = float(peer.orient[0]) if len(peer.orient) > 0 else 0.0
            heading_j = np.array([math.cos(theta_j), math.sin(theta_j)], dtype=float)
            heading_align = abs(float(np.dot(heading_vec, heading_j)))
            lat_sep = abs(
                float(rel_pos[0] * (-heading_vec[1]) + rel_pos[1] * heading_vec[0])
            )
            if heading_align > 0.88 and lat_sep >= d_contact + 0.06:
                continue

            forward_cos = float(np.dot(heading_vec, unit_rel))
            if forward_cos <= -0.05:
                continue

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

            prio_weight = 2.15 if is_lower_prio else 0.35
            prox_ratio = max(
                0.0,
                (self.sensing_radius - min(dist, cpa_dist))
                / max(self.sensing_radius - d_safe, 0.5),
            )
            phi_ij = prio_weight * max(0.2, forward_cos) * (prox_ratio ** 1.2)
            consensus_penalty += phi_ij

        cons_scale = float(
            np.clip(1.0 - self.consensus_gain * consensus_penalty, 0.0, 1.0)
        )
        return min(scale, cons_scale)

    def _apply_cbf_qp_safety_filter(
        self, pose: np.ndarray, v_nom: float, w_nom: float
    ) -> Tuple[float, float]:
        """
        Filter the nominal unicycle command `(v_nom, w_nom)` through the distributed CBF-QP
        (`cbf_qp_filter`) using Near-Identity Diffeomorphism.
        """
        if not self.peer_robots:
            return v_nom, w_nom

        theta_i = float(pose[2])
        heading_i = np.array([math.cos(theta_i), math.sin(theta_i)], dtype=float)
        p_i = unicycle_to_lookahead_point(pose, self.lookahead_l)
        u_nom = unicycle_vel_to_single_integrator(
            theta_i, v_nom, w_nom, self.lookahead_l
        )
        r_i = getattr(self.robot_model, "radius", 0.44)

        neighbors: List[Tuple[np.ndarray, np.ndarray]] = []
        weights: List[float] = []
        d_safe_eff = 2.0 * r_i + self.safe_margin

        for peer_id, peer in self.peer_robots.items():
            if str(peer_id) == self.self_id:
                continue
            pos_j = peer.pos[:2]
            dist_center = float(np.linalg.norm(pos_j - pose[:2]))
            if dist_center > self.sensing_radius:
                continue

            r_j = getattr(peer, "radius", 0.44)
            d_contact = r_i + r_j
            theta_j = float(peer.orient[0]) if len(peer.orient) > 0 else 0.0
            heading_j = np.array([math.cos(theta_j), math.sin(theta_j)], dtype=float)

            # Skip parallel opposing lane peers that already have clear body-frame lateral separation
            rel_pos = pos_j - pose[:2]
            lat_sep = abs(
                float(rel_pos[0] * (-heading_i[1]) + rel_pos[1] * heading_i[0])
            )
            if abs(float(np.dot(heading_i, heading_j))) > 0.88 and lat_sep >= d_contact + 0.06:
                continue

            peer_pose = np.array([pos_j[0], pos_j[1], theta_j], dtype=float)
            p_j = unicycle_to_lookahead_point(peer_pose, self.lookahead_l)
            u_j = peer.lin_vel[:2].copy()
            d_safe_eff = max(d_safe_eff, d_contact + self.safe_margin)

            peer_prio = self.priorities.get(str(peer_id), 1)
            is_lower_prio = (self.priority < peer_prio) or (
                self.priority == peer_prio and str(self.self_id) > str(peer_id)
            )
            # Lower-priority agent takes smaller gamma slack (yields earlier)
            weights.append(0.35 if is_lower_prio else 0.65)
            neighbors.append((p_j, u_j))

        if not neighbors:
            return v_nom, w_nom

        t0 = time.perf_counter()
        u_safe, cbf_info = cbf_qp_filter(
            p_i=p_i,
            u_nom=u_nom,
            neighbors=neighbors,
            d_safe=d_safe_eff,
            gamma=self.cbf_gamma,
            u_max=self.max_lin_speed,
            weights=weights,
        )
        self.last_solve_time_us += (time.perf_counter() - t0) * 1e6
        self.last_cbf_min_h = cbf_info["min_h"]
        self.last_cbf_status = cbf_info["status_val"]

        if np.linalg.norm(u_safe - u_nom) < 1e-4:
            return v_nom, w_nom

        v_safe, w_safe = single_integrator_to_unicycle_vel(
            theta_i,
            u_safe,
            lookahead_l=self.lookahead_l,
            v_max=self.max_lin_speed,
            omega_max=self.max_ang_speed,
            allow_reverse=False,
        )
        # Preserve path-tracking steering curvature when CBF reduces forward speed
        if v_safe > 0.05 and abs(w_nom) > 0.05:
            w_safe = float(
                np.clip(
                    0.65 * w_nom + 0.35 * w_safe,
                    -self.max_ang_speed,
                    self.max_ang_speed,
                )
            )
        return v_safe, w_safe

    def get_action(self, obs: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if self.goal is None:
            return np.zeros(self.action_space.shape), self.goal

        pose, vel, pos, orient, lin_vel, ang_vel = self.get_pose_velocity(obs)
        s_frenet, _, _ = self.compute_frenet_state(pos, float(pose[2]))
        self._progress_s = max(self._progress_s, s_frenet)

        s_cap: Optional[float] = None
        if self.adg is not None:
            self.adg.update_progress(self.self_id, self._progress_s)
            s_cap = self.adg.max_allowed_s(self.self_id)

        speed_scale = self._compute_consensus_speed_scale(pose, vel, s_cap=s_cap)
        ref_window, target_pose = self._reference_window(
            pos, self.max_lin_speed * max(0.25, speed_scale), s_cap=s_cap
        )
        vel_robot = FrameTransformer.vel_world_to_robot(self.dim, vel, orient)

        v_mpc, w_mpc, _ = self._solve_linear_mpc_osqp(
            pose,
            ref_window,
            target_pose,
            target_speed_scale=speed_scale,
        )
        best_v, best_w = self._apply_cbf_qp_safety_filter(pose, v_mpc, w_mpc)

        u_applied = np.tile([best_v, best_w], (self.horizon_steps, 1))
        self.pred_traj = self._rollout_sequence(pose, u_applied)
        self.prev_cmd = np.array([best_v, best_w], dtype=float)

        desired_vel = np.array([best_v, 0.0, best_w], dtype=float)
        desired_vel = self._stop_if_reached(desired_vel, pose)
        action = self._get_desired_action(desired_vel, vel_robot, orient)
        return action, target_pose


class ORCAController(MPC):
    """
    Reciprocal Velocity Obstacles / Optimal Reciprocal Collision Avoidance (ORCA)
    multi-agent baseline controller (van den Berg et al., ISRR 2011) mapped to differential-drive
    unicycle kinematics via Near-Identity Diffeomorphism and solved via OSQP.
    """

    def __init__(
        self,
        *args,
        self_id: str = "1",
        peer_robots: Optional[Dict[str, BaseRobot]] = None,
        time_horizon: float = 2.8,
        safe_margin: float = 0.16,
        lookahead_l: float = 0.22,
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.self_id = str(self_id)
        self.peer_robots: Dict[str, BaseRobot] = (
            peer_robots if peer_robots is not None else {}
        )
        self.time_horizon = float(time_horizon)
        self.safe_margin = float(safe_margin)
        self.lookahead_l = float(lookahead_l)

    def __str__(self) -> str:
        return "ORCA"

    def get_action(self, obs: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if self.goal is None:
            return np.zeros(self.action_space.shape), self.goal

        pose, vel, pos, orient, lin_vel, ang_vel = self.get_pose_velocity(obs)
        ref_window, target_pose = self._reference_window(pos, self.max_lin_speed)
        vel_robot = FrameTransformer.vel_world_to_robot(self.dim, vel, orient)

        dist_to_goal = float(np.linalg.norm(pos - self.goal[:2]))
        if dist_to_goal < self.goal_dist_tol:
            desired_vel = np.zeros(3, dtype=float)
            return self._get_desired_action(desired_vel, vel_robot, orient), target_pose

        # Preferred velocity toward lookahead target
        to_target = target_pose[:2] - pos[:2]
        dist_t = float(np.linalg.norm(to_target))
        v_pref_mag = min(self.max_lin_speed, dist_to_goal / max(self.dt * 2.0, 0.2))
        v_pref = (to_target / max(dist_t, 1e-6)) * v_pref_mag
        v_A = vel[:2].copy()
        r_A = getattr(self.robot_model, "radius", 0.44)

        A_rows: List[np.ndarray] = []
        lb: List[float] = []
        ub: List[float] = []
        inv_tau = 1.0 / max(self.time_horizon, 0.2)

        for peer_id, peer in self.peer_robots.items():
            if str(peer_id) == self.self_id:
                continue
            p_B = peer.pos[:2]
            v_B = peer.lin_vel[:2]
            r_B = getattr(peer, "radius", 0.44)
            R_ab = r_A + r_B + self.safe_margin

            rel_p = p_B - pos[:2]
            dist = float(np.linalg.norm(rel_p))
            if dist > 8.5 or dist < 1e-6:
                continue

            rel_v = v_A - v_B
            # Vector from cutoff circle center `rel_p / tau` to `rel_v`
            w_vec = rel_v - inv_tau * rel_p
            w_len = float(np.linalg.norm(w_vec))
            unit_p = rel_p / dist

            if dist <= R_ab:
                # Already inside combined radius: push velocity outward along -unit_p
                n_out = -unit_p
                u_orca = (R_ab - dist) / max(self.dt, 0.05) * n_out - rel_v
            else:
                if w_len < R_ab * inv_tau and w_len > 1e-6:
                    n_out = w_vec / w_len
                    u_orca = (R_ab * inv_tau - w_len) * n_out
                else:
                    continue

            # ORCA half-plane: (v - (v_A + 0.5 * u_orca)) . n_out >= 0  <=>  -n_out . v <= -(v_A + 0.5 * u_orca) . n_out
            rhs = -float(np.dot(v_A + 0.5 * u_orca, n_out))
            A_rows.append(-n_out)
            lb.append(-np.inf)
            ub.append(rhs)

        # Static obstacle clearance velocity constraint via ESDF gradient
        if self.obstacle_grid is not None:
            d_obs = self._get_dist_to_nearest_obstacle(pos[:2])
            if d_obs < r_A + 0.35:
                gx, gy = float(pos[0]), float(pos[1])
                dx_e = self._get_dist_to_nearest_obstacle(
                    np.array([gx + 0.25, gy])
                ) - self._get_dist_to_nearest_obstacle(np.array([gx - 0.25, gy]))
                dy_e = self._get_dist_to_nearest_obstacle(
                    np.array([gx, gy + 0.25])
                ) - self._get_dist_to_nearest_obstacle(np.array([gx, gy - 0.25]))
                g_norm = math.hypot(dx_e, dy_e)
                if g_norm > 1e-5:
                    n_obs = np.array([dx_e / g_norm, dy_e / g_norm], dtype=float)
                    A_rows.append(-n_obs)
                    lb.append(-np.inf)
                    ub.append(1.2 * (d_obs - (r_A + 0.15)))

        A_rows.append(np.array([1.0, 0.0], dtype=float))
        lb.append(-self.max_lin_speed)
        ub.append(self.max_lin_speed)
        A_rows.append(np.array([0.0, 1.0], dtype=float))
        lb.append(-self.max_lin_speed)
        ub.append(self.max_lin_speed)

        P = sp.csc_matrix(2.0 * np.eye(2, dtype=float))
        q = -2.0 * v_pref
        A = sp.csc_matrix(np.vstack(A_rows))
        res = _osqp_solve(
            P=P,
            q=q,
            A=A,
            lb=np.asarray(lb, dtype=float),
            ub=np.asarray(ub, dtype=float),
            eps=1e-4,
            polish=False,
        )
        if res.info.status_val in (1, 2) and res.x is not None:
            v_orca = np.clip(
                np.asarray(res.x, dtype=float), -self.max_lin_speed, self.max_lin_speed
            )
        else:
            v_orca = np.zeros(2, dtype=float)

        v_cmd, w_cmd = single_integrator_to_unicycle_vel(
            float(pose[2]),
            v_orca,
            lookahead_l=self.lookahead_l,
            v_max=self.max_lin_speed,
            omega_max=self.max_ang_speed,
            allow_reverse=False,
        )
        u_seq = np.tile([v_cmd, w_cmd], (self.horizon_steps, 1))
        self.pred_traj = self._rollout_sequence(pose, u_seq)

        desired_vel = np.array([v_cmd, 0.0, w_cmd], dtype=float)
        desired_vel = self._stop_if_reached(desired_vel, pose)
        action = self._get_desired_action(desired_vel, vel_robot, orient)
        return action, target_pose
