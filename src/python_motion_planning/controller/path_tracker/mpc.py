"""
@file: mpc.py
@description: Linearized Error-State Model Predictive Control (MPC via OSQP with persistent workspace update),
              Strict Distributed Control Barrier Function Quadratic Program (CBF-QP) Safety Filter,
              Arc-Length Action Dependency Graph (ADG) Schedule Synchronization, and Non-Holonomic ORCA Baseline.
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
    [6] Alonso-Mora, J., Breitenmoser, A., Rufli, M., Beardsley, P., & Siegwart, R. (2013).
        Optimal Reciprocal Collision Avoidance for Multiple Non-Holonomic Robots. Distributed
        Autonomous Robotic Systems (DARS), Springer, 203-216.
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


class _Fast2DQPWorkspace:
    """
    Persistent 2-variable OSQP workspace (`u in R^2`, up to `max_rows` linear constraints)
    that calls `prob.setup(...)` once and uses `prob.update(Px=..., q=..., Ax=..., l=..., u=...)`
    on subsequent calls to eliminate Python/C setup overhead.
    """

    def __init__(self, max_rows: int = 12, eps: float = 1e-5) -> None:
        self.max_rows = int(max_rows)
        self.eps = float(eps)
        self.prob = osqp.OSQP()
        P_init = sp.csc_matrix(np.array([[2.0, 0.1], [0.1, 2.0]], dtype=float))
        A_init = sp.csc_matrix(np.ones((self.max_rows, 2), dtype=float))
        l_init = -1e9 * np.ones(self.max_rows, dtype=float)
        u_init = 1e9 * np.ones(self.max_rows, dtype=float)
        kwargs = {
            "P": P_init,
            "q": np.zeros(2, dtype=float),
            "A": A_init,
            "l": l_init,
            "u": u_init,
            "verbose": False,
            "eps_abs": self.eps,
            "eps_rel": self.eps,
        }
        if _OSQP_V1:
            kwargs["warm_starting"] = True
            kwargs["polishing"] = False
        else:
            kwargs["warm_start"] = True
            kwargs["polish"] = False
        self.prob.setup(**kwargs)

    def solve(
        self,
        M_metric: np.ndarray,
        q: np.ndarray,
        A_dense: np.ndarray,
        lb_arr: np.ndarray,
        ub_arr: np.ndarray,
    ):
        n_rows = len(A_dense)
        if n_rows > self.max_rows:
            return _osqp_solve(
                P=sp.csc_matrix(2.0 * M_metric),
                q=q,
                A=sp.csc_matrix(A_dense),
                lb=lb_arr,
                ub=ub_arr,
                eps=self.eps,
                polish=False,
            )
        A_pad = np.zeros((self.max_rows, 2), dtype=float)
        A_pad[:n_rows, :] = A_dense
        l_pad = -1e9 * np.ones(self.max_rows, dtype=float)
        u_pad = 1e9 * np.ones(self.max_rows, dtype=float)
        l_pad[:n_rows] = np.maximum(lb_arr, -1e9)
        u_pad[:n_rows] = np.minimum(ub_arr, 1e9)
        Px = np.array(
            [2.0 * M_metric[0, 0], 2.0 * M_metric[0, 1], 2.0 * M_metric[1, 1]],
            dtype=float,
        )
        self.prob.update(
            Px=Px,
            q=q,
            Ax=A_pad.ravel(order="F"),
            l=l_pad,
            u=u_pad,
        )
        return self.prob.solve(raise_error=False) if _OSQP_V1 else self.prob.solve()


_CBF_WS = _Fast2DQPWorkspace(max_rows=12, eps=1e-5)
_ORCA_WS = _Fast2DQPWorkspace(max_rows=12, eps=1e-4)


def unicycle_to_lookahead_point(pose: np.ndarray, lookahead_l: float = 0.14) -> np.ndarray:
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
    theta: float, v: float, omega: float, lookahead_l: float = 0.14
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
    lookahead_l: float = 0.14,
    v_max: float = 2.6,
    omega_max: float = 3.14,
    allow_reverse: bool = False,
) -> Tuple[float, float]:
    """
    Exact inverse Near-Identity Diffeomorphism:
        `[v, omega]^T = diag(1, 1/l) R(theta)^T u_vec`
    When `cbf_qp_filter` is called with `theta` and `omega_max`, the linear inequalities
    `0 <= v <= v_max` and `-omega_max <= omega <= omega_max` are already enforced inside the QP,
    so `(v_raw, w_raw)` already satisfies the actuation bounds up to solver tolerance `eps`.
    """
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
    theta: Optional[float] = None,
    lookahead_l: float = 0.14,
    omega_max: Optional[float] = None,
) -> Tuple[np.ndarray, Dict[str, float]]:
    """
    Strict Distributed Control Barrier Function Quadratic Program (CBF-QP) safety filter
    (Wang, Ames, & Egerstedt, IEEE T-RO 2017):

        min_{u in R^2}  ||u - u_nom||_2^2
        s.t.            -2 (p_i - p_j)^T u <= alpha_ij * gamma * h_ij - 2 (p_i - p_j)^T u_j_hat

    Actuation bounds inside the QP:
      - If `theta` and `omega_max` are provided (unicycle NID mode), the exact linear unicycle
        bounds are enforced directly inside the QP:
            0 <= [cos(theta), sin(theta)] u <= u_max
            -omega_max <= [-sin(theta)/l, cos(theta)/l] u <= omega_max
        so that `[v, omega]^T = diag(1, 1/l) R(theta)^T u` satisfies `0 <= v <= u_max` and
        `|omega| <= omega_max` without post-clipping or post-blending.
      - Otherwise (single-integrator mode), enforces `-u_max <= u_x, u_y <= u_max`.

    If the strict QP is infeasible, returns `zeros(2)` (`v = 0, omega = 0`, safe stop).
    """
    t0 = time.perf_counter()
    p_i = np.asarray(p_i, dtype=float)[:2]
    u_nom = np.asarray(u_nom, dtype=float)[:2]
    if not neighbors:
        if theta is not None and omega_max is not None:
            v_c, w_c = single_integrator_to_unicycle_vel(
                theta, u_nom, lookahead_l=lookahead_l, v_max=u_max, omega_max=omega_max
            )
            u_clip = unicycle_vel_to_single_integrator(theta, v_c, w_c, lookahead_l=lookahead_l)
        else:
            u_clip = np.clip(u_nom, -u_max, u_max)
        return u_clip, {
            "status_val": 1.0,
            "solve_time_us": 0.0,
            "wall_time_us": (time.perf_counter() - t0) * 1e6,
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

    if theta is not None and omega_max is not None:
        ct, st = math.cos(float(theta)), math.sin(float(theta))
        inv_l = 1.0 / max(float(lookahead_l), 1e-4)
        # Exact linear unicycle bounds: 0 <= v <= u_max and -omega_max <= omega <= omega_max
        r_v = np.array([ct, st], dtype=float)
        r_w = np.array([-st * inv_l, ct * inv_l], dtype=float)
        A_rows.append(r_v)
        lb.append(0.0)
        ub.append(float(u_max))
        A_rows.append(r_w)
        lb.append(-float(omega_max))
        ub.append(float(omega_max))
        # Actuator-metric objective ||v - v_nom||^2 + rho_w ||omega - omega_nom||^2 in u-coordinates
        rho_w = 1.2
        M_metric = np.outer(r_v, r_v) + rho_w * np.outer(r_w, r_w) + 1e-6 * np.eye(2)
        q = -2.0 * (M_metric @ u_nom)
    else:
        A_rows.append(np.array([1.0, 0.0], dtype=float))
        lb.append(-float(u_max))
        ub.append(float(u_max))
        A_rows.append(np.array([0.0, 1.0], dtype=float))
        lb.append(-float(u_max))
        ub.append(float(u_max))
        M_metric = np.eye(2, dtype=float)
        q = -2.0 * u_nom

    A_dense = np.vstack(A_rows)
    lb_arr = np.asarray(lb, dtype=float)
    ub_arr = np.asarray(ub, dtype=float)

    # Exact optimality check: since M_metric > 0, if u_nom already satisfies lb <= A u_nom <= ub,
    # u_nom is the unique global optimum with objective 0.0.
    Au_nom = A_dense @ u_nom
    if np.all(Au_nom >= lb_arr - 1e-7) and np.all(Au_nom <= ub_arr + 1e-7):
        wall_us = (time.perf_counter() - t0) * 1e6
        return u_nom.copy(), {
            "status_val": 1.0,
            "solve_time_us": 0.0,
            "wall_time_us": wall_us,
            "min_h": min_h,
        }

    res = _CBF_WS.solve(M_metric, q, A_dense, lb_arr, ub_arr)
    wall_us = (time.perf_counter() - t0) * 1e6
    c_solve_us = float(getattr(res.info, "solve_time", 0.0)) * 1e6

    if res.info.status_val in (1, 2) and res.x is not None:
        u_sol = np.asarray(res.x, dtype=float)
        if theta is None:
            u_sol = np.clip(u_sol, -u_max, u_max)
        return u_sol, {
            "status_val": float(res.info.status_val),
            "solve_time_us": c_solve_us if c_solve_us > 0.0 else wall_us,
            "wall_time_us": wall_us,
            "min_h": min_h,
        }
    return np.zeros(2, dtype=float), {
        "status_val": float(res.info.status_val),
        "solve_time_us": c_solve_us if c_solve_us > 0.0 else wall_us,
        "wall_time_us": wall_us,
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
    Arc-Length Action Dependency Graph (ADG-inspired precedence graph, adapted from
    Honig et al., IEEE RA-L 2019 for continuous arc-length parameterized trajectories).

    Extracts the complete partial order of shared cell/corridor visits from the discrete `ECBS`
    space-time trajectories (for all `t_A < t_B`, with no time-horizon truncation) and maps each
    precedence event onto cumulative arc-length `s` along each agent's continuous trajectory.

    If agent `A` visits a shared region at step `t_A` before agent `B` visits it at step `t_B > t_A`,
    then agent `B` cannot advance past `succ_hold_s` until agent `A` has advanced past `pred_leave_s`.
    """

    def __init__(
        self,
        grid: Grid,
        ecbs_paths: Dict[str, Sequence[Tuple[int, int]]],
        world_paths: Dict[str, Sequence[Tuple[float, float, float]]],
        conflict_radius: float = 0.95,
        hold_buffer_m: float = 1.35,
        leave_buffer_m: float = 1.25,
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
                s_vec = np.concatenate(
                    [[0.0], np.cumsum(np.linalg.norm(np.diff(w_arr, axis=0), axis=1))]
                )
            else:
                s_vec = np.zeros(len(w_arr), dtype=float)
            self._world_xy[aid] = w_arr
            self.total_s[aid] = float(s_vec[-1]) if len(s_vec) > 0 else 0.0

            gpath = ecbs_paths[aid]
            step_s = np.zeros(len(gpath), dtype=float)
            search_start = 0
            for k, gpt in enumerate(gpath):
                wpt = np.asarray(grid.map_to_world(gpt), dtype=float)
                window_end = min(
                    len(w_arr),
                    search_start + max(40, len(w_arr) // max(1, len(gpath)) * 6),
                )
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

        # Build Type-2 inter-agent ordering dependencies across the full plan horizon (no time cutoff)
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

                diff = pts_a[:, None, :] - pts_b[None, :, :]
                dist_sq = np.sum(diff * diff, axis=2)
                close_pairs = np.argwhere(dist_sq <= r_sq)
                if len(close_pairs) == 0:
                    continue

                # Coalesce contiguous cells belonging to the same conflict zone
                last_ta, last_tb = -999, -999
                for ta, tb in close_pairs:
                    ta_i, tb_i = int(ta), int(tb)
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
    solved via OSQP with persistent workspace updates (`prob.update()`) and warm-starting
    (Kuhne et al., 2004).

    At each control step:
      1. Projects the robot state onto the piecewise-linear reference trajectory to obtain true
         Frenet coordinates `(s, d_lat, e_theta)` and extracts an `N`-step reference window
         `(x_{r,k}, y_{r,k}, theta_{r,k}, v_{r,k}, omega_{r,k})` along cumulative arc-length `s`.
      2. Linearizes the discrete-time unicycle error dynamics around the reference window:
             e_{k+1} = A_k e_k + B_k u_tilde_k
      3. Updates the pre-allocated `2N`-variable OSQP workspace via `update(Px, q, l, u)` and
         returns the pure optimal first-step control `u_0 = u_{r,0} + u_tilde_0` (no post-QP
         steering blending or heuristic overrides).
    """

    def __init__(
        self,
        *args,
        horizon_steps: int = 8,
        q_pos: float = 36.0,
        q_theta: float = 22.0,
        r_vel: float = 0.15,
        r_omega: float = 0.025,
        r_smooth: float = 0.02,
        w_esdf: float = 0.0,
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

        # Precompute constant matrices and persistent OSQP workspace for 2N variables
        N = self.horizon_steps
        n_vars = 2 * N
        q_diag = np.tile([self.q_pos, self.q_pos, self.q_theta], N)
        q_diag[-3:] *= 1.5
        self._Q_bar = np.diag(q_diag)
        self._R_bar = np.diag(np.tile([self.r_vel, self.r_omega], N))

        D_u = np.eye(n_vars, dtype=float)
        for k in range(1, N):
            D_u[2 * k : 2 * (k + 1), 2 * (k - 1) : 2 * k] = -np.eye(2, dtype=float)
        self._D_u = D_u
        self._S_bar = np.diag(np.tile([self.r_smooth, 0.35 * self.r_smooth], N))
        self._reg_H = (
            self._R_bar
            + self._D_u.T @ self._S_bar @ self._D_u
            + 1e-6 * np.eye(n_vars, dtype=float)
        )

        # Column-major upper-triangular index mapping for OSQP `update(Px=...)`
        self._cols_cm, self._rows_cm = np.tril_indices(n_vars)
        H_init = np.eye(n_vars, dtype=float) + 1e-4 * np.ones((n_vars, n_vars), dtype=float)
        P_init = sp.triu(2.0 * H_init, format="csc")
        A_init = sp.eye(n_vars, format="csc")
        self._mpc_osqp = osqp.OSQP()
        setup_kwargs = {
            "P": P_init,
            "q": np.zeros(n_vars, dtype=float),
            "A": A_init,
            "l": -np.ones(n_vars, dtype=float),
            "u": np.ones(n_vars, dtype=float),
            "verbose": False,
            "eps_abs": 1e-4,
            "eps_rel": 1e-4,
        }
        if _OSQP_V1:
            setup_kwargs["warm_starting"] = True
            setup_kwargs["polishing"] = False
        else:
            setup_kwargs["warm_start"] = True
            setup_kwargs["polish"] = False
        self._mpc_osqp.setup(**setup_kwargs)

        # OSQP solver diagnostics & warm-start buffers
        self.last_solve_time_us: float = 0.0
        self.last_wall_time_us: float = 0.0
        self.last_status: str = "uninitialized"
        self.last_iter: int = 0
        self.solve_times_us: List[float] = []
        self.wall_times_us: List[float] = []
        self._warm_x: Optional[np.ndarray] = None

    def __str__(self) -> str:
        return "Linear-MPC (OSQP)"

    def reset(self) -> None:
        super().reset()
        self.prev_cmd = np.zeros(2, dtype=float)
        self._progress_s = 0.0
        self.last_solve_time_us = 0.0
        self.last_wall_time_us = 0.0
        self.last_status = "reset"
        self.last_iter = 0
        self.solve_times_us.clear()
        self.wall_times_us.clear()
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

        s_frenet, _, _ = self.compute_frenet_state(pos, 0.0)
        self._progress_s = max(self._progress_s, s_frenet)

        s_total = float(s_arr[-1])
        if s_cap is not None:
            s_limit = min(s_total, max(self._progress_s, float(s_cap)))
        else:
            s_limit = s_total

        step_ds = max(0.12, ref_speed * self.dt)
        ref_window = np.zeros((self.horizon_steps, 3), dtype=float)
        for k in range(self.horizon_steps):
            s_k = min(s_limit, self._progress_s + (k + 1) * step_ds)
            ref_window[k] = self._interpolate_at_s(s_k)

        lookahead_s = min(
            s_limit, self._progress_s + max(1.4, self.lookahead_distance * 0.8)
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
        Solve the condensed `2N`-variable linearized error-state MPC Quadratic Program via
        persistent OSQP workspace update (`self._mpc_osqp.update()`) and return the pure QP
        first-step command `(v_0, omega_0)`.
        """
        t_start = time.perf_counter()
        pos = pose[:2]
        goal_pos = self.goal[:2]
        dist_to_goal = float(np.linalg.norm(pos - goal_pos))
        N = self.horizon_steps

        stop_tol = self.goal_dist_tol
        if dist_to_goal < stop_tol or target_speed_scale <= 0.01:
            wall_us = (time.perf_counter() - t_start) * 1e6
            self.last_wall_time_us = wall_us
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
        curv_factor = 1.0 / (1.0 + 0.45 * abs(kappa) + 0.65 * abs(heading_err))
        v_nom = min(
            v_cap,
            max(0.28 * v_cap, v_cap * curv_factor),
            dist_to_goal / max(self.dt * 1.5, 0.15),
        )
        if abs(heading_err) > 1.25:
            v_nom *= 0.40

        dt = float(self.dt)
        u_ref = np.zeros((N, 2), dtype=float)
        A_list: List[np.ndarray] = []
        B_list: List[np.ndarray] = []
        e_free = np.zeros(3 * N, dtype=float)

        # Roll out kinematically consistent nominal trajectory x_bar_{k+1} = f(x_bar_k, u_ref_k)
        # from x_bar_0 = pose, and linearize unicycle dynamics around (x_bar_k, u_ref_k)
        x_bar = float(pose[0])
        y_bar = float(pose[1])
        th_bar = float(pose[2])

        for k in range(N):
            dx_r = float(ref_window[k, 0] - x_bar)
            dy_r = float(ref_window[k, 1] - y_bar)
            ref_th_k = float(ref_window[k, 2])
            fwd_proj = dx_r * math.cos(ref_th_k) + dy_r * math.sin(ref_th_k)
            dist_step = math.hypot(dx_r, dy_r)
            if fwd_proj > 0.12 and dist_step > 0.12:
                desired_yaw = math.atan2(dy_r, dx_r)
            else:
                desired_yaw = ref_th_k
            yaw_err = float(
                Geometry.regularize_orient(np.array([desired_yaw - th_bar]))[0]
            )
            w_rk = float(
                np.clip(1.8 * yaw_err, -self.max_ang_speed, self.max_ang_speed)
            )
            step_v_lim = dist_step / dt if fwd_proj <= 0.25 else v_nom
            v_rk = float(
                np.clip(
                    min(v_nom, step_v_lim) * max(0.15, math.cos(min(abs(yaw_err), 1.35))),
                    0.0,
                    v_cap,
                )
            )
            u_ref[k, 0] = v_rk
            u_ref[k, 1] = w_rk

            ct, st = math.cos(th_bar), math.sin(th_bar)
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

            # Advance nominal state x_bar_{k+1}
            x_bar += v_rk * ct * dt
            y_bar += v_rk * st * dt
            th_bar = float(
                Geometry.regularize_orient(np.array([th_bar + w_rk * dt]))[0]
            )
            e_free[3 * k] = x_bar - float(ref_window[k, 0])
            e_free[3 * k + 1] = y_bar - float(ref_window[k, 1])
            e_free[3 * k + 2] = float(
                Geometry.regularize_orient(
                    np.array([th_bar - float(ref_window[k, 2])])
                )[0]
            )

        Gamma = np.zeros((3 * N, 2 * N), dtype=float)
        for k in range(N):
            G_step = np.eye(3, dtype=float)
            for j in range(k, -1, -1):
                if j < k:
                    G_step = G_step @ A_list[j + 1]
                Gamma[3 * k : 3 * (k + 1), 2 * j : 2 * (j + 1)] = G_step @ B_list[j]

        u_prev_offset = np.zeros(2 * N, dtype=float)
        u_prev_offset[:2] = self.prev_cmd
        u_ref_flat = u_ref.ravel()
        d_const = self._D_u @ u_ref_flat - u_prev_offset

        H = Gamma.T @ self._Q_bar @ Gamma + self._reg_H
        g = Gamma.T @ (self._Q_bar @ e_free) + self._D_u.T @ (self._S_bar @ d_const)

        lb = np.zeros(2 * N, dtype=float)
        ub = np.zeros(2 * N, dtype=float)
        for k in range(N):
            lb[2 * k] = 0.0 - u_ref[k, 0]
            ub[2 * k] = v_cap - u_ref[k, 0]
            lb[2 * k + 1] = -self.max_ang_speed - u_ref[k, 1]
            ub[2 * k + 1] = self.max_ang_speed - u_ref[k, 1]

        P_two = 2.0 * H
        q_vec = 2.0 * g
        self._mpc_osqp.update(
            Px=P_two[self._rows_cm, self._cols_cm],
            q=q_vec,
            l=lb,
            u=ub,
        )
        if self._warm_x is not None and len(self._warm_x) == 2 * N:
            self._mpc_osqp.warm_start(x=self._warm_x)

        res = self._mpc_osqp.solve(raise_error=False) if _OSQP_V1 else self._mpc_osqp.solve()
        wall_us = (time.perf_counter() - t_start) * 1e6
        c_solve_us = float(getattr(res.info, "solve_time", 0.0)) * 1e6
        self.last_solve_time_us = c_solve_us if c_solve_us > 0.0 else wall_us
        self.last_wall_time_us = wall_us
        self.solve_times_us.append(self.last_solve_time_us)
        self.wall_times_us.append(wall_us)
        self.last_status = str(res.info.status)
        self.last_iter = int(res.info.iter)

        if res.info.status_val in (1, 2) and res.x is not None:
            u_tilde = np.clip(np.asarray(res.x, dtype=float), lb, ub)
            self._warm_x = np.concatenate([u_tilde[2:], u_tilde[-2:]])
            u_seq = u_ref + u_tilde.reshape(N, 2)
        else:
            u_seq = u_ref.copy()

        # Pure OSQP MPC first-step control (zero post-blending or heuristic steering override)
        v_qp = float(np.clip(u_seq[0, 0], 0.0, v_cap))
        w_qp = float(np.clip(u_seq[0, 1], -self.max_ang_speed, self.max_ang_speed))
        u_seq[0, 0] = v_qp
        u_seq[0, 1] = w_qp
        pred_traj = self._rollout_sequence(pose, u_seq)
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
      1. Optional Arc-Length Action Dependency Graph (`adg`) schedule synchronization,
      2. Priority-weighted velocity consensus + Linearized Error-State OSQP MPC (`MPC`),
      3. Optional Strict Distributed Control Barrier Function Quadratic Program (`cbf_qp_filter`,
         Wang et al., 2017) via Near-Identity Diffeomorphism (`lookahead_l`) with exact linear
         unicycle bounds inside the QP and full `2 * lookahead_l` clearance compensation.
    """

    def __init__(
        self,
        *args,
        self_id: str = "1",
        peer_robots: Optional[Dict[str, BaseRobot]] = None,
        priority: int = 1,
        priorities: Optional[Dict[str, int]] = None,
        adg: Optional[ActionDependencyGraph] = None,
        use_cbf: bool = True,
        use_consensus: bool = False,
        sensing_radius: float = 3.0,
        safe_margin: float = 0.02,
        cbf_gamma: float = 2.2,
        lookahead_l: float = 0.04,
        consensus_gain: float = 0.20,
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
        self.use_cbf = bool(use_cbf)
        self.use_consensus = bool(use_consensus)
        self.sensing_radius = float(sensing_radius)
        self.safe_margin = float(safe_margin)
        self.cbf_gamma = float(cbf_alpha * 4.0 if cbf_alpha is not None else cbf_gamma)
        self.lookahead_l = float(lookahead_l)
        self.consensus_gain = float(consensus_gain)
        self.last_cbf_min_h: float = float("inf")
        self.last_cbf_status: float = 1.0

    def _is_lower_priority_than(self, peer_id: str) -> bool:
        """
        Determine whether `self_id` should yield to `peer_id`.
        If an `ActionDependencyGraph` (`self.adg`) is active and defines an active ordering
        between `self_id` and `peer_id`, follow the ADG precedence order so velocity consensus
        and CBF-QP weights never contradict the ADG schedule.
        """
        peer_s = str(peer_id)
        if self.adg is not None:
            for dep in self.adg.dependencies_by_succ.get(self.self_id, ()):
                if dep.pred_agent == peer_s and self.adg.progress_s.get(peer_s, 0.0) < dep.pred_leave_s:
                    return True
            for dep in self.adg.dependencies_by_succ.get(peer_s, ()):
                if dep.pred_agent == self.self_id and self.adg.progress_s.get(self.self_id, 0.0) < dep.pred_leave_s:
                    return False
        peer_prio = self.priorities.get(peer_s, 1)
        return (self.priority < peer_prio) or (
            self.priority == peer_prio and str(self.self_id) > peer_s
        )

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

        if not self.use_consensus or not self.peer_robots:
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
            d_safe = d_contact + 2.0 * self.lookahead_l + self.safe_margin
            unit_rel = rel_pos / dist

            theta_j = float(peer.orient[0]) if len(peer.orient) > 0 else 0.0
            heading_j = np.array([math.cos(theta_j), math.sin(theta_j)], dtype=float)
            heading_align = abs(float(np.dot(heading_vec, heading_j)))
            lat_sep = abs(
                float(rel_pos[0] * (-heading_vec[1]) + rel_pos[1] * heading_vec[0])
            )
            if heading_align > 0.88 and lat_sep >= d_contact + 0.12:
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

            is_lower_prio = self._is_lower_priority_than(str(peer_id))
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
        Filter the nominal unicycle command `(v_nom, w_nom)` through the strict distributed CBF-QP
        (`cbf_qp_filter`) using Near-Identity Diffeomorphism:
          - Includes ALL peers within `self.sensing_radius` (no heuristic lane skipping).
          - Uses peer `j`'s true lookahead point velocity `p_dot_j = R(theta_j) diag(1, l) [v_j, w_j]^T`.
          - Adds `2 * lookahead_l` to `d_safe` so lookahead safety implies center-to-center safety.
          - Enforces `0 <= v <= v_max` and `|omega| <= omega_max` directly inside the QP linear constraints
            and returns the exact inverse-NID command `(v_safe, w_safe)` without post-blending.
        """
        if not self.use_cbf or not self.peer_robots:
            return v_nom, w_nom

        theta_i = float(pose[2])
        p_i = unicycle_to_lookahead_point(pose, self.lookahead_l)
        u_nom = unicycle_vel_to_single_integrator(
            theta_i, v_nom, w_nom, self.lookahead_l
        )
        r_i = getattr(self.robot_model, "radius", 0.44)

        neighbors: List[Tuple[np.ndarray, np.ndarray]] = []
        weights: List[float] = []
        # Full lookahead offset compensation: ||x_i - x_j|| >= ||p_i - p_j|| - 2*l
        d_safe_eff = 2.0 * r_i + 2.0 * self.lookahead_l + self.safe_margin

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

            peer_pose = np.array([pos_j[0], pos_j[1], theta_j], dtype=float)
            p_j = unicycle_to_lookahead_point(peer_pose, self.lookahead_l)
            v_j_body = float(np.dot(peer.lin_vel[:2], heading_j))
            w_j_body = float(peer.ang_vel[0]) if len(peer.ang_vel) > 0 else 0.0
            u_j = unicycle_vel_to_single_integrator(
                theta_j, v_j_body, w_j_body, self.lookahead_l
            )
            d_safe_eff = max(
                d_safe_eff, d_contact + 2.0 * self.lookahead_l + self.safe_margin
            )

            is_lower_prio = self._is_lower_priority_than(str(peer_id))
            weights.append(0.35 if is_lower_prio else 0.65)
            neighbors.append((p_j, u_j))

        if not neighbors:
            return v_nom, w_nom

        u_safe, cbf_info = cbf_qp_filter(
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
        self.last_solve_time_us += cbf_info["solve_time_us"]
        self.last_wall_time_us += cbf_info["wall_time_us"]
        if self.solve_times_us:
            self.solve_times_us[-1] = self.last_solve_time_us
        if self.wall_times_us:
            self.wall_times_us[-1] = self.last_wall_time_us
        self.last_cbf_min_h = cbf_info["min_h"]
        self.last_cbf_status = cbf_info["status_val"]

        v_safe, w_safe = single_integrator_to_unicycle_vel(
            theta_i,
            u_safe,
            lookahead_l=self.lookahead_l,
            v_max=self.max_lin_speed,
            omega_max=self.max_ang_speed,
            allow_reverse=False,
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
    Non-Holonomic Optimal Reciprocal Collision Avoidance (NH-ORCA) multi-agent controller
    (van den Berg et al., ISRR 2011; Alonso-Mora et al., DARS 2013) mapped to differential-drive
    lookahead kinematics via Near-Identity Diffeomorphism and solved via OSQP with exact linear
    unicycle actuation constraints (`0 <= v <= v_max`, `|omega| <= omega_max`).
    """

    def __init__(
        self,
        *args,
        self_id: str = "1",
        peer_robots: Optional[Dict[str, BaseRobot]] = None,
        adg: Optional[ActionDependencyGraph] = None,
        time_horizon: float = 2.4,
        safe_margin: float = 0.08,
        lookahead_l: float = 0.14,
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.self_id = str(self_id)
        self.peer_robots: Dict[str, BaseRobot] = (
            peer_robots if peer_robots is not None else {}
        )
        self.adg: Optional[ActionDependencyGraph] = adg
        self.time_horizon = float(time_horizon)
        self.safe_margin = float(safe_margin)
        self.lookahead_l = float(lookahead_l)

    def __str__(self) -> str:
        return "ADG+ORCA" if self.adg is not None else "ORCA"

    def get_action(self, obs: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if self.goal is None:
            return np.zeros(self.action_space.shape), self.goal

        pose, vel, pos, orient, lin_vel, ang_vel = self.get_pose_velocity(obs)
        theta_A = float(pose[2])
        s_frenet, _, _ = self.compute_frenet_state(pos, theta_A)
        self._progress_s = max(self._progress_s, s_frenet)

        s_cap: Optional[float] = None
        if self.adg is not None:
            self.adg.update_progress(self.self_id, self._progress_s)
            s_cap = self.adg.max_allowed_s(self.self_id)

        ref_window, target_pose = self._reference_window(
            pos, self.max_lin_speed, s_cap=s_cap
        )
        vel_robot = FrameTransformer.vel_world_to_robot(self.dim, vel, orient)

        dist_to_goal = float(np.linalg.norm(pos - self.goal[:2]))
        if dist_to_goal < self.goal_dist_tol or (
            s_cap is not None and s_cap - self._progress_s <= 0.20
        ):
            desired_vel = np.zeros(3, dtype=float)
            return self._get_desired_action(desired_vel, vel_robot, orient), target_pose

        p_A = unicycle_to_lookahead_point(pose, self.lookahead_l)
        heading_A = np.array([math.cos(theta_A), math.sin(theta_A)], dtype=float)
        v_A_body = float(np.dot(vel[:2], heading_A))
        w_A_body = float(ang_vel[0]) if len(ang_vel) > 0 else 0.0
        u_A = unicycle_vel_to_single_integrator(
            theta_A, v_A_body, w_A_body, self.lookahead_l
        )

        # Preferred lookahead velocity via pure pursuit curvature toward lookahead target
        rel_target = FrameTransformer.pose_world_to_robot(self.dim, target_pose, pose)
        rx, ry = float(rel_target[0]), float(rel_target[1])
        L = max(math.hypot(rx, ry), 0.2)
        kappa = 2.0 * ry / (L * L)
        heading_err = math.atan2(ry, rx)
        v_pref_s = min(
            self.max_lin_speed / (1.0 + 0.7 * abs(kappa) + 0.9 * abs(heading_err)),
            dist_to_goal / max(self.dt * 2.0, 0.2),
        )
        w_pref_s = float(
            np.clip(
                v_pref_s * kappa + 1.6 * heading_err,
                -self.max_ang_speed,
                self.max_ang_speed,
            )
        )
        v_pref = unicycle_vel_to_single_integrator(
            theta_A, v_pref_s, w_pref_s, self.lookahead_l
        )
        r_A = getattr(self.robot_model, "radius", 0.44)

        A_rows: List[np.ndarray] = []
        lb: List[float] = []
        ub: List[float] = []
        inv_tau = 1.0 / max(self.time_horizon, 0.2)

        for peer_id, peer in self.peer_robots.items():
            if str(peer_id) == self.self_id:
                continue
            pos_B = peer.pos[:2]
            if float(np.linalg.norm(pos_B - pos[:2])) > 7.5:
                continue
            theta_B = float(peer.orient[0]) if len(peer.orient) > 0 else 0.0
            heading_B = np.array([math.cos(theta_B), math.sin(theta_B)], dtype=float)
            p_B = unicycle_to_lookahead_point(
                np.array([pos_B[0], pos_B[1], theta_B]), self.lookahead_l
            )
            v_B_body = float(np.dot(peer.lin_vel[:2], heading_B))
            w_B_body = float(peer.ang_vel[0]) if len(peer.ang_vel) > 0 else 0.0
            u_B = unicycle_vel_to_single_integrator(
                theta_B, v_B_body, w_B_body, self.lookahead_l
            )
            r_B = getattr(peer, "radius", 0.44)
            R_ab = r_A + r_B + 2.0 * self.lookahead_l + self.safe_margin

            rel_p = p_B - p_A
            dist = float(np.linalg.norm(rel_p))
            if dist < 1e-6:
                continue

            rel_v = u_A - u_B
            w_vec = rel_v - inv_tau * rel_p
            w_len = float(np.linalg.norm(w_vec))
            unit_p = rel_p / dist

            if dist <= R_ab:
                n_out = -unit_p
                u_orca = (R_ab - dist) / max(self.dt, 0.05) * n_out - rel_v
            else:
                leg_sq = max(0.0, dist * dist - R_ab * R_ab)
                cos_half = math.sqrt(leg_sq) / dist
                # Project onto velocity obstacle cone boundary or cutoff circle
                dot_wp = float(np.dot(rel_v, unit_p))
                if dot_wp > inv_tau * R_ab * cos_half and w_len < R_ab * inv_tau and w_len > 1e-6:
                    n_out = w_vec / w_len
                    u_orca = (R_ab * inv_tau - w_len) * n_out
                elif w_len < R_ab * inv_tau and w_len > 1e-6:
                    n_out = w_vec / w_len
                    u_orca = (R_ab * inv_tau - w_len) * n_out
                else:
                    continue

            rhs = -float(np.dot(u_A + 0.5 * u_orca, n_out))
            A_rows.append(-n_out)
            lb.append(-np.inf)
            ub.append(rhs)

        # Exact linear unicycle actuation bounds inside the ORCA QP
        ct, st = math.cos(theta_A), math.sin(theta_A)
        inv_l = 1.0 / max(self.lookahead_l, 1e-4)
        r_v = np.array([ct, st], dtype=float)
        r_w = np.array([-st * inv_l, ct * inv_l], dtype=float)
        A_rows.append(r_v)
        lb.append(0.0)
        ub.append(self.max_lin_speed)
        A_rows.append(r_w)
        lb.append(-self.max_ang_speed)
        ub.append(self.max_ang_speed)

        t_orca = time.perf_counter()
        A_dense = np.vstack(A_rows)
        lb_arr = np.asarray(lb, dtype=float)
        ub_arr = np.asarray(ub, dtype=float)
        Av_pref = A_dense @ v_pref
        if np.all(Av_pref >= lb_arr - 1e-7) and np.all(Av_pref <= ub_arr + 1e-7):
            wall_us = (time.perf_counter() - t_orca) * 1e6
            self.last_solve_time_us = 0.0
            self.last_wall_time_us = wall_us
            self.solve_times_us.append(0.0)
            self.wall_times_us.append(wall_us)
            v_cmd, w_cmd = v_pref_s, w_pref_s
        else:
            rho_w = 0.85
            M_metric = np.outer(r_v, r_v) + rho_w * np.outer(r_w, r_w) + 1e-6 * np.eye(2)
            q = -2.0 * (M_metric @ v_pref)
            res = _ORCA_WS.solve(M_metric, q, A_dense, lb_arr, ub_arr)
            wall_us = (time.perf_counter() - t_orca) * 1e6
            c_solve_us = float(getattr(res.info, "solve_time", 0.0)) * 1e6
            self.last_solve_time_us = c_solve_us if c_solve_us > 0.0 else wall_us
            self.last_wall_time_us = wall_us
            self.solve_times_us.append(self.last_solve_time_us)
            self.wall_times_us.append(wall_us)

            if res.info.status_val in (1, 2) and res.x is not None:
                v_orca = np.asarray(res.x, dtype=float)
                v_cmd, w_cmd = single_integrator_to_unicycle_vel(
                    theta_A,
                    v_orca,
                    lookahead_l=self.lookahead_l,
                    v_max=self.max_lin_speed,
                    omega_max=self.max_ang_speed,
                    allow_reverse=False,
                )
            else:
                v_cmd, w_cmd = 0.0, 0.0

        u_seq = np.tile([v_cmd, w_cmd], (self.horizon_steps, 1))
        self.pred_traj = self._rollout_sequence(pose, u_seq)

        desired_vel = np.array([v_cmd, 0.0, w_cmd], dtype=float)
        desired_vel = self._stop_if_reached(desired_vel, pose)
        action = self._get_desired_action(desired_vel, vel_robot, orient)
        return action, target_pose
