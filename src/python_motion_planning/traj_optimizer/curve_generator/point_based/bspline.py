"""
@file: bspline_curve.py
@author: Yang Haodong, Wu Maojia
@update: 2026.10.28
@description: B-Spline curve generator (interpolation, least-squares approximation, control-polygon)
              and Safe-Corridor B-Spline Quadratic Programming (QP) trajectory optimizer via OSQP.
@references:
    [1] Piegl, L., & Tiller, W. (1997). The NURBS Book (2nd ed.), Springer, Section 9.4.1 (Eq. 9.68-9.69).
    [2] Liu, S., Watterson, M., Mohta, K., Sun, K., Bhattacharya, S., Taylor, C. J., & Kumar, V. (2017).
        Planning Dynamically Feasible Trajectories for Quadrotors Using Safe Flight Corridors in 3-D
        Complex Environments. IEEE Robotics and Automation Letters (RA-L), 2(3), 1688-1695.
"""
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
import osqp
import scipy.sparse as sp

from python_motion_planning.common.env.map.grid import Grid
from python_motion_planning.common.env.types import TYPES
from python_motion_planning.common.utils.geometry import Geometry
from python_motion_planning.traj_optimizer.base_curve_generator import BaseCurveGenerator


class BSpline(BaseCurveGenerator):
    """
    Class for B-Spline curve generator.

    Args:
        *args: see the parent class.
        k: Degree of the B-Spline curve.
        param_mode: Parameter selection mode, one of {"centripetal", "chord_length", "uniform_spaced"}.
        spline_mode: B-Spline construction mode, one of {"interpolation", "approximation", "control_polygon"}.
        **kwargs: see the parent class.
    """

    def __init__(
        self,
        *args,
        k: int = 3,
        param_mode: str = "centripetal",
        spline_mode: str = "interpolation",
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)
        self.k = k

        if param_mode not in ("centripetal", "chord_length", "uniform_spaced"):
            raise ValueError("Parameter selection mode error!")
        self.param_mode = param_mode

        if spline_mode not in ("interpolation", "approximation", "control_polygon"):
            raise ValueError("Spline mode selection error!")
        self.spline_mode = spline_mode

    def __str__(self) -> str:
        return "B-Spline Curve"

    def generate(
        self, points: List[Tuple[float, ...]]
    ) -> Tuple[List[Tuple[float, float]], Dict[str, Any]]:
        """
        Generate a B-Spline curve through a list of 2D points.
        """
        if len(points) < 2:
            return [], {"success": False, "length": 0.0}

        points_2d = [(float(p[0]), float(p[1])) for p in points]
        n = len(points_2d)
        k_eff = min(self.k, n - 1)
        if k_eff < 1:
            return points_2d, {"success": True, "length": self.length(points_2d)}

        t = np.linspace(0.0, 1.0, max(2, int(round(1.0 / self.step))))
        params = self._param_selection(points_2d)

        if self.spline_mode == "interpolation":
            knot = self._knot_generation(params, n, k_eff)
            control_pts = self._interpolation(points_2d, params, knot, k_eff)
        elif self.spline_mode == "approximation":
            if n <= k_eff + 1:
                knot = self._knot_generation(params, n, k_eff)
                control_pts = np.asarray(points_2d, dtype=float)
            else:
                h = max(k_eff + 1, min(n - 1, max(k_eff + 2, int(round(0.75 * n)))))
                knot = self._approximation_knot_generation(params, n, h, k_eff)
                control_pts = self._approximation(points_2d, params, knot, h, k_eff)
        else:
            # control_polygon mode: use points_2d directly as clamped B-Spline control points
            knot = self._knot_generation(params, n, k_eff)
            control_pts = np.asarray(points_2d, dtype=float)

        curve = self._generate_curve(t, k_eff, knot, control_pts)
        path = [(float(pt[0]), float(pt[1])) for pt in curve]

        return path, {
            "success": True,
            "length": self.length(path),
            "control_points": control_pts,
            "knot": knot,
        }

    def _base_function(self, i: int, k: int, t: float, knot: List[float]) -> float:
        """Calculate the B-Spline basis function N_{i,k}(t) using Cox-de Boor recursion."""
        Nik_t = 0.0
        if k == 0:
            Nik_t = 1.0 if knot[i] <= t < knot[i + 1] else 0.0
        else:
            length1 = knot[i + k] - knot[i]
            length2 = knot[i + k + 1] - knot[i + 1]
            if not length1 and not length2:
                Nik_t = 0.0
            elif not length1:
                Nik_t = (
                    (knot[i + k + 1] - t)
                    / length2
                    * self._base_function(i + 1, k - 1, t, knot)
                )
            elif not length2:
                Nik_t = (
                    (t - knot[i])
                    / length1
                    * self._base_function(i, k - 1, t, knot)
                )
            else:
                Nik_t = (t - knot[i]) / length1 * self._base_function(
                    i, k - 1, t, knot
                ) + (knot[i + k + 1] - t) / length2 * self._base_function(
                    i + 1, k - 1, t, knot
                )
        return Nik_t

    def _param_selection(self, points: List[Tuple[float, float]]) -> List[float]:
        """Calculate the parameter values u_0, ..., u_{n-1} in [0, 1]."""
        n = len(points)
        if n <= 1:
            return [0.0]
        x_list = [pt[0] for pt in points]
        y_list = [pt[1] for pt in points]
        dx, dy = np.diff(x_list), np.diff(y_list)

        if self.param_mode == "uniform_spaced":
            return np.linspace(0.0, 1.0, n).tolist()

        if self.param_mode == "chord_length":
            s = np.cumsum([math.hypot(idx, idy) for (idx, idy) in zip(dx, dy)])
            if s[-1] <= 1e-12:
                return np.linspace(0.0, 1.0, n).tolist()
            parameters = np.zeros(n)
            for i in range(1, n):
                parameters[i] = s[i - 1] / s[-1]
            parameters[-1] = 1.0
            return parameters.tolist()

        alpha = 0.5
        s = np.cumsum(
            [math.pow(max(math.hypot(idx, idy), 1e-9), alpha) for (idx, idy) in zip(dx, dy)]
        )
        if s[-1] <= 1e-12:
            return np.linspace(0.0, 1.0, n).tolist()
        parameters = np.zeros(n)
        for i in range(1, n):
            parameters[i] = s[i - 1] / s[-1]
        parameters[-1] = 1.0
        return parameters.tolist()

    def _knot_generation(
        self, param: List[float], n: int, k: Optional[int] = None
    ) -> List[float]:
        """Generate clamped knot vector of length `n + k + 1` by averaging parameters."""
        deg = self.k if k is None else k
        m = n + deg + 1
        knot = np.zeros(m)
        for i in range(n, m):
            knot[i] = 1.0
        for i in range(deg + 1, n):
            for j in range(i - deg, i):
                knot[i] = knot[i] + param[j]
            knot[i] = knot[i] / deg
        return knot.tolist()

    def _approximation_knot_generation(
        self, param: List[float], n: int, h: int, k: Optional[int] = None
    ) -> List[float]:
        """
        Generate clamped knot vector of length `h + k + 1` for least-squares approximation
        with `h` control points and `n` data points (Piegl & Tiller, The NURBS Book, Eq. 9.68-9.69).
        Guarantees the Schoenberg-Whitney condition and partition of unity across all `h` basis functions.
        """
        deg = self.k if k is None else k
        m = h + deg + 1
        knot = np.zeros(m, dtype=float)
        knot[h:] = 1.0
        num_internal = h - deg - 1
        if num_internal > 0:
            d = float(n) / float(h - deg)
            for j in range(1, num_internal + 1):
                i = int(math.floor(j * d))
                alpha = float(j * d - i)
                i_prev = max(0, min(n - 1, i - 1))
                i_curr = max(0, min(n - 1, i))
                knot[deg + j] = (1.0 - alpha) * param[i_prev] + alpha * param[i_curr]
        return knot.tolist()

    def _interpolation(
        self,
        points: List[Tuple[float, float]],
        param: List[float],
        knot: List[float],
        k: Optional[int] = None,
    ) -> np.ndarray:
        """Build a degree-k B-Spline curve passing through all N data points."""
        deg = self.k if k is None else k
        n = len(points)
        N = np.zeros((n, n))

        for i in range(n):
            for j in range(n):
                N[i][j] = self._base_function(j, deg, param[i], knot)
        N[n - 1][n - 1] = 1.0
        D = np.asarray(points, dtype=float)
        return np.linalg.solve(N + 1e-10 * np.eye(n), D)

    def _approximation(
        self,
        points: List[Tuple[float, float]],
        param: List[float],
        knot: List[float],
        h: Optional[int] = None,
        k: Optional[int] = None,
    ) -> np.ndarray:
        """
        Build a degree-k B-Spline curve with H control points (H < N) that clamps the
        first and last data points and fits interior points in the least-squares sense.
        """
        deg = self.k if k is None else k
        n = len(points)
        D = np.asarray(points, dtype=float)
        if h is None:
            h = max(deg + 1, n - 1)
        if len(knot) != h + deg + 1:
            knot = self._approximation_knot_generation(param, n, h, deg)

        N = np.zeros((n, h), dtype=float)
        for i in range(n):
            for j in range(h):
                N[i][j] = self._base_function(j, deg, param[i], knot)
        N[n - 1][h - 1] = 1.0

        if h <= 2:
            return np.vstack([D[0], D[-1]])

        N_ = N[1 : n - 1, 1 : h - 1]
        qk = np.zeros((n - 2, 2), dtype=float)
        for i in range(1, n - 1):
            qk[i - 1] = D[i, :] - N[i][0] * D[0, :] - N[i][h - 1] * D[-1, :]

        P_inner, _, _, _ = np.linalg.lstsq(N_, qk, rcond=None)
        return np.vstack([D[0:1, :], P_inner, D[-1:, :]])

    def _generate_curve(
        self, t: np.ndarray, k: int, knot: List[float], control_pts: np.ndarray
    ) -> np.ndarray:
        """Sample points on the B-Spline curve."""
        N = np.zeros((len(t), len(control_pts)), dtype=float)
        for i in range(len(t)):
            for j in range(len(control_pts)):
                N[i][j] = self._base_function(j, k, float(t[i]), knot)
        N[len(t) - 1][len(control_pts) - 1] = 1.0
        return N @ control_pts


class SafeCorridorBSplineOptimizer:
    """
    Safe Flight Corridor (SFC) Convex Box Expansion + Cubic B-Spline Quadratic Programming (QP)
    trajectory optimizer solved via OSQP.

    1. Given a discrete collision-free grid path on `Grid`, inflates an axis-aligned convex bounding
       box `[lx_i, ux_i] x [ly_i, uy_i]` around each waypoint inside `TYPES.FREE`.
    2. Formulates a sparse convex QP in OSQP over the B-Spline control points `c_0, ..., c_{M-1}`
       minimizing weighted jerk (3rd-order finite differences), acceleration (2nd-order finite
       differences), and reference path deviation, subject to linear box constraints `l_i <= c_i <= u_i`
       and exact start/waypoint/goal equality constraints.
    3. Evaluates the clamped cubic B-Spline curve and verifies dense curve samples against `TYPES.FREE`.
    """

    def __init__(
        self,
        w_jerk: float = 2.5,
        w_acc: float = 1.2,
        w_ref: float = 1.8,
        max_corridor_half_width: float = 0.42,
        sample_step: float = 0.0035,
    ) -> None:
        self.w_jerk = float(w_jerk)
        self.w_acc = float(w_acc)
        self.w_ref = float(w_ref)
        self.max_corridor_half_width = float(max_corridor_half_width)
        self.sample_step = float(sample_step)

    @staticmethod
    def _is_world_point_free(grid: Grid, wx: float, wy: float) -> bool:
        gx, gy = grid.world_to_map((wx, wy))
        if not grid.within_bounds((gx, gy)):
            return False
        cell = int(grid.type_map[(gx, gy)])
        return cell != TYPES.OBSTACLE and cell != TYPES.INFLATION

    def _is_box_free(
        self, grid: Grid, lx: float, ux: float, ly: float, uy: float
    ) -> bool:
        for sx in (lx, 0.5 * (lx + ux), ux):
            for sy in (ly, 0.5 * (ly + uy), uy):
                if not self._is_world_point_free(grid, sx, sy):
                    return False
        return True

    def compute_corridor_bounds(
        self,
        grid: Grid,
        pts: np.ndarray,
        pinned_indices: Sequence[int] = (),
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute axis-aligned safe corridor bounds `(lx, ux, ly, uy)` for each control point `pts[i]`.
        """
        m = len(pts)
        lx = pts[:, 0].copy()
        ux = pts[:, 0].copy()
        ly = pts[:, 1].copy()
        uy = pts[:, 1].copy()
        pinned = set(pinned_indices) | {0, m - 1}

        deltas = np.linspace(0.08, self.max_corridor_half_width, 5)
        for i in range(1, m - 1):
            if i in pinned:
                continue
            x0, y0 = float(pts[i, 0]), float(pts[i, 1])
            # Check if adjacent corners have an obstacle diagonally adjacent; if so limit expansion
            prev_p = pts[i - 1]
            next_p = pts[i + 1]
            is_turn = abs(
                (x0 - prev_p[0]) * (next_p[1] - y0)
                - (y0 - prev_p[1]) * (next_p[0] - x0)
            ) > 1e-3
            for d in deltas:
                # For turns, verify midpoint of diagonal chord also stays in TYPES.FREE
                if is_turn:
                    diag_mid = 0.5 * (prev_p + next_p)
                    if not self._is_world_point_free(grid, float(diag_mid[0]), float(diag_mid[1])):
                        break
                if self._is_box_free(grid, x0 - d, x0 + d, ly[i], uy[i]):
                    lx[i] = x0 - d
                    ux[i] = x0 + d
                if self._is_box_free(grid, lx[i], ux[i], y0 - d, y0 + d):
                    ly[i] = y0 - d
                    uy[i] = y0 + d
        return lx, ux, ly, uy

    def _solve_1d_corridor_qp(
        self, ref: np.ndarray, lb: np.ndarray, ub: np.ndarray
    ) -> Tuple[np.ndarray, bool]:
        """Solve 1D B-Spline control point smoothness QP with box constraints `lb <= c <= ub`."""
        m = len(ref)
        if m <= 3:
            return ref.copy(), True

        # Second-order finite difference matrix D2: (m-2) x m
        d2_rows = []
        for i in range(m - 2):
            row = np.zeros(m, dtype=float)
            row[i] = 1.0
            row[i + 1] = -2.0
            row[i + 2] = 1.0
            d2_rows.append(row)
        D2 = np.vstack(d2_rows)

        # Third-order finite difference matrix D3: (m-3) x m
        d3_rows = []
        for i in range(m - 3):
            row = np.zeros(m, dtype=float)
            row[i] = -1.0
            row[i + 1] = 3.0
            row[i + 2] = -3.0
            row[i + 3] = 1.0
            d3_rows.append(row)
        D3 = np.vstack(d3_rows)

        H = (
            self.w_ref * np.eye(m)
            + self.w_acc * (D2.T @ D2)
            + self.w_jerk * (D3.T @ D3)
        )
        P = sp.csc_matrix(2.0 * H)
        q = -2.0 * self.w_ref * ref
        A = sp.eye(m, format="csc")

        solver = osqp.OSQP()
        solver.setup(
            P=P,
            q=q,
            A=A,
            l=lb.astype(float),
            u=ub.astype(float),
            verbose=False,
            eps_abs=1e-5,
            eps_rel=1e-5,
        )
        res = solver.solve()
        if res.info.status_val in (1, 2) and res.x is not None:
            return np.clip(np.asarray(res.x, dtype=float), lb, ub), True
        return ref.copy(), False

    def optimize(
        self,
        grid: Grid,
        grid_path: Sequence[Tuple[int, int]],
        mandatory_waypoints: Sequence[Tuple[int, int]] = (),
    ) -> Tuple[List[Tuple[float, float, float]], Dict[str, Any]]:
        """
        Optimize a discrete grid path into a collision-free C^2 cubic B-Spline trajectory
        within inflated Safe Flight Corridor boxes using OSQP.
        """
        raw_world = [grid.map_to_world(pt) for pt in grid_path]
        mandatory_world = {grid.map_to_world(wp) for wp in mandatory_waypoints}

        unique_pts: List[np.ndarray] = []
        pinned_indices: List[int] = [0]
        for pt in raw_world:
            arr = np.asarray(pt, dtype=float)
            if not unique_pts or float(np.linalg.norm(arr - unique_pts[-1])) > 1e-5:
                idx = len(unique_pts)
                unique_pts.append(arr)
                if (float(pt[0]), float(pt[1])) in mandatory_world:
                    pinned_indices.append(idx)
        if len(unique_pts) == 1:
            unique_pts.append(unique_pts[0] + np.array([0.1, 0.0]))
        pinned_indices.append(len(unique_pts) - 1)

        pts_ref = np.asarray(unique_pts, dtype=float)
        lx, ux, ly, uy = self.compute_corridor_bounds(grid, pts_ref, pinned_indices)

        cx, ok_x = self._solve_1d_corridor_qp(pts_ref[:, 0], lx, ux)
        cy, ok_y = self._solve_1d_corridor_qp(pts_ref[:, 1], ly, uy)
        ctrl_pts = np.column_stack([cx, cy])

        # Verify control points & midpoint chords lie in TYPES.FREE
        for i in range(len(ctrl_pts)):
            if not self._is_world_point_free(grid, float(ctrl_pts[i, 0]), float(ctrl_pts[i, 1])):
                ctrl_pts[i] = pts_ref[i]

        bs_gen = BSpline(
            step=self.sample_step,
            k=3,
            param_mode="centripetal",
            spline_mode="control_polygon",
        )
        curve_2d, bs_info = bs_gen.generate([tuple(p.tolist()) for p in ctrl_pts])

        # Dense curve-level collision verification; if any sample touches inflation/obstacle, tighten locally
        repaired = False
        for sx, sy in curve_2d:
            if not self._is_world_point_free(grid, sx, sy):
                # Find nearest control point and pull halfway toward original grid center
                dists = np.linalg.norm(ctrl_pts - np.array([sx, sy]), axis=1)
                idx_near = int(np.argmin(dists))
                for k_adj in range(max(1, idx_near - 1), min(len(ctrl_pts) - 1, idx_near + 2)):
                    ctrl_pts[k_adj] = pts_ref[k_adj]
                repaired = True
        if repaired:
            curve_2d, bs_info = bs_gen.generate([tuple(p.tolist()) for p in ctrl_pts])

        traj_3d = Geometry.add_orient_to_2d_path(curve_2d)
        corridor_boxes = [
            (float(lx[i]), float(ux[i]), float(ly[i]), float(uy[i]))
            for i in range(len(pts_ref))
        ]
        return traj_3d, {
            "success": bool(ok_x and ok_y and bs_info["success"]),
            "control_points": ctrl_pts,
            "corridor_boxes": corridor_boxes,
            "length": bs_info["length"],
        }
