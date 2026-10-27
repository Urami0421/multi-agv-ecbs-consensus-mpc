"""Unit tests for the ECBS space-time planner and Consensus-CBF-MPC controller."""
from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from python_motion_planning.common import Grid, TYPES, DiffDriveRobot, ToySimulator
from python_motion_planning.path_planner import CBS, ECBS
from python_motion_planning.traj_optimizer import BSpline
from python_motion_planning.controller import MPC, ConsensusMPC


def test_ecbs_resolves_head_on_and_intersection_conflicts() -> None:
    grid = Grid(bounds=[[0, 15], [0, 15]], resolution=1.0)
    grid.fill_boundary_with_obstacles()
    starts = {"1": (2, 7), "2": (12, 7), "3": (7, 2)}
    goals = {"1": (12, 7), "2": (2, 7), "3": (7, 12)}

    planner = ECBS(grid, starts, goals, w=1.08, turn_penalty=0.35)
    paths, info = planner.plan()

    assert info["success"] is True
    conflicts, first = planner._detect_conflicts(paths)
    assert conflicts == 0
    assert first is None
    for aid in starts:
        assert paths[aid][0] == starts[aid]
        assert paths[aid][-1] == goals[aid]


def test_ecbs_reduces_turns_compared_to_zero_penalty_cbs() -> None:
    grid = Grid(bounds=[[0, 20], [0, 20]], resolution=1.0)
    grid.fill_boundary_with_obstacles()
    grid[8:12, 8:12] = TYPES.OBSTACLE
    starts = {"1": (3, 3), "2": (16, 16)}
    goals = {"1": (16, 16), "2": (3, 3)}

    cbs = CBS(grid, starts, goals)
    _, cbs_info = cbs.plan()

    ecbs = ECBS(grid, starts, goals, w=1.08, turn_penalty=0.5)
    _, ecbs_info = ecbs.plan()

    assert cbs_info["success"] is True
    assert ecbs_info["success"] is True
    assert ecbs_info["total_turns"] <= cbs_info["total_turns"]


def test_consensus_mpc_tracks_path_without_inter_robot_collision() -> None:
    grid = Grid(bounds=[[0, 25], [0, 25]], resolution=1.0)
    grid.fill_boundary_with_obstacles()
    _ = grid.esdf

    env = ToySimulator(dim=2, dt=0.1, obstacle_grid=grid, robot_collisions=True)
    r1 = DiffDriveRobot(
        dim=2,
        radius=0.9,
        pose=np.array([4.5, 12.5, 0.0]),
        vel=np.zeros(3),
        max_lin_speed=2.5,
        max_ang_speed=3.14,
        action_min=np.array([-3.0, 0.0, -6.28]),
        action_max=np.array([3.0, 0.0, 6.28]),
    )
    r2 = DiffDriveRobot(
        dim=2,
        radius=0.9,
        pose=np.array([12.5, 4.5, 1.57]),
        vel=np.zeros(3),
        max_lin_speed=2.5,
        max_ang_speed=3.14,
        action_min=np.array([-3.0, 0.0, -6.28]),
        action_max=np.array([3.0, 0.0, 6.28]),
    )
    robots = {"1": r1, "2": r2}
    for rid, rob in robots.items():
        env.add_robot(rid, rob)

    path1 = [(float(x), 12.5) for x in np.linspace(4.5, 20.5, 25)]
    path2 = [(12.5, float(y)) for y in np.linspace(4.5, 20.5, 25)]
    priorities = {"1": 2, "2": 1}

    obs1, act1 = env.build_robot_spaces(r1)
    obs2, act2 = env.build_robot_spaces(r2)
    c1 = ConsensusMPC(
        obs1, act1, env.dt, path1, robot_model=r1, obstacle_grid=grid,
        max_lin_speed=2.2, max_ang_speed=3.14, self_id="1", peer_robots=robots,
        priority=2, priorities=priorities,
    )
    c2 = ConsensusMPC(
        obs2, act2, env.dt, path2, robot_model=r2, obstacle_grid=grid,
        max_lin_speed=2.2, max_ang_speed=3.14, self_id="2", peer_robots=robots,
        priority=1, priorities=priorities,
    )

    env.reset()
    min_dist = float("inf")
    for _ in range(110):
        a1, _ = c1.get_action(r1.get_observation(env))
        a2, _ = c2.get_action(r2.get_observation(env))
        d = float(np.linalg.norm(r1.pos[:2] - r2.pos[:2]))
        min_dist = min(min_dist, d)
        env.step({"1": a1, "2": a2})

    assert min_dist >= 1.8
    assert np.linalg.norm(r1.pos[:2] - np.array([20.5, 12.5])) < 2.0
    assert np.linalg.norm(r2.pos[:2] - np.array([12.5, 20.5])) < 2.5


def test_bspline_clamped_approximation_has_zero_collinear_oscillation() -> None:
    collinear_pts = [(float(x), 14.5) for x in range(10, 45)]
    bs = BSpline(step=0.01, k=3, param_mode="centripetal", spline_mode="approximation")
    curve, info = bs.generate(collinear_pts)
    assert info["success"] is True
    arr = np.array(curve)
    assert np.allclose(arr[0], [10.0, 14.5], atol=1e-4)
    assert np.allclose(arr[-1], [44.0, 14.5], atol=1e-4)
    assert float(np.max(np.abs(arr[:, 1] - 14.5))) < 1e-6

