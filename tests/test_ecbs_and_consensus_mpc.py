"""Unit tests for Turn-Penalized Focal ECBS, SafeCorridorBSplineOptimizer, CBF-QP, Linear-MPC, and ADG."""
from __future__ import annotations

import math
import numpy as np

from python_motion_planning.common import Grid, TYPES, DiffDriveRobot, ToySimulator
from python_motion_planning.path_planner import CBS, ECBS
from python_motion_planning.traj_optimizer import BSpline, SafeCorridorBSplineOptimizer
from python_motion_planning.controller import (
    MPC,
    ConsensusMPC,
    ORCAController,
    ActionDependencyGraph,
    cbf_qp_filter,
)


def test_ecbs_resolves_head_on_and_intersection_conflicts() -> None:
    """Case 1: 3-agent symmetric cross intersection resolves all vertex and edge conflicts."""
    grid = Grid(bounds=[[0, 15], [0, 15]], resolution=1.0)
    grid.fill_boundary_with_obstacles()
    starts = {"1": (2, 7), "2": (12, 7), "3": (7, 2)}
    goals = {"1": (12, 7), "2": (2, 7), "3": (7, 12)}

    planner = ECBS(grid, starts, goals, w=1.08, turn_penalty=0.45)
    paths, info = planner.plan()

    assert info["success"] is True
    conflicts, first = planner.detect_conflicts(paths)
    assert conflicts == 0
    assert first is None
    for aid in starts:
        assert paths[aid][0] == starts[aid]
        assert paths[aid][-1] == goals[aid]


def test_ecbs_resolves_narrow_corridor_edge_swap_via_passing_bay() -> None:
    """Case 2: Two agents swapping endpoints in a single-cell corridor must use a passing bay."""
    grid = Grid(bounds=[[0, 11], [0, 7]], resolution=1.0)
    grid.fill_boundary_with_obstacles()
    # Block everything except y=3 corridor and a single passing bay at (5, 4)
    for x in range(1, 10):
        for y in range(1, 6):
            if y != 3 and not (x == 5 and y == 4):
                grid.type_map[x, y] = TYPES.OBSTACLE

    starts = {"1": (2, 3), "2": (8, 3)}
    goals = {"1": (8, 3), "2": (2, 3)}

    planner = ECBS(grid, starts, goals, w=1.05, turn_penalty=0.0, min_separation=1.0)
    paths, info = planner.plan()

    assert info["success"] is True
    conflicts, first = planner.detect_conflicts(paths)
    assert conflicts == 0
    assert first is None
    # At least one agent must have entered the passing bay (5, 4)
    visited_bay = any((5, 4) in paths[aid] for aid in ("1", "2"))
    assert visited_bay is True


def test_ecbs_multi_stage_waypoints_visited_in_strict_chronological_order() -> None:
    """Case 3: Two-stage Start -> Pickup -> Delivery tasks must visit Pickup strictly before Delivery."""
    grid = Grid(bounds=[[0, 18], [0, 18]], resolution=1.0)
    grid.fill_boundary_with_obstacles()
    grid.type_map[7:10, 7:10] = TYPES.OBSTACLE

    starts = {"1": (2, 2), "2": (15, 2)}
    pickups = {"1": (2, 14), "2": (15, 14)}
    deliveries = {"1": (14, 14), "2": (3, 14)}
    waypoints = {aid: [pickups[aid]] for aid in starts}

    planner = ECBS(grid, starts, deliveries, waypoints=waypoints, w=1.08, turn_penalty=0.45)
    paths, info = planner.plan()

    assert info["success"] is True
    conflicts, _ = planner.detect_conflicts(paths)
    assert conflicts == 0
    for aid in starts:
        path = paths[aid]
        assert path[0] == starts[aid]
        t_pickup = path.index(pickups[aid])
        t_delivery = len(path) - 1 - path[::-1].index(deliveries[aid])
        assert 0 < t_pickup < t_delivery
        assert path[-1] == deliveries[aid]


def test_ecbs_min_separation_enforces_2_cell_clearance() -> None:
    """Case 4: When min_separation=2.0, Euclidean distance between all pairs at every step >= 2.0."""
    grid = Grid(bounds=[[0, 18], [0, 18]], resolution=1.0)
    grid.fill_boundary_with_obstacles()
    starts = {"1": (3, 8), "2": (14, 8), "3": (8, 3)}
    goals = {"1": (14, 8), "2": (3, 8), "3": (8, 14)}

    planner = ECBS(grid, starts, goals, w=1.08, turn_penalty=0.45, min_separation=2.0)
    paths, info = planner.plan()

    assert info["success"] is True
    max_t = max(len(p) for p in paths.values())
    aids = list(starts.keys())
    for t in range(max_t):
        for i in range(len(aids)):
            for j in range(i + 1, len(aids)):
                pi = paths[aids[i]][min(t, len(paths[aids[i]]) - 1)]
                pj = paths[aids[j]][min(t, len(paths[aids[j]]) - 1)]
                dist = math.hypot(pi[0] - pj[0], pi[1] - pj[1])
                assert dist >= 2.0 - 1e-9


def test_ecbs_unsolvable_scenario_terminates_cleanly() -> None:
    """Case 5: Totally walled-off goal returns success=False without infinite looping."""
    grid = Grid(bounds=[[0, 12], [0, 12]], resolution=1.0)
    grid.fill_boundary_with_obstacles()
    # Enclose goal (9, 9) completely with obstacles
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx != 0 or dy != 0:
                grid.type_map[9 + dx, 9 + dy] = TYPES.OBSTACLE

    starts = {"1": (2, 2)}
    goals = {"1": (9, 9)}
    planner = ECBS(grid, starts, goals, w=1.08, max_high_level_nodes=50)
    paths, info = planner.plan()

    assert info["success"] is False
    assert paths == {}


def test_ecbs_bounded_suboptimality_guarantee() -> None:
    """Case 6: For any w >= 1.0, ECBS solution cost satisfies SOC_ECBS <= w * LB <= w * SOC_CBS."""
    grid = Grid(bounds=[[0, 16], [0, 16]], resolution=1.0)
    grid.fill_boundary_with_obstacles()
    grid.type_map[6:9, 6:9] = TYPES.OBSTACLE
    starts = {"1": (2, 5), "2": (13, 5), "3": (5, 2), "4": (5, 13)}
    goals = {"1": (13, 5), "2": (2, 5), "3": (5, 13), "4": (5, 2)}

    cbs = ECBS(grid, starts, goals, w=1.0, turn_penalty=0.0)
    _, cbs_info = cbs.plan()
    assert cbs_info["success"] is True
    soc_cbs = float(cbs_info["cost"])

    for w in (1.05, 1.2, 1.5):
        ecbs = ECBS(grid, starts, goals, w=w, turn_penalty=0.0)
        _, ecbs_info = ecbs.plan()
        assert ecbs_info["success"] is True
        soc_ecbs = float(ecbs_info["cost"])
        lb = float(ecbs_info["lower_bound"])
        assert soc_ecbs <= w * lb + 1e-6
        assert soc_ecbs <= w * soc_cbs + 1e-6


def test_ecbs_reduces_turns_strictly_compared_to_zero_penalty_cbs() -> None:
    """Case 7: Turn penalty strictly reduces total 90-degree turns in obstacle fields."""
    grid = Grid(bounds=[[0, 20], [0, 15]], resolution=1.0)
    grid.fill_boundary_with_obstacles()
    for ox, oy in [(6, 10), (10, 11), (14, 10), (8, 9), (12, 12)]:
        grid.type_map[ox, oy] = TYPES.OBSTACLE

    starts = {"1": (2, 10), "2": (16, 4)}
    goals = {"1": (16, 10), "2": (2, 4)}

    cbs_no_turn = CBS(grid, starts, goals)
    _, info_zero = cbs_no_turn.plan()

    ecbs_turn = ECBS(grid, starts, goals, w=1.08, turn_penalty=0.6)
    _, info_turn = ecbs_turn.plan()

    assert info_zero["success"] is True
    assert info_turn["success"] is True
    assert info_turn["total_turns"] < info_zero["total_turns"]


def test_cbf_qp_filter_forward_invariance_and_infeasible_fallback() -> None:
    """Case 8: Distributed CBF-QP filter maintains h_ij >= 0 and falls back to 0 on contradiction."""
    dt = 0.05
    d_safe = 1.6
    p1 = np.array([-1.0, 0.0], dtype=float)
    p2 = np.array([1.0, 0.0], dtype=float)
    u1_nom = np.array([2.2, 0.05], dtype=float)
    u2_nom = np.array([-2.2, -0.05], dtype=float)

    for _ in range(60):
        u1_safe, info1 = cbf_qp_filter(
            p_i=p1,
            u_nom=u1_nom,
            neighbors=[(p2, u2_nom)],
            d_safe=d_safe,
            gamma=1.8,
            u_max=2.5,
            weights=[0.5],
        )
        u2_safe, info2 = cbf_qp_filter(
            p_i=p2,
            u_nom=u2_nom,
            neighbors=[(p1, u1_nom)],
            d_safe=d_safe,
            gamma=1.8,
            u_max=2.5,
            weights=[0.5],
        )
        assert info1["status_val"] in (1.0, 2.0)
        assert info2["status_val"] in (1.0, 2.0)
        p1 = p1 + dt * u1_safe
        p2 = p2 + dt * u2_safe
        h_val = float(np.dot(p1 - p2, p1 - p2) - d_safe * d_safe)
        assert h_val >= -1e-2

    # Infeasible sandwich: two overlapping neighbors demanding opposite infinite velocity
    u_fallback, info_infeas = cbf_qp_filter(
        p_i=np.array([0.0, 0.0]),
        u_nom=np.array([1.0, 0.0]),
        neighbors=[
            (np.array([0.1, 0.0]), np.array([-50.0, 0.0])),
            (np.array([-0.1, 0.0]), np.array([50.0, 0.0])),
        ],
        d_safe=3.0,
        gamma=10.0,
        u_max=1.0,
        weights=[0.5, 0.5],
    )
    assert np.allclose(u_fallback, np.zeros(2))


def test_linear_mpc_osqp_closed_loop_convergence() -> None:
    """Case 9: Single-agent Linearized Error-State MPC solves via OSQP and converges to goal."""
    grid = Grid(bounds=[[0, 25], [0, 25]], resolution=1.0)
    grid.fill_boundary_with_obstacles()
    _ = grid.esdf

    env = ToySimulator(dim=2, dt=0.1, obstacle_grid=grid, robot_collisions=False)
    robot = DiffDriveRobot(
        dim=2,
        radius=0.45,
        pose=np.array([3.5, 5.5, 0.0]),
        vel=np.zeros(3),
        max_lin_speed=2.4,
        max_ang_speed=3.14,
        action_min=np.array([-3.5, 0.0, -6.28]),
        action_max=np.array([3.5, 0.0, 6.28]),
    )
    env.add_robot("1", robot)
    # L-shaped path with a 90-degree turn
    path = [(float(x), 5.5) for x in np.linspace(3.5, 14.5, 23)] + [
        (14.5, float(y)) for y in np.linspace(6.0, 16.5, 22)
    ]
    obs_space, act_space = env.build_robot_spaces(robot)
    mpc = MPC(
        obs_space,
        act_space,
        env.dt,
        path,
        robot_model=robot,
        obstacle_grid=grid,
        max_lin_speed=2.2,
        max_ang_speed=3.14,
        goal_dist_tol=0.6,
    )

    env.reset()
    for _ in range(140):
        action, _ = mpc.get_action(robot.get_observation(env))
        env.step({"1": action})

    assert len(mpc.solve_times_us) > 0
    assert mpc.last_solve_time_us > 0.0
    assert float(np.linalg.norm(robot.pos[:2] - np.array([14.5, 16.5]))) < 0.75
    assert float(np.linalg.norm(robot.lin_vel[:2])) < 0.25


def test_consensus_mpc_and_orca_track_intersection_without_collision() -> None:
    """Case 10: ConsensusMPC (ADG + Linear-MPC + CBF-QP) and ORCA avoid inter-robot collisions."""
    grid = Grid(bounds=[[0, 25], [0, 25]], resolution=1.0)
    grid.fill_boundary_with_obstacles()
    _ = grid.esdf

    env = ToySimulator(dim=2, dt=0.1, obstacle_grid=grid, robot_collisions=True)
    r1 = DiffDriveRobot(
        dim=2,
        radius=0.8,
        pose=np.array([4.5, 12.5, 0.0]),
        vel=np.zeros(3),
        max_lin_speed=2.5,
        max_ang_speed=3.14,
        action_min=np.array([-3.0, 0.0, -6.28]),
        action_max=np.array([3.0, 0.0, 6.28]),
    )
    r2 = DiffDriveRobot(
        dim=2,
        radius=0.8,
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
        obs1,
        act1,
        env.dt,
        path1,
        robot_model=r1,
        obstacle_grid=grid,
        max_lin_speed=2.2,
        max_ang_speed=3.14,
        self_id="1",
        peer_robots=robots,
        priority=2,
        priorities=priorities,
    )
    c2 = ConsensusMPC(
        obs2,
        act2,
        env.dt,
        path2,
        robot_model=r2,
        obstacle_grid=grid,
        max_lin_speed=2.2,
        max_ang_speed=3.14,
        self_id="2",
        peer_robots=robots,
        priority=1,
        priorities=priorities,
    )

    env.reset()
    min_dist = float("inf")
    for _ in range(110):
        a1, _ = c1.get_action(r1.get_observation(env))
        a2, _ = c2.get_action(r2.get_observation(env))
        d = float(np.linalg.norm(r1.pos[:2] - r2.pos[:2]))
        min_dist = min(min_dist, d)
        env.step({"1": a1, "2": a2})

    assert min_dist >= 1.6
    assert np.linalg.norm(r1.pos[:2] - np.array([20.5, 12.5])) < 2.0
    assert np.linalg.norm(r2.pos[:2] - np.array([12.5, 20.5])) < 2.5


def test_bspline_and_safe_corridor_optimizer_respect_free_space() -> None:
    """Case 11: BSpline approximation has zero collinear error and SafeCorridorBSplineOptimizer stays in FREE."""
    collinear_pts = [(float(x), 14.5) for x in range(10, 45)]
    for mode in ("approximation", "interpolation", "control_polygon"):
        bs = BSpline(step=0.01, k=3, param_mode="centripetal", spline_mode=mode)
        curve, info = bs.generate(collinear_pts)
        assert info["success"] is True
        arr = np.array(curve)
        assert np.allclose(arr[0], [10.0, 14.5], atol=1e-4)
        assert np.allclose(arr[-1], [44.0, 14.5], atol=1e-4)
        assert float(np.max(np.abs(arr[:, 1] - 14.5))) < 1e-6

    grid = Grid(bounds=[[0, 20], [0, 20]], resolution=1.0)
    grid.fill_boundary_with_obstacles()
    grid.type_map[6:14, 6:14] = TYPES.OBSTACLE
    grid.inflate_obstacles(radius=1)

    # Path hugging the inflated shelf corner
    grid_path = [(4, y) for y in range(3, 16)] + [(x, 15) for x in range(5, 16)]
    opt = SafeCorridorBSplineOptimizer(max_corridor_half_width=0.42, sample_step=0.01)
    traj_3d, opt_info = opt.optimize(grid, grid_path, mandatory_waypoints=[(4, 3), (4, 15), (15, 15)])

    assert opt_info["success"] is True
    for x, y, _ in traj_3d:
        gx, gy = grid.world_to_map((x, y))
        assert grid.within_bounds((gx, gy))
        assert int(grid.type_map[gx, gy]) == TYPES.FREE


def test_action_dependency_graph_delays_successor_until_predecessor_clears() -> None:
    """Case 12: ActionDependencyGraph (ADG) blocks agent 2 before intersection when agent 1 is delayed."""
    grid = Grid(bounds=[[0, 20], [0, 20]], resolution=1.0)
    grid.fill_boundary_with_obstacles()
    # Agent 1 crosses (10, 10) at t=3; Agent 2 crosses (10, 10) at t=6
    ecbs_paths = {
        "1": [(7, 10), (8, 10), (9, 10), (10, 10), (11, 10), (12, 10), (13, 10)],
        "2": [(10, 4), (10, 5), (10, 6), (10, 7), (10, 8), (10, 9), (10, 10), (10, 11), (10, 12)],
    }
    world_paths = {
        aid: [(x + 0.5, y + 0.5, 0.0) for (x, y) in p]
        for aid, p in ecbs_paths.items()
    }
    adg = ActionDependencyGraph(grid, ecbs_paths, world_paths)
    assert len(adg.dependencies_by_succ["2"]) > 0

    # If Agent 1 is frozen at s=0.5 (before clearing intersection), Agent 2's s_cap is held before intersection
    adg.update_progress("1", 0.5)
    adg.update_progress("2", 4.0)
    cap_held = adg.max_allowed_s("2")
    assert cap_held < 6.0

    # Once Agent 1 advances past intersection (s=6.0), Agent 2 is released to full trajectory length
    adg.update_progress("1", 6.0)
    cap_free = adg.max_allowed_s("2")
    assert cap_free >= 7.9
