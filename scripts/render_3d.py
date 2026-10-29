"""
3D PyVista/VTK Off-Screen Warehouse Animation Renderer (`assets/multi_agv_consensus_mpc_3d.gif`).
"""
from __future__ import annotations

from pathlib import Path
import random
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))
if str(ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts"))

from python_motion_planning.common import DiffDriveRobot, ToySimulator, Visualizer3D
from python_motion_planning.controller import ActionDependencyGraph, ConsensusMPC
from benchmark_mapf import (
    load_warehouse_config,
    build_warehouse_grid,
    get_rack_blocks,
    get_fleet_mission,
    optimize_corridor_bspline_paths,
    run_planning_ablation,
)


def render_smooth_consensus_mpc_3d_gif(
    world_paths: Dict[str, List[Tuple[float, float, float]]],
    waypoints: Dict[str, List[Tuple[int, int]]],
    goals: Dict[str, Tuple[int, int]],
    priorities: Dict[str, int],
    output_gif: Path,
    ecbs_paths: Optional[Dict[str, List[Tuple[int, int]]]] = None,
    steps: int = 305,
    robot_radius: float = 0.44,
) -> None:
    """
    Render the 3D PyVista/VTK warehouse simulation with 48 AWS RoboMaker 3D shelves (`ShelfD`, `ShelfE`),
    12 pallet stacks (`ClutteringC`), and 8 Neobotix `MP-400` 3D AGV meshes.
    """
    random.seed(0)
    np.random.seed(0)
    cfg = load_warehouse_config()
    grid = build_warehouse_grid(inflate_radius=1, cfg=cfg)
    _ = grid.esdf
    _, _, _, _, agv_colors = get_fleet_mission(cfg)

    env = ToySimulator(
        dim=2,
        dt=float(cfg["simulation"]["dt"]),
        obstacle_grid=grid,
        friction=0.015,
        restitution=0.3,
        noise=0.001,
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

    adg = ActionDependencyGraph(grid, ecbs_paths, world_paths) if ecbs_paths is not None else None
    controllers = {}
    for rid, robot in robots.items():
        obs_space, act_space = env.build_robot_spaces(robot)
        controllers[rid] = ConsensusMPC(
            obs_space,
            act_space,
            env.dt,
            world_paths[rid],
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
            adg=adg,
            sensing_radius=9.5,
            safe_margin=0.45,
        )

    env.reset()
    pickup_world = {rid: grid.map_to_world(waypoints[rid][0]) for rid in robots}
    delivery_world = {rid: grid.map_to_world(goals[rid]) for rid in robots}

    vis3d = Visualizer3D(window_size=(800, 560), off_screen=True, show_axes=False)
    vis3d.render_warehouse_3d_simulator(
        env=env,
        controllers=controllers,
        world_paths=world_paths,
        pickup_world=pickup_world,
        delivery_world=delivery_world,
        rack_blocks=get_rack_blocks(cfg),
        meshes_dir=str(ROOT / "assets" / "models"),
        output_gif=str(output_gif),
        steps=steps,
        frame_stride=4,
        fps=15,
    )
    vis3d.close()


def main() -> int:
    cfg = load_warehouse_config()
    grid = build_warehouse_grid(inflate_radius=1, cfg=cfg)
    _, waypoints, goals, priorities, _ = get_fleet_mission(cfg)
    ecbs_paths, _, _ = run_planning_ablation(repeats=1, write_csv=False)
    world_paths, _ = optimize_corridor_bspline_paths(grid, ecbs_paths, waypoints, cfg)
    out_gif = ROOT / "assets" / "multi_agv_consensus_mpc_3d.gif"
    render_smooth_consensus_mpc_3d_gif(
        world_paths,
        waypoints,
        goals,
        priorities,
        out_gif,
        ecbs_paths=ecbs_paths,
        steps=int(cfg["simulation"]["steps"]),
    )
    print(f"Saved {out_gif}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
