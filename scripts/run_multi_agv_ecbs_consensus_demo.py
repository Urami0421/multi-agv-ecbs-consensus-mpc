"""
Full-Stack Multi-AGV Planning, Safe-Corridor B-Spline Optimization, and Control Benchmark Orchestrator.

Delegates to the modular scripts in `scripts/`:
  - `scripts/benchmark_mapf.py`: 4-Group MAPF Ablation (10 repeats), (w, turn_penalty) sweep,
    and MovingAI `.map`/`.scen` evaluation (`results/mapf_*.csv`, `results/movingai_benchmark.csv`).
  - `scripts/benchmark_control.py`: 6-Method continuous controller ablation and 5-seed random
    delay/perturbation robustness evaluation (`results/control_ablation.csv`,
    `results/perturbation_robustness.csv`).
  - `scripts/render_2d.py`: 2D Matplotlib simulation renderer (`assets/multi_agv_consensus_mpc_2d.gif`).
  - `scripts/render_3d.py`: 3D PyVista/VTK off-screen warehouse renderer (`assets/multi_agv_consensus_mpc_3d.gif`).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))
if str(ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts"))

from benchmark_mapf import (
    load_warehouse_config,
    build_warehouse_grid,
    get_rack_blocks,
    get_hazard_blocks,
    get_fleet_mission,
    optimize_corridor_bspline_paths,
    run_planning_ablation,
    run_parameter_sweep,
    run_synthetic_grid_benchmark,
    plot_ecbs_vs_cbs_comparison,
)
from benchmark_control import (
    simulate_controller,
    run_control_ablation,
    run_perturbation_experiment,
    plot_ablation_dashboard,
)
from render_2d import render_smooth_consensus_mpc_2d_gif
from render_3d import render_smooth_consensus_mpc_3d_gif


def densify_and_smooth_world_path(grid, grid_path):
    """Backward-compatible helper wrapping `SafeCorridorBSplineOptimizer`."""
    world_paths, _ = optimize_corridor_bspline_paths(grid, {"1": grid_path})
    return world_paths["1"]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Run Multi-AGV Planning & Control Benchmarks and Visualizations.")
    parser.add_argument("--repeats", type=int, default=10, help="Number of timing repetitions for MAPF ablation.")
    parser.add_argument("--skip-gifs", action="store_true", help="Skip 2D/3D GIF rendering (run benchmarks & PNG plots only).")
    parser.add_argument("--skip-3d", action="store_true", help="Skip 3D PyVista GIF rendering.")
    args = parser.parse_args(argv)

    cfg = load_warehouse_config()
    grid = build_warehouse_grid(inflate_radius=1, cfg=cfg)
    _, waypoints, goals, priorities, _ = get_fleet_mission(cfg)

    print(
        f"=== [1/4] Running 4-Group MAPF Ablation ({args.repeats} repeats), Parameter Sweep & Synthetic Grid Benchmark ===",
        flush=True,
    )
    ecbs_paths, cbs_paths, planning_summary = run_planning_ablation(repeats=args.repeats)
    print(json.dumps(planning_summary, indent=2), flush=True)
    run_parameter_sweep(repeats=3)
    run_synthetic_grid_benchmark()

    world_paths, _ = optimize_corridor_bspline_paths(grid, ecbs_paths, waypoints, cfg)

    print(
        "\n=== [2/4] Running 10-Variant Continuous Controller Ablation & 20-Seed Perturbation Suite ===",
        flush=True,
    )
    steps = int(cfg["simulation"]["steps"])
    control_results = run_control_ablation(world_paths, priorities, ecbs_paths=ecbs_paths, steps=steps)
    for ctrl_name, res in control_results.items():
        print(
            f"  {ctrl_name:20s} | collisions={res['collision_events']:2d} | "
            f"min_dist={res['min_inter_robot_dist']:5.2f}m | "
            f"2stage_succ={res['two_stage_success_rate']*100:5.1f}% | "
            f"makespan={res['makespan_s']:5.1f}s | "
            f"nav_err={res['mean_nav_error']:5.2f}m | "
            f"osqp_c={res['osqp_mean_us']:5.1f}us | "
            f"wall={res['wall_mean_us']:6.1f}us",
            flush=True,
        )
    pert_rows = run_perturbation_experiment(world_paths, priorities, ecbs_paths, num_trials=20)
    print("  Perturbation robustness summary:", json.dumps(pert_rows, indent=2), flush=True)

    assets_dir = ROOT / "assets"
    print("\n=== [3/4] Rendering Quantitative Comparison & Ablation Figures ===", flush=True)
    plot_ecbs_vs_cbs_comparison(
        cbs_paths, ecbs_paths, planning_summary, assets_dir / "ecbs_vs_cbs_warehouse.png"
    )
    plot_ablation_dashboard(
        planning_summary, control_results, assets_dir / "multi_agv_ablation_dashboard.png"
    )

    if not args.skip_gifs:
        print("\n=== [4/4] Rendering 2D & 3D Warehouse Animations ===", flush=True)
        render_smooth_consensus_mpc_2d_gif(
            world_paths,
            waypoints,
            goals,
            priorities,
            assets_dir / "multi_agv_consensus_mpc_2d.gif",
            ecbs_paths=ecbs_paths,
            steps=steps,
        )
        if not args.skip_3d:
            render_smooth_consensus_mpc_3d_gif(
                world_paths,
                waypoints,
                goals,
                priorities,
                assets_dir / "multi_agv_consensus_mpc_3d.gif",
                ecbs_paths=ecbs_paths,
                steps=steps,
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
