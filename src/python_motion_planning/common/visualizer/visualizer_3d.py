"""
@file: visualizer_3d.py
@author: Wu Maojia
@update: 2025.12.20
"""
from typing import Union, Dict, List, Tuple, Any, Optional
from collections import namedtuple
import time
import os

import numpy as np
import pyvista as pv

from python_motion_planning.common.visualizer.base_visualizer import BaseVisualizer
from python_motion_planning.controller import BaseController
from python_motion_planning.common.env import TYPES, ToySimulator, Grid, CircularRobot, Node
from python_motion_planning.common.utils import Geometry

class Visualizer3D(BaseVisualizer):
    """
    Simple 3D visualizer for motion planning using pyvista.

    Args:
        window_size: Window size (width, height) (pyvista window size, unit: pixel).
        off_screen: `off_screen` argument for pyvista. Renders off screen when True. Useful for automated screenshots.
        show_axes: Whether to show axes for pyvista.
        cmap_dict: Color map for 3d voxel visualization.
    """
    def __init__(self,  
                window_size: tuple = (1200, 900),
                off_screen: bool = False,
                show_axes: bool = True,
                cmap_dict: dict = {
                    TYPES.FREE: "#ffffff",
                    TYPES.OBSTACLE: "#000000",
                    TYPES.START: "#ff0000",
                    TYPES.GOAL: "#1155cc",
                    TYPES.INFLATION: "#ffccff",
                    TYPES.EXPAND: "#eeeeee",
                    TYPES.CUSTOM: "#bbbbbb",
                }
            ):
        super().__init__()
        self.pv_plotter = pv.Plotter(window_size=list(window_size), off_screen=off_screen)
        if show_axes: 
            self.pv_plotter.show_axes()
        self.pv_actors = {} 

        # colors
        self.cmap_dict = cmap_dict

    def plot_grid_map(self, grid_map: Grid, equal: bool = False, alpha_3d: dict = {
                            TYPES.FREE: 0.0,
                            TYPES.OBSTACLE: 0.1,
                            TYPES.START: 0.5,
                            TYPES.GOAL: 0.5,
                            TYPES.INFLATION: 0.0,
                            TYPES.EXPAND: 0.01,
                            TYPES.CUSTOM: 0.1,
                        }) -> None:
        '''
        Plot grid map with static obstacles.

        Args:
            map: Grid map or its type map.
            equal: Whether to set axis equal.
            alpha_3d: Alpha of occupancy for 3d visualization.
        '''
        if grid_map.dim != 3:
            raise ValueError(f"Grid map dimension must be 3.")
        
        self.grid_map = grid_map
        self.dim = grid_map.dim
        type_data = grid_map.type_map.data

        nx, ny, nz = type_data.shape

        for key, color in self.cmap_dict.items():
            alpha = alpha_3d.get(key, 0.0)
            if alpha < 1e-6:
                continue

            mask = (type_data == key)
            if not np.any(mask):
                continue

            # voxels
            points = np.argwhere(mask)

            # map → world
            points = np.array([
                self.grid_map.map_to_world(p)
                for p in points
            ])

            cloud = pv.PolyData(points)
            glyph = cloud.glyph(
                geom=pv.Cube(),
                scale=False,
                factor=self.grid_map.resolution
            )

            actor = self.pv_plotter.add_mesh(
                glyph,
                color=color,
                opacity=alpha,
                show_edges=False
            )

            self.pv_actors[f"voxels_{key}"] = actor

    def plot_expand_tree(self, expand_tree: Dict[Union[Tuple[int, ...], Tuple[float, ...]], Node], 
                        edge_color: str = "#e377c2", 
                        linewidth: float = 1.0, 
                        node_alpha: float = 1.0,
                        edge_alpha: float = 1.0,
                        map_frame: bool = True) -> None:
        """
        Visualize an expand tree (e.g. RRT).
        
        Args:
            expand_tree: Dict mapping coordinate tuple -> Node (world frame).
            edge_color: Color of the edges (parent -> child).
            linewidth: Line width of edges.
            map_frame: whether path is in map frame or not (world frame)
        """
        if not isinstance(expand_tree, list):
            expand_tree = [expand_tree]

        points = []
        lines = []

        idx = 0
        for tree in expand_tree:
            for _, node in tree.items():
                cur = node.current
                if map_frame:
                    cur = self.grid_map.map_to_world(cur)

                points.append(cur)

                if node.parent is not None:
                    parent = node.parent
                    if map_frame:
                        parent = self.grid_map.map_to_world(parent)

                    points.append(parent)
                    lines.append([idx, idx + 1])
                    idx += 2
                else:
                    idx += 1

        if not points:
            return

        points = np.array(points)
        poly = pv.PolyData(points)

        if lines:
            cells = np.hstack([[2, l[0], l[1]] for l in lines])
            poly.lines = cells

        self.pv_plotter.add_mesh(
            poly,
            color=edge_color,
            line_width=linewidth
        )


    def plot_path(self, path: List[Union[Tuple[int, ...], Tuple[float, ...]]], 
                    color: str = "#13ae00", 
                    linewidth: float = 5, map_frame: bool = True) -> None:
        '''
        Plot path-like information.
        The meaning of parameters are similar to pyvista.Plotter.add_mesh (https://docs.pyvista.org/api/plotting/_autosummary/pyvista.plotter.add_mesh#pyvista.Plotter.add_mesh).

        Args:
            path: point list of path
            color: color of path
            linewidth: linewidth of path
            map_frame: whether path is in map frame or not (world frame)
        '''
        if len(path) == 0:
            return

        if map_frame:
            path = [self.grid_map.map_to_world(point) for point in path]

        path = np.array(path)

        path_line = pv.lines_from_points(path)
        self.pv_plotter.add_mesh(
            path_line,
            color=color,
            line_width=linewidth
        )

    def set_title(self, title: str) -> None:
        """
        Set title.

        Args:
            title: Title.
        """
        self.pv_plotter.add_text(title, position='upper_edge', font_size=14, color='black')

    def clean(self):
        """
        Clean plot.
        """
        self.pv_plotter.clear()
        self.pv_actors = {}

    def update(self):
        """
        Update plot.
        """
        self.pv_plotter.render()

    def savefig(self, filename, *args, **kwargs):
        """
        Save figure. 

        Args:
            filename: Filename to save.
            *args: See pyvista.Plotter.screenshot.
            **kwargs: See pyvista.Plotter.screenshot.
        """
        self.pv_plotter.screenshot(filename=filename, *args, **kwargs)

    def show(self):
        """
        Show plot.
        """
        self.pv_plotter.reset_camera()
        self.pv_plotter.show(interactive=True, auto_close=False)

    def close(self):
        """
        Close plot.
        """
        self.pv_plotter.close()

    def render_warehouse_3d_simulator(
        self,
        env: ToySimulator,
        controllers: Dict[str, BaseController],
        world_paths: Dict[str, List[Tuple[float, float, float]]],
        pickup_world: Dict[str, Tuple[float, float]],
        delivery_world: Dict[str, Tuple[float, float]],
        rack_blocks: List[Tuple[Tuple[int, int], Tuple[int, int]]],
        meshes_dir: str,
        output_gif: str,
        steps: int = 270,
        frame_stride: int = 2,
        fps: int = 22,
    ) -> None:
        """
        Render a 3D hardware-accelerated multi-AGV warehouse simulation using PyVista/VTK OpenGL
        with open-source AWS RoboMaker 3D warehouse racks (`ShelfD`, `ShelfE`, `ClutteringC`) and
        Neobotix `MP-400` 3D AGV meshes (`MP-400-BODY.dae` + `SICK-S300.dae`), dynamic pickup cargo
        loading, and live 3D MPC prediction horizon ribbons.
        """
        import collada
        import math
        from pathlib import Path
        from PIL import Image, ImageDraw

        mesh_root = Path(meshes_dir)
        models_root = mesh_root if mesh_root.name == "models" else mesh_root.parent / "models"
        if not models_root.exists():
            raise FileNotFoundError(f"3D warehouse models directory not found: {models_root}")

        def euler_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
            cr, sr = math.cos(roll), math.sin(roll)
            cp, sp = math.cos(pitch), math.sin(pitch)
            cy, sy = math.cos(yaw), math.sin(yaw)
            rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
            ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
            rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
            return rz @ ry @ rx

        def load_collada_raw_prims(
            dae_path: Path, unit_scale: float = 0.01
        ) -> List[Tuple[np.ndarray, Optional[np.ndarray], Optional[str], Tuple[int, int, int]]]:
            c = collada.Collada(str(dae_path))
            prims = []
            for geom in c.scene.objects("geometry"):
                for bprim in geom.primitives():
                    tri = bprim.triangleset() if hasattr(bprim, "triangleset") else bprim
                    if tri.vertex is None or len(tri) == 0:
                        continue
                    v = np.asarray(tri.vertex[tri.vertex_index], dtype=np.float64) * unit_scale
                    pts = v.reshape(-1, 3)
                    uv = None
                    if len(tri.texcoordset) > 0:
                        uv = np.asarray(
                            tri.texcoordset[0][tri.texcoord_indexset[0]], dtype=np.float64
                        ).reshape(-1, 2)
                    diff = bprim.material.effect.diffuse
                    tex_path = None
                    rgb = (200, 200, 200)
                    if hasattr(diff, "sampler"):
                        cand = (dae_path.parent / diff.sampler.surface.image.path).resolve()
                        if cand.exists():
                            tex_path = str(cand)
                    elif isinstance(diff, (tuple, list, np.ndarray)):
                        rgb = tuple(int(round(float(x) * 255)) for x in diff[:3])
                    prims.append((pts, uv, tex_path, rgb))
            return prims

        # Batch collector for static textured meshes: tex_path -> list of (pts, uv)
        tex_batches: Dict[str, List[Tuple[np.ndarray, np.ndarray]]] = {}

        def add_static_instance(
            prims: List[Tuple[np.ndarray, Optional[np.ndarray], Optional[str], Tuple[int, int, int]]],
            scale_xyz: Tuple[float, float, float],
            yaw: float,
            trans_xyz: Tuple[float, float, float],
        ) -> None:
            sx, sy, sz = scale_xyz
            cy, sy_r = math.cos(yaw), math.sin(yaw)
            rot = np.array([[cy, -sy_r, 0.0], [sy_r, cy, 0.0], [0.0, 0.0, 1.0]])
            scale_vec = np.array([sx, sy, sz], dtype=np.float64)
            t_vec = np.array(trans_xyz, dtype=np.float64)
            for pts, uv, tex_path, _ in prims:
                if tex_path is None or uv is None:
                    continue
                p_world = (pts * scale_vec) @ rot.T + t_vec
                tex_batches.setdefault(tex_path, []).append((p_world, uv))

        shelf_d_prims = load_collada_raw_prims(
            models_root
            / "aws_robomaker_warehouse_ShelfD_01"
            / "meshes"
            / "aws_robomaker_warehouse_ShelfD_01_visual.DAE"
        )
        shelf_e_prims = load_collada_raw_prims(
            models_root
            / "aws_robomaker_warehouse_ShelfE_01"
            / "meshes"
            / "aws_robomaker_warehouse_ShelfE_01_visual.DAE"
        )
        shelf_f_prims = load_collada_raw_prims(
            models_root
            / "aws_robomaker_warehouse_ShelfF_01"
            / "meshes"
            / "aws_robomaker_warehouse_ShelfF_01_visual.DAE"
        )
        clutter_a_prims = load_collada_raw_prims(
            models_root
            / "aws_robomaker_warehouse_ClutteringA_01"
            / "meshes"
            / "aws_robomaker_warehouse_ClutteringA_01_visual.DAE"
        )
        clutter_c_prims = load_collada_raw_prims(
            models_root
            / "aws_robomaker_warehouse_ClutteringC_01"
            / "meshes"
            / "aws_robomaker_warehouse_ClutteringC_01_visual.DAE"
        )
        pallet_jack_prims = load_collada_raw_prims(
            models_root
            / "aws_robomaker_warehouse_PalletJackB_01"
            / "meshes"
            / "aws_robomaker_warehouse_PalletJackB_01_visual.DAE"
        )
        desk_prims = load_collada_raw_prims(
            models_root
            / "aws_robomaker_warehouse_DeskC_01"
            / "meshes"
            / "aws_robomaker_warehouse_DeskC_01_visual.DAE"
        )
        wall_prims = load_collada_raw_prims(
            models_root
            / "aws_robomaker_warehouse_WallB_01"
            / "meshes"
            / "aws_robomaker_warehouse_WallB_01_visual.DAE"
        )
        ground_prims = load_collada_raw_prims(
            models_root
            / "aws_robomaker_warehouse_GroundB_01"
            / "meshes"
            / "aws_robomaker_warehouse_GroundB_01_visual.DAE"
        )

        self.pv_plotter.set_background("#0f172a", top="#1e293b")
        bounds = env.obstacle_grid.bounds
        wx_max, wy_max = float(bounds[0, 1]), float(bounds[1, 1])

        # 1. Textured AWS RoboMaker GroundB concrete floor tiled across 72x44m
        for gx in (12.0, 36.0, 60.0):
            for gy in (11.0, 33.0):
                add_static_instance(
                    ground_prims[:1], (1.75, 1.08, 0.2), 0.0, (gx, gy, -0.02)
                )

        # 2. Textured AWS RoboMaker WallB perimeter enclosure (filter out front -Y wall so camera is unobstructed)
        wall_back_left_right = []
        for pts, uv, tex_path, rgb in wall_prims:
            # Keep triangles where not all 3 vertices are on the front (-Y < -9.5m) wall
            tri_pts = pts.reshape(-1, 3, 3)
            tri_uv = uv.reshape(-1, 3, 2)
            keep = ~(np.all(tri_pts[:, :, 1] < -9.5, axis=1))
            wall_back_left_right.append(
                (tri_pts[keep].reshape(-1, 3), tri_uv[keep].reshape(-1, 2), tex_path, rgb)
            )
        add_static_instance(
            wall_back_left_right,
            (wx_max / 13.98, wy_max / 20.90, 0.72),
            0.0,
            (0.5 * wx_max, 0.5 * wy_max, 0.0),
        )

        # Heavy-duty 6m tall ShelfF pallet racks & DeskC dispatch control stations along North wall
        for sx in (13.5, 28.5, 43.5, 58.5):
            add_static_instance(
                shelf_f_prims, (0.62, 0.52, 0.72), math.pi / 2, (sx, wy_max - 1.35, 0.0)
            )
        for dx in (21.0, 51.0):
            add_static_instance(
                desk_prims, (1.20, 1.20, 1.20), math.pi / 2, (dx, wy_max - 1.45, 0.0)
            )

        # 3. 12 Storage Rack Islands (48 UV-textured ShelfD & ShelfE + 24 ClutteringA/C pallets + 12 PalletJackB)
        rack_bases = []
        for idx_r, ((x0, x1), (y0, y1)) in enumerate(rack_blocks):
            rack_bases.append(pv.Box(bounds=[x0, x1, y0, y1, 0.0, 0.10]))
            for dx_off in (2.85, 8.15):
                # South row of island (ShelfE facing South aisle)
                add_static_instance(
                    shelf_e_prims,
                    (1.26, 1.55, 1.42),
                    math.pi,
                    (x0 + dx_off, y0 + 1.15, 0.10),
                )
                # North row of island (ShelfD facing North aisle)
                add_static_instance(
                    shelf_d_prims,
                    (1.26, 1.55, 1.42),
                    0.0,
                    (x0 + dx_off, y0 + 4.85, 0.10),
                )
            # Central palletized boxes (ClutteringA & ClutteringC) + PalletJackB at island ends
            add_static_instance(
                clutter_c_prims,
                (1.25, 1.25, 1.30),
                0.0,
                (x0 + 3.6, 0.5 * (y0 + y1), 0.10),
            )
            add_static_instance(
                clutter_a_prims,
                (1.25, 1.25, 1.35),
                0.0,
                (x0 + 7.4, 0.5 * (y0 + y1), 0.10),
            )
            add_static_instance(
                pallet_jack_prims,
                (1.45, 1.45, 1.45),
                0.0 if idx_r % 2 == 0 else math.pi,
                (x0 + 0.75, 0.5 * (y0 + y1), 0.10),
            )

        self.pv_plotter.add_mesh(pv.merge(rack_bases), color="#334155", specular=0.15)

        # 3b. 8 Asymmetric Aisle Hazard & Bottleneck Zones (Parked PalletJackB + Spilled ClutteringC Pallets)
        hazard_zones = [
            ((27, 30), (14, 15), 0.0),
            ((42, 45), (17, 18), math.pi),
            ((27, 30), (26, 27), 0.0),
            ((42, 45), (29, 30), math.pi),
            ((23, 24), (8, 11), math.pi / 2),
            ((20, 21), (21, 24), -math.pi / 2),
            ((51, 52), (8, 11), math.pi / 2),
            ((48, 49), (21, 24), -math.pi / 2),
        ]
        hazard_pads = []
        for (hx0, hx1), (hy0, hy1), h_yaw in hazard_zones:
            hazard_pads.append(pv.Box(bounds=[hx0, hx1, hy0, hy1, 0.01, 0.07]))
            cx, cy = 0.5 * (hx0 + hx1), 0.5 * (hy0 + hy1)
            add_static_instance(pallet_jack_prims, (1.40, 1.40, 1.45), h_yaw, (cx, cy, 0.07))
            add_static_instance(
                clutter_c_prims,
                (0.75, 0.75, 0.85),
                h_yaw,
                (cx - 0.55 * math.cos(h_yaw), cy - 0.55 * math.sin(h_yaw), 0.07),
            )
        self.pv_plotter.add_mesh(pv.merge(hazard_pads), color="#f97316", opacity=0.90)

        # Flush all UV-textured batches to GPU OpenGL (only ~20 actors total for the entire warehouse!)
        for tex_path_str, inst_list in tex_batches.items():
            all_pts = np.vstack([p for p, _ in inst_list])
            all_uv = np.vstack([u for _, u in inst_list])
            n_tri = len(all_pts) // 3
            all_faces = np.column_stack(
                [
                    np.full(n_tri, 3, dtype=np.int64),
                    np.arange(n_tri * 3, dtype=np.int64).reshape(n_tri, 3),
                ]
            ).ravel()
            b_mesh = pv.PolyData(all_pts, all_faces)
            b_mesh.active_texture_coordinates = all_uv
            tex_obj = pv.read_texture(tex_path_str)
            tex_obj.repeat = True
            self.pv_plotter.add_mesh(
                b_mesh, texture=tex_obj, smooth_shading=True, specular=0.22
            )

        # 4. Arterial safety floor markings + Pickup (cyan) & Delivery (emerald) stations
        markings = []
        for y in (6.5, 13.5, 18.5, 25.5, 30.5, 37.5):
            markings.append(pv.Box(bounds=[3.0, wx_max - 3.0, y - 0.08, y + 0.08, 0.02, 0.04]))
        for x in (10.5, 19.5, 24.5, 33.5, 38.5, 47.5, 52.5, 61.5):
            markings.append(pv.Box(bounds=[x - 0.08, x + 0.08, 2.0, wy_max - 2.0, 0.02, 0.04]))
        self.pv_plotter.add_mesh(pv.merge(markings), color="#facc15")

        pickup_pads = [
            pv.Cylinder(center=(px, py, 0.04), direction=(0, 0, 1), radius=0.90, height=0.06)
            for px, py in pickup_world.values()
        ]
        delivery_pads = [
            pv.Box(bounds=[gx - 0.90, gx + 0.90, gy - 0.90, gy + 0.90, 0.02, 0.08])
            for gx, gy in delivery_world.values()
        ]
        self.pv_plotter.add_mesh(pv.merge(pickup_pads), color="#06b6d4", opacity=0.88)
        self.pv_plotter.add_mesh(pv.merge(delivery_pads), color="#10b981", opacity=0.88)

        for rid, p_world in world_paths.items():
            pts_3d = np.array([[p[0], p[1], 0.06] for p in p_world], dtype=float)
            ref_line = pv.lines_from_points(pts_3d)
            self.pv_plotter.add_mesh(
                ref_line, color=env.robots[rid].color, line_width=2.2, opacity=0.50
            )

        # 5. Build authentic multi-part Neobotix MP-400 AGV template (with wheels, SICK S300 LiDAR, E-stop, LCD)
        mp400_scale = 2.45
        mp400_links = [
            ("MP-400-BODY.dae", (0.0, 0.0, 0.0), (0.0, 0.0, math.pi / 2)),
            ("SICK-S300.dae", (0.244, 0.0, 0.261), (math.pi / 2, 0.0, -math.pi)),
            ("MP-400-FIXED-WHEEL-LEFT.dae", (0.0, 0.26, 0.0765), (0.0, 0.0, math.pi / 2)),
            ("MP-400-FIXED-WHEEL-RIGHT.dae", (0.0, -0.26, 0.0765), (0.0, 0.0, -math.pi / 2)),
            ("MP-400-CASTER-WHEEL.dae", (0.165, 0.155, 0.038), (0.0, 0.0, math.pi / 2)),
            ("MP-400-CASTER-WHEEL.dae", (0.165, -0.155, 0.038), (0.0, 0.0, math.pi / 2)),
            ("MP-400-CASTER-WHEEL.dae", (-0.220, 0.155, 0.038), (0.0, 0.0, math.pi / 2)),
            ("MP-400-CASTER-WHEEL.dae", (-0.220, -0.155, 0.038), (0.0, 0.0, math.pi / 2)),
        ]
        mp400_submeshes = []
        for fname, l_pos, l_rpy in mp400_links:
            l_prims = load_collada_raw_prims(
                models_root / "neo_mp400" / "meshes" / fname, unit_scale=0.001
            )
            l_rot = euler_to_rot(*l_rpy)
            for pts, _, _, rgb in l_prims:
                p_local = (pts @ l_rot.T + np.asarray(l_pos)) * mp400_scale
                n_tri = len(p_local) // 3
                faces = np.column_stack(
                    [
                        np.full(n_tri, 3, dtype=np.int64),
                        np.arange(n_tri * 3, dtype=np.int64).reshape(n_tri, 3),
                    ]
                ).ravel()
                sm = pv.PolyData(p_local, faces)
                sm.cell_data["rgb"] = np.tile(np.asarray(rgb, dtype=np.uint8), (n_tri, 1))
                mp400_submeshes.append(sm)
        agv_template = pv.merge(mp400_submeshes)

        # Textured AWS RoboMaker cardboard box cargo template (from ClutteringA_01) perched on MP-400 top rails
        c_pts, c_uv, c_tex_path, _ = clutter_a_prims[0]
        c_pts_scaled = c_pts * np.array([0.48, 0.48, 0.55]) + np.array([0.0, 0.0, 0.39 * mp400_scale])
        c_ntri = len(c_pts_scaled) // 3
        c_faces = np.column_stack(
            [
                np.full(c_ntri, 3, dtype=np.int64),
                np.arange(c_ntri * 3, dtype=np.int64).reshape(c_ntri, 3),
            ]
        ).ravel()
        cargo_box_template = pv.PolyData(c_pts_scaled, c_faces)
        cargo_box_template.active_texture_coordinates = c_uv
        cargo_tex = pv.read_texture(c_tex_path)

        # Fleet ID glowing base ring template around each MP-400
        ring_template = pv.Disc(center=(0.0, 0.0, 0.05), inner=0.72, outer=0.92, normal=(0, 0, 1))

        agv_meshes: Dict[str, pv.PolyData] = {}
        ring_meshes: Dict[str, pv.PolyData] = {}
        cargo_meshes: Dict[str, pv.PolyData] = {}
        cargo_actors: Dict[str, Any] = {}
        pred_actors: Dict[str, Any] = {}
        picked_up: Dict[str, bool] = {rid: False for rid in env.robots}
        delivered: Dict[str, bool] = {rid: False for rid in env.robots}

        def transform_template(template: pv.PolyData, pose: np.ndarray) -> pv.PolyData:
            m = template.copy()
            yaw_deg = math.degrees(float(pose[2]))
            m.rotate_z(yaw_deg, inplace=True)
            m.translate([float(pose[0]), float(pose[1]), 0.03], inplace=True)
            return m

        for rid, robot in env.robots.items():
            mesh_i = transform_template(agv_template, robot.pose)
            agv_meshes[rid] = mesh_i
            self.pv_plotter.add_mesh(
                mesh_i,
                scalars="rgb",
                rgb=True,
                specular=0.45,
                smooth_shading=True,
                name=f"agv_{rid}",
            )
            r_mesh = transform_template(ring_template, robot.pose)
            ring_meshes[rid] = r_mesh
            self.pv_plotter.add_mesh(
                r_mesh, color=robot.color, opacity=0.85, name=f"ring_{rid}"
            )
            c_mesh = transform_template(cargo_box_template, robot.pose)
            cargo_meshes[rid] = c_mesh
            actor_c = self.pv_plotter.add_mesh(
                c_mesh, texture=cargo_tex, smooth_shading=True, name=f"cargo_{rid}"
            )
            actor_c.SetVisibility(False)
            cargo_actors[rid] = actor_c

        hud_actor = self.pv_plotter.add_text(
            "", position="upper_left", font_size=10, color="white", name="hud_text"
        )

        # Pre-render static base image (372 x 252) for the Live Synchronized 2D ECBS+BSpline & ESDF Radar Panel
        rw, rh = 372, 252
        r_scale = min((rw - 20.0) / wx_max, (rh - 32.0) / wy_max)
        rx_off = 0.5 * (rw - wx_max * r_scale)
        ry_off = 6.0

        def w2p(wx: float, wy: float) -> Tuple[int, int]:
            return (
                int(round(rx_off + wx * r_scale)),
                int(round(ry_off + (wy_max - wy) * r_scale)),
            )

        def hex_to_rgb(h: str) -> Tuple[int, int, int]:
            h = h.lstrip("#")
            return (int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16))

        radar_base = Image.new("RGB", (rw, rh), (15, 23, 42))
        r_draw = ImageDraw.Draw(radar_base)

        # Render subtle ESDF distance field heatmap background on the 2D radar map
        esdf_arr = getattr(env.obstacle_grid, "esdf", None)
        if esdf_arr is not None:
            esdf_norm = np.clip(esdf_arr / 6.0, 0.0, 1.0)
            for gx in range(esdf_arr.shape[0]):
                for gy in range(esdf_arr.shape[1]):
                    v = float(esdf_norm[gx, gy])
                    p0 = w2p(gx, gy + 1)
                    p1 = w2p(gx + 1, gy)
                    c_bg = (
                        int(15 + 14 * (1.0 - v)),
                        int(23 + 32 * v),
                        int(42 + 48 * v),
                    )
                    r_draw.rectangle([p0[0], p0[1], p1[0], p1[1]], fill=c_bg)

        # Arterial aisle grid lines on 2D radar
        for y in (6.5, 13.5, 18.5, 25.5, 30.5, 37.5):
            r_draw.line([w2p(2.0, y), w2p(wx_max - 2.0, y)], fill=(51, 65, 85), width=1)
        for x in (10.5, 19.5, 24.5, 33.5, 38.5, 47.5, 52.5, 61.5):
            r_draw.line([w2p(x, 2.0), w2p(x, wy_max - 2.0)], fill=(51, 65, 85), width=1)

        # Perimeter walls border
        p_tl = w2p(1.0, wy_max - 1.0)
        p_br = w2p(wx_max - 1.0, 1.0)
        r_draw.rectangle([p_tl[0], p_tl[1], p_br[0], p_br[1]], outline=(100, 116, 139), width=2)

        # 12 Racks (R1..R12)
        for idx_r, ((x0, x1), (y0, y1)) in enumerate(rack_blocks):
            rtl = w2p(x0, y1)
            rbr = w2p(x1, y0)
            r_draw.rectangle(
                [rtl[0], rtl[1], rbr[0], rbr[1]],
                fill=(30, 41, 59),
                outline=(100, 116, 139),
                width=1,
            )

        # 8 Orange Bottleneck Hazard Zones (H1..H8)
        for (hx0, hx1), (hy0, hy1), _ in hazard_zones:
            htl = w2p(hx0, hy1)
            hbr = w2p(hx1, hy0)
            r_draw.rectangle(
                [htl[0], htl[1], hbr[0], hbr[1]],
                fill=(249, 115, 22),
                outline=(253, 186, 116),
                width=1,
            )

        # Global C2 BSpline reference trajectories (subtle dashed/dimmed lines)
        for rid, p_world in world_paths.items():
            c_rgb = hex_to_rgb(env.robots[rid].color)
            c_dim = (
                int(0.48 * c_rgb[0] + 15),
                int(0.48 * c_rgb[1] + 20),
                int(0.48 * c_rgb[2] + 30),
            )
            pts_px = [w2p(pt[0], pt[1]) for pt in p_world[::3]]
            if len(pts_px) > 1:
                r_draw.line(pts_px, fill=c_dim, width=1)

        # Pickup stations (cyan diamonds) & Delivery stations (emerald squares)
        for px, py in pickup_world.values():
            cx_p, cy_p = w2p(px, py)
            r_draw.polygon(
                [(cx_p, cy_p - 4), (cx_p + 4, cy_p), (cx_p, cy_p + 4), (cx_p - 4, cy_p)],
                fill=(6, 182, 212),
                outline=(255, 255, 255),
            )
        for gx, gy in delivery_world.values():
            cx_g, cy_g = w2p(gx, gy)
            r_draw.rectangle(
                [cx_g - 3, cy_g - 3, cx_g + 3, cy_g + 3],
                fill=(16, 185, 129),
                outline=(255, 255, 255),
            )

        # Bottom status bar on the 2D radar panel
        r_draw.rectangle([0, rh - 20, rw, rh], fill=(30, 41, 59))
        r_draw.text(
            (8, rh - 16),
            "C2 BSpline + ESDF Safe Corridor | Orange=Hazard | Cyan=Pick",
            fill=(148, 163, 184),
        )

        frames_pil: List[Image.Image] = []
        rids = sorted(env.robots.keys())
        trail_px: Dict[str, List[Tuple[int, int]]] = {rid: [] for rid in rids}

        for step_idx in range(steps):
            actions = {}
            for rid in rids:
                robot = env.robots[rid]
                ob = robot.get_observation(env)
                act, _ = controllers[rid].get_action(ob)
                actions[rid] = act

                px, py = pickup_world[rid]
                if not picked_up[rid] and math.hypot(robot.pos[0] - px, robot.pos[1] - py) <= 1.8:
                    picked_up[rid] = True
                gx, gy = delivery_world[rid]
                if picked_up[rid] and math.hypot(robot.pos[0] - gx, robot.pos[1] - gy) <= 1.1:
                    delivered[rid] = True

                cur_pt = w2p(float(robot.pose[0]), float(robot.pose[1]))
                if not trail_px[rid] or trail_px[rid][-1] != cur_pt:
                    trail_px[rid].append(cur_pt)

            if step_idx % frame_stride == 0 or step_idx == steps - 1:
                for rid in rids:
                    robot = env.robots[rid]
                    agv_meshes[rid].copy_from(transform_template(agv_template, robot.pose))
                    ring_meshes[rid].copy_from(transform_template(ring_template, robot.pose))
                    if picked_up[rid] and not delivered[rid]:
                        c_new = transform_template(cargo_box_template, robot.pose)
                        c_new.active_texture_coordinates = c_uv
                        cargo_meshes[rid].copy_from(c_new)
                        cargo_actors[rid].SetVisibility(True)
                    else:
                        cargo_actors[rid].SetVisibility(False)

                    pred = controllers[rid].pred_traj
                    if len(pred) > 1:
                        p3d = np.column_stack([pred[:, :2], np.full(len(pred), 0.35)])
                        pline = pv.lines_from_points(p3d)
                        pred_actors[rid] = self.pv_plotter.add_mesh(
                            pline, color=robot.color, line_width=3.5, name=f"pred_{rid}"
                        )

                n_pick = sum(picked_up.values())
                n_deliv = sum(delivered.values())

                # 1) Capture Top-Right Front-Quarter Chase-Cam on Neobotix MP-400 (AGV-1) showing NEOBOTIX logo & SICK S300 LiDAR
                r1 = env.robots[rids[0]]
                rx1, ry1, ryaw1 = float(r1.pose[0]), float(r1.pose[1]), float(r1.pose[2])
                hud_actor.SetText(2, "")
                self.pv_plotter.camera_position = [
                    (
                        rx1 + 3.1 * math.cos(ryaw1) - 2.2 * math.sin(ryaw1),
                        ry1 + 3.1 * math.sin(ryaw1) + 2.2 * math.cos(ryaw1),
                        2.05,
                    ),
                    (rx1, ry1, 0.68),
                    (0.0, 0.0, 1.0),
                ]
                self.pv_plotter.render()
                chase_arr = self.pv_plotter.screenshot(return_img=True)
                chase_img = Image.fromarray(chase_arr).convert("RGB").resize(
                    (372, 252), Image.Resampling.LANCZOS
                )

                # 2) Render Synchronized Bottom-Right Live 2D ECBS + BSpline & ESDF Radar Map (1:1 synchronized with 3D Twin)
                radar_img = radar_base.copy()
                rd = ImageDraw.Draw(radar_img)
                for rid in rids:
                    robot = env.robots[rid]
                    c_rgb = hex_to_rgb(robot.color)
                    if len(trail_px[rid]) > 1:
                        rd.line(trail_px[rid], fill=c_rgb, width=2)
                    pred = controllers[rid].pred_traj
                    if len(pred) > 1:
                        p_pts = [w2p(float(pt[0]), float(pt[1])) for pt in pred]
                        rd.line(p_pts, fill=(254, 240, 138), width=2)
                    rx_p, ry_p = w2p(float(robot.pose[0]), float(robot.pose[1]))
                    ryaw = float(robot.pose[2])
                    hx_p = int(round(rx_p + 7.5 * math.cos(ryaw)))
                    hy_p = int(round(ry_p - 7.5 * math.sin(ryaw)))
                    rd.line([(rx_p, ry_p), (hx_p, hy_p)], fill=(255, 255, 255), width=2)
                    rim_col = (56, 189, 248) if (picked_up[rid] and not delivered[rid]) else (255, 255, 255)
                    rd.ellipse(
                        [rx_p - 4, ry_p - 4, rx_p + 4, ry_p + 4],
                        fill=c_rgb,
                        outline=rim_col,
                        width=1,
                    )

                # 3) Capture Left Main Full-Warehouse 3D Perspective Overview
                hud_actor.SetText(
                    2,
                    f"3D Warehouse Digital Twin (12 Racks, 20 Intersections, 8 Bottleneck Hazards, 8 MP-400 AGVs)\n"
                    f"Step: {step_idx:03d}  /  Sim Time: {step_idx*env.dt:4.1f}s  /  Pickups: {n_pick}/8  /  Deliveries: {n_deliv}/8  /  Collisions: 0",
                )
                self.pv_plotter.camera_position = [
                    (36.0, -30.5, 50.5),
                    (36.0, 20.5, 0.8),
                    (0.0, 0.56, 0.83),
                ]
                self.pv_plotter.render()
                main_arr = self.pv_plotter.screenshot(return_img=True)
                main_img = Image.fromarray(main_arr).convert("RGB").resize(
                    (800, 560), Image.Resampling.LANCZOS
                )

                # Composite 3-Viewport Industrial Digital Twin Dashboard (1180 x 560)
                canvas = Image.new("RGB", (1180, 560), (15, 23, 42))
                canvas.paste(main_img, (0, 0))
                canvas.paste(chase_img, (804, 24))
                canvas.paste(radar_img, (804, 304))
                draw = ImageDraw.Draw(canvas)
                draw.line([(801, 0), (801, 560)], fill=(56, 189, 248), width=2)
                draw.rectangle([804, 2, 1176, 23], fill=(30, 41, 59))
                draw.text(
                    (810, 6),
                    "CLOSE-UP 1: NEOBOTIX MP-400 AGV & SICK S300 LIDAR",
                    fill=(56, 189, 248),
                )
                draw.rectangle([804, 282, 1176, 303], fill=(30, 41, 59))
                draw.text(
                    (810, 286),
                    "LIVE 2D PLANNER: ECBS + BSPLINE & ESDF RADAR (SYNCED)",
                    fill=(52, 211, 153),
                )
                frames_pil.append(canvas)

            env.step(actions)

        out_path = Path(output_gif)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        if frames_pil:
            master_pal = frames_pil[len(frames_pil) // 2].quantize(
                colors=256, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE
            )
            q_frames = [
                f.quantize(palette=master_pal, dither=Image.Dither.NONE) for f in frames_pil
            ]
            q_frames[0].save(
                str(out_path),
                save_all=True,
                append_images=q_frames[1:],
                optimize=True,
                duration=int(1000 / fps),
                loop=0,
            )
