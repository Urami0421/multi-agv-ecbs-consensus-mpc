"""
@file: ecbs.py
@description: Enhanced Conflict-Based Search (ECBS) with Two-Level Focal Search
              and Kinematic Turn Penalty for Multi-AGV Path Planning.
@references:
    [1] Barer, M., Sharon, G., Stern, R., & Felner, A. (2014). Suboptimal Variants
        of the Conflict-Based Search Algorithm for the Multi-Agent Pathfinding Problem. ECAI.
    [2] Cohen, L., Uras, T., Kumar, T. K. S., Xu, H., Ayanian, N., & Koenig, S. (2016).
        Improved Solvers for Bounded-Suboptimal Multi-Agent Path Finding. IJCAI.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import itertools
import time
from typing import Dict, List, Optional, Sequence, Set, Tuple, Any

from python_motion_planning.common import Grid, Node, TYPES

GridPoint = Tuple[int, int]


@dataclass(frozen=True)
class SpaceTimeConstraint:
    """Vertex or edge-swap space-time constraint for a single agent."""
    agent_id: str
    time_step: int
    position: GridPoint
    next_position: Optional[GridPoint] = None  # None = vertex constraint; else edge-swap constraint


@dataclass(frozen=True)
class SpaceTimeConflict:
    """First detected vertex or edge-swap conflict between two agents."""
    time_step: int
    agent_a: str
    agent_b: str
    pos_a: GridPoint
    pos_b: GridPoint
    is_edge: bool = False


@dataclass
class _CTNode:
    """High-level Constraint Tree (CT) node for CBS / ECBS."""
    cost: float
    lower_bound: float
    conflict_count: int
    paths: Dict[str, List[GridPoint]]
    low_level_fmin: Dict[str, float]
    low_level_turns: Dict[str, int]
    constraints: Set[SpaceTimeConstraint] = field(default_factory=set)
    expanded_low_level: int = 0


class ECBS:
    """
    Enhanced Conflict-Based Search (ECBS) multi-agent path planner on a 2D `Grid`.

    When `w = 1.0` and `turn_penalty = 0.0`, ECBS behaves as exact optimal CBS.
    When `w > 1.0` (e.g. `w = 1.08`), ECBS uses a two-level Focal Search guided by a
    Conflict Avoidance Table (CAT) and a kinematic 90-degree turn penalty, drastically
    reducing high-level branching and producing smoother corridor paths for differential-drive AGVs.

    Args:
        map_: 2D `Grid` map from `python_motion_planning.common.env.map`.
        starts: Dict mapping `agent_id -> (x, y)` start grid cell.
        goals: Dict mapping `agent_id -> (x, y)` goal grid cell (or final delivery cell).
        waypoints: Optional Dict mapping `agent_id -> [(wx, wy), ...]` intermediate stops (e.g. pickup).
        w: Bounded suboptimality factor (`w >= 1.0`).
        turn_penalty: Cost penalty for 90-degree / 180-degree heading changes (`>= 0.0`).
        max_time: Maximum space-time horizon.
        max_high_level_nodes: Maximum high-level CT node expansions.
    """

    _DIRS: Tuple[GridPoint, ...] = ((1, 0), (-1, 0), (0, 1), (0, -1), (0, 0))

    def __init__(
        self,
        map_: Grid,
        starts: Dict[str, GridPoint],
        goals: Dict[str, GridPoint],
        waypoints: Optional[Dict[str, Sequence[GridPoint]]] = None,
        w: float = 1.08,
        turn_penalty: float = 0.35,
        min_separation: float = 1.0,
        max_time: int = 160,
        max_high_level_nodes: int = 800,
    ) -> None:
        if map_.dim != 2:
            raise ValueError("ECBS currently supports 2D Grid maps.")
        if w < 1.0:
            raise ValueError("Suboptimality factor w must be >= 1.0.")
        self.map_ = map_
        self.starts = {k: (int(v[0]), int(v[1])) for k, v in starts.items()}
        self.goals = {k: (int(v[0]), int(v[1])) for k, v in goals.items()}
        self.waypoints: Dict[str, Tuple[GridPoint, ...]] = {}
        for aid in self.starts:
            wps = waypoints.get(aid, ()) if waypoints else ()
            self.waypoints[aid] = tuple((int(p[0]), int(p[1])) for p in wps)
        self.agent_ids = sorted(self.starts.keys())
        self.w = float(w)
        self.turn_penalty = float(turn_penalty)
        self.min_separation = float(max(1.0, min_separation))
        self.max_time = int(max_time)
        self.max_high_level_nodes = int(max_high_level_nodes)

    def __str__(self) -> str:
        return f"ECBS(w={self.w:.2f}, turn_penalty={self.turn_penalty:.2f})"

    def _stops_for(self, agent_id: str) -> Tuple[GridPoint, ...]:
        return (self.starts[agent_id], *self.waypoints[agent_id], self.goals[agent_id])

    def _is_free(self, pt: GridPoint) -> bool:
        if not self.map_.within_bounds(pt):
            return False
        cell_type = int(self.map_.type_map[pt])
        return cell_type != TYPES.OBSTACLE and cell_type != TYPES.INFLATION

    @staticmethod
    def _manhattan(a: GridPoint, b: GridPoint) -> int:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def _turn_aware_h(self, pos: GridPoint, dir_idx: int, target: GridPoint) -> float:
        dx = target[0] - pos[0]
        dy = target[1] - pos[1]
        base = float(abs(dx) + abs(dy))
        if self.turn_penalty <= 0.0 or (dx == 0 and dy == 0):
            return base
        if dx != 0 and dy != 0:
            return base + self.turn_penalty
        # Single axis remaining (dx == 0 or dy == 0)
        if dir_idx == -1:
            return base
        cur_dx, cur_dy = self._DIRS[dir_idx]
        if (dx != 0 and cur_dx * dx > 0) or (dy != 0 and cur_dy * dy > 0):
            return base
        return base + self.turn_penalty

    @staticmethod
    def _pos_at(path: Sequence[GridPoint], t: int) -> GridPoint:
        return path[t] if t < len(path) else path[-1]

    @staticmethod
    def _count_turns(path: Sequence[GridPoint]) -> int:
        turns = 0
        prev_dir: Optional[GridPoint] = None
        for i in range(1, len(path)):
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
            if dx == 0 and dy == 0:
                continue
            cur_dir = (dx, dy)
            if prev_dir is not None and cur_dir != prev_dir:
                turns += 1
            prev_dir = cur_dir
        return turns

    def _detect_conflicts(
        self, paths: Dict[str, List[GridPoint]]
    ) -> Tuple[int, Optional[SpaceTimeConflict]]:
        """Count total pairwise space-time conflicts and return the earliest conflict."""
        horizon = max((len(p) for p in paths.values()), default=1)
        total_conflicts = 0
        first_conflict: Optional[SpaceTimeConflict] = None

        for t in range(horizon):
            for idx_a in range(len(self.agent_ids)):
                aid_a = self.agent_ids[idx_a]
                path_a = paths[aid_a]
                pos_a_t = self._pos_at(path_a, t)
                pos_a_next = self._pos_at(path_a, t + 1)

                for idx_b in range(idx_a + 1, len(self.agent_ids)):
                    aid_b = self.agent_ids[idx_b]
                    path_b = paths[aid_b]
                    pos_b_t = self._pos_at(path_b, t)
                    pos_b_next = self._pos_at(path_b, t + 1)

                    dx = pos_a_t[0] - pos_b_t[0]
                    dy = pos_a_t[1] - pos_b_t[1]
                    if dx * dx + dy * dy < (self.min_separation - 1e-6) ** 2:
                        total_conflicts += 1
                        if first_conflict is None:
                            first_conflict = SpaceTimeConflict(
                                time_step=t,
                                agent_a=aid_a,
                                agent_b=aid_b,
                                pos_a=pos_a_t,
                                pos_b=pos_b_t,
                                is_edge=False,
                            )
                    if t + 1 < horizon and pos_a_t == pos_b_next and pos_b_t == pos_a_next and pos_a_t != pos_a_next:
                        total_conflicts += 1
                        if first_conflict is None:
                            first_conflict = SpaceTimeConflict(
                                time_step=t + 1,
                                agent_a=aid_a,
                                agent_b=aid_b,
                                pos_a=pos_a_t,
                                pos_b=pos_b_t,
                                is_edge=True,
                            )
        return total_conflicts, first_conflict

    def _focal_low_level_astar(
        self,
        agent_id: str,
        constraints: Set[SpaceTimeConstraint],
        other_paths: Dict[str, List[GridPoint]],
    ) -> Tuple[Optional[List[GridPoint]], float, int, Dict[GridPoint, Node]]:
        """
        Low-level Multi-Label Focal Space-Time A* with Conflict Avoidance Table (CAT)
        and kinematic turn penalty.
        """
        stops = self._stops_for(agent_id)
        for pt in stops:
            if not self._is_free(pt):
                return None, float("inf"), 0, {}

        final_stage = len(stops) - 1
        initial_stage = 1
        while initial_stage < final_stage and stops[initial_stage] == stops[0]:
            initial_stage += 1

        remaining_leg = [0.0] * len(stops)
        for stage in range(final_stage - 1, 0, -1):
            leg_turn = (
                self.turn_penalty
                if (
                    stops[stage][0] != stops[stage + 1][0]
                    and stops[stage][1] != stops[stage + 1][1]
                )
                else 0.0
            )
            remaining_leg[stage] = (
                float(self._manhattan(stops[stage], stops[stage + 1]))
                + leg_turn
                + remaining_leg[stage + 1]
            )

        vertex_blocked: Set[Tuple[int, GridPoint]] = set()
        edge_blocked: Set[Tuple[int, GridPoint, GridPoint]] = set()
        max_constraint_t_at_goal = 0
        goal_pt = stops[-1]
        for c in constraints:
            if c.agent_id != agent_id:
                continue
            if c.next_position is None:
                vertex_blocked.add((c.time_step, c.position))
                if c.position == goal_pt:
                    max_constraint_t_at_goal = max(max_constraint_t_at_goal, c.time_step)
            else:
                edge_blocked.add((c.time_step, c.position, c.next_position))

        # Precompute Conflict Avoidance Table (CAT) from other agents' current trajectories
        cat_vertex: Dict[Tuple[int, GridPoint], int] = {}
        cat_edge: Dict[Tuple[int, GridPoint, GridPoint], int] = {}
        cat_horizon = max((len(p) for p in other_paths.values()), default=1) + 4
        sep_rad = int(self.min_separation - 1e-6)
        sep_sq = (self.min_separation - 1e-6) ** 2
        for other_id, opath in other_paths.items():
            if other_id == agent_id or not opath:
                continue
            for t in range(cat_horizon):
                p_cur = self._pos_at(opath, t)
                if sep_rad <= 0:
                    cat_vertex[(t, p_cur)] = cat_vertex.get((t, p_cur), 0) + 1
                else:
                    for ox in range(-sep_rad, sep_rad + 1):
                        for oy in range(-sep_rad, sep_rad + 1):
                            if ox * ox + oy * oy < sep_sq:
                                nb = (p_cur[0] + ox, p_cur[1] + oy)
                                cat_vertex[(t, nb)] = cat_vertex.get((t, nb), 0) + 1
                p_nxt = self._pos_at(opath, t + 1)
                cat_edge[(t + 1, p_nxt, p_cur)] = cat_edge.get((t + 1, p_nxt, p_cur), 0) + 1

        # State: (pos, time_step, stage, dir_idx) where dir_idx in {0,1,2,3} or -1 (initial/wait)
        start_pt = stops[0]
        h0 = float(
            self._turn_aware_h(start_pt, -1, stops[initial_stage])
            + remaining_leg[initial_stage]
        )
        start_state = (start_pt, 0, initial_stage, -1)

        seq = itertools.count()
        g_sign = -1.0 if self.w > 1.0 else 1.0
        # OPEN entries: (f, g_tie, hc, seq_id, g_val, state)
        open_heap: List[Tuple[float, float, int, int, float, Tuple[GridPoint, int, int, int]]] = [
            (h0, 0.0, 0, next(seq), 0.0, start_state)
        ]
        best_cost: Dict[Tuple[GridPoint, int, int, int], float] = {start_state: 0.0}
        came_from: Dict[Tuple[GridPoint, int, int, int], Tuple[GridPoint, int, int, int]] = {}
        expanded_2d: Dict[GridPoint, Node] = {start_pt: Node(start_pt, None, 0.0, h0)}
        expanded_count = 0
        f_min_lower_bound: Optional[float] = None

        while open_heap:
            # Focal selection: inspect up to K lowest-f candidates within w * f_min
            f_head = open_heap[0][0]
            if f_min_lower_bound is None:
                f_min_lower_bound = f_head
            else:
                f_min_lower_bound = max(f_min_lower_bound, f_head)

            if self.w > 1.0 and len(open_heap) > 1:
                bound = self.w * f_head
                popped_buffer = []
                best_idx = 0
                best_tuple = None
                limit = min(len(open_heap), 32)
                for i in range(limit):
                    cand = heapq.heappop(open_heap)
                    popped_buffer.append(cand)
                    if cand[0] <= bound:
                        # Prefer fewer CAT conflicts (cand[2]), lower f (cand[0]), and higher progress (-g_val)
                        key = (cand[2], cand[0], -cand[4])
                        if best_tuple is None or key < best_tuple:
                            best_tuple = key
                            best_idx = i
                    else:
                        break
                chosen = popped_buffer.pop(best_idx)
                for item in popped_buffer:
                    heapq.heappush(open_heap, item)
                f_val, _, hc_val, _, g_val, state = chosen
            else:
                f_val, _, hc_val, _, g_val, state = heapq.heappop(open_heap)

            if g_val > best_cost.get(state, float("inf")) + 1e-9:
                continue

            pos, t, stage, dir_idx = state
            expanded_count += 1

            if stage == final_stage and pos == goal_pt and t >= max_constraint_t_at_goal:
                # Reconstruct path
                rev_path = [pos]
                cur = state
                while cur in came_from:
                    cur = came_from[cur]
                    rev_path.append(cur[0])
                rev_path.reverse()
                return rev_path, float(f_min_lower_bound or len(rev_path) - 1), expanded_count, expanded_2d

            if t >= self.max_time:
                continue

            next_t = t + 1
            for d_i, (dx, dy) in enumerate(self._DIRS):
                nxt_pos: GridPoint = (pos[0] + dx, pos[1] + dy)
                if not self._is_free(nxt_pos):
                    continue
                if (next_t, nxt_pos) in vertex_blocked:
                    continue
                if (next_t, pos, nxt_pos) in edge_blocked:
                    continue

                nxt_stage = stage
                while nxt_stage < final_stage and nxt_pos == stops[nxt_stage]:
                    nxt_stage += 1

                is_move = d_i < 4
                nxt_dir = d_i if is_move else dir_idx
                turn_cost = 0.0
                if is_move and dir_idx != -1 and d_i != dir_idx:
                    turn_cost = self.turn_penalty

                step_cost = 1.0 + turn_cost
                nxt_g = g_val + step_cost
                nxt_state = (nxt_pos, next_t, nxt_stage, nxt_dir)

                if nxt_g + 1e-9 >= best_cost.get(nxt_state, float("inf")):
                    continue

                best_cost[nxt_state] = nxt_g
                came_from[nxt_state] = state

                if nxt_pos not in expanded_2d:
                    expanded_2d[nxt_pos] = Node(nxt_pos, pos, nxt_g, 0.0)

                if nxt_stage == final_stage:
                    h_rem = float(
                        self._turn_aware_h(nxt_pos, nxt_dir, stops[final_stage])
                    )
                else:
                    h_rem = float(
                        self._turn_aware_h(nxt_pos, nxt_dir, stops[nxt_stage])
                        + remaining_leg[nxt_stage]
                    )

                nxt_hc = (
                    hc_val
                    + cat_vertex.get((next_t, nxt_pos), 0)
                    + cat_edge.get((next_t, pos, nxt_pos), 0)
                )
                heapq.heappush(
                    open_heap,
                    (nxt_g + h_rem, g_sign * nxt_g, nxt_hc, next(seq), nxt_g, nxt_state),
                )

        return None, float("inf"), expanded_count, expanded_2d

    def plan(self) -> Tuple[Dict[str, List[GridPoint]], Dict[str, Any]]:
        """
        Run ECBS multi-agent space-time planning.

        Returns:
            paths: Dict mapping `agent_id -> [(x0, y0), (x1, y1), ...]`.
            info: Dict of planning diagnostics (expanded high/low-level nodes, turns, runtime, etc.).
        """
        t_start = time.perf_counter()
        root_paths: Dict[str, List[GridPoint]] = {}
        root_fmin: Dict[str, float] = {}
        root_turns: Dict[str, int] = {}
        expanded_map_union: Dict[GridPoint, Node] = {}
        total_low_level_expanded = 0

        for aid in self.agent_ids:
            path, fmin, exp_cnt, exp_nodes = self._focal_low_level_astar(
                aid, set(), root_paths
            )
            total_low_level_expanded += exp_cnt
            expanded_map_union.update(exp_nodes)
            if path is None:
                return {}, {
                    "success": False,
                    "reason": f"no_initial_path:{aid}",
                    "planning_time_ms": (time.perf_counter() - t_start) * 1000.0,
                }
            root_paths[aid] = path
            root_fmin[aid] = fmin
            root_turns[aid] = self._count_turns(path)

        conflict_cnt, _ = self._detect_conflicts(root_paths)
        root_cost = sum(
            (len(p) - 1) + self.turn_penalty * root_turns[aid]
            for aid, p in root_paths.items()
        )
        root_lb = sum(root_fmin.values())

        root_node = _CTNode(
            cost=root_cost,
            lower_bound=root_lb,
            conflict_count=conflict_cnt,
            paths=root_paths,
            low_level_fmin=root_fmin,
            low_level_turns=root_turns,
            constraints=set(),
            expanded_low_level=total_low_level_expanded,
        )

        seq = itertools.count()
        # High-level OPEN ordered by (lower_bound, conflict_count, cost, seq, node)
        open_ct: List[Tuple[float, int, float, int, _CTNode]] = [
            (root_node.lower_bound, root_node.conflict_count, root_node.cost, next(seq), root_node)
        ]
        high_level_expanded = 0

        while open_ct and high_level_expanded < self.max_high_level_nodes:
            # High-level Focal selection: among CT nodes with cost <= w * min_lb, pick lowest conflict_count
            min_lb = open_ct[0][0]
            if self.w > 1.0 and len(open_ct) > 1:
                bound = self.w * min_lb
                buffer = []
                best_idx = 0
                best_key = None
                limit = min(len(open_ct), 16)
                for i in range(limit):
                    cand = heapq.heappop(open_ct)
                    buffer.append(cand)
                    if cand[2] <= bound:
                        key = (cand[1], cand[2], cand[0])  # (conflict_count, cost, lb)
                        if best_key is None or key < best_key:
                            best_key = key
                            best_idx = i
                chosen = buffer.pop(best_idx)
                for item in buffer:
                    heapq.heappush(open_ct, item)
                _, _, _, _, current = chosen
            else:
                _, _, _, _, current = heapq.heappop(open_ct)

            high_level_expanded += 1
            num_conflicts, conflict = self._detect_conflicts(current.paths)
            if conflict is None:
                elapsed_ms = (time.perf_counter() - t_start) * 1000.0
                total_steps = sum(len(p) - 1 for p in current.paths.values())
                total_turns = sum(current.low_level_turns.values())
                return current.paths, {
                    "success": True,
                    "planner": str(self),
                    "w": self.w,
                    "turn_penalty": self.turn_penalty,
                    "high_level_expanded": high_level_expanded,
                    "low_level_expanded": total_low_level_expanded,
                    "total_steps": total_steps,
                    "total_turns": total_turns,
                    "makespan": max((len(p) - 1 for p in current.paths.values()), default=0),
                    "planning_time_ms": elapsed_ms,
                    "expand": expanded_map_union,
                }

            # Branch into two child CT nodes
            branches = (
                (
                    conflict.agent_a,
                    SpaceTimeConstraint(
                        agent_id=conflict.agent_a,
                        time_step=conflict.time_step,
                        position=conflict.pos_a,
                        next_position=conflict.pos_b if conflict.is_edge else None,
                    ),
                ),
                (
                    conflict.agent_b,
                    SpaceTimeConstraint(
                        agent_id=conflict.agent_b,
                        time_step=conflict.time_step,
                        position=conflict.pos_b,
                        next_position=conflict.pos_a if conflict.is_edge else None,
                    ),
                ),
            )

            for constrained_agent, new_constraint in branches:
                if new_constraint in current.constraints:
                    continue
                next_constraints = set(current.constraints)
                next_constraints.add(new_constraint)
                other_paths = {
                    k: v for k, v in current.paths.items() if k != constrained_agent
                }
                new_path, new_fmin, exp_cnt, exp_nodes = self._focal_low_level_astar(
                    constrained_agent, next_constraints, other_paths
                )
                total_low_level_expanded += exp_cnt
                expanded_map_union.update(exp_nodes)
                if new_path is None:
                    continue

                next_paths = dict(current.paths)
                next_paths[constrained_agent] = new_path
                next_fmin = dict(current.low_level_fmin)
                next_fmin[constrained_agent] = new_fmin
                next_turns = dict(current.low_level_turns)
                next_turns[constrained_agent] = self._count_turns(new_path)

                c_cnt, _ = self._detect_conflicts(next_paths)
                c_cost = sum(
                    (len(p) - 1) + self.turn_penalty * next_turns[aid]
                    for aid, p in next_paths.items()
                )
                c_lb = sum(next_fmin.values())
                child = _CTNode(
                    cost=c_cost,
                    lower_bound=c_lb,
                    conflict_count=c_cnt,
                    paths=next_paths,
                    low_level_fmin=next_fmin,
                    low_level_turns=next_turns,
                    constraints=next_constraints,
                    expanded_low_level=exp_cnt,
                )
                heapq.heappush(
                    open_ct,
                    (child.lower_bound, child.conflict_count, child.cost, next(seq), child),
                )

        elapsed_ms = (time.perf_counter() - t_start) * 1000.0
        return {}, {
            "success": False,
            "reason": "high_level_limit_exceeded",
            "high_level_expanded": high_level_expanded,
            "low_level_expanded": total_low_level_expanded,
            "planning_time_ms": elapsed_ms,
            "expand": expanded_map_union,
        }


class CBS(ECBS):
    """Exact Conflict-Based Search (`w = 1.0, turn_penalty = 0.0`)."""

    def __init__(self, map_: Grid, starts: Dict[str, GridPoint], goals: Dict[str, GridPoint], **kwargs) -> None:
        kwargs.setdefault("w", 1.0)
        kwargs.setdefault("turn_penalty", 0.0)
        super().__init__(map_=map_, starts=starts, goals=goals, **kwargs)

    def __str__(self) -> str:
        return "CBS(exact)"
