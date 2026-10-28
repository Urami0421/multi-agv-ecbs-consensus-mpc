"""
@file: ecbs.py
@description: Enhanced Conflict-Based Search (ECBS) with Two-Level Focal Search,
              Conflict Avoidance Table (CAT), and Admissible Turn-Aware Heuristic.
@references:
    [1] Sharon, G., Stern, R., Felner, A., & Sturtevant, N. R. (2015). Conflict-Based Search
        for Optimal Multi-Agent Pathfinding. Artificial Intelligence, 219, 40-66.
    [2] Barer, M., Sharon, G., Stern, R., & Felner, A. (2014). Suboptimal Variants
        of the Conflict-Based Search Algorithm for the Multi-Agent Pathfinding Problem. SoCS / ECAI.
    [3] Cohen, L., Uras, T., Kumar, T. K. S., Xu, H., Ayanian, N., & Koenig, S. (2016).
        Improved Solvers for Bounded-Suboptimal Multi-Agent Path Finding. IJCAI.
    [4] Pearl, J., & Kim, J. H. (1982). Studies in Semi-Admissible Heuristics.
        IEEE Transactions on Pattern Analysis and Machine Intelligence, PAMI-4(4), 392-399.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import itertools
import time
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

from python_motion_planning.common import Grid, Node, TYPES

GridPoint = Tuple[int, int]
LowLevelState = Tuple[GridPoint, int, int, int]  # (pos, time_step, stage_idx, dir_idx)


@dataclass(frozen=True)
class SpaceTimeConstraint:
    """Vertex or edge-swap space-time constraint for a single agent."""
    agent_id: str
    time_step: int
    position: GridPoint
    next_position: Optional[GridPoint] = None  # None = vertex constraint; else edge-swap constraint


@dataclass(frozen=True)
class SpaceTimeConflict:
    """Detected vertex or edge-swap conflict between two agents."""
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
    first_conflict: Optional[SpaceTimeConflict]
    paths: Dict[str, List[GridPoint]]
    low_level_costs: Dict[str, float]
    low_level_fmin: Dict[str, float]
    low_level_turns: Dict[str, int]
    constraints: Set[SpaceTimeConstraint] = field(default_factory=set)
    expanded_low_level: int = 0


class ECBS:
    """
    Enhanced Conflict-Based Search (ECBS) multi-agent path planner on a 2D `Grid`.

    Maintains complete OPEN and FOCAL priority queues at both the low level (space-time focal A*)
    and high level (Constraint Tree focal search), guaranteeing the bounded-suboptimality
    condition `cost <= w * lower_bound <= w * optimal_cost`.

    When `w = 1.0` and `turn_penalty = 0.0`, ECBS reduces to exact optimal CBS.

    Args:
        map_: 2D `Grid` map from `python_motion_planning.common.env.map`.
        starts: Dict mapping `agent_id -> (x, y)` start grid cell.
        goals: Dict mapping `agent_id -> (x, y)` goal grid cell.
        waypoints: Optional Dict mapping `agent_id -> [(wx, wy), ...]` ordered intermediate stops.
        w: Bounded suboptimality factor (`w >= 1.0`).
        turn_penalty: Cost penalty for 90-degree / 180-degree heading changes (`>= 0.0`).
        min_separation: Minimum Euclidean cell distance between agents at the same time step.
        max_time: Maximum space-time horizon.
        max_high_level_nodes: Maximum high-level CT node expansions.
        record_expanded: Whether to record 2D `Node` expansion dictionary for visualization.
    """

    _DIRS: Tuple[GridPoint, ...] = ((1, 0), (-1, 0), (0, 1), (0, -1), (0, 0))

    def __init__(
        self,
        map_: Grid,
        starts: Dict[str, GridPoint],
        goals: Dict[str, GridPoint],
        waypoints: Optional[Dict[str, Sequence[GridPoint]]] = None,
        w: float = 1.08,
        turn_penalty: float = 0.45,
        min_separation: float = 1.0,
        max_time: int = 160,
        max_high_level_nodes: int = 800,
        record_expanded: bool = False,
    ) -> None:
        if map_.dim != 2:
            raise ValueError("ECBS currently supports 2D Grid maps.")
        if w < 1.0:
            raise ValueError("Suboptimality factor w must be >= 1.0.")
        if set(starts.keys()) != set(goals.keys()):
            raise ValueError("starts and goals must have identical agent_id keys.")

        self.map_ = map_
        self.starts = {k: (int(v[0]), int(v[1])) for k, v in starts.items()}
        self.goals = {k: (int(v[0]), int(v[1])) for k, v in goals.items()}
        self.waypoints: Dict[str, Tuple[GridPoint, ...]] = {}
        for aid in self.starts:
            wps = waypoints.get(aid, ()) if waypoints else ()
            self.waypoints[aid] = tuple((int(p[0]), int(p[1])) for p in wps)
        self.agent_ids = sorted(self.starts.keys())
        self.w = float(w)
        self.turn_penalty = float(max(0.0, turn_penalty))
        self.min_separation = float(max(1.0, min_separation))
        self.max_time = int(max_time)
        self.max_high_level_nodes = int(max_high_level_nodes)
        self.record_expanded = bool(record_expanded)

        # Precompute integer offset disk for min_separation conflict/CAT lookup
        sep_rad = int(self.min_separation - 1e-6)
        sep_sq = (self.min_separation - 1e-6) ** 2
        offsets: List[GridPoint] = []
        for ox in range(-sep_rad, sep_rad + 1):
            for oy in range(-sep_rad, sep_rad + 1):
                if ox * ox + oy * oy < sep_sq:
                    offsets.append((ox, oy))
        self._sep_offsets: Tuple[GridPoint, ...] = tuple(offsets) if offsets else ((0, 0),)

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
        """
        Admissible lower bound on cost-to-go from `(pos, dir_idx)` to `target`.
        If both dx != 0 and dy != 0, at least one 90-degree turn is required.
        If only one axis remains but current heading `dir_idx` is not aligned with that axis,
        at least one turn is also required.
        """
        dx = target[0] - pos[0]
        dy = target[1] - pos[1]
        base = float(abs(dx) + abs(dy))
        if self.turn_penalty <= 0.0 or (dx == 0 and dy == 0):
            return base
        if dx != 0 and dy != 0:
            return base + self.turn_penalty
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
    def count_turns(path: Sequence[GridPoint]) -> int:
        """Count the number of heading changes along a discrete grid trajectory (ignoring waits)."""
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

    @classmethod
    def _count_turns(cls, path: Sequence[GridPoint]) -> int:
        return cls.count_turns(path)

    def path_cost(self, path: Sequence[GridPoint]) -> float:
        """Compute single-agent path cost `(len(path) - 1) + turn_penalty * turns`."""
        if not path:
            return float("inf")
        return float(len(path) - 1) + self.turn_penalty * float(self.count_turns(path))

    def detect_conflicts(
        self, paths: Dict[str, List[GridPoint]]
    ) -> Tuple[int, Optional[SpaceTimeConflict]]:
        """
        Count total pairwise space-time conflicts and return the earliest conflict.
        Uses spatial hash tables per timestep for O(T * N) detection when `min_separation == 1.0`.
        """
        if not paths:
            return 0, None
        horizon = max((len(p) for p in paths.values()), default=1)
        total_conflicts = 0
        first_conflict: Optional[SpaceTimeConflict] = None
        agent_list = [aid for aid in self.agent_ids if aid in paths]

        if len(self._sep_offsets) == 1:
            # Fast O(T * N) spatial hash lookup for standard unit separation
            for t in range(horizon):
                occ: Dict[GridPoint, str] = {}
                for aid in agent_list:
                    pos_t = self._pos_at(paths[aid], t)
                    prev_aid = occ.get(pos_t)
                    if prev_aid is not None:
                        total_conflicts += 1
                        if first_conflict is None:
                            first_conflict = SpaceTimeConflict(
                                time_step=t,
                                agent_a=prev_aid,
                                agent_b=aid,
                                pos_a=pos_t,
                                pos_b=pos_t,
                                is_edge=False,
                            )
                    else:
                        occ[pos_t] = aid

                if t + 1 < horizon:
                    moves: Dict[Tuple[GridPoint, GridPoint], str] = {}
                    for aid in agent_list:
                        p0 = self._pos_at(paths[aid], t)
                        p1 = self._pos_at(paths[aid], t + 1)
                        if p0 == p1:
                            continue
                        rev_aid = moves.get((p1, p0))
                        if rev_aid is not None:
                            total_conflicts += 1
                            if first_conflict is None:
                                first_conflict = SpaceTimeConflict(
                                    time_step=t + 1,
                                    agent_a=rev_aid,
                                    agent_b=aid,
                                    pos_a=p1,
                                    pos_b=p0,
                                    is_edge=True,
                                )
                        else:
                            moves[(p0, p1)] = aid
            return total_conflicts, first_conflict

        # General separation radius case (`min_separation > 1.0`)
        sep_sq = (self.min_separation - 1e-6) ** 2
        for t in range(horizon):
            for idx_a in range(len(agent_list)):
                aid_a = agent_list[idx_a]
                path_a = paths[aid_a]
                pos_a_t = self._pos_at(path_a, t)
                pos_a_next = self._pos_at(path_a, t + 1)

                for idx_b in range(idx_a + 1, len(agent_list)):
                    aid_b = agent_list[idx_b]
                    path_b = paths[aid_b]
                    pos_b_t = self._pos_at(path_b, t)
                    pos_b_next = self._pos_at(path_b, t + 1)

                    dx = pos_a_t[0] - pos_b_t[0]
                    dy = pos_a_t[1] - pos_b_t[1]
                    if dx * dx + dy * dy < sep_sq:
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
                    if (
                        t + 1 < horizon
                        and pos_a_t == pos_b_next
                        and pos_b_t == pos_a_next
                        and pos_a_t != pos_a_next
                    ):
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

    def _detect_conflicts(
        self, paths: Dict[str, List[GridPoint]]
    ) -> Tuple[int, Optional[SpaceTimeConflict]]:
        return self.detect_conflicts(paths)

    def _focal_low_level_astar(
        self,
        agent_id: str,
        constraints: Set[SpaceTimeConstraint],
        other_paths: Dict[str, List[GridPoint]],
    ) -> Tuple[Optional[List[GridPoint]], float, int, Dict[GridPoint, Node]]:
        """
        Low-level Two-Queue Focal Space-Time A* with full OPEN and FOCAL heaps.

        Maintains:
            - `open_heap`: ordered by `(f, g_tie, seq_id, state)` to track `f_min = min_{n in OPEN} f(n)`
            - `focal_heap`: ordered by `(h_c, f, g_tie, seq_id, state)` containing ALL active states
              in `OPEN` that satisfy `f(n) <= w * f_min`.
        Whenever `f_min` increases, all newly eligible states in `OPEN` are added to `focal_heap`.
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

        # Precompute Conflict Avoidance Table (CAT) from other agents' trajectories
        cat_vertex: Dict[Tuple[int, GridPoint], int] = {}
        cat_edge: Dict[Tuple[int, GridPoint, GridPoint], int] = {}
        if self.w > 1.0 and other_paths:
            cat_horizon = max((len(p) for p in other_paths.values()), default=1) + 4
            for other_id, opath in other_paths.items():
                if other_id == agent_id or not opath:
                    continue
                for t in range(cat_horizon):
                    p_cur = self._pos_at(opath, t)
                    for ox, oy in self._sep_offsets:
                        nb = (p_cur[0] + ox, p_cur[1] + oy)
                        cat_vertex[(t, nb)] = cat_vertex.get((t, nb), 0) + 1
                    p_nxt = self._pos_at(opath, t + 1)
                    if p_nxt != p_cur:
                        cat_edge[(t + 1, p_nxt, p_cur)] = cat_edge.get((t + 1, p_nxt, p_cur), 0) + 1

        start_pt = stops[0]
        h0 = float(
            self._turn_aware_h(start_pt, -1, stops[initial_stage])
            + remaining_leg[initial_stage]
        )
        start_state: LowLevelState = (start_pt, 0, initial_stage, -1)

        seq = itertools.count()
        # Standard A* tie-breaking: prefer larger g (deeper progress toward goal) on equal f
        g_sign = -1.0
        s0 = next(seq)

        # Active OPEN table: state -> (f_val, g_tie, hc_val, seq_id, g_val)
        open_entries: Dict[LowLevelState, Tuple[float, float, int, int, float]] = {
            start_state: (h0, 0.0, 0, s0, 0.0)
        }
        open_heap: List[Tuple[float, float, int, LowLevelState]] = [
            (h0, 0.0, s0, start_state)
        ]
        focal_heap: List[Tuple[int, float, float, int, LowLevelState]] = [
            (0, h0, 0.0, s0, start_state)
        ]
        in_focal: Set[int] = {s0}

        best_cost: Dict[LowLevelState, float] = {start_state: 0.0}
        came_from: Dict[LowLevelState, LowLevelState] = {}
        expanded_2d: Dict[GridPoint, Node] = (
            {start_pt: Node(start_pt, None, 0.0, h0)} if self.record_expanded else {}
        )
        expanded_count = 0
        f_min = h0

        def clean_open_top() -> Optional[float]:
            while open_heap:
                f_top, _, seq_top, st_top = open_heap[0]
                cur = open_entries.get(st_top)
                if cur is not None and cur[3] == seq_top:
                    return f_top
                heapq.heappop(open_heap)
            return None

        def clean_focal_top() -> Optional[Tuple[int, float, float, int, LowLevelState]]:
            while focal_heap:
                top = focal_heap[0]
                seq_top, st_top = top[3], top[4]
                cur = open_entries.get(st_top)
                if cur is not None and cur[3] == seq_top:
                    return heapq.heappop(focal_heap)
                heapq.heappop(focal_heap)
            return None

        while open_entries:
            cur_f_min = clean_open_top()
            if cur_f_min is None:
                break

            if cur_f_min > f_min + 1e-9:
                f_min = cur_f_min
                if self.w > 1.0:
                    bound = self.w * f_min + 1e-9
                    for st, (f_v, g_t, hc_v, s_id, _) in open_entries.items():
                        if s_id not in in_focal and f_v <= bound:
                            in_focal.add(s_id)
                            heapq.heappush(focal_heap, (hc_v, f_v, g_t, s_id, st))

            if self.w > 1.0:
                chosen = clean_focal_top()
                if chosen is None:
                    # Populate focal with current f_min bound
                    bound = self.w * f_min + 1e-9
                    for st, (f_v, g_t, hc_v, s_id, _) in open_entries.items():
                        if s_id not in in_focal and f_v <= bound:
                            in_focal.add(s_id)
                            heapq.heappush(focal_heap, (hc_v, f_v, g_t, s_id, st))
                    chosen = clean_focal_top()
                    if chosen is None:
                        break
                hc_val, f_val, _, seq_id, state = chosen
                g_val = open_entries.pop(state)[4]
            else:
                f_val, _, seq_id, state = heapq.heappop(open_heap)
                entry = open_entries.pop(state, None)
                if entry is None:
                    continue
                hc_val, g_val = entry[2], entry[4]

            pos, t, stage, dir_idx = state
            expanded_count += 1

            if stage == final_stage and pos == goal_pt and t >= max_constraint_t_at_goal:
                rev_path = [pos]
                cur = state
                while cur in came_from:
                    cur = came_from[cur]
                    rev_path.append(cur[0])
                rev_path.reverse()
                return rev_path, float(f_min), expanded_count, expanded_2d

            if t >= self.max_time:
                continue

            next_t = t + 1
            bound = self.w * f_min + 1e-9
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
                nxt_state: LowLevelState = (nxt_pos, next_t, nxt_stage, nxt_dir)

                if nxt_g + 1e-9 >= best_cost.get(nxt_state, float("inf")):
                    continue

                best_cost[nxt_state] = nxt_g
                came_from[nxt_state] = state

                if self.record_expanded and nxt_pos not in expanded_2d:
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

                nxt_f = nxt_g + h_rem
                nxt_gtie = g_sign * nxt_g
                nxt_hc = (
                    hc_val
                    + cat_vertex.get((next_t, nxt_pos), 0)
                    + cat_edge.get((next_t, pos, nxt_pos), 0)
                )
                s_id = next(seq)
                open_entries[nxt_state] = (nxt_f, nxt_gtie, nxt_hc, s_id, nxt_g)
                heapq.heappush(open_heap, (nxt_f, nxt_gtie, s_id, nxt_state))
                if self.w > 1.0 and nxt_f <= bound:
                    in_focal.add(s_id)
                    heapq.heappush(
                        focal_heap, (nxt_hc, nxt_f, nxt_gtie, s_id, nxt_state)
                    )

        return None, float("inf"), expanded_count, expanded_2d

    def plan(self) -> Tuple[Dict[str, List[GridPoint]], Dict[str, Any]]:
        """
        Run ECBS multi-agent space-time planning with full high-level OPEN and FOCAL queues.

        Returns:
            paths: Dict mapping `agent_id -> [(x0, y0), (x1, y1), ...]`.
            info: Dict of planning diagnostics (expanded high/low-level nodes, turns, lower_bound,
                  soc, suboptimality_ratio, runtime, etc.).
        """
        t_start = time.perf_counter()
        root_paths: Dict[str, List[GridPoint]] = {}
        root_costs: Dict[str, float] = {}
        root_fmin: Dict[str, float] = {}
        root_turns: Dict[str, int] = {}
        expanded_map_union: Dict[GridPoint, Node] = {}
        total_low_level_expanded = 0

        for aid in self.agent_ids:
            path, fmin, exp_cnt, exp_nodes = self._focal_low_level_astar(
                aid, set(), root_paths
            )
            total_low_level_expanded += exp_cnt
            if self.record_expanded:
                expanded_map_union.update(exp_nodes)
            if path is None:
                return {}, {
                    "success": False,
                    "reason": f"no_initial_path:{aid}",
                    "high_level_expanded": 0,
                    "low_level_expanded": total_low_level_expanded,
                    "planning_time_ms": (time.perf_counter() - t_start) * 1000.0,
                }
            root_paths[aid] = path
            turns_i = self.count_turns(path)
            root_turns[aid] = turns_i
            root_costs[aid] = float(len(path) - 1) + self.turn_penalty * float(turns_i)
            root_fmin[aid] = float(fmin)

        conflict_cnt, first_conf = self.detect_conflicts(root_paths)
        root_cost = sum(root_costs.values())
        root_lb = sum(root_fmin.values())

        root_node = _CTNode(
            cost=root_cost,
            lower_bound=root_lb,
            conflict_count=conflict_cnt,
            first_conflict=first_conf,
            paths=root_paths,
            low_level_costs=root_costs,
            low_level_fmin=root_fmin,
            low_level_turns=root_turns,
            constraints=set(),
            expanded_low_level=total_low_level_expanded,
        )

        seq = itertools.count()
        root_id = next(seq)
        active_ct: Dict[int, _CTNode] = {root_id: root_node}
        # High-level OPEN ordered by (lower_bound, conflict_count, cost, node_id)
        open_ct: List[Tuple[float, int, float, int]] = [
            (root_node.lower_bound, root_node.conflict_count, root_node.cost, root_id)
        ]
        # High-level FOCAL ordered by (conflict_count, cost, lower_bound, node_id)
        focal_ct: List[Tuple[int, float, float, int]] = [
            (root_node.conflict_count, root_node.cost, root_node.lower_bound, root_id)
        ]
        in_focal_ct: Set[int] = {root_id}
        min_lb = root_node.lower_bound
        high_level_expanded = 0

        def clean_open_ct_top() -> Optional[float]:
            while open_ct:
                lb_top, _, _, nid_top = open_ct[0]
                if nid_top in active_ct:
                    return lb_top
                heapq.heappop(open_ct)
            return None

        def refill_focal_ct(bound_val: float) -> None:
            for nid, node in active_ct.items():
                if nid not in in_focal_ct and node.cost <= bound_val + 1e-9:
                    in_focal_ct.add(nid)
                    heapq.heappush(
                        focal_ct,
                        (node.conflict_count, node.cost, node.lower_bound, nid),
                    )

        def pop_focal_ct() -> Optional[_CTNode]:
            while focal_ct:
                _, _, _, nid = heapq.heappop(focal_ct)
                node = active_ct.pop(nid, None)
                if node is not None:
                    return node
            return None

        while active_ct and high_level_expanded < self.max_high_level_nodes:
            cur_min_lb = clean_open_ct_top()
            if cur_min_lb is None:
                break

            if cur_min_lb > min_lb + 1e-9:
                min_lb = cur_min_lb
                if self.w > 1.0:
                    refill_focal_ct(self.w * min_lb)

            if self.w > 1.0:
                current = pop_focal_ct()
                if current is None:
                    refill_focal_ct(self.w * min_lb)
                    current = pop_focal_ct()
                    if current is None:
                        break
            else:
                _, _, _, nid = heapq.heappop(open_ct)
                current = active_ct.pop(nid, None)
                if current is None:
                    continue

            high_level_expanded += 1
            conflict = current.first_conflict
            if conflict is None:
                elapsed_ms = (time.perf_counter() - t_start) * 1000.0
                total_steps = sum(len(p) - 1 for p in current.paths.values())
                total_turns = sum(current.low_level_turns.values())
                global_lb = max(root_lb, min_lb)
                return current.paths, {
                    "success": True,
                    "planner": str(self),
                    "w": self.w,
                    "turn_penalty": self.turn_penalty,
                    "high_level_expanded": high_level_expanded,
                    "low_level_expanded": total_low_level_expanded,
                    "total_steps": total_steps,
                    "total_turns": total_turns,
                    "soc": current.cost,
                    "cost": current.cost,
                    "lower_bound": global_lb,
                    "suboptimality_ratio": current.cost / max(global_lb, 1e-9),
                    "makespan": max((len(p) - 1 for p in current.paths.values()), default=0),
                    "planning_time_ms": elapsed_ms,
                    "expand": expanded_map_union,
                }

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
                if self.record_expanded:
                    expanded_map_union.update(exp_nodes)
                if new_path is None:
                    continue

                next_paths = dict(current.paths)
                next_paths[constrained_agent] = new_path
                next_fmin = dict(current.low_level_fmin)
                next_fmin[constrained_agent] = float(new_fmin)
                next_turns = dict(current.low_level_turns)
                turns_new = self.count_turns(new_path)
                next_turns[constrained_agent] = turns_new
                next_costs = dict(current.low_level_costs)
                next_costs[constrained_agent] = (
                    float(len(new_path) - 1) + self.turn_penalty * float(turns_new)
                )

                c_cnt, first_c = self.detect_conflicts(next_paths)
                c_cost = sum(next_costs.values())
                c_lb = sum(next_fmin.values())
                child = _CTNode(
                    cost=c_cost,
                    lower_bound=c_lb,
                    conflict_count=c_cnt,
                    first_conflict=first_c,
                    paths=next_paths,
                    low_level_costs=next_costs,
                    low_level_fmin=next_fmin,
                    low_level_turns=next_turns,
                    constraints=next_constraints,
                    expanded_low_level=exp_cnt,
                )
                cid = next(seq)
                active_ct[cid] = child
                heapq.heappush(
                    open_ct,
                    (child.lower_bound, child.conflict_count, child.cost, cid),
                )
                if self.w > 1.0 and child.cost <= self.w * min_lb + 1e-9:
                    in_focal_ct.add(cid)
                    heapq.heappush(
                        focal_ct,
                        (child.conflict_count, child.cost, child.lower_bound, cid),
                    )

        elapsed_ms = (time.perf_counter() - t_start) * 1000.0
        return {}, {
            "success": False,
            "reason": "high_level_limit_exceeded" if high_level_expanded >= self.max_high_level_nodes else "open_exhausted",
            "high_level_expanded": high_level_expanded,
            "low_level_expanded": total_low_level_expanded,
            "planning_time_ms": elapsed_ms,
            "expand": expanded_map_union,
        }


class CBS(ECBS):
    """Exact Conflict-Based Search (`w = 1.0, turn_penalty = 0.0`)."""

    def __init__(
        self,
        map_: Grid,
        starts: Dict[str, GridPoint],
        goals: Dict[str, GridPoint],
        **kwargs,
    ) -> None:
        kwargs.setdefault("w", 1.0)
        kwargs.setdefault("turn_penalty", 0.0)
        super().__init__(map_=map_, starts=starts, goals=goals, **kwargs)

    def __str__(self) -> str:
        if self.turn_penalty > 0.0:
            return f"CBS(w=1.00, turn_penalty={self.turn_penalty:.2f})"
        return "CBS(exact)"
