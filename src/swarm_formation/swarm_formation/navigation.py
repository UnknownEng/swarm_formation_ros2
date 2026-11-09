# Copyright 2026 Mansoor Ahmed Rind
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Per-robot navigation layer: route around static obstacles toward the formation target.

The formation controller and the CBF are purely reactive: a robot whose formation slot
lies behind an obstacle cluster pushes straight at it and stops (in Gazebo, robot 5 was
trapped at a 0.66 m gap the CBF cannot pass). This layer gives each robot a route:

* the known static map (world_model) is rasterised once into an occupancy grid, with
  every obstacle inflated by the clearance the CBF needs, so corridors the CBF would
  block are blocked for the planner too;
* the *target* is where the formation controller wants the robot to be, computed from
  its own relative measurements: p_i + mean_j(p_ij - d_ij);
* if the straight line to that target is free, nothing changes (pure formation control);
  otherwise the robot follows an A* route, steering at a carrot point along it.

Each robot plans only for itself with its own EKF position estimate and the map; other
robots are handled by the CBF, not by the planner.
"""

import heapq
import math
from typing import Dict, List, Optional, Tuple

from .world_model import BOXES, closest_point_on_box, closest_point_on_oriented_box
from .world_model import OBSTACLES, WALLS

Point = Tuple[float, float]


def clearance_to_static_world(x: float, y: float) -> float:
    """Distance from (x, y) to the nearest static obstacle or wall surface (<0 inside)."""
    best = float('inf')
    for oid, (ox, oy, rad) in OBSTACLES.items():
        if oid in BOXES:
            cx, cy = closest_point_on_oriented_box(x, y, BOXES[oid])
            d = math.hypot(cx - x, cy - y)
            if d == 0.0:
                d = -1e-3
        else:
            d = math.hypot(ox - x, oy - y) - rad
        best = min(best, d)
    for box in WALLS.values():
        cx, cy = closest_point_on_box(x, y, box)
        d = math.hypot(cx - x, cy - y)
        best = min(best, d if d > 0.0 else -1e-3)
    return best


def required_clearance(
    safety_dist_obs: float = 0.30,
    lookahead: float = 0.15,
    max_linear_vel: float = 1.4,
    delay_bound_sec: float = 0.05,
    slack: float = 0.05,
) -> float:
    """
    Clearance from obstacle surfaces the planner must keep so the CBF will let robots pass.

    The look-ahead CBF keeps its look-ahead point at least
    ``safety_dist_obs + lookahead + max_linear_vel * delay_bound_sec`` from every obstacle
    surface (compute_cbf_safe_twist). A robot driving along the centre line of a gap has
    its look-ahead point on that line too, so the planner must keep the same distance,
    plus ``slack`` for grid discretisation. A smaller inflation produces routes into gaps
    the CBF refuses to pass.
    """
    return safety_dist_obs + lookahead + max_linear_vel * delay_bound_sec + slack


class OccupancyGrid:
    """Inflated occupancy grid of the static world."""

    def __init__(
        self,
        inflation: float,
        resolution: float = 0.05,
        bounds: Tuple[float, float, float, float] = (-4.6, 4.6, -4.6, 4.6),
    ) -> None:
        """
        Rasterise the world.

        :param inflation: Required clearance from any surface, in meters (robot radius
            plus the CBF margins).
        :param resolution: Cell size in meters.
        :param bounds: (x_min, x_max, y_min, y_max) of the planning area.
        """
        self.inflation = inflation
        self.res = resolution
        self.x0, x1, self.y0, y1 = bounds
        self.nx = int(math.ceil((x1 - self.x0) / resolution))
        self.ny = int(math.ceil((y1 - self.y0) / resolution))
        self.blocked = [
            [clearance_to_static_world(*self.cell_center(ix, iy)) < inflation
             for iy in range(self.ny)]
            for ix in range(self.nx)
        ]

    def cell_center(self, ix: int, iy: int) -> Point:
        """World coordinates of a cell centre."""
        return self.x0 + (ix + 0.5) * self.res, self.y0 + (iy + 0.5) * self.res

    def cell_of(self, p: Point) -> Tuple[int, int]:
        """Grid cell containing a world point (clamped to the grid)."""
        ix = int((p[0] - self.x0) / self.res)
        iy = int((p[1] - self.y0) / self.res)
        return min(max(ix, 0), self.nx - 1), min(max(iy, 0), self.ny - 1)

    def is_free(self, p: Point) -> bool:
        """Return True if the point's cell is free."""
        ix, iy = self.cell_of(p)
        return not self.blocked[ix][iy]

    def nearest_free_cell(self, cell: Tuple[int, int], max_rings: int = 40) -> Optional[
        Tuple[int, int]
    ]:
        """Closest free cell to ``cell`` (searching outward ring by ring)."""
        cx, cy = cell
        if not self.blocked[cx][cy]:
            return cell
        for r in range(1, max_rings + 1):
            best = None
            best_d = float('inf')
            for ix in range(cx - r, cx + r + 1):
                for iy in (cy - r, cy + r) if abs(ix - cx) != r else range(cy - r, cy + r + 1):
                    if 0 <= ix < self.nx and 0 <= iy < self.ny and not self.blocked[ix][iy]:
                        d = (ix - cx) ** 2 + (iy - cy) ** 2
                        if d < best_d:
                            best, best_d = (ix, iy), d
            if best is not None:
                return best
        return None

    def segment_free(self, a: Point, b: Point, skip_ends: bool = True) -> bool:
        """
        Return True if the straight segment a -> b crosses no blocked cell.

        The robot may stand inside the inflated band of an obstacle (the CBF holds it
        close to the barrier) and its target may lie inside a band too (a formation slot
        near an obstacle). With ``skip_ends``, blocked samples are ignored only within
        each endpoint's own depth inside the band (inflation minus its clearance, plus
        one cell), so a blocked gap just after the start or just before the target still
        blocks the line.
        """
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        if length < 1e-9:
            return True
        steps = max(1, int(length / (self.res * 0.5)))
        skip_a = skip_b = 0.0
        if skip_ends:
            skip_a = max(0.0, self.inflation - clearance_to_static_world(*a)) + self.res
            skip_b = max(0.0, self.inflation - clearance_to_static_world(*b)) + self.res
        for k in range(steps + 1):
            t = k / steps
            dist_from_a = t * length
            if dist_from_a <= skip_a or (length - dist_from_a) <= skip_b:
                continue
            p = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
            if not self.is_free(p):
                return False
        return True

    def plan(self, start: Point, goal: Point) -> Optional[List[Point]]:
        """8-connected A* from start to goal; returns world waypoints or None."""
        s = self.nearest_free_cell(self.cell_of(start))
        g = self.nearest_free_cell(self.cell_of(goal))
        if s is None or g is None:
            return None
        diag = math.sqrt(2.0)
        moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, diag), (1, -1, diag), (-1, 1, diag), (-1, -1, diag)]

        def h(c: Tuple[int, int]) -> float:
            return math.hypot(c[0] - g[0], c[1] - g[1])

        open_heap: List[Tuple[float, float, Tuple[int, int]]] = [(h(s), 0.0, s)]
        came: Dict[Tuple[int, int], Tuple[int, int]] = {}
        cost: Dict[Tuple[int, int], float] = {s: 0.0}
        while open_heap:
            _, c_cost, cur = heapq.heappop(open_heap)
            if cur == g:
                cells = [cur]
                while cur in came:
                    cur = came[cur]
                    cells.append(cur)
                cells.reverse()
                return [self.cell_center(*c) for c in cells]
            if c_cost > cost.get(cur, float('inf')):
                continue
            for dx, dy, step in moves:
                nb = (cur[0] + dx, cur[1] + dy)
                if not (0 <= nb[0] < self.nx and 0 <= nb[1] < self.ny):
                    continue
                if self.blocked[nb[0]][nb[1]]:
                    continue
                if dx and dy and (self.blocked[cur[0] + dx][cur[1]]
                                  or self.blocked[cur[0]][cur[1] + dy]):
                    continue  # no corner cutting
                new_cost = c_cost + step
                if new_cost < cost.get(nb, float('inf')):
                    cost[nb] = new_cost
                    came[nb] = cur
                    heapq.heappush(open_heap, (new_cost + h(nb), new_cost, nb))
        return None


def fit_template_rotation(
    rel_displacements: Dict[int, Point],
    target_offsets: Dict[int, Point],
    active_mask: Optional[Dict[int, bool]] = None,
) -> float:
    """
    Best-fit rotation (rad) of the formation template onto the measured peers.

    Least-squares angle R minimising sum_j ||R d_ij + t - p_ij||^2 over translation and
    rotation; 0 with fewer than two peers. Used by the distance controller, which leaves
    the formation's orientation free.
    """
    pairs = [
        (rel_displacements[pid], target_offsets[pid])
        for pid in sorted(rel_displacements)
        if pid in target_offsets
        and (active_mask is None or active_mask.get(pid, True))
    ]
    if len(pairs) < 2:
        return 0.0
    count = len(pairs)
    rbx = sum(r[0] for r, _ in pairs) / count
    rby = sum(r[1] for r, _ in pairs) / count
    obx = sum(o[0] for _, o in pairs) / count
    oby = sum(o[1] for _, o in pairs) / count
    num = den = 0.0
    for (rx, ry), (ox, oy) in pairs:
        ox, oy, rx, ry = ox - obx, oy - oby, rx - rbx, ry - rby
        num += ox * ry - oy * rx
        den += ox * rx + oy * ry
    return math.atan2(num, den)


def formation_target(
    est_pos: Point,
    rel_displacements: Dict[int, Point],
    target_offsets: Dict[int, Point],
    active_mask: Optional[Dict[int, bool]] = None,
    consensus_gain: float = 1.0,
    anchor_slot: Optional[Point] = None,
    anchor_gain: float = 0.0,
    fit_rotation: bool = False,
) -> Optional[Point]:
    """
    World point where the formation (and optional anchor) forces would balance.

    The robot's slot is found by fitting the formation template to the measured peers:
    peer j should sit at R d_ij + t, and the robot's own slot is t. Without rotation
    (displacement and bearing control, which fix the formation's orientation) the fit is
    t = mean_j(p_ij - d_ij), the point where consensus balances. With ``fit_rotation``
    (distance control, which leaves the orientation free) R is the best-fit rotation, so
    the target does not depend on a common heading either.

    With an anchor pulling toward ``anchor_slot`` the linear forces k_p * n * (t - p) +
    g * (slot - p) vanish at p_i + (k_p * n * t + g * (slot - p_i)) / (k_p * n + g);
    routing toward that point lets the anchor move a robot around obstacles instead of
    pinning it against one. Returns None when there is neither a neighbor nor an anchor.
    """
    pairs = [
        (rel_displacements[pid], target_offsets[pid])
        for pid in sorted(rel_displacements)
        if pid in target_offsets
        and (active_mask is None or active_mask.get(pid, True))
    ]
    count = len(pairs)
    use_anchor = anchor_slot is not None and anchor_gain > 0.0
    if count == 0 and not use_anchor:
        return None
    tx = ty = 0.0
    if count > 0:
        rbx = sum(r[0] for r, _ in pairs) / count
        rby = sum(r[1] for r, _ in pairs) / count
        obx = sum(o[0] for _, o in pairs) / count
        oby = sum(o[1] for _, o in pairs) / count
        c, s_ = 1.0, 0.0
        if fit_rotation:
            theta = fit_template_rotation(rel_displacements, target_offsets, active_mask)
            c, s_ = math.cos(theta), math.sin(theta)
        tx = rbx - (c * obx - s_ * oby)
        ty = rby - (s_ * obx + c * oby)
    wx = consensus_gain * count * tx
    wy = consensus_gain * count * ty
    weight = consensus_gain * count
    if use_anchor:
        wx += anchor_gain * (anchor_slot[0] - est_pos[0])
        wy += anchor_gain * (anchor_slot[1] - est_pos[1])
        weight += anchor_gain
    return est_pos[0] + wx / weight, est_pos[1] + wy / weight


def anchor_slot_position(
    robot_id: int, total_robots: int, radius: float, center: Point, rotation: float = 0.0
) -> Point:
    """World position of a robot's slot around a fixed centre (pattern rotated by rotation)."""
    angle = 2.0 * math.pi * ((robot_id - 1) % total_robots) / total_robots + rotation
    return center[0] + radius * math.cos(angle), center[1] + radius * math.sin(angle)


class NavigationGuide:
    """Decides when to override formation control and where to steer."""

    def __init__(
        self,
        grid: OccupancyGrid,
        carrot_dist: float = 0.6,
        replan_period: float = 0.5,
        replan_goal_shift: float = 0.3,
    ) -> None:
        """
        Create a guide on a shared, read-only grid.

        :param carrot_dist: Look-ahead distance along the route for the steering point.
        :param replan_period: Minimum time between replans, in seconds.
        :param replan_goal_shift: Replan early if the target moved more than this (m).
        """
        self.grid = grid
        self.carrot_dist = carrot_dist
        self.replan_period = replan_period
        self.replan_goal_shift = replan_goal_shift
        self.path: Optional[List[Point]] = None
        self._goal: Optional[Point] = None
        self._plan_time = -1e9

    def steer(self, now: float, pos: Point, target: Optional[Point]) -> Optional[Point]:
        """
        Return a steering point, or None when formation control should act unchanged.

        :param now: Current time in seconds.
        :param pos: Robot's estimated world position.
        :param target: Formation target from :func:`formation_target`.
        """
        if target is None or self.grid.segment_free(pos, target):
            self.path = None
            return None
        stale = (
            self.path is None
            or self._goal is None
            or now - self._plan_time >= self.replan_period
            or math.hypot(target[0] - self._goal[0], target[1] - self._goal[1])
            > self.replan_goal_shift
        )
        if stale:
            self.path = self.grid.plan(pos, target)
            self._goal = target
            self._plan_time = now
        if not self.path:
            return None  # no route: fall back to formation control (CBF keeps it safe)
        # Carrot: the route point carrot_dist further along than the closest route point
        path = self.path
        k0 = min(
            range(len(path)),
            key=lambda k: (path[k][0] - pos[0]) ** 2 + (path[k][1] - pos[1]) ** 2,
        )
        carrot = path[-1]
        travelled = 0.0
        for k in range(k0 + 1, len(path)):
            travelled += math.hypot(path[k][0] - path[k - 1][0], path[k][1] - path[k - 1][1])
            if travelled >= self.carrot_dist:
                carrot = path[k]
                break
        return carrot


def navigation_force(
    formation_force: Point, pos: Point, steer_point: Point, min_magnitude: float = 0.4
) -> Point:
    """Point the formation force at the steering point, keeping (at least) its magnitude."""
    dx, dy = steer_point[0] - pos[0], steer_point[1] - pos[1]
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return formation_force
    mag = max(math.hypot(*formation_force), min_magnitude)
    return mag * dx / d, mag * dy / d
