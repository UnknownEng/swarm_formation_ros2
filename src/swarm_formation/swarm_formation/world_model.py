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
Single source of truth for the static geometry of worlds/collapsed_building.sdf.

Avoidance uses the exact collision shapes: pillars are circles, slabs and rubble blocks
are oriented boxes, and boundary walls are axis-aligned boxes. For every box a robot
senses the closest point of the box surface as a zero-radius obstacle. (Bounding circles
were too conservative: they closed the gaps between neighbouring obstacles and trapped
robots in Gazebo.) Obstacle centres are separately available as localization landmarks.
"""

import math
from typing import Dict, List, Tuple

# obstacle_id -> (x, y, bounding_radius): landmark centres (IDs 1..7) and coarse bounds.
OBSTACLES: Dict[int, Tuple[float, float, float]] = {
    1: (-1.66, 1.40, 0.15),   # steel_pillar_1, cylinder r 0.15
    2: (1.92, 1.87, 0.15),    # steel_pillar_2, cylinder r 0.15
    3: (0.25, 1.96, 0.43),    # fallen_slab_1, box 0.50 x 0.70
    4: (0.95, -0.33, 0.43),   # fallen_slab_2, box 0.70 x 0.50
    5: (-1.80, -1.09, 0.25),  # rubble_block_1, box 0.35 x 0.35
    6: (-0.12, -1.89, 0.25),  # rubble_block_2, box 0.35 x 0.35
    7: (1.87, -1.24, 0.25),   # rubble_block_3, box 0.35 x 0.35
}

# Exact oriented-box collision shapes: obstacle_id -> (x, y, size_x, size_y, yaw).
# Obstacles not listed here (the pillars) are true circles.
BOXES: Dict[int, Tuple[float, float, float, float, float]] = {
    3: (0.25, 1.96, 0.50, 0.70, 0.45),
    4: (0.95, -0.33, 0.70, 0.50, -0.38),
    5: (-1.80, -1.09, 0.35, 0.35, 0.72),
    6: (-0.12, -1.89, 0.35, 0.35, -0.55),
    7: (1.87, -1.24, 0.35, 0.35, 0.28),
}

# wall_id -> (x_min, x_max, y_min, y_max) of the collision box
WALLS: Dict[int, Tuple[float, float, float, float]] = {
    101: (-5.0, 5.0, 4.6, 5.0),    # boundary_north
    102: (-5.0, 5.0, -5.0, -4.6),  # boundary_south
    103: (4.6, 5.0, -5.0, 5.0),    # boundary_east
    104: (-5.0, -4.6, -5.0, 5.0),  # boundary_west
}

# Formation centre: the centroid of the launch spawn points, so the robots travel the
# least and do not have to cross the rubble field. Every slot keeps >= 0.52 m from all
# obstacle surfaces. (The max-clearance centre (-1.0, -1.7) forced robots through the
# rubble and left the formation 1.1 m off target; the arena centre (0, 0) would put a
# slot inside a slab.)
DEFAULT_FORMATION_CENTER: Tuple[float, float] = (0.44, -0.44)


def closest_point_on_box(
    px: float, py: float, box: Tuple[float, float, float, float]
) -> Tuple[float, float]:
    """Return the point of an axis-aligned box closest to (px, py)."""
    x_min, x_max, y_min, y_max = box
    return min(max(px, x_min), x_max), min(max(py, y_min), y_max)


def closest_point_on_oriented_box(
    px: float, py: float, box: Tuple[float, float, float, float, float]
) -> Tuple[float, float]:
    """Return the point of an oriented box (x, y, size_x, size_y, yaw) closest to (px, py)."""
    bx, by, sx, sy, yaw = box
    c, s = math.cos(yaw), math.sin(yaw)
    lx = c * (px - bx) + s * (py - by)
    ly = -s * (px - bx) + c * (py - by)
    lx = min(max(lx, -sx / 2.0), sx / 2.0)
    ly = min(max(ly, -sy / 2.0), sy / 2.0)
    return bx + c * lx - s * ly, by + s * lx + c * ly


def static_obstacles_in_range(
    px: float, py: float, sensing_range: float
) -> List[Tuple[int, float, float, float]]:
    """
    List static obstacles a robot at (px, py) can sense.

    :return: List of (obstacle_id, x, y, radius) in the world frame for collision
        avoidance. Pillars are returned as centre plus radius; every box (slab, rubble
        block, wall) within range is returned as its closest surface point, radius 0.
    """
    found: List[Tuple[int, float, float, float]] = []
    for oid, (ox, oy, rad) in OBSTACLES.items():
        if oid in BOXES:
            cx, cy = closest_point_on_oriented_box(px, py, BOXES[oid])
            if math.hypot(cx - px, cy - py) <= sensing_range:
                found.append((oid, cx, cy, 0.0))
        elif math.hypot(ox - px, oy - py) - rad <= sensing_range:
            found.append((oid, ox, oy, rad))
    for wid, box in WALLS.items():
        cx, cy = closest_point_on_box(px, py, box)
        if math.hypot(cx - px, cy - py) <= sensing_range:
            found.append((wid, cx, cy, 0.0))
    return found


def landmarks_in_range(
    px: float, py: float, sensing_range: float
) -> List[Tuple[int, float, float]]:
    """List landmark centres (obstacle_id, x, y) a robot at (px, py) can detect."""
    return [
        (oid, ox, oy) for oid, (ox, oy, rad) in OBSTACLES.items()
        if math.hypot(ox - px, oy - py) - rad <= sensing_range
    ]


def landmark_positions() -> Dict[int, Tuple[float, float]]:
    """Return obstacle centres usable as known localization landmarks."""
    return {oid: (x, y) for oid, (x, y, _) in OBSTACLES.items()}
