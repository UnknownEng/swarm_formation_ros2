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

"""Tests for the static world model, the formation anchor, and drift prevention."""

import math
import os
import re

from swarm_formation.campaign_simulator import CampaignRunConfig, run_single_simulation
from swarm_formation.consensus import compute_anchor_force
from swarm_formation.world_model import (
    BOXES,
    closest_point_on_box,
    closest_point_on_oriented_box,
    DEFAULT_FORMATION_CENTER,
    landmarks_in_range,
    OBSTACLES,
    static_obstacles_in_range,
    WALLS,
)

WORLD = os.path.join(os.path.dirname(__file__), '..', 'worlds', 'collapsed_building.sdf')


def test_obstacle_radii_bound_the_collision_shapes() -> None:
    """Each modelled radius covers the whole collision shape in the SDF world."""
    sdf = open(WORLD, encoding='utf-8').read()
    shapes = []
    for name, body in re.findall(r'<model name="([^"]+)">(.*?)</model>', sdf, re.S):
        pose = re.search(r'<pose>([^<]+)</pose>', body)
        col = re.search(r'<collision.*?</collision>', body, re.S)
        if not pose or not col or name.startswith('boundary'):
            continue
        x, y = (float(v) for v in pose.group(1).split()[:2])
        box = re.search(r'<size>([^<]+)</size>', col.group(0))
        cyl = re.search(r'<radius>([^<]+)</radius>', col.group(0))
        if box:
            sx, sy, _ = (float(v) for v in box.group(1).split())
            shapes.append((x, y, math.hypot(sx, sy) / 2.0))
        elif cyl:
            shapes.append((x, y, float(cyl.group(1))))
    assert len(shapes) == len(OBSTACLES)
    for x, y, needed in shapes:
        match = [r for ox, oy, r in OBSTACLES.values() if math.hypot(ox - x, oy - y) < 1e-6]
        assert match, f'obstacle at ({x}, {y}) missing from world_model'
        assert match[0] >= needed - 0.005, f'radius {match[0]} < bounding {needed:.3f}'


def test_walls_are_sensed_near_the_boundary() -> None:
    """A robot near the east wall senses the wall's closest point on its inner face."""
    found = static_obstacles_in_range(4.0, 0.5, sensing_range=6.0)
    east = [f for f in found if f[0] == 103]
    assert east and math.isclose(east[0][1], 4.6) and math.isclose(east[0][2], 0.5)
    assert closest_point_on_box(0.0, 0.0, WALLS[101]) == (0.0, 4.6)
    # Far from every wall nothing is reported for walls beyond range
    near = static_obstacles_in_range(0.0, 0.0, sensing_range=1.0)
    assert all(f[0] < 100 for f in near)


def test_anchor_force_zero_at_slot_and_saturated_far_away() -> None:
    """The anchor vanishes at the robot's slot and is bounded far from it."""
    cx, cy = DEFAULT_FORMATION_CENTER
    slot_2 = (cx + 2.0 * math.cos(2 * math.pi / 5), cy + 2.0 * math.sin(2 * math.pi / 5))
    fx, fy = compute_anchor_force(2, 5, 2.0, (cx, cy), slot_2, gain=0.4)
    assert math.hypot(fx, fy) < 1e-9
    fx, fy = compute_anchor_force(2, 5, 2.0, (cx, cy), (50.0, 50.0), gain=0.4)
    assert math.isclose(math.hypot(fx, fy), 0.4, rel_tol=1e-9)
    assert compute_anchor_force(2, 5, 2.0, (cx, cy), (50.0, 50.0), gain=0.0) == (0.0, 0.0)


def test_anchor_stops_formation_drift() -> None:
    """Without the anchor the formation drifts; with it, it settles at the chosen centre."""
    def centroid_error(gain: float) -> float:
        res = run_single_simulation(
            CampaignRunConfig(duration_sec=25.0, use_cbf=True, anchor_gain=gain), 1042
        )
        assert res['is_converged'] and res['empirically_collision_free']
        cx, cy = res['final_centroid']
        return math.hypot(cx - DEFAULT_FORMATION_CENTER[0], cy - DEFAULT_FORMATION_CENTER[1])

    assert centroid_error(0.0) > 1.0
    assert centroid_error(0.4) < 0.3


def test_cbf_keeps_clear_of_walls_and_obstacles_in_the_arena() -> None:
    """With the real obstacles and walls sensed, the CBF stack never touches them."""
    for seed in (1042, 2042):
        res = run_single_simulation(
            CampaignRunConfig(duration_sec=30.0, use_cbf=True, use_world_obstacles=True), seed
        )
        assert res['obstacle_collision_free'], res['min_obstacle_clearance']
        assert res['empirically_collision_free']
        assert res['is_converged'], res['final_shape_error']


def test_oriented_box_closest_point_matches_geometry() -> None:
    """Closest point on a rotated box lies on its surface, along the box axes."""
    box = (1.0, 2.0, 0.5, 0.7, math.pi / 2)  # rotated 90 deg: size_x now spans world y
    cx, cy = closest_point_on_oriented_box(3.0, 2.0, box)
    assert math.isclose(cx, 1.35, abs_tol=1e-9) and math.isclose(cy, 2.0, abs_tol=1e-9)
    cx, cy = closest_point_on_oriented_box(1.0, 5.0, box)
    assert math.isclose(cx, 1.0, abs_tol=1e-9) and math.isclose(cy, 2.25, abs_tol=1e-9)


def test_boxes_sensed_as_surface_points_and_landmarks_as_centres() -> None:
    """Avoidance gets exact box surfaces; localization gets obstacle centres."""
    bx, by, sx, sy, yaw = BOXES[4]
    px, py = bx + 1.0, by
    sensed = {oid: (x, y, r) for oid, x, y, r in static_obstacles_in_range(px, py, 6.0)}
    x, y, r = sensed[4]
    assert r == 0.0
    expected = closest_point_on_oriented_box(px, py, BOXES[4])
    assert math.isclose(x, expected[0]) and math.isclose(y, expected[1])
    # surface is closer than the bounding circle would suggest, but not inside the box
    assert math.hypot(x - px, y - py) >= 1.0 - OBSTACLES[4][2]
    landmarks = {oid: (lx, ly) for oid, lx, ly in landmarks_in_range(px, py, 6.0)}
    assert landmarks[4] == (bx, by)
