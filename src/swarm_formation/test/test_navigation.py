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

"""Tests for the navigation layer (occupancy grid, A*, handover to formation control)."""

import math

import pytest

from swarm_formation.campaign_simulator import CampaignRunConfig, run_single_simulation
from swarm_formation.navigation import (
    clearance_to_static_world,
    formation_target,
    navigation_force,
    NavigationGuide,
    OccupancyGrid,
)
from swarm_formation.world_model import OBSTACLES


@pytest.fixture(scope='module')
def grid() -> OccupancyGrid:
    """Planning grid with the default inflation (CBF clearance)."""
    return OccupancyGrid(inflation=0.50)


def _gap_open(grid: OccupancyGrid, a: int, b: int) -> bool:
    ax, ay, _ = OBSTACLES[a]
    bx, by, _ = OBSTACLES[b]
    return any(grid.is_free((ax + (bx - ax) * k / 200, ay + (by - ay) * k / 200))
               for k in range(201))


def test_grid_blocks_gaps_the_cbf_cannot_pass(grid: OccupancyGrid) -> None:
    """The 0.66 m slab/rubble gap is closed; the 1.17 m slab/pillar gap stays open."""
    assert not _gap_open(grid, 4, 7)
    assert _gap_open(grid, 3, 2)


def test_plan_keeps_required_clearance(grid: OccupancyGrid) -> None:
    """Every waypoint of a route around the blocked gap keeps the inflation clearance."""
    path = grid.plan((2.2, -2.2), (0.3, -0.9))
    assert path is not None and len(path) > 2
    assert math.hypot(path[-1][0] - 0.3, path[-1][1] + 0.9) < 0.2
    for x, y in path:
        assert clearance_to_static_world(x, y) >= 0.50 - grid.res
    for a, b in zip(path, path[1:]):
        assert math.hypot(b[0] - a[0], b[1] - a[1]) <= grid.res * math.sqrt(2) + 1e-9


def test_guide_only_overrides_when_line_of_sight_is_blocked(grid: OccupancyGrid) -> None:
    """Open space: formation control unchanged. Blocked: a steering point on the route."""
    guide = NavigationGuide(grid)
    assert guide.steer(0.0, (-3.0, 3.0), (-2.0, 3.5)) is None
    steer = guide.steer(0.0, (2.2, -2.2), (0.3, -0.9))
    assert steer is not None
    assert grid.is_free(steer)
    assert 0.3 < math.hypot(steer[0] - 2.2, steer[1] + 2.2) < 1.0


def test_formation_target_and_navigation_force() -> None:
    """Target is p_i + mean(p_ij - d_ij); the force keeps magnitude, changes direction."""
    rel = {2: (1.0, 0.0), 3: (0.0, 1.0)}
    offsets = {2: (2.0, 0.0), 3: (0.0, 2.0)}
    tx, ty = formation_target((5.0, 5.0), rel, offsets)
    assert math.isclose(tx, 4.5) and math.isclose(ty, 4.5)
    assert formation_target((0.0, 0.0), {}, offsets) is None
    fx, fy = navigation_force((3.0, 4.0), (0.0, 0.0), (0.0, 2.0))
    assert math.isclose(fx, 0.0, abs_tol=1e-12) and math.isclose(fy, 5.0)
    fx, fy = navigation_force((0.0, 0.0), (0.0, 0.0), (1.0, 0.0), min_magnitude=0.4)
    assert math.isclose(fx, 0.4) and math.isclose(fy, 0.0)


def test_navigation_does_not_change_open_space_runs() -> None:
    """Without obstacles the guide never overrides, so results are identical."""
    base = run_single_simulation(CampaignRunConfig(duration_sec=10.0, use_cbf=True), 77)
    nav = run_single_simulation(
        CampaignRunConfig(duration_sec=10.0, use_cbf=True, use_navigation=True), 77
    )
    # The walls are far from the open-space formation, so line of sight stays clear
    assert nav['final_shape_error'] == pytest.approx(base['final_shape_error'], abs=0.05)


def test_navigation_keeps_arena_runs_safe_and_converging() -> None:
    """With obstacles, walls and navigation the swarm converges without touching anything."""
    res = run_single_simulation(
        CampaignRunConfig(duration_sec=30.0, use_cbf=True, use_world_obstacles=True,
                          use_navigation=True), 1042,
    )
    assert res['obstacle_collision_free'] and res['empirically_collision_free']
    assert res['is_converged']


def test_anchored_formation_holds_its_place_in_the_arena() -> None:
    """Deployed defaults (CBF + navigation + anchor at the spawn centroid) stop the drift."""
    from swarm_formation.world_model import DEFAULT_FORMATION_CENTER as center

    for seed in (1042, 4042):
        res = run_single_simulation(
            CampaignRunConfig(duration_sec=30.0, use_cbf=True, use_world_obstacles=True,
                              use_navigation=True, anchor_gain=0.4), seed,
        )
        assert res['is_converged'] and res['final_shape_error'] < 0.1
        assert res['obstacle_collision_free'] and res['empirically_collision_free']
        cx, cy = res['final_centroid']
        assert math.hypot(cx - center[0], cy - center[1]) < 0.3


def test_planner_clearance_matches_cbf_needs() -> None:
    """Default clearance covers the CBF requirement and grows with the delay bound."""
    from swarm_formation.navigation import required_clearance

    base = required_clearance()
    assert base == pytest.approx(0.30 + 0.15 + 1.4 * 0.05 + 0.05)
    assert required_clearance(delay_bound_sec=0.12) > base
    # The 1.165 m slab/pillar gap is passable at defaults but not under 120 ms delay
    open_grid = OccupancyGrid(inflation=base)
    closed_grid = OccupancyGrid(inflation=required_clearance(delay_bound_sec=0.12))
    assert _gap_open(open_grid, 3, 2)
    assert not _gap_open(closed_grid, 3, 2)


def test_line_of_sight_skips_are_bounded_at_both_ends(grid: OccupancyGrid) -> None:
    """Endpoints inside an obstacle band are tolerated; a gap next to them is not."""
    # Target slightly inside the band of slab 4, approached from open space: free
    bx, by, _ = OBSTACLES[4]
    near_slab = (bx, by - 0.25 - 0.40)          # ~0.40 m below the slab's lower face
    assert not grid.is_free(near_slab)
    assert grid.segment_free((bx, by - 2.5), near_slab)
    # From the robot-5 trap spot through the closed slab/rubble gap: still blocked
    assert not grid.segment_free((1.06, -1.35), (0.3, -0.4))


def test_impaired_arena_converges_with_derived_clearance() -> None:
    """Delay + obstacles failed when the planner clearance ignored the CBF delay margin."""
    res = run_single_simulation(
        CampaignRunConfig(duration_sec=40.0, use_cbf=True, use_world_obstacles=True,
                          use_navigation=True, anchor_gain=0.4, twist_mapper='calm',
                          drop_rate=0.15, delay_sec=0.08, jitter_sec=0.04,
                          range_noise_std=0.05, bearing_noise_std=0.02), 1042,
    )
    assert res['is_converged'] and res['final_shape_error'] < 0.25
    assert res['obstacle_collision_free'] and res['empirically_collision_free']


def test_delay_compensation_rescues_a_200ms_arena_run() -> None:
    """Seed 1042 at 200 ms delay failed without compensation and converges with it."""
    kw = {
        'duration_sec': 40.0, 'use_cbf': True, 'use_world_obstacles': True,
        'use_navigation': True, 'anchor_gain': 0.4, 'twist_mapper': 'calm', 'delay_sec': 0.2,
    }
    without = run_single_simulation(CampaignRunConfig(**kw), 1042)
    with_comp = run_single_simulation(CampaignRunConfig(delay_compensation=True, **kw), 1042)
    assert without['final_shape_error'] > 0.25
    assert with_comp['is_converged'] and with_comp['final_shape_error'] < 0.15
    assert with_comp['empirically_collision_free'] and with_comp['obstacle_collision_free']
