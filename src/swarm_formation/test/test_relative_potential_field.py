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

"""Unit tests for relative artificial potential field (APF) modules."""

from swarm_formation.potential_field import (
    compute_peer_safety_scale,
    compute_relative_obstacle_apf,
    compute_relative_peer_apf,
)


def test_relative_peer_apf_zero_beyond_influence() -> None:
    """Verify zero APF repulsion when all relative peers are beyond influence distance."""
    # Peer at relative displacement dx=2.5m (d_infl = 1.80m)
    relative_peers = {2: (2.5, 0.0, 0.0, 0.0)}
    fx, fy, min_d = compute_relative_peer_apf(relative_peers, d_infl=1.80)
    assert abs(fx) < 1e-6
    assert abs(fy) < 1e-6
    assert abs(min_d - 2.5) < 1e-6


def test_relative_peer_apf_repulsion_direction() -> None:
    """Verify repulsive force pushes robot away from peer."""
    # Peer is at relative displacement dx=1.0m, dy=0.0m (East of robot)
    # The repulsive force on this robot should push it West (fx < 0)
    relative_peers = {2: (1.0, 0.0, 0.0, 0.0)}
    fx, fy, min_d = compute_relative_peer_apf(
        relative_peers,
        d_infl=1.80,
        d_safe=1.10,
        k_rep=1.0,
        lateral_ratio=0.0,
    )
    assert fx < 0.0, f'Expected fx < 0, got {fx}'
    assert abs(fy) < 1e-6
    assert abs(min_d - 1.0) < 1e-6


def test_relative_obstacle_apf_repulsion() -> None:
    """Verify obstacle APF repulsion and tangent guidance."""
    # Obstacle center at relative dx=0.35m, dy=0.0m, radius 0.15m (clearance 0.20m < 0.45m)
    relative_obstacles = [(0.35, 0.0, 0.15)]
    fx, fy = compute_relative_obstacle_apf(
        relative_obstacles,
        vel_x=0.0,
        vel_y=0.0,
        fx_guide=1.0,
        fy_guide=0.0,
        d_obs_infl=0.45,
        d_obs_safe=0.30,
    )
    # Normal force must repel away from obstacle (push West, fx < 0)
    assert fx < 0.0, f'Expected fx < 0, got {fx}'


def test_peer_safety_scale_critical_zone() -> None:
    """Verify safety priority attenuation scaling."""
    # Above safe distance: scale is 1.0
    scale_safe = compute_peer_safety_scale(1.50, d_safe=1.10, d_collision=0.50)
    assert scale_safe == 1.0

    # At collision boundary: scale drops to minimum (0.08)
    scale_critical = compute_peer_safety_scale(0.50, d_safe=1.10, d_collision=0.50)
    assert abs(scale_critical - 0.08) < 1e-6
