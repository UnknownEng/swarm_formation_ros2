# Copyright 2026 Open Source Robotics Foundation, Inc.
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

"""Unit tests for Artificial Potential Field safety barrier functions."""

import math

from swarm_formation.potential_field import (
    compute_obstacle_apf,
    compute_peer_apf,
    compute_peer_safety_scale,
)


def test_peer_apf_outside_influence():
    """Verify zero repulsive force when peer distance is beyond d_infl."""
    # Distance is 2.5m > d_infl (1.80m)
    filtered_states = {2: (2.5, 0.0, 0.0, 0.0)}
    fx, fy, min_d = compute_peer_apf(
        0.0, 0.0, 0.0, 0.0, filtered_states, d_infl=1.80, d_safe=1.10
    )
    assert fx == 0.0
    assert fy == 0.0
    assert math.isclose(min_d, 2.5)


def test_peer_apf_repulsion_and_lateral_deflection():
    """Verify repulsive force and anti-symmetric lateral deflection for oncoming peer."""
    # Peer is at (1.0, 0.0), distance 1.0m (within safe range d_safe=1.10m)
    # nx_hat = (0.0 - 1.0)/1.0 = -1.0, ny_hat = 0.0
    # tx_peer = -ny_hat = 0.0, ty_peer = nx_hat = -1.0
    filtered_states = {2: (1.0, 0.0, 0.0, 0.0)}
    fx, fy, min_d = compute_peer_apf(
        0.0, 0.0, 0.0, 0.0, filtered_states, d_infl=1.80, d_safe=1.10
    )
    assert fx < 0.0  # Repelled away from peer along -x
    assert fy < 0.0  # Lateral deflection along -y (right deflection)
    assert math.isclose(min_d, 1.0)


def test_peer_safety_scale():
    """Verify formation force attenuation when peers are inside critical safe distance."""
    # Outside d_safe (1.10m) -> scale is 1.0
    assert math.isclose(compute_peer_safety_scale(1.50, d_safe=1.10, d_collision=0.50), 1.0)
    # Near collision limit (0.50m) -> scale is clamped to 0.08
    assert math.isclose(compute_peer_safety_scale(0.50, d_safe=1.10, d_collision=0.50), 0.08)


def test_obstacle_apf_tangent_circulation():
    """Verify repulsive and smooth tangential circulation forces around disaster obstacles."""
    obstacles = [(1.0, 0.0, 0.2)]  # Obstacle at (1.0, 0.0), radius 0.2m
    # Robot at (0.7, 0.0), clearance = 0.3 - 0.2 = 0.1m < d_obs_infl (0.45m)
    # nx = -1.0, ny = 0.0, tx = 0.0, ty = -1.0
    fx, fy = compute_obstacle_apf(
        0.7, 0.0, 0.0, 0.0, fx_guide=1.0, fy_guide=0.0, obstacles=obstacles
    )
    assert fx < 0.0  # Normal repulsion pushes away from obstacle
    assert fy != 0.0  # Tangential circulation deflects robot around obstacle
