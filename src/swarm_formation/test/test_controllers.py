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

"""Unit tests for swarm formation controllers (displacement, distance, bearing)."""

import math

import pytest
from swarm_formation.consensus import compute_target_offsets
from swarm_formation.controllers import (
    BearingController,
    compute_target_bearings,
    compute_target_distances,
    create_controller,
    DisplacementController,
    DistanceController,
)


def test_compute_target_geometry() -> None:
    """Verify target inter-agent distance and bearing geometry for a regular pentagon."""
    radius = 2.0
    distances = compute_target_distances(1, 5, radius)
    bearings = compute_target_bearings(1, 5, radius)

    assert len(distances) == 4
    assert len(bearings) == 4

    # Regular pentagon adjacent side length: 2 * R * sin(pi / 5) = 2 * 2.0 * sin(36 deg) ~ 2.3511m
    d_adjacent_expected = 2.0 * radius * math.sin(math.pi / 5.0)
    assert math.isclose(distances[2], d_adjacent_expected, rel_tol=1e-4)
    assert math.isclose(distances[5], d_adjacent_expected, rel_tol=1e-4)

    # Unit bearing vector norm must be 1.0
    for peer_id, (gx, gy, d) in bearings.items():
        assert math.isclose(math.hypot(gx, gy), 1.0, abs_tol=1e-6)
        assert math.isclose(d, distances[peer_id], abs_tol=1e-6)


def test_displacement_controller_equilibrium() -> None:
    """Verify displacement controller produces zero force at formation equilibrium."""
    controller = DisplacementController()
    target_offsets = compute_target_offsets(1, 5, 2.0)

    fx, fy = controller.compute_control_force(1, 5, 2.0, target_offsets, gain=1.8)
    assert math.isclose(fx, 0.0, abs_tol=1e-6)
    assert math.isclose(fy, 0.0, abs_tol=1e-6)


def test_displacement_controller_proportional_and_saturation() -> None:
    """Verify restoring force and Huber saturation for displacement controller."""
    controller = DisplacementController()
    target_offsets = compute_target_offsets(1, 5, 2.0)

    # Displace peer 2 by +1.0m in X
    displaced = dict(target_offsets)
    t2_x, t2_y = displaced[2]
    displaced[2] = (t2_x + 1.0, t2_y)

    # Within linear region (delta = 2.0)
    fx, fy = controller.compute_control_force(
        1, 5, 2.0, displaced, gain=1.8, huber_delta=2.0
    )
    assert math.isclose(fx, 1.8 * 1.0, abs_tol=1e-4)
    assert math.isclose(fy, 0.0, abs_tol=1e-4)

    # Saturated region (displace by +5.0m, delta = 1.0)
    displaced[2] = (t2_x + 5.0, t2_y)
    fx_sat, fy_sat = controller.compute_control_force(
        1, 5, 2.0, displaced, gain=1.8, huber_delta=1.0
    )
    assert math.isclose(fx_sat, 1.8 * 1.0, abs_tol=1e-4)


def test_distance_controller_restoring_force() -> None:
    """Verify distance controller pulls when too far and pushes when too close."""
    controller = DistanceController(orientation_gain_ratio=0.0)
    target_distances = compute_target_distances(1, 5, 2.0)
    d_star = target_distances[2]

    # Peer 2 is located along the X axis
    # 1. At equilibrium distance: force must be zero
    rel_eq = {2: (d_star, 0.0)}
    fx, fy = controller.compute_control_force(1, 5, 2.0, rel_eq, gain=2.0)
    assert math.isclose(fx, 0.0, abs_tol=1e-6)
    assert math.isclose(fy, 0.0, abs_tol=1e-6)

    # 2. Too far (d_star + 0.5m): robot 1 should be pulled towards robot 2 (+X direction)
    rel_far = {2: (d_star + 0.5, 0.0)}
    fx_far, fy_far = controller.compute_control_force(1, 5, 2.0, rel_far, gain=2.0)
    assert fx_far > 0.0
    assert math.isclose(fx_far, 2.0 * 0.5, abs_tol=1e-4)

    # 3. Too close (d_star - 0.5m): robot 1 should be pushed away from robot 2 (-X direction)
    rel_close = {2: (d_star - 0.5, 0.0)}
    fx_close, _ = controller.compute_control_force(1, 5, 2.0, rel_close, gain=2.0)
    assert fx_close < 0.0
    assert math.isclose(fx_close, -2.0 * 0.5, abs_tol=1e-4)


def test_bearing_controller_orthogonal_and_scale() -> None:
    """Verify bearing controller generates orthogonal restoring and radial scale forces."""
    controller = BearingController(scale_gain_ratio=1.0)
    target_bearings = compute_target_bearings(1, 5, 2.0)
    gx_star, gy_star, d_star = target_bearings[2]

    # 1. At exact target bearing and distance: force is zero
    rel_eq = {2: (d_star * gx_star, d_star * gy_star)}
    fx, fy = controller.compute_control_force(1, 5, 2.0, rel_eq, gain=1.5)
    assert math.isclose(fx, 0.0, abs_tol=1e-6)
    assert math.isclose(fy, 0.0, abs_tol=1e-6)

    # 2. Scale deviation along bearing: force is directed along (gx_star, gy_star)
    rel_dilated = {2: ((d_star + 0.4) * gx_star, (d_star + 0.4) * gy_star)}
    fx_dil, fy_dil = controller.compute_control_force(1, 5, 2.0, rel_dilated, gain=1.5)
    assert math.isclose(fx_dil, 1.5 * 0.4 * gx_star, abs_tol=1e-4)
    assert math.isclose(fy_dil, 1.5 * 0.4 * gy_star, abs_tol=1e-4)


def test_create_controller_factory() -> None:
    """Verify factory returns appropriate controller instances or raises on invalid input."""
    c_disp = create_controller('displacement')
    assert isinstance(c_disp, DisplacementController)
    assert c_disp.name == 'displacement'

    c_dist = create_controller('distance')
    assert isinstance(c_dist, DistanceController)
    assert c_dist.name == 'distance'

    c_bear = create_controller('bearing')
    assert isinstance(c_bear, BearingController)
    assert c_bear.name == 'bearing'

    with pytest.raises(ValueError):
        create_controller('unknown_controller_family')
