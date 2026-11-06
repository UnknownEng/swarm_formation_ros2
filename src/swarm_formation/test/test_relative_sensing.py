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

"""Unit tests for relative range-bearing sensing module."""

import math
import random

from swarm_formation.relative_sensing import (
    compute_relative_bearing,
    compute_relative_measurement,
    compute_relative_obstacle,
    transform_body_to_reference,
)


def test_compute_relative_bearing() -> None:
    """Verify relative bearing calculation in body frame across cardinal directions."""
    # Robot facing East (0 rad), target at North (dx=0, dy=2) -> bearing should be +pi/2
    bearing = compute_relative_bearing(0.0, 2.0, 0.0)
    assert abs(bearing - math.pi / 2.0) < 1e-6

    # Robot facing North (pi/2 rad), target at North (dx=0, dy=2) -> bearing should be 0
    bearing = compute_relative_bearing(0.0, 2.0, math.pi / 2.0)
    assert abs(bearing) < 1e-6

    # Robot facing North, target at East (dx=2, dy=0) -> bearing should be -pi/2
    bearing = compute_relative_bearing(2.0, 0.0, math.pi / 2.0)
    assert abs(bearing - (-math.pi / 2.0)) < 1e-6


def test_compute_relative_measurement_within_range() -> None:
    """Verify relative measurement when peer is inside sensing range."""
    self_pos = (1.0, 2.0)
    self_yaw = 0.0
    target_pos = (4.0, 6.0)  # dx=3, dy=4, distance=5.0
    sensing_range = 6.0

    meas = compute_relative_measurement(
        self_pos=self_pos,
        self_yaw=self_yaw,
        target_pos=target_pos,
        sensing_range=sensing_range,
        range_noise_std=0.0,
        bearing_noise_std=0.0,
    )

    assert meas is not None
    assert abs(meas['distance'] - 5.0) < 1e-6
    expected_bearing = math.atan2(4.0, 3.0)
    assert abs(meas['bearing'] - expected_bearing) < 1e-6
    assert abs(meas['relative_x'] - 3.0) < 1e-5
    assert abs(meas['relative_y'] - 4.0) < 1e-5


def test_compute_relative_measurement_out_of_range() -> None:
    """Verify that peers outside sensing range are completely invisible (return None)."""
    self_pos = (0.0, 0.0)
    self_yaw = 0.0
    target_pos = (5.1, 0.0)  # distance = 5.1m
    sensing_range = 5.0

    meas = compute_relative_measurement(
        self_pos=self_pos,
        self_yaw=self_yaw,
        target_pos=target_pos,
        sensing_range=sensing_range,
    )
    assert meas is None, 'Robot beyond sensing range must be invisible (None)'


def test_compute_relative_measurement_with_noise() -> None:
    """Verify that Gaussian noise is applied to range and bearing."""
    rng = random.Random(123)
    self_pos = (0.0, 0.0)
    self_yaw = 0.0
    target_pos = (3.0, 0.0)
    sensing_range = 5.0

    meas = compute_relative_measurement(
        self_pos=self_pos,
        self_yaw=self_yaw,
        target_pos=target_pos,
        sensing_range=sensing_range,
        range_noise_std=0.10,
        bearing_noise_std=0.05,
        rng=rng,
    )

    assert meas is not None
    assert meas['range_variance'] == 0.10 ** 2
    assert meas['bearing_variance'] == 0.05 ** 2
    # Distance should be close to 3.0 within 3-sigma
    assert 2.5 < meas['distance'] < 3.5


def test_compute_relative_obstacle() -> None:
    """Verify obstacle detection and out-of-range invisibility."""
    self_pos = (0.0, 0.0)
    self_yaw = 0.0
    obs_center = (2.0, 0.0)
    obs_radius = 0.2
    sensing_range = 3.0

    meas = compute_relative_obstacle(
        self_pos=self_pos,
        self_yaw=self_yaw,
        obs_center=obs_center,
        obs_radius=obs_radius,
        sensing_range=sensing_range,
    )
    assert meas is not None
    assert abs(meas['distance'] - 2.0) < 1e-6
    assert abs(meas['radius'] - 0.2) < 1e-6

    # Test out of range
    far_obs = (10.0, 0.0)
    far_meas = compute_relative_obstacle(
        self_pos=self_pos,
        self_yaw=self_yaw,
        obs_center=far_obs,
        obs_radius=obs_radius,
        sensing_range=sensing_range,
    )
    assert far_meas is None, 'Obstacle outside sensing range must be invisible'


def test_transform_body_to_reference() -> None:
    """Verify 2D vector coordinate rotation from body frame to reference frame."""
    # Point at (1, 0) in body frame with robot heading pi/2 (facing North)
    # Reference frame coordinates should be (0, 1)
    dx, dy = transform_body_to_reference(1.0, 0.0, math.pi / 2.0)
    assert abs(dx) < 1e-6
    assert abs(dy - 1.0) < 1e-6

    # Point at (0, 1) in body frame (robot left) with heading pi/2 (North)
    # Reference frame coordinates should be (-1, 0) (West)
    dx2, dy2 = transform_body_to_reference(0.0, 1.0, math.pi / 2.0)
    assert abs(dx2 - (-1.0)) < 1e-6
    assert abs(dy2) < 1e-6
