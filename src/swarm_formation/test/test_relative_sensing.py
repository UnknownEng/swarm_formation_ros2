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

"""Unit tests for relative range-bearing sensing module."""

import math
import random

from swarm_formation.relative_sensing import (
    compute_heartbeat_mask,
    compute_relative_bearing,
    compute_relative_measurement,
    compute_relative_obstacle,
    transform_body_to_reference,
    update_relative_store,
    YawHistory,
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


def test_yaw_history_interpolates_and_wraps() -> None:
    """Heading lookup interpolates between samples, including across +/- pi."""
    hist = YawHistory()
    assert hist.lookup(1.0) is None
    hist.add(0.0, 0.0)
    hist.add(1.0, 1.0)
    assert math.isclose(hist.lookup(0.5), 0.5, abs_tol=1e-9)
    assert math.isclose(hist.lookup(-5.0), 0.0, abs_tol=1e-9)
    assert math.isclose(hist.lookup(9.0), 1.0, abs_tol=1e-9)
    wrap = YawHistory()
    wrap.add(0.0, math.pi - 0.1)
    wrap.add(1.0, -math.pi + 0.1)
    assert abs(abs(wrap.lookup(0.5)) - math.pi) < 1e-9


def test_delayed_measurement_uses_heading_at_measurement_time() -> None:
    """A peer straight ahead when measured stays at the same world bearing after turning."""
    hist = YawHistory()
    hist.add(0.0, 0.0)          # facing +x when the measurement is taken
    hist.add(0.2, math.pi / 2)  # turned 90 deg by the time it is delivered
    store = update_relative_store({}, [(2, 1.0, 0.0, 0.0)], hist.lookup)
    dx, dy = store[2][0], store[2][1]
    assert math.isclose(dx, 1.0, abs_tol=1e-9) and math.isclose(dy, 0.0, abs_tol=1e-9)


def test_relative_store_drops_absent_peers_and_keeps_velocity_on_republish() -> None:
    """Peers missing from the list are removed; re-published data keeps its velocity."""
    def yaw_zero(_: float) -> float:
        return 0.0

    store = update_relative_store({}, [(2, 1.0, 0.0, 0.0), (3, 2.0, 0.0, 0.0)], yaw_zero)
    store = update_relative_store(store, [(2, 1.1, 0.0, 0.1)], yaw_zero)
    assert set(store) == {2}
    assert math.isclose(store[2][2], 1.0, abs_tol=1e-9)  # 0.1 m in 0.1 s
    store = update_relative_store(store, [(2, 1.1, 0.0, 0.1)], yaw_zero)
    assert math.isclose(store[2][2], 1.0, abs_tol=1e-9)  # same stamp: velocity kept


def test_heartbeat_mask_uses_measurement_age() -> None:
    """A re-published but old measurement times out; absent peers are inactive."""
    store = {2: (1.0, 0.0, 0.0, 0.0, 10.0), 3: (1.0, 1.0, 0.0, 0.0, 8.6)}
    mask = compute_heartbeat_mask(store, [2, 3, 4], now=10.0, timeout=1.2)
    assert mask == {2: True, 3: False, 4: False}


def test_delay_compensation_subtracts_own_motion() -> None:
    """Own motion since the measurement is removed exactly; no history leaves it raw."""
    from swarm_formation.relative_sensing import compensate_delay, PoseHistory

    hist = PoseHistory()
    hist.add(0.0, 0.0, 0.0)
    hist.add(1.0, 1.0, 0.5)
    store = {2: (2.0, 0.0, 0.0, 0.0, 0.5)}      # measured at t = 0.5, now = 1.0
    comp = compensate_delay(store, 1.0, hist.lookup)
    assert math.isclose(comp[2][0], 1.5) and math.isclose(comp[2][1], -0.25)
    assert compensate_delay(store, 1.0, PoseHistory().lookup) == {2: (2.0, 0.0)}


def test_delay_compensation_peer_extrapolation_and_age_clip() -> None:
    """Neighbour motion is extrapolated only when asked; very old entries are clipped."""
    from swarm_formation.relative_sensing import compensate_delay, PoseHistory

    hist = PoseHistory()
    hist.add(0.0, 0.0, 0.0)
    hist.add(10.0, 0.0, 0.0)                     # this robot stands still
    store = {2: (2.0, 0.0, 1.0, 0.0, 9.8)}       # peer drifting away at 1 m/s, age 0.2 s
    assert compensate_delay(store, 10.0, hist.lookup)[2] == (2.0, 0.0)
    moved = compensate_delay(store, 10.0, hist.lookup, peer_velocity_gain=1.0)[2]
    assert math.isclose(moved[0], 2.2, abs_tol=1e-9)
    old = {2: (2.0, 0.0, 1.0, 0.0, 0.0)}         # age 10 s, clipped to 0.5 s
    clipped = compensate_delay(old, 10.0, hist.lookup, peer_velocity_gain=1.0)[2]
    assert math.isclose(clipped[0], 2.5, abs_tol=1e-9)
