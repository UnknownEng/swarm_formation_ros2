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

"""Unit tests for consensus formation algorithms."""

import math

from swarm_formation.consensus import (
    compute_baseline_consensus,
    compute_huber_consensus,
    compute_target_offsets,
)


def test_target_offsets_regular_pentagon():
    """Verify geometric properties of regular pentagon formation offsets."""
    offsets = compute_target_offsets(robot_id=1, total_robots=5, radius=2.0)
    assert len(offsets) == 4
    for peer_id, (dx, dy) in offsets.items():
        # Distance between vertices in regular pentagon with R=2.0m:
        # Adjacent edge: 2 * R * sin(pi / 5) ~= 2.3511 m
        # Diagonal edge: 2 * R * sin(2 * pi / 5) ~= 3.8042 m
        dist = math.hypot(dx, dy)
        assert math.isclose(dist, 2.3511, abs_tol=1e-3) or math.isclose(dist, 3.8042, abs_tol=1e-3)


def test_baseline_consensus_zero_error():
    """Verify zero force when all agents are perfectly placed at target offsets."""
    target_offsets = {2: (2.0, 0.0), 3: (0.0, 2.0)}
    # Peer 2 at (2.0, 0.0), Peer 3 at (0.0, 2.0) relative to robot 1 at (0.0, 0.0)
    neighbor_states = {
        2: (2.0, 0.0, 0.0, 0.0, 0.0),
        3: (0.0, 2.0, 0.0, 0.0, 0.0),
    }
    fx, fy = compute_baseline_consensus(0.0, 0.0, target_offsets, neighbor_states, k_p=1.8)
    assert math.isclose(fx, 0.0, abs_tol=1e-6)
    assert math.isclose(fy, 0.0, abs_tol=1e-6)


def test_baseline_consensus_proportional():
    """Verify proportional consensus drive force when agents are offset from targets."""
    target_offsets = {2: (2.0, 0.0)}
    # Peer 2 is at (3.0, 0.0), so dx - delta_x = 3.0 - 2.0 = 1.0
    neighbor_states = {2: (3.0, 0.0, 0.0, 0.0, 0.0)}
    fx, fy = compute_baseline_consensus(0.0, 0.0, target_offsets, neighbor_states, k_p=1.8)
    assert math.isclose(fx, 1.8, abs_tol=1e-6)
    assert math.isclose(fy, 0.0, abs_tol=1e-6)


def test_huber_consensus_clipping():
    """Verify that Huber loss bounds influence for large outlier errors."""
    target_offsets = {2: (0.0, 0.0)}
    # Error norm is 10.0m > huber_delta (2.0m)
    filtered_states = {2: (10.0, 0.0, 0.0, 0.0)}
    active_mask = {2: True}

    fx, fy, count = compute_huber_consensus(
        0.0, 0.0, target_offsets, filtered_states, active_mask, k_p=1.8, huber_delta=2.0
    )
    assert count == 1
    # Huber scales error: err_norm = 10 -> clipped to huber_delta = 2.0 -> force = 1.8 * 2.0 = 3.6
    assert math.isclose(fx, 3.6, abs_tol=1e-6)
    assert math.isclose(fy, 0.0, abs_tol=1e-6)
