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

"""Unit tests for relative consensus algorithms."""

from swarm_formation.consensus import (
    compute_relative_consensus,
    compute_relative_huber_consensus,
)


def test_relative_consensus_at_equilibrium() -> None:
    """Verify zero consensus force when relative displacements match target offsets."""
    target_offsets = {2: (1.5, 0.0), 3: (0.0, 2.0)}
    # Measured relative displacements match target offsets exactly
    relative_displacements = {2: (1.5, 0.0), 3: (0.0, 2.0)}

    fx, fy = compute_relative_consensus(target_offsets, relative_displacements, k_p=1.8)
    assert abs(fx) < 1e-6
    assert abs(fy) < 1e-6


def test_relative_consensus_proportional_force() -> None:
    """Verify linear proportional force response to relative displacement error."""
    target_offsets = {2: (2.0, 0.0)}
    # Measured displacement is 3.0 (error is +1.0 in x)
    relative_displacements = {2: (3.0, 0.0)}

    fx, fy = compute_relative_consensus(target_offsets, relative_displacements, k_p=2.0)
    assert abs(fx - 2.0) < 1e-6
    assert abs(fy) < 1e-6


def test_relative_huber_consensus_saturation() -> None:
    """Verify that Huber consensus caps force gradient for large relative errors."""
    target_offsets = {2: (0.0, 0.0)}
    # Large relative error (10.0m) with huber_delta=2.0
    relative_displacements = {2: (10.0, 0.0)}
    active_mask = {2: True}

    fx, fy, count = compute_relative_huber_consensus(
        target_offsets,
        relative_displacements,
        active_mask,
        k_p=1.0,
        huber_delta=2.0,
    )
    assert count == 1
    # Huber influence function caps at huber_delta = 2.0
    assert abs(fx - 2.0) < 1e-6
    assert abs(fy) < 1e-6


def test_relative_huber_consensus_inactive_mask() -> None:
    """Verify that inactive or dropped neighbors are excluded from consensus."""
    target_offsets = {2: (1.0, 0.0), 3: (0.0, 1.0)}
    relative_displacements = {2: (2.0, 0.0), 3: (0.0, 5.0)}
    # Neighbor 3 is inactive (e.g. heartbeat timed out or dropped out)
    active_mask = {2: True, 3: False}

    fx, fy, count = compute_relative_huber_consensus(
        target_offsets,
        relative_displacements,
        active_mask,
        k_p=1.0,
        huber_delta=2.0,
    )
    assert count == 1
    # Only neighbor 2 contributes: error is 2.0 - 1.0 = 1.0
    assert abs(fx - 1.0) < 1e-6
    assert abs(fy) < 1e-6
