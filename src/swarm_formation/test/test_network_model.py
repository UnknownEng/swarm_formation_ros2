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

"""Unit tests for standalone network channel and impairment model."""

import random

from swarm_formation.network_model import (
    ChannelLink,
    compute_packet_latency,
    should_drop_packet,
)


def test_channel_link_defaults() -> None:
    """Verify default parameters of a ChannelLink."""
    link = ChannelLink(from_id=1, to_id=2)
    assert link.from_id == 1
    assert link.to_id == 2
    assert link.enabled is True
    assert link.drop_rate == 0.0
    assert link.delay_sec == 0.0
    assert link.jitter_sec == 0.0


def test_should_drop_packet_disabled() -> None:
    """Verify that disabled link always drops packets."""
    link = ChannelLink(from_id=1, to_id=2, enabled=False, drop_rate=0.0)
    assert should_drop_packet(link) is True


def test_should_drop_packet_bounds() -> None:
    """Verify deterministic boundary drop behavior."""
    link_clean = ChannelLink(from_id=1, to_id=2, drop_rate=0.0)
    assert should_drop_packet(link_clean) is False

    link_lossy = ChannelLink(from_id=1, to_id=2, drop_rate=1.0)
    assert should_drop_packet(link_lossy) is True


def test_should_drop_packet_stochastic() -> None:
    """Verify statistical packet drop rate matches expected probability."""
    rng = random.Random(42)
    p_drop = 0.25
    link = ChannelLink(from_id=1, to_id=2, drop_rate=p_drop)

    trials = 2000
    drops = sum(1 for _ in range(trials) if should_drop_packet(link, rng))
    empirical_p = drops / trials

    # Standard error for p=0.25, N=2000 is ~0.0097; within 3 sigma (~0.03)
    assert abs(empirical_p - p_drop) < 0.03


def test_compute_packet_latency_deterministic() -> None:
    """Verify fixed latency without jitter."""
    link = ChannelLink(from_id=1, to_id=2, delay_sec=0.10, jitter_sec=0.0)
    latency = compute_packet_latency(link)
    assert abs(latency - 0.10) < 1e-6


def test_compute_packet_latency_jitter_bounds() -> None:
    """Verify latency with jitter strictly adheres to [delay - jitter, delay + jitter]."""
    rng = random.Random(999)
    base_delay = 0.10
    jitter = 0.03
    link = ChannelLink(from_id=1, to_id=2, delay_sec=base_delay, jitter_sec=jitter)

    for _ in range(500):
        lat = compute_packet_latency(link, rng)
        assert (base_delay - jitter) - 1e-6 <= lat <= (base_delay + jitter) + 1e-6
        assert lat >= 0.0
