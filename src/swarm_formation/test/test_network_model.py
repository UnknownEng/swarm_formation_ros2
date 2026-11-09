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
    apply_link_overrides,
    ChannelLink,
    compute_packet_latency,
    parse_network_profile,
    randomize_links,
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


def _mesh(n: int = 5):
    return {(s, d): ChannelLink(from_id=s, to_id=d) for s in range(1, n + 1)
            for d in range(1, n + 1) if s != d}


def test_network_profile_one_way_links() -> None:
    """A profile cuts one direction of a pair and leaves the other working."""
    text = """
network_profile:
  name: test
  links:
    - {from: 1, to: 0, drop_rate: 1.0}
    - {from: 3, to: 4, drop_rate: 0.5, delay_sec: 0.2, jitter_sec: 0.05}
"""
    overrides = parse_network_profile(text)
    assert overrides == [(1, 0, 1.0, None, None), (3, 4, 0.5, 0.2, 0.05)]
    links = _mesh()
    assert apply_link_overrides(links, overrides) == 4 + 1
    for d in range(2, 6):
        assert not links[(1, d)].enabled               # nobody hears robot 1
        assert links[(d, 1)].enabled                   # robot 1 hears everybody
    assert (links[(3, 4)].drop_rate, links[(3, 4)].delay_sec) == (0.5, 0.2)
    assert (links[(4, 3)].drop_rate, links[(4, 3)].delay_sec) == (0.0, 0.0)


def test_shipped_network_profiles_parse() -> None:
    """Every profile installed with the package parses to at least one rule."""
    import glob
    import os
    paths = glob.glob(os.path.join(os.path.dirname(__file__), '..', 'config',
                                   'network_profiles', '*.yaml'))
    assert len(paths) >= 4
    for path in paths:
        assert parse_network_profile(path), path


def test_randomized_links_are_asymmetric() -> None:
    """Random asymmetry draws each direction independently and within bounds."""
    links = _mesh()
    randomize_links(links, 0.5, 0.15, random.Random(3))
    for link in links.values():
        assert 0.0 <= link.drop_rate <= 0.5 and 0.0 <= link.delay_sec <= 0.15
    assert any(abs(links[(s, d)].drop_rate - links[(d, s)].drop_rate) > 0.05
               for (s, d) in links)


def test_profile_keeps_unspecified_values() -> None:
    """A profile entry without a delay leaves an existing delay in place."""
    links = _mesh()
    for link in links.values():
        link.delay_sec = 0.08
    apply_link_overrides(links, parse_network_profile(
        'network_profile: {links: [{from: 1, to: 2, drop_rate: 0.3}]}'))
    assert links[(1, 2)].drop_rate == 0.3
    assert links[(1, 2)].delay_sec == 0.08


def test_profile_rejects_bad_input() -> None:
    """Missing files, malformed entries and out-of-range values raise clear errors."""
    import pytest
    with pytest.raises(FileNotFoundError):
        parse_network_profile('/no/such/profile.yaml')
    with pytest.raises(ValueError):
        parse_network_profile('network_profile: {links: [null]}')
    with pytest.raises(ValueError):
        parse_network_profile('network_profile: {links: [{from: 1, to: 2, drop_rate: 1.5}]}')
    with pytest.raises(ValueError):
        parse_network_profile('network_profile: {links: [{from: 1, to: 2, delay_sec: -1}]}')
