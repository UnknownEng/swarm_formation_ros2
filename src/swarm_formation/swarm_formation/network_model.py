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

"""Network channel link and impairment mathematical model."""

from dataclasses import dataclass
import math
import os
import random
from typing import Dict, List, Optional, Tuple

import yaml


@dataclass
class ChannelLink:
    """Represents a directed communication link between two swarm robots."""

    from_id: int
    to_id: int
    enabled: bool = True
    drop_rate: float = 0.0
    delay_sec: float = 0.0
    jitter_sec: float = 0.0


def should_drop_packet(
    link: ChannelLink, rng: Optional[random.Random] = None
) -> bool:
    """
    Determine whether a transmission packet across this link should be dropped.

    :param link: Channel link configuration.
    :param rng: Random number generator instance.
    :return: True if the packet is dropped (or link disabled), False otherwise.
    """
    if not link.enabled:
        return True
    if link.drop_rate <= 0.0:
        return False
    if link.drop_rate >= 1.0:
        return True
    if rng is not None:
        return rng.random() < link.drop_rate
    return False


def compute_packet_latency(
    link: ChannelLink, rng: Optional[random.Random] = None
) -> float:
    """
    Compute total packet transmission latency including base delay and random jitter.

    :param link: Channel link configuration.
    :param rng: Random number generator instance.
    :return: Total transmission delay in seconds (>= 0.0).
    """
    base_delay = max(0.0, link.delay_sec)
    if link.jitter_sec > 0.0 and rng is not None:
        jitter = rng.uniform(-link.jitter_sec, link.jitter_sec)
        return max(0.0, base_delay + jitter)
    return base_delay


# (src, dst, drop_rate, delay_sec, jitter_sec); 0 = any robot, None = keep the current
# value, later entries win
LinkOverride = Tuple[int, int, Optional[float], Optional[float], Optional[float]]


def _optional_float(item: dict, key: str) -> Optional[float]:
    value = item.get(key)
    if value is None:
        return None
    value = float(value)
    if not math.isfinite(value) or value < 0.0 or (key == 'drop_rate' and value > 1.0):
        raise ValueError(f'invalid {key}: {value}')
    return value


def parse_network_profile(yaml_path_or_str: str) -> List[LinkOverride]:
    """
    Parse a network profile (per-direction link settings) from a YAML file or string.

    Format::

        network_profile:
          name: robot1_mute
          links:
            - {from: 1, to: 0, drop_rate: 1.0, delay_sec: 0.0, jitter_sec: 0.0}

    Link ``from -> to`` carries information about robot ``from`` to robot ``to``; 0 means
    any robot. A value that is left out keeps the link's current setting.

    :param yaml_path_or_str: Filesystem path to a YAML file or raw YAML text.
    :return: List of link overrides in file order.
    :raises FileNotFoundError: if a ``.yaml``/``.yml`` path does not exist.
    :raises ValueError: if an entry is malformed or a value is out of range.
    """
    if os.path.exists(yaml_path_or_str):
        with open(yaml_path_or_str, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
    elif yaml_path_or_str.strip().endswith(('.yaml', '.yml')):
        raise FileNotFoundError(f'network profile not found: {yaml_path_or_str}')
    else:
        data = yaml.safe_load(yaml_path_or_str)
    if not isinstance(data, dict) or 'network_profile' not in data:
        return []
    overrides: List[LinkOverride] = []
    for item in (data['network_profile'] or {}).get('links', []) or []:
        if not isinstance(item, dict):
            raise ValueError(f'link entry must be a mapping, got {item!r}')
        src, dst = int(item.get('from', 0)), int(item.get('to', 0))
        if src < 0 or dst < 0:
            raise ValueError(f'robot ids must be >= 0, got {src} -> {dst}')
        overrides.append((
            src,
            dst,
            _optional_float(item, 'drop_rate'),
            _optional_float(item, 'delay_sec'),
            _optional_float(item, 'jitter_sec'),
        ))
    return overrides


def apply_link_overrides(
    links: Dict[Tuple[int, int], ChannelLink], overrides: List[LinkOverride]
) -> int:
    """
    Apply per-direction overrides to a directed link table in place.

    A drop rate of 1 disables the link; ``None`` values leave the setting unchanged.
    Returns the number of link updates.
    """
    count = 0
    for src, dst, drop, delay, jitter in overrides:
        for (s, d), link in links.items():
            if (src == 0 or s == src) and (dst == 0 or d == dst):
                if drop is not None:
                    link.drop_rate = drop
                    link.enabled = drop < 1.0
                if delay is not None:
                    link.delay_sec = delay
                if jitter is not None:
                    link.jitter_sec = jitter
                count += 1
    return count


def randomize_links(
    links: Dict[Tuple[int, int], ChannelLink],
    max_drop: float,
    max_delay: float,
    rng: random.Random,
) -> None:
    """Give every directed link its own drop rate and extra delay (asymmetric channel)."""
    for key in sorted(links):
        link = links[key]
        link.drop_rate = max(link.drop_rate, rng.uniform(0.0, max_drop))
        link.delay_sec = link.delay_sec + rng.uniform(0.0, max_delay)
