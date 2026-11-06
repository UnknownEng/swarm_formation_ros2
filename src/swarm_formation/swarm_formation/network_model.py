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

"""Network channel link and impairment mathematical model."""

from dataclasses import dataclass
import random
from typing import Optional


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
