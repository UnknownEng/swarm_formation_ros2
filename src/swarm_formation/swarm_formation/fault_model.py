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

"""Configurable fault injection mathematical models and YAML profile parser."""

from dataclasses import dataclass
import os
from typing import List, Tuple

import yaml

from .kinematics import wrap_angle


@dataclass
class FaultConfig:
    """Configuration record for an individual robot fault scenario."""

    robot_id: int
    fault_type: str
    start_time: float
    end_time: float
    range_bias: float = 0.0
    bearing_bias: float = 0.0
    period: float = 1.0
    duty_cycle: float = 0.5


def is_fault_active(fault: FaultConfig, current_time: float) -> bool:
    """
    Check if a configured fault is actively in effect at the given time.

    :param fault: Fault specification.
    :param current_time: Current simulation or wall time in seconds.
    :return: True if the fault is active, False otherwise.
    """
    if current_time < fault.start_time or current_time > fault.end_time:
        return False

    if fault.fault_type in ('robot_death', 'sensor_bias'):
        return True

    if fault.fault_type == 'intermittent_failure':
        period = max(fault.period, 1e-4)
        phase = (current_time - fault.start_time) % period
        return phase < (period * fault.duty_cycle)

    return False


def apply_sensor_bias(
    distance: float, bearing: float, range_bias: float, bearing_bias: float
) -> Tuple[float, float]:
    """
    Inject additive distance and bearing bias into relative measurements.

    :param distance: True measured distance in meters.
    :param bearing: True measured bearing in radians.
    :param range_bias: Additive distance offset in meters.
    :param bearing_bias: Additive bearing offset in radians.
    :return: Tuple of (biased_distance, biased_bearing).
    """
    biased_dist = max(0.01, distance + range_bias)
    biased_bearing = wrap_angle(bearing + bearing_bias)
    return biased_dist, biased_bearing


def parse_fault_yaml(yaml_path_or_str: str) -> List[FaultConfig]:
    """
    Parse a YAML configuration file or string into a list of FaultConfig objects.

    :param yaml_path_or_str: Filesystem path to YAML file or raw YAML string.
    :return: List of validated FaultConfig instances.
    """
    if os.path.exists(yaml_path_or_str):
        with open(yaml_path_or_str, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
    else:
        data = yaml.safe_load(yaml_path_or_str)

    if not data or 'fault_profile' not in data:
        return []

    faults_list = data['fault_profile'].get('faults', [])
    configs: List[FaultConfig] = []

    for item in faults_list:
        cfg = FaultConfig(
            robot_id=int(item['robot_id']),
            fault_type=str(item['fault_type']),
            start_time=float(item['start_time']),
            end_time=float(item['end_time']),
            range_bias=float(item.get('range_bias', 0.0)),
            bearing_bias=float(item.get('bearing_bias', 0.0)),
            period=float(item.get('period', 1.0)),
            duty_cycle=float(item.get('duty_cycle', 0.5)),
        )
        configs.append(cfg)

    return configs
