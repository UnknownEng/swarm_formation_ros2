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

"""Unit tests for configurable fault injection mathematical models and parser."""

import math
import os
import tempfile

from swarm_formation.fault_model import (
    apply_sensor_bias,
    FaultConfig,
    is_fault_active,
    parse_fault_yaml,
)


def test_fault_config_defaults() -> None:
    """Verify FaultConfig dataclass initialization and default parameter values."""
    cfg = FaultConfig(robot_id=4, fault_type='robot_death', start_time=5.0, end_time=10.0)
    assert cfg.robot_id == 4
    assert cfg.fault_type == 'robot_death'
    assert cfg.start_time == 5.0
    assert cfg.end_time == 10.0
    assert cfg.range_bias == 0.0
    assert cfg.bearing_bias == 0.0
    assert cfg.period == 1.0
    assert cfg.duty_cycle == 0.5


def test_is_fault_active_continuous() -> None:
    """Verify continuous activation bounds for robot_death and sensor_bias."""
    death = FaultConfig(robot_id=4, fault_type='robot_death', start_time=5.0, end_time=10.0)
    assert not is_fault_active(death, 4.99)
    assert is_fault_active(death, 5.0)
    assert is_fault_active(death, 7.5)
    assert is_fault_active(death, 10.0)
    assert not is_fault_active(death, 10.01)

    bias = FaultConfig(
        robot_id=2, fault_type='sensor_bias', start_time=2.0, end_time=6.0, range_bias=0.3
    )
    assert not is_fault_active(bias, 1.9)
    assert is_fault_active(bias, 3.0)
    assert not is_fault_active(bias, 6.1)


def test_is_fault_active_intermittent() -> None:
    """Verify periodic duty cycle evaluation for intermittent failures."""
    intermittent = FaultConfig(
        robot_id=3,
        fault_type='intermittent_failure',
        start_time=10.0,
        end_time=20.0,
        period=2.0,
        duty_cycle=0.5,
    )
    # Total window: 10.0 to 20.0
    assert not is_fault_active(intermittent, 9.9)
    assert not is_fault_active(intermittent, 20.1)

    # In period 1 (10.0 - 12.0): [10.0, 11.0) active, [11.0, 12.0) inactive
    assert is_fault_active(intermittent, 10.0)
    assert is_fault_active(intermittent, 10.5)
    assert not is_fault_active(intermittent, 11.1)
    assert not is_fault_active(intermittent, 11.9)

    # In period 2 (12.0 - 14.0): [12.0, 13.0) active, [13.0, 14.0) inactive
    assert is_fault_active(intermittent, 12.2)
    assert not is_fault_active(intermittent, 13.2)


def test_apply_sensor_bias() -> None:
    """Verify range and bearing additive bias injection with wrapping and minimum clamp."""
    dist, bearing = apply_sensor_bias(distance=2.0, bearing=0.0, range_bias=0.25, bearing_bias=0.1)
    assert math.isclose(dist, 2.25, abs_tol=1e-6)
    assert math.isclose(bearing, 0.1, abs_tol=1e-6)

    # Negative range clamp test (must stay positive >= 0.01m)
    dist_clamped, _ = apply_sensor_bias(
        distance=0.1, bearing=0.0, range_bias=-1.0, bearing_bias=0.0
    )
    assert dist_clamped >= 0.01

    # Bearing wrap-around test: pi + 0.2 rad -> -pi + 0.2 rad
    _, bearing_wrapped = apply_sensor_bias(
        distance=1.0, bearing=math.pi - 0.1, range_bias=0.0, bearing_bias=0.3
    )
    assert -math.pi <= bearing_wrapped <= math.pi
    assert math.isclose(bearing_wrapped, -math.pi + 0.2, abs_tol=1e-5)


def test_parse_fault_yaml() -> None:
    """Verify loading and parsing of YAML fault configurations from string and file."""
    yaml_text = """
    fault_profile:
      name: "test_profile"
      faults:
        - robot_id: 4
          fault_type: "robot_death"
          start_time: 5.0
          end_time: 10.0
        - robot_id: 2
          fault_type: "sensor_bias"
          start_time: 2.0
          end_time: 8.0
          range_bias: 0.2
          bearing_bias: -0.1
    """
    configs = parse_fault_yaml(yaml_text)
    assert len(configs) == 2
    assert configs[0].robot_id == 4
    assert configs[0].fault_type == 'robot_death'
    assert configs[1].robot_id == 2
    assert math.isclose(configs[1].range_bias, 0.2)
    assert math.isclose(configs[1].bearing_bias, -0.1)

    # Test file-based parsing
    with tempfile.NamedTemporaryFile('w', delete=False, suffix='.yaml') as tmp:
        tmp.write(yaml_text)
        tmp_path = tmp.name

    try:
        file_configs = parse_fault_yaml(tmp_path)
        assert len(file_configs) == 2
        assert file_configs[0].robot_id == 4
    finally:
        os.remove(tmp_path)
