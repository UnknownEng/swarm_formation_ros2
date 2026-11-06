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

"""Acceptance tests for Milestone M1 (Relative Neighbor Sensing)."""

import ast
import os

from swarm_formation.relative_sensing import compute_relative_measurement


def test_controller_does_not_read_global_xy() -> None:
    """Verify AST of robot_node.py to ensure no controller accesses global (x, y) coordinates."""
    robot_node_path = os.path.join(
        os.path.dirname(__file__), '..', 'swarm_formation', 'robot_node.py'
    )
    with open(robot_node_path, 'r', encoding='utf-8') as f:
        source = f.read()

    tree = ast.parse(source)

    # 1. Ensure '/robot_all_poses' topic is NOT present
    assert '/robot_all_poses' not in source, (
        'Violation: robot_node must not publish or subscribe to global /robot_all_poses'
    )

    # 2. Inspect attribute accesses in GazeboSwarmNode methods
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == 'GazeboSwarmNode':
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name in [
                    'control_loop',
                    '_compute_resilient_control',
                ]:
                    func_source = ast.get_source_segment(source, item)
                    assert 'self.pos_x' not in func_source, (
                        f'Violation: {item.name} accesses self.pos_x (global x)'
                    )
                    assert 'self.pos_y' not in func_source, (
                        f'Violation: {item.name} accesses self.pos_y (global y)'
                    )


def test_out_of_range_robots_are_invisible() -> None:
    """Verify that robots outside the configured sensing range are invisible (return None)."""
    self_pos = (0.0, 0.0)
    self_yaw = 0.0
    sensing_range = 5.0

    # Peer exactly inside boundary
    inside_peer = (4.99, 0.0)
    meas_in = compute_relative_measurement(
        self_pos=self_pos,
        self_yaw=self_yaw,
        target_pos=inside_peer,
        sensing_range=sensing_range,
    )
    assert meas_in is not None, 'Peer within sensing range must be visible'
    assert abs(meas_in['distance'] - 4.99) < 1e-6

    # Peer outside boundary
    outside_peer = (5.01, 0.0)
    meas_out = compute_relative_measurement(
        self_pos=self_pos,
        self_yaw=self_yaw,
        target_pos=outside_peer,
        sensing_range=sensing_range,
    )
    assert meas_out is None, 'Peer beyond sensing range must be invisible (None)'


def test_relative_sensing_noise_parameters() -> None:
    """Verify that range and bearing noise parameters function properly."""
    self_pos = (0.0, 0.0)
    self_yaw = 0.0
    peer = (3.0, 4.0)  # distance = 5.0m
    sensing_range = 8.0

    meas_clean = compute_relative_measurement(
        self_pos=self_pos,
        self_yaw=self_yaw,
        target_pos=peer,
        sensing_range=sensing_range,
        range_noise_std=0.0,
        bearing_noise_std=0.0,
    )
    assert meas_clean is not None
    assert abs(meas_clean['distance'] - 5.0) < 1e-6
    assert meas_clean['range_variance'] == 0.0
    assert meas_clean['bearing_variance'] == 0.0
