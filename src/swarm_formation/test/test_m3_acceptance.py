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

"""Acceptance tests for Milestone M3 (Configurable Faults / Noise)."""

import ast
import math
import os

import rclpy
from swarm_formation.fault_injector_node import FaultInjectorNode
from swarm_formation.neighbor_sensor_node import NeighborSensorNode
from swarm_interfaces.msg import FaultInjection
from swarm_interfaces.srv import TriggerFault


def test_robot_node_decoupled_from_faults() -> None:
    """Verify that robot_node has no internal dropout timer and receives faults externally."""
    node_file = os.path.join(
        os.path.dirname(__file__), '..', 'swarm_formation', 'robot_node.py'
    )
    with open(node_file, 'r', encoding='utf-8') as f:
        tree = ast.parse(f.read())

    func_names = [n.name for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)]
    assert '_check_agent_dropout' not in func_names, (
        'robot_node.py must NOT contain internal _check_agent_dropout function'
    )

    # Check for hardcoded parameter names in string literals
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            assert 'fault_dropout_robot_id' not in node.value, (
                'robot_node.py must not declare fault_dropout_robot_id'
            )
            assert 'fault_dropout_start_sec' not in node.value, (
                'robot_node.py must not declare fault_dropout_start_sec'
            )


def test_fault_injector_runtime_trigger() -> None:
    """Verify runtime interactive trigger and release of faults via TriggerFault service."""
    if not rclpy.ok():
        rclpy.init()

    node = FaultInjectorNode()
    try:
        # 1. Trigger dynamic robot_death for Robot 4
        req = TriggerFault.Request()
        req.robot_id = 4
        req.fault_type = 'robot_death'
        req.enable = True
        resp = TriggerFault.Response()

        resp = node._handle_trigger_fault(req, resp)
        assert resp.success is True
        assert len(node.dynamic_faults) == 1
        assert node.dynamic_faults[0].robot_id == 4

        # Run evaluation loop
        node._fault_evaluation_loop()
        assert node.robot_death_state[4] is True

        # 2. Release robot_death for Robot 4
        req_rel = TriggerFault.Request()
        req_rel.robot_id = 4
        req_rel.fault_type = 'robot_death'
        req_rel.enable = False
        resp_rel = TriggerFault.Response()

        resp_rel = node._handle_trigger_fault(req_rel, resp_rel)
        assert resp_rel.success is True
        assert len(node.dynamic_faults) == 0

        # Run evaluation loop again
        node._fault_evaluation_loop()
        assert node.robot_death_state[4] is False
    finally:
        node.destroy_node()


def test_neighbor_sensor_fault_response() -> None:
    """Verify NeighborSensorNode applies sensor bias and intermittent dropouts on demand."""
    if not rclpy.ok():
        rclpy.init()

    sensor_node = NeighborSensorNode()
    try:
        sensor_node.robot_id = 1
        sensor_node.sensing_range = 10.0
        sensor_node.has_own_pose = True
        sensor_node.pos_x = 0.0
        sensor_node.pos_y = 0.0
        sensor_node.yaw = 0.0

        # Target peer 2 at (2.0, 0.0) -> relative dist = 2.0, bearing = 0.0
        sensor_node.peer_poses[2] = (2.0, 0.0)

        # Baseline capture
        captured = []

        class MockPublisher:

            def publish(self, msg) -> None:
                captured.append(msg)

        sensor_node.pub_neighbors = MockPublisher()

        # 1. Nominal sensor reading (no fault)
        sensor_node._publish_sensor_readings()
        assert len(captured) == 1
        meas_nom = captured[-1].neighbors[0]
        assert math.isclose(meas_nom.distance, 2.0, abs_tol=1e-3)
        assert math.isclose(meas_nom.bearing, 0.0, abs_tol=1e-3)

        # 2. Inject sensor bias (+0.5m range, +0.2 rad bearing)
        fault_msg = FaultInjection()
        fault_msg.robot_id = 1
        fault_msg.fault_type = 'sensor_bias'
        fault_msg.active = True
        fault_msg.range_bias = 0.5
        fault_msg.bearing_bias = 0.2
        sensor_node._sensor_fault_callback(fault_msg)

        captured.clear()
        sensor_node._publish_sensor_readings()
        assert len(captured) == 1
        meas_biased = captured[-1].neighbors[0]
        assert math.isclose(meas_biased.distance, 2.5, abs_tol=1e-3)
        assert math.isclose(meas_biased.bearing, 0.2, abs_tol=1e-3)

        # 3. Inject intermittent failure
        fault_msg.fault_type = 'intermittent_failure'
        sensor_node._sensor_fault_callback(fault_msg)

        captured.clear()
        sensor_node._publish_sensor_readings()
        # Should drop readings completely during active phase
        assert len(captured) == 0

        # 4. Release fault
        fault_msg.active = False
        sensor_node._sensor_fault_callback(fault_msg)

        captured.clear()
        sensor_node._publish_sensor_readings()
        assert len(captured) == 1
        meas_recovered = captured[-1].neighbors[0]
        assert math.isclose(meas_recovered.distance, 2.0, abs_tol=1e-3)
    finally:
        sensor_node.destroy_node()
