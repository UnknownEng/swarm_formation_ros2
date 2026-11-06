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

"""Acceptance tests for Milestone M2 (Network Impairment Node)."""

import ast
import os

import rclpy
from swarm_formation.network_impairment_node import NetworkImpairmentNode
from swarm_interfaces.msg import LinkState, NeighborList, NeighborMeasurement
from swarm_interfaces.srv import SetLinkState


def test_network_impairment_runtime_link_toggle() -> None:
    """Verify runtime link on/off toggling via SetLinkState service."""
    if not rclpy.ok():
        rclpy.init()

    node = NetworkImpairmentNode()
    try:
        # Initial state: link 2 -> 1 is enabled
        assert node.links[(2, 1)].enabled is True

        # Toggle link 2 -> 1 to DISABLED via service
        req = SetLinkState.Request()
        req.from_robot_id = 2
        req.to_robot_id = 1
        req.enabled = False
        req.drop_rate = 0.0
        req.delay_sec = 0.0
        req.jitter_sec = 0.0
        resp = SetLinkState.Response()

        resp = node._handle_set_link(req, resp)
        assert resp.success is True
        assert node.links[(2, 1)].enabled is False

        # Feed raw neighbor measurements for robot 1 containing peer 2 and peer 3
        raw_msg = NeighborList()
        raw_msg.robot_id = 1

        n2 = NeighborMeasurement()
        n2.neighbor_id = 2
        n2.distance = 2.0
        raw_msg.neighbors.append(n2)

        n3 = NeighborMeasurement()
        n3.neighbor_id = 3
        n3.distance = 2.5
        raw_msg.neighbors.append(n3)

        node._raw_neighbors_callback(raw_msg, dst_id=1)

        # Peer 2 was on disabled link (2 -> 1) so it was dropped
        assert 2 not in node.delivered_states[1], 'Disabled link 2->1 must drop peer 2'
        # Peer 3 was on enabled link (3 -> 1) so it was forwarded
        assert 3 in node.delivered_states[1], 'Enabled link 3->1 must forward peer 3'

        # Re-enable link 2 -> 1
        req.enabled = True
        resp = node._handle_set_link(req, resp)
        assert resp.success is True
        assert node.links[(2, 1)].enabled is True

        # Deliver raw message again
        node._raw_neighbors_callback(raw_msg, dst_id=1)
        assert 2 in node.delivered_states[1], 'Re-enabled link 2->1 must forward peer 2'
        assert 3 in node.delivered_states[1]

    finally:
        node.destroy_node()


def test_network_impairment_link_command_topic() -> None:
    """Verify runtime link modification via topic command."""
    if not rclpy.ok():
        rclpy.init()

    node = NetworkImpairmentNode()
    try:
        cmd = LinkState()
        cmd.from_robot_id = 4
        cmd.to_robot_id = 1
        cmd.enabled = False
        cmd.drop_rate = 0.50
        cmd.delay_sec = 0.12
        cmd.jitter_sec = 0.02

        node._handle_link_command(cmd)

        link = node.links[(4, 1)]
        assert link.enabled is False
        assert link.drop_rate == 0.50
        assert link.delay_sec == 0.12
        assert link.jitter_sec == 0.02

    finally:
        node.destroy_node()


def test_robot_code_untouched_by_impairments() -> None:
    """Verify that robot controller code contains zero link toggle services or impairment logic."""
    robot_node_path = os.path.join(
        os.path.dirname(__file__), '..', 'swarm_formation', 'robot_node.py'
    )
    with open(robot_node_path, 'r', encoding='utf-8') as f:
        source = f.read()

    tree = ast.parse(source)

    # Controller should not have network service handlers or link impairment logic
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef):
            assert 'set_link' not in node.name.lower(), (
                f'Found link toggle service {node.name} inside robot controller'
            )
            assert 'impairment' not in node.name.lower()

    # Robot node simply subscribes to /robot_{id}/neighbors
    assert 'sub_neighbors' in source
    assert 'NetworkImpairmentNode' not in source
