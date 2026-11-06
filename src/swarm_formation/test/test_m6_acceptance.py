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

"""Acceptance tests for Milestone M6 (Safety Layer: CBF-QP Filter)."""

import math
from typing import List

from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from std_msgs.msg import Float32
from swarm_formation.cbf_qp import compute_cbf_safe_twist
from swarm_formation.robot_node import GazeboSwarmNode
from swarm_formation.safety_filter_node import SafetyFilterNode
from swarm_interfaces.msg import NeighborList, NeighborMeasurement


def test_robot_node_cmd_vel_raw_routing() -> None:
    """Verify robot_node directs velocity commands to /cmd_vel_raw when safety filter is active."""
    if not rclpy.ok():
        rclpy.init()

    # Case A: Safety filter enabled (default in M6)
    robot_filter_on = GazeboSwarmNode()
    assert 'cmd_vel_raw' in robot_filter_on.pub_cmd_vel.topic_name
    robot_filter_on.destroy_node()


def test_safety_filter_node_execution() -> None:
    """Verify SafetyFilterNode receives raw commands and publishes safe commands."""
    if not rclpy.ok():
        rclpy.init()

    filter_node = SafetyFilterNode()
    try:
        published_cmds: List[Twist] = []
        published_interventions: List[Float32] = []

        class MockCmdPub:
            def publish(self, msg: Twist) -> None:
                published_cmds.append(msg)

        class MockIntervPub:
            def publish(self, msg: Float32) -> None:
                published_interventions.append(msg)

        filter_node.pub_cmd_vel = MockCmdPub()
        filter_node.pub_intervention = MockIntervPub()

        # 1. Provide estimated pose (yaw = 0.0)
        pose_msg = PoseStamped()
        pose_msg.pose.orientation.w = 1.0
        filter_node._estimated_pose_callback(pose_msg)

        # 2. Provide neighbor measurement: Peer 2 directly ahead at 0.65m
        n_msg = NeighborList()
        m = NeighborMeasurement()
        m.neighbor_id = 2
        m.relative_x = 0.65
        m.relative_y = 0.0
        m.distance = 0.65
        m.bearing = 0.0
        n_msg.neighbors.append(m)
        filter_node._neighbors_callback(n_msg)

        # 3. Provide raw commanded velocity heading directly into peer at +1.0 m/s
        raw_cmd = Twist()
        raw_cmd.linear.x = 1.0
        raw_cmd.angular.z = 0.0
        filter_node._raw_cmd_callback(raw_cmd)

        # 4. Trigger filter loop
        filter_node._filter_loop()

        assert len(published_cmds) == 1, 'Safe cmd_vel was not published'
        assert len(published_interventions) == 1, 'Intervention metric was not published'

        safe_cmd = published_cmds[0]
        # Linear speed must be reduced to obey barrier
        assert safe_cmd.linear.x < 1.0
        assert published_interventions[0].data > 0.0

    finally:
        filter_node.destroy_node()


def test_head_on_collision_course_avoidance_simulation() -> None:
    """Simulate two unicycle robots on head-on collision course, verifying empirical safety."""
    # Robot 1 at (-1.5, 0), Robot 2 at (1.5, 0)
    r1_x, r1_y, r1_yaw = -1.5, 0.0, 0.0
    r2_x, r2_y, r2_yaw = 1.5, 0.0, math.pi

    dt = 0.02
    min_observed_distance = float('inf')

    for _ in range(150):  # 3.0 seconds simulation
        dist = math.hypot(r2_x - r1_x, r2_y - r1_y)
        if dist < min_observed_distance:
            min_observed_distance = dist

        # Desired aggressive commands: head straight toward each other at 1.0 m/s
        v1_des, w1_des = 1.0, 0.0
        v2_des, w2_des = 1.0, 0.0

        # Robot 1 CBF-QP
        rel_peers_1 = {2: (r2_x - r1_x, r2_y - r1_y)}
        safe_v1, safe_w1, _ = compute_cbf_safe_twist(
            v_raw=v1_des,
            w_raw=w1_des,
            robot_yaw=r1_yaw,
            relative_peers=rel_peers_1,
            relative_obstacles=[],
            safety_dist_peer=0.55,
            delay_bound_sec=0.02,
        )

        # Robot 2 CBF-QP
        rel_peers_2 = {1: (r1_x - r2_x, r1_y - r2_y)}
        safe_v2, safe_w2, _ = compute_cbf_safe_twist(
            v_raw=v2_des,
            w_raw=w2_des,
            robot_yaw=r2_yaw,
            relative_peers=rel_peers_2,
            relative_obstacles=[],
            safety_dist_peer=0.55,
            delay_bound_sec=0.02,
        )

        # Integrate kinematics
        r1_x += safe_v1 * math.cos(r1_yaw) * dt
        r1_y += safe_v1 * math.sin(r1_yaw) * dt
        r1_yaw += safe_w1 * dt

        r2_x += safe_v2 * math.cos(r2_yaw) * dt
        r2_y += safe_v2 * math.sin(r2_yaw) * dt
        r2_yaw += safe_w2 * dt

    # Empirically collision-free check: distance never drops below collision threshold 0.50m
    assert min_observed_distance >= 0.50, (
        f'Safety violation: min distance {min_observed_distance}m < 0.50m collision limit'
    )
