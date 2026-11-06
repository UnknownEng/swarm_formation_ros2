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

"""Acceptance tests for Milestone M5 (Localization Law)."""

import ast
import math
import os
from typing import List

from geometry_msgs.msg import Pose, PoseStamped, Twist, Vector3
from nav_msgs.msg import Odometry
import rclpy
from swarm_formation.localization_node import LocalizationNode
from swarm_formation.robot_node import GazeboSwarmNode
from swarm_interfaces.msg import NeighborList, NeighborMeasurement, ObstacleMeasurement


def test_robot_node_does_not_subscribe_to_ground_truth() -> None:
    """Verify that robot_node does NOT subscribe to ground truth /pose for control."""
    robot_node_path = os.path.join(
        os.path.dirname(__file__), '..', 'swarm_formation', 'robot_node.py'
    )
    with open(robot_node_path, 'r', encoding='utf-8') as f:
        source = f.read()

    tree = ast.parse(source)

    # 1. Ensure no subscription to ground truth pose topic
    assert "f'/robot_{self.robot_id}/pose'" not in source, (
        'Violation: robot_node must NOT subscribe to ground truth /pose'
    )

    # 2. Ensure subscription to estimated_pose is present
    assert "f'/robot_{self.robot_id}/estimated_pose'" in source, (
        'robot_node must subscribe to /estimated_pose from localization node'
    )

    # 3. Check AST for any calls to ground truth pose
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == 'pose_callback':
            raise AssertionError(
                'robot_node.py must NOT contain pose_callback for ground truth'
            )


def test_localization_node_ast_ground_truth_isolation() -> None:
    """Verify ground truth in localization_node is isolated for evaluation ONLY, never control."""
    loc_node_path = os.path.join(
        os.path.dirname(__file__), '..', 'swarm_formation', 'localization_node.py'
    )
    with open(loc_node_path, 'r', encoding='utf-8') as f:
        source = f.read()

    tree = ast.parse(source)

    gt_func: ast.FunctionDef = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == '_ground_truth_callback':
            gt_func = node
            break

    assert gt_func is not None, 'localization_node must contain _ground_truth_callback'

    gt_source = ast.get_source_segment(source, gt_func)
    assert 'self.ekf.predict' not in gt_source, (
        'Violation: EKF predict must NOT be called from ground truth callback'
    )
    assert 'self.ekf.update_range_bearing' not in gt_source, (
        'Violation: EKF update must NOT be called from ground truth callback'
    )


def test_localization_node_execution() -> None:
    """Verify that LocalizationNode processes odometry, neighbor measurements, and publishes."""
    if not rclpy.ok():
        rclpy.init()

    node = LocalizationNode()
    try:
        # Mock publisher recording
        published_poses: List[PoseStamped] = []
        published_errors: List[Vector3] = []

        class MockPosePublisher:
            def publish(self, msg: PoseStamped) -> None:
                published_poses.append(msg)

        class MockErrorPublisher:
            def publish(self, msg: Vector3) -> None:
                published_errors.append(msg)

        node.pub_estimated_pose = MockPosePublisher()
        node.pub_error = MockErrorPublisher()

        # 1. Send ground truth (for evaluation)
        gt_msg = Pose()
        gt_msg.position.x = 1.0
        gt_msg.position.y = 2.0
        gt_msg.orientation.w = 1.0
        node._ground_truth_callback(gt_msg)

        # 2. Send wheel odometry
        odom_msg = Odometry()
        odom_msg.twist.twist.linear.x = 0.5
        odom_msg.twist.twist.angular.z = 0.1
        node._odom_callback(odom_msg)

        # 3. Send neighbor measurements
        n_msg = NeighborList()
        meas = NeighborMeasurement()
        meas.neighbor_id = 2
        meas.distance = 1.414
        meas.bearing = 0.5
        n_msg.neighbors.append(meas)

        obs = ObstacleMeasurement()
        obs.obstacle_id = 1
        obs.distance = 2.0
        obs.bearing = 0.0
        n_msg.obstacles.append(obs)

        node._neighbors_callback(n_msg)

        # 4. Trigger periodic timer callback twice
        node._timer_callback()
        node._timer_callback()

        # Verify publications
        assert len(published_poses) >= 1, 'Estimated pose was not published'
        assert len(published_errors) >= 1, 'Estimation error was not published'

        latest_pose = published_poses[-1]
        assert latest_pose.header.frame_id == 'world'
        assert isinstance(latest_pose.pose.position.x, float)

        latest_error = published_errors[-1]
        assert latest_error.x >= 0.0  # Euclidean position error
        assert latest_error.y >= 0.0  # Absolute heading error
        assert latest_error.z > 0.0  # Covariance trace

    finally:
        node.destroy_node()


def test_robot_node_receives_estimated_pose() -> None:
    """Verify robot_node accepts PoseStamped estimated pose and computes control."""
    if not rclpy.ok():
        rclpy.init()

    robot = GazeboSwarmNode()
    try:
        published_cmds: List[Twist] = []

        class MockCmdPublisher:
            def publish(self, msg: Twist) -> None:
                published_cmds.append(msg)

        robot.pub_cmd_vel = MockCmdPublisher()

        # Send estimated pose
        est_pose = PoseStamped()
        est_pose.header.frame_id = 'world'
        est_pose.pose.position.x = 1.0
        est_pose.pose.position.y = 1.0
        # 45 degrees yaw (0.785 rad) -> half_yaw = 0.3927 -> sin = 0.38268, cos = 0.92388
        est_pose.pose.orientation.z = math.sin(math.pi / 8)
        est_pose.pose.orientation.w = math.cos(math.pi / 8)

        robot._estimated_pose_callback(est_pose)
        assert robot.has_received_yaw
        assert math.isclose(robot.yaw, math.pi / 4, abs_tol=1e-3)

        # Send neighbor measurement
        n_msg = NeighborList()
        m = NeighborMeasurement()
        m.neighbor_id = 2
        m.relative_x = 1.0
        m.relative_y = 0.0
        m.distance = 1.0
        m.bearing = 0.0
        n_msg.neighbors.append(m)
        robot.neighbor_callback(n_msg)

        # Execute control loop
        robot.control_loop()
        assert len(published_cmds) == 1, 'Control command was not published'
        assert isinstance(published_cmds[0].linear.x, float)
        assert isinstance(published_cmds[0].angular.z, float)

    finally:
        robot.destroy_node()
