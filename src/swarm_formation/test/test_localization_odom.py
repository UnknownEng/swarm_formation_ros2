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

"""Regression tests for the odometry path into the localization EKF."""

import os

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy

from swarm_formation.localization_node import LocalizationNode

LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'launch', 'gazebo_swarm_launch.py')


def _twist(v: float, w: float) -> Twist:
    msg = Twist()
    msg.linear.x = v
    msg.angular.z = w
    return msg


def test_cmd_vel_fallback_tracks_every_command_without_odometry() -> None:
    """Without odometry the EKF must follow each new command, not freeze on the first."""
    if not rclpy.ok():
        rclpy.init()
    node = LocalizationNode()
    try:
        node._cmd_vel_callback(_twist(0.5, 0.1))
        assert (node.current_v, node.current_w) == (0.5, 0.1)
        node._cmd_vel_callback(_twist(-0.2, 1.5))
        assert (node.current_v, node.current_w) == (-0.2, 1.5)
    finally:
        node.destroy_node()


def test_fresh_odometry_takes_precedence_over_cmd_vel() -> None:
    """While odometry is fresh, commanded velocities are ignored."""
    if not rclpy.ok():
        rclpy.init()
    node = LocalizationNode()
    try:
        odom = Odometry()
        odom.twist.twist.linear.x = 0.3
        odom.twist.twist.angular.z = 0.2
        odom.pose.pose.orientation.w = 1.0
        node._odom_callback(odom)
        node._cmd_vel_callback(_twist(1.0, -1.0))
        assert (node.current_v, node.current_w) == (0.3, 0.2)
    finally:
        node.destroy_node()


def test_bridge_uses_the_diff_drive_odometry_message_type() -> None:
    """The DiffDrive plugin publishes gz.msgs.Odometry; other bridge types deliver nothing."""
    src = open(LAUNCH, encoding='utf-8').read()
    assert "odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'" in src
    assert 'OdometryWithCovariance' not in src
