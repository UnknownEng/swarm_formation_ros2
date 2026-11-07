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

"""
Standalone Control Barrier Function (CBF-QP) safety filter node.

Intercepts raw commanded velocity (/cmd_vel_raw) from high-level swarm controller
and computes safe minimal-correction velocity (/cmd_vel) respecting barrier certificates.
"""

import math
from typing import Dict, List, Optional, Tuple

from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Float32
from swarm_interfaces.msg import NeighborList

from .cbf_qp import compute_cbf_safe_twist
from .node_runner import run_node
from .relative_sensing import transform_body_to_reference, YawHistory


class SafetyFilterNode(Node):
    """Executes high-frequency Control Barrier Function Quadratic Programming safety filtering."""

    def __init__(self) -> None:
        """Initialize parameters, publishers, subscribers, and periodic filter loop."""
        super().__init__('safety_filter_node')

        self.declare_parameter('robot_id', 1)
        self.declare_parameter('total_robots', 5)
        self.declare_parameter('safety_dist_peer', 0.55)
        self.declare_parameter('safety_dist_obs', 0.30)
        self.declare_parameter('cbf_gamma_peer', 1.5)
        self.declare_parameter('cbf_gamma_obs', 2.0)
        self.declare_parameter('delay_bound_sec', 0.05)
        self.declare_parameter('max_linear_vel', 1.4)
        self.declare_parameter('max_angular_vel', 3.0)
        self.declare_parameter('yaw_gain_kyaw', 4.0)
        self.declare_parameter('filter_rate', 50.0)
        self.declare_parameter('cbf_lookahead', 0.15)

        self.robot_id = int(self.get_parameter('robot_id').value)
        self.total_robots = int(self.get_parameter('total_robots').value)
        self.safety_dist_peer = float(self.get_parameter('safety_dist_peer').value)
        self.safety_dist_obs = float(self.get_parameter('safety_dist_obs').value)
        self.cbf_gamma_peer = float(self.get_parameter('cbf_gamma_peer').value)
        self.cbf_gamma_obs = float(self.get_parameter('cbf_gamma_obs').value)
        self.delay_bound_sec = float(self.get_parameter('delay_bound_sec').value)
        self.max_lin_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_ang_vel = float(self.get_parameter('max_angular_vel').value)
        self.k_yaw = float(self.get_parameter('yaw_gain_kyaw').value)
        self.filter_rate = float(self.get_parameter('filter_rate').value)
        self.cbf_lookahead = float(self.get_parameter('cbf_lookahead').value)

        self.dt = 1.0 / self.filter_rate

        # Tracking state
        self.raw_v = 0.0
        self.raw_w = 0.0
        self.last_raw_cmd_time: Optional[float] = None

        self.robot_yaw = 0.0
        self.has_received_pose = False
        self.yaw_history = YawHistory()

        # Relative measurements in reference frame: peer_id -> (dx, dy)
        self.relative_peers: Dict[int, Tuple[float, float]] = {}
        # Relative obstacles in reference frame: list of (ox, oy, radius)
        self.relative_obstacles: List[Tuple[float, float, float]] = []

        # Intercept and publish
        self.sub_raw_cmd = self.create_subscription(
            Twist, f'/robot_{self.robot_id}/cmd_vel_raw', self._raw_cmd_callback, 10
        )
        self.sub_estimated_pose = self.create_subscription(
            PoseStamped,
            f'/robot_{self.robot_id}/estimated_pose',
            self._estimated_pose_callback,
            10,
        )
        self.sub_neighbors = self.create_subscription(
            NeighborList, f'/robot_{self.robot_id}/neighbors', self._neighbors_callback, 10
        )

        self.pub_cmd_vel = self.create_publisher(
            Twist, f'/robot_{self.robot_id}/cmd_vel', 10
        )
        self.pub_intervention = self.create_publisher(
            Float32, f'/robot_{self.robot_id}/safety_intervention', 10
        )

        self.timer = self.create_timer(self.dt, self._filter_loop)
        self.get_logger().info(
            f'SafetyFilterNode initialized for Robot {self.robot_id} | '
            f'Peer Dist: {self.safety_dist_peer}m | Obs Dist: {self.safety_dist_obs}m | '
            f'Rate: {self.filter_rate}Hz'
        )

    def _raw_cmd_callback(self, msg: Twist) -> None:
        """Receive raw command from swarm controller."""
        self.raw_v = msg.linear.x
        self.raw_w = msg.angular.z
        self.last_raw_cmd_time = self.get_clock().now().nanoseconds * 1e-9

    def _estimated_pose_callback(self, msg: PoseStamped) -> None:
        """Update robot heading from onboard state estimator."""
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.has_received_pose = True
        self.yaw_history.add(self.get_clock().now().nanoseconds * 1e-9, self.robot_yaw)

    def _neighbors_callback(self, msg: NeighborList) -> None:
        """Update relative peer and obstacle detections."""
        if not self.has_received_pose:
            return

        new_peers: Dict[int, Tuple[float, float]] = {}
        for n in msg.neighbors:
            # Rotate with the heading at measurement time (the network may delay it)
            t_meas = n.stamp.sec + n.stamp.nanosec * 1e-9
            yaw = self.yaw_history.lookup(t_meas) if t_meas > 0.0 else None
            if yaw is None:
                yaw = self.robot_yaw
            dx, dy = transform_body_to_reference(n.relative_x, n.relative_y, yaw)
            new_peers[n.neighbor_id] = (dx, dy)
        self.relative_peers = new_peers

        new_obs: List[Tuple[float, float, float]] = []
        for o in msg.obstacles:
            ox, oy = transform_body_to_reference(o.relative_x, o.relative_y, self.robot_yaw)
            new_obs.append((ox, oy, o.radius))
        self.relative_obstacles = new_obs

    def _filter_loop(self) -> None:
        """Execute CBF-QP filter and publish safe velocity command."""
        now = self.get_clock().now().nanoseconds * 1e-9

        # Stale command protection (>0.6s without controller command = stop)
        if self.last_raw_cmd_time is None or (now - self.last_raw_cmd_time) > 0.6:
            cmd = Twist()
            self.pub_cmd_vel.publish(cmd)
            interv_msg = Float32()
            interv_msg.data = 0.0
            self.pub_intervention.publish(interv_msg)
            return

        safe_v, safe_w, intervention = compute_cbf_safe_twist(
            v_raw=self.raw_v,
            w_raw=self.raw_w,
            robot_yaw=self.robot_yaw,
            relative_peers=self.relative_peers,
            relative_obstacles=self.relative_obstacles,
            safety_dist_peer=self.safety_dist_peer,
            safety_dist_obs=self.safety_dist_obs,
            gamma_peer=self.cbf_gamma_peer,
            gamma_obs=self.cbf_gamma_obs,
            delay_bound_sec=self.delay_bound_sec,
            max_linear_vel=self.max_lin_vel,
            max_angular_vel=self.max_ang_vel,
            yaw_gain_kyaw=self.k_yaw,
            lookahead_dist=self.cbf_lookahead,
        )

        cmd = Twist()
        cmd.linear.x = safe_v
        cmd.angular.z = safe_w
        self.pub_cmd_vel.publish(cmd)

        interv_msg = Float32()
        interv_msg.data = float(intervention)
        self.pub_intervention.publish(interv_msg)


def main(args: Optional[List[str]] = None) -> None:
    """Entry point for SafetyFilterNode."""
    run_node(SafetyFilterNode, args)


if __name__ == '__main__':
    main()
