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
Standalone cooperative EKF localization node for decentralized swarm robots.

Fuses wheel odometry dead-reckoning with relative range-bearing neighbor
and landmark observations without accessing global ground truth for control.
"""

import math
from typing import Dict, List, Optional, Tuple

from geometry_msgs.msg import Pose, PoseStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from swarm_interfaces.msg import NeighborList

from .kinematics import wrap_angle
from .localization import compute_estimation_error, RobotEKF
from .node_runner import run_node


class LocalizationNode(Node):
    """
    Onboard state estimator executing cooperative EKF localization for a swarm robot.

    Ground truth is subscribed strictly for estimation error quantification.
    """

    def __init__(self) -> None:
        """Initialize parameters, EKF filter, publishers, subscribers, and timers."""
        super().__init__('localization_node')

        self.declare_parameter('robot_id', 1)
        self.declare_parameter('total_robots', 5)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('use_landmarks', True)

        self.robot_id = int(self.get_parameter('robot_id').value)
        self.total_robots = int(self.get_parameter('total_robots').value)
        init_x = float(self.get_parameter('initial_x').value)
        init_y = float(self.get_parameter('initial_y').value)
        init_yaw = float(self.get_parameter('initial_yaw').value)
        self.initial_yaw = init_yaw
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.use_landmarks = bool(self.get_parameter('use_landmarks').value)

        # Initialize EKF filter
        self.ekf = RobotEKF(
            init_x=init_x,
            init_y=init_y,
            init_yaw=init_yaw,
            process_var_pos=0.04,
            process_var_yaw=0.02,
            meas_var_range=0.05,
            meas_var_bearing=0.03,
        )

        # Odometry tracking state
        self.last_predict_time: Optional[float] = None
        self.current_v = 0.0
        self.current_w = 0.0

        # Peer estimated poses: peer_id -> (x, y, yaw, timestamp)
        self.peer_estimates: Dict[int, Tuple[float, float, float, float]] = {}

        # Known environmental landmarks in world frame: obstacle_id -> (ox, oy, radius)
        self.known_landmarks: Dict[int, Tuple[float, float, float]] = {
            1: (-1.66, 1.40, 0.15),
            2: (1.92, 1.87, 0.15),
            3: (0.25, 1.96, 0.22),
            4: (0.95, -0.33, 0.22),
            5: (-1.80, -1.09, 0.16),
            6: (-0.12, -1.89, 0.16),
            7: (1.87, -1.24, 0.16),
        }

        # Ground truth tracking for evaluation only
        self.has_ground_truth = False
        self.gt_x = 0.0
        self.gt_y = 0.0
        self.gt_yaw = 0.0

        # Performance metric accumulators
        self.sample_count = 0
        self.sum_pos_err = 0.0
        self.sum_sq_pos_err = 0.0
        self.max_pos_err = 0.0
        self.last_pos_err = 0.0
        self.last_yaw_err = 0.0

        # Publishers
        self.pub_estimated_pose = self.create_publisher(
            PoseStamped, f'/robot_{self.robot_id}/estimated_pose', 10
        )
        self.pub_error = self.create_publisher(
            Vector3, f'/robot_{self.robot_id}/estimation_error', 10
        )

        # Subscriptions for control state estimation
        # 1. Odometry from diff-drive
        self.sub_odom = self.create_subscription(
            Odometry, f'/robot_{self.robot_id}/odom', self._odom_callback, 10
        )
        # 2. Cmd_vel backup feed
        self.sub_cmd_vel = self.create_subscription(
            Twist, f'/robot_{self.robot_id}/cmd_vel', self._cmd_vel_callback, 10
        )
        # 3. Relative neighbor and obstacle measurements
        self.sub_neighbors = self.create_subscription(
            NeighborList, f'/robot_{self.robot_id}/neighbors', self._neighbors_callback, 10
        )

        # 4. Peer estimated poses from network
        self.peer_subs = []
        for j in range(1, self.total_robots + 1):
            if j != self.robot_id:
                sub = self.create_subscription(
                    PoseStamped,
                    f'/robot_{j}/estimated_pose',
                    lambda msg, peer_id=j: self._peer_estimate_callback(msg, peer_id),
                    10,
                )
                self.peer_subs.append(sub)

        # 5. Gazebo Ground Truth (EVALUATION ONLY)
        self.sub_gt = self.create_subscription(
            Pose, f'/robot_{self.robot_id}/pose', self._ground_truth_callback, 10
        )

        dt = 1.0 / self.publish_rate
        self.timer = self.create_timer(dt, self._timer_callback)

        self.get_logger().info(
            f'LocalizationNode initialized for Robot {self.robot_id} | '
            f'Init: ({init_x:.2f}, {init_y:.2f}, {init_yaw:.2f} rad) | '
            f'Rate: {self.publish_rate}Hz'
        )

    def _odom_callback(self, msg: Odometry) -> None:
        """Update velocity inputs and fused heading from wheel odometry."""
        self.current_v = msg.twist.twist.linear.x
        self.current_w = msg.twist.twist.angular.z

        # Extract odometric integrated heading
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        odom_yaw = math.atan2(siny_cosp, cosy_cosp)
        total_yaw = wrap_angle(self.initial_yaw + odom_yaw)
        self.ekf.update_heading(total_yaw, var_yaw=0.01)

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """Update velocity input fallback if odometry is delayed."""
        if self.current_v == 0.0 and self.current_w == 0.0:
            self.current_v = msg.linear.x
            self.current_w = msg.angular.z

    def _ground_truth_callback(self, msg: Pose) -> None:
        """
        Record Gazebo ground truth pose strictly for evaluation and metric logging.

        IMPORTANT: This callback never feeds into the EKF estimator or robot control.
        """
        self.gt_x = msg.position.x
        self.gt_y = msg.position.y

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.gt_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.has_ground_truth = True

    def _peer_estimate_callback(self, msg: PoseStamped, peer_id: int) -> None:
        """Record received peer estimated reference position."""
        px = msg.pose.position.x
        py = msg.pose.position.y
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        pyaw = math.atan2(siny_cosp, cosy_cosp)
        now = self.get_clock().now().nanoseconds * 1e-9
        self.peer_estimates[peer_id] = (px, py, pyaw, now)

    def _neighbors_callback(self, msg: NeighborList) -> None:
        """Process onboard relative measurements and execute EKF correction step."""
        # 1. Update with known environmental landmarks
        if self.use_landmarks:
            for obs in msg.obstacles:
                o_id = obs.obstacle_id
                if o_id in self.known_landmarks:
                    lx, ly, _ = self.known_landmarks[o_id]
                    self.ekf.update_range_bearing(
                        target_x=lx,
                        target_y=ly,
                        measured_dist=obs.distance,
                        measured_bearing=obs.bearing,
                        var_range=0.04,
                        var_bearing=0.02,
                    )

        # 2. Update with cooperative peer estimates
        for n in msg.neighbors:
            p_id = n.neighbor_id
            if p_id in self.peer_estimates:
                peer_x, peer_y, _, last_time = self.peer_estimates[p_id]
                now = self.get_clock().now().nanoseconds * 1e-9
                # Only use recent peer estimates (<1.5s old)
                if (now - last_time) < 1.5:
                    self.ekf.update_range_bearing(
                        target_x=peer_x,
                        target_y=peer_y,
                        measured_dist=n.distance,
                        measured_bearing=n.bearing,
                        var_range=n.range_variance if n.range_variance > 0 else 0.05,
                        var_bearing=n.bearing_variance if n.bearing_variance > 0 else 0.03,
                    )

    def _timer_callback(self) -> None:
        """Execute periodic prediction, publish estimated pose, and record evaluation error."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_predict_time is None:
            self.last_predict_time = now
            return

        dt = now - self.last_predict_time
        self.last_predict_time = now

        # EKF Prediction step
        self.ekf.predict(self.current_v, self.current_w, dt)

        est_x, est_y, est_yaw = self.ekf.get_pose()

        # Publish estimated pose for control and peer sharing
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = est_x
        msg.pose.position.y = est_y
        msg.pose.position.z = 0.0

        half_yaw = est_yaw * 0.5
        msg.pose.orientation.z = math.sin(half_yaw)
        msg.pose.orientation.w = math.cos(half_yaw)
        self.pub_estimated_pose.publish(msg)

        # Evaluate estimation error against ground truth (EVALUATION ONLY)
        if self.has_ground_truth:
            pos_err, yaw_err = compute_estimation_error(
                est_x, est_y, est_yaw, self.gt_x, self.gt_y, self.gt_yaw
            )
            self.last_pos_err = pos_err
            self.last_yaw_err = yaw_err
            self.sample_count += 1
            self.sum_pos_err += pos_err
            self.sum_sq_pos_err += pos_err * pos_err
            if pos_err > self.max_pos_err:
                self.max_pos_err = pos_err

            cov = self.ekf.get_covariance()
            trace_cov = cov[0][0] + cov[1][1]

            err_msg = Vector3()
            err_msg.x = pos_err
            err_msg.y = yaw_err
            err_msg.z = trace_cov
            self.pub_error.publish(err_msg)


def main(args: Optional[List[str]] = None) -> None:
    """Entry point for LocalizationNode."""
    run_node(LocalizationNode, args)


if __name__ == '__main__':
    main()
