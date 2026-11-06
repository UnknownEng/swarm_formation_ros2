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

"""Onboard relative neighbor and obstacle sensing node for decentralized swarm robots."""

import math
import random
from typing import Dict, List, Optional, Tuple

from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import NeighborList, NeighborMeasurement, ObstacleMeasurement

from .relative_sensing import compute_relative_measurement, compute_relative_obstacle


class NeighborSensorNode(Node):
    """Simulates physical onboard relative range-bearing sensors for a swarm robot."""

    def __init__(self) -> None:
        """Initialize parameters, publishers, subscribers, and sensor publishing timer."""
        super().__init__('neighbor_sensor_node')

        self.declare_parameter('robot_id', 1)
        self.declare_parameter('total_robots', 5)
        self.declare_parameter('sensing_range', 6.0)
        self.declare_parameter('range_noise_std', 0.0)
        self.declare_parameter('bearing_noise_std', 0.0)
        self.declare_parameter('sensor_rate', 20.0)
        self.declare_parameter('output_topic', '')

        self.robot_id = int(self.get_parameter('robot_id').value)
        self.total_robots = int(self.get_parameter('total_robots').value)
        self.sensing_range = float(self.get_parameter('sensing_range').value)
        self.range_noise_std = float(self.get_parameter('range_noise_std').value)
        self.bearing_noise_std = float(self.get_parameter('bearing_noise_std').value)
        self.sensor_rate = float(self.get_parameter('sensor_rate').value)
        output_topic = str(self.get_parameter('output_topic').value)
        if not output_topic:
            output_topic = f'/robot_{self.robot_id}/neighbors_raw'

        self.rng = random.Random(42 + self.robot_id)

        # Ground truth state tracking strictly inside sensor simulation
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        self.has_own_pose = False

        self.peer_poses: Dict[int, Tuple[float, float]] = {}

        # Environmental Obstacles (Disaster Zone)
        self.disaster_obstacles: List[Tuple[float, float, float]] = [
            (-1.66, 1.40, 0.15),
            (1.92, 1.87, 0.15),
            (0.25, 1.96, 0.22),
            (0.95, -0.33, 0.22),
            (-1.80, -1.09, 0.16),
            (-0.12, -1.89, 0.16),
            (1.87, -1.24, 0.16),
        ]

        # Publisher for onboard relative measurements
        self.pub_neighbors = self.create_publisher(
            NeighborList, output_topic, 10
        )

        # Subscriptions to ground truth poses from simulation environment
        self.sub_own_pose = self.create_subscription(
            Pose, f'/robot_{self.robot_id}/pose', self._own_pose_callback, 10
        )

        self.peer_subs = []
        for j in range(1, self.total_robots + 1):
            if j != self.robot_id:
                sub = self.create_subscription(
                    Pose,
                    f'/robot_{j}/pose',
                    lambda msg, peer_id=j: self._peer_pose_callback(msg, peer_id),
                    10,
                )
                self.peer_subs.append(sub)

        dt = 1.0 / self.sensor_rate
        self.timer = self.create_timer(dt, self._publish_sensor_readings)

        self.get_logger().info(
            f'NeighborSensorNode initialized for Robot {self.robot_id} | '
            f'Range: {self.sensing_range}m | Rate: {self.sensor_rate}Hz'
        )

    def _own_pose_callback(self, msg: Pose) -> None:
        """Update observing robot ground truth pose."""
        self.pos_x = msg.position.x
        self.pos_y = msg.position.y

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.has_own_pose = True

    def _peer_pose_callback(self, msg: Pose, peer_id: int) -> None:
        """Update peer robot ground truth pose."""
        self.peer_poses[peer_id] = (msg.position.x, msg.position.y)

    def _publish_sensor_readings(self) -> None:
        """Evaluate relative sensing range and publish detected peers and obstacles."""
        if not self.has_own_pose:
            return

        msg = NeighborList()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'robot_{self.robot_id}/base_link'
        msg.robot_id = self.robot_id

        self_pos = (self.pos_x, self.pos_y)

        # 1. Peer neighbor detection within finite sensing range
        for peer_id in range(1, self.total_robots + 1):
            if peer_id == self.robot_id:
                continue
            if peer_id not in self.peer_poses:
                continue

            target_pos = self.peer_poses[peer_id]
            meas = compute_relative_measurement(
                self_pos=self_pos,
                self_yaw=self.yaw,
                target_pos=target_pos,
                sensing_range=self.sensing_range,
                range_noise_std=self.range_noise_std,
                bearing_noise_std=self.bearing_noise_std,
                rng=self.rng,
            )

            if meas is not None:
                n_msg = NeighborMeasurement()
                n_msg.neighbor_id = peer_id
                n_msg.distance = meas['distance']
                n_msg.bearing = meas['bearing']
                n_msg.relative_x = meas['relative_x']
                n_msg.relative_y = meas['relative_y']
                n_msg.range_variance = meas['range_variance']
                n_msg.bearing_variance = meas['bearing_variance']
                msg.neighbors.append(n_msg)
            # Out-of-range robots are NOT added to msg.neighbors (completely invisible)

        # 2. Environmental obstacle detection within finite sensing range
        for obs_idx, (ox, oy, orad) in enumerate(self.disaster_obstacles, start=1):
            obs_meas = compute_relative_obstacle(
                self_pos=self_pos,
                self_yaw=self.yaw,
                obs_center=(ox, oy),
                obs_radius=orad,
                sensing_range=self.sensing_range,
                range_noise_std=self.range_noise_std,
                bearing_noise_std=self.bearing_noise_std,
                rng=self.rng,
            )

            if obs_meas is not None:
                o_msg = ObstacleMeasurement()
                o_msg.obstacle_id = obs_idx
                o_msg.distance = obs_meas['distance']
                o_msg.bearing = obs_meas['bearing']
                o_msg.relative_x = obs_meas['relative_x']
                o_msg.relative_y = obs_meas['relative_y']
                o_msg.radius = obs_meas['radius']
                msg.obstacles.append(o_msg)

        self.pub_neighbors.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    """Entry point for NeighborSensorNode."""
    rclpy.init(args=args)
    node = NeighborSensorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception, BaseException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
