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

"""Onboard relative neighbor and obstacle sensing node for decentralized swarm robots."""

import math
import random
from typing import Dict, List, Optional, Tuple

from geometry_msgs.msg import Pose
from rclpy.node import Node
from swarm_interfaces.msg import (
    FaultInjection,
    NeighborList,
    NeighborMeasurement,
    ObstacleMeasurement,
)

from .fault_model import apply_sensor_bias
from .node_runner import run_node
from .relative_sensing import compute_relative_measurement, compute_relative_obstacle
from .world_model import landmarks_in_range, static_obstacles_in_range


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
        self.own_pose_time = 0.0

        # Last two (time, x, y) samples per peer, to evaluate peers at the own-pose time
        self.peer_poses: Dict[int, Tuple[float, float]] = {}
        self.peer_samples: Dict[int, List[Tuple[float, float, float]]] = {}
        self.active_sensor_fault: Optional[FaultInjection] = None

        # Publisher for onboard relative measurements
        self.pub_neighbors = self.create_publisher(
            NeighborList, output_topic, 10
        )

        # Subscriptions to ground truth poses from simulation environment
        self.sub_own_pose = self.create_subscription(
            Pose, f'/robot_{self.robot_id}/pose', self._own_pose_callback, 10
        )

        self.sub_sensor_fault = self.create_subscription(
            FaultInjection,
            f'/robot_{self.robot_id}/sensor_fault',
            self._sensor_fault_callback,
            10,
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
        self.own_pose_time = self.get_clock().now().nanoseconds * 1e-9

    def _peer_pose_callback(self, msg: Pose, peer_id: int) -> None:
        """Update peer robot ground truth pose."""
        self.peer_poses[peer_id] = (msg.position.x, msg.position.y)
        t = self.get_clock().now().nanoseconds * 1e-9
        hist = self.peer_samples.setdefault(peer_id, [])
        hist.append((t, msg.position.x, msg.position.y))
        if len(hist) > 2:
            del hist[0]

    def _peer_position_at(self, peer_id: int, t: float) -> Tuple[float, float]:
        """
        Peer position at time t, interpolated/extrapolated from its last two samples.

        Extrapolation is limited to 50 ms; with a single sample the latest pose is used.
        """
        hist = self.peer_samples.get(peer_id)
        if not hist or len(hist) < 2:
            return self.peer_poses[peer_id]
        (t0, x0, y0), (t1, x1, y1) = hist
        if t1 - t0 < 1e-6:
            return x1, y1
        tq = min(max(t, t0 - 0.05), t1 + 0.05)
        frac = (tq - t0) / (t1 - t0)
        return x0 + frac * (x1 - x0), y0 + frac * (y1 - y0)

    def _sensor_fault_callback(self, msg: FaultInjection) -> None:
        """Update active sensor fault state from fault injector node."""
        if msg.active:
            self.active_sensor_fault = msg
        else:
            self.active_sensor_fault = None

    def _publish_sensor_readings(self) -> None:
        """Evaluate relative sensing range and publish detected peers and obstacles."""
        if not self.has_own_pose:
            return

        if self.active_sensor_fault is not None:
            if self.active_sensor_fault.fault_type == 'intermittent_failure':
                # Blackout drop during active intermittent fault phase
                return

        msg = NeighborList()
        # Stamp with the time of the own pose the measurement is computed from, and
        # evaluate peers at that same time: a real sensor measures at one instant. Using
        # the publish time with poses of different ages (poses arrive at 50 Hz, the sensor
        # runs at 20 Hz) gave ~9 cm measurement errors once robots moved and turned.
        stamp_ns = int(self.own_pose_time * 1e9)
        msg.header.stamp.sec = stamp_ns // 1_000_000_000
        msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
        msg.header.frame_id = f'robot_{self.robot_id}/base_link'
        msg.robot_id = self.robot_id

        self_pos = (self.pos_x, self.pos_y)

        range_bias = 0.0
        bearing_bias = 0.0
        if (
            self.active_sensor_fault is not None
            and self.active_sensor_fault.fault_type == 'sensor_bias'
        ):
            range_bias = self.active_sensor_fault.range_bias
            bearing_bias = self.active_sensor_fault.bearing_bias

        # 1. Peer neighbor detection within finite sensing range
        for peer_id in range(1, self.total_robots + 1):
            if peer_id == self.robot_id:
                continue
            if peer_id not in self.peer_poses:
                continue

            target_pos = self._peer_position_at(peer_id, self.own_pose_time)
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
                dist = meas['distance']
                bearing = meas['bearing']
                if range_bias != 0.0 or bearing_bias != 0.0:
                    dist, bearing = apply_sensor_bias(dist, bearing, range_bias, bearing_bias)

                rel_x = dist * math.cos(bearing)
                rel_y = dist * math.sin(bearing)

                n_msg = NeighborMeasurement()
                n_msg.stamp = msg.header.stamp  # measurement time survives network delay
                n_msg.neighbor_id = peer_id
                n_msg.distance = dist
                n_msg.bearing = bearing
                n_msg.relative_x = rel_x
                n_msg.relative_y = rel_y
                n_msg.range_variance = meas['range_variance']
                n_msg.bearing_variance = meas['bearing_variance']
                msg.neighbors.append(n_msg)
            # Out-of-range robots are NOT added to msg.neighbors (completely invisible)

        # 2. Static obstacles and boundary walls within finite sensing range
        for obs_idx, ox, oy, orad in static_obstacles_in_range(
            self.pos_x, self.pos_y, self.sensing_range
        ):
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
                dist = obs_meas['distance']
                bearing = obs_meas['bearing']
                if range_bias != 0.0 or bearing_bias != 0.0:
                    dist, bearing = apply_sensor_bias(dist, bearing, range_bias, bearing_bias)

                rel_x = dist * math.cos(bearing)
                rel_y = dist * math.sin(bearing)

                o_msg = ObstacleMeasurement()
                o_msg.obstacle_id = obs_idx
                o_msg.distance = dist
                o_msg.bearing = bearing
                o_msg.relative_x = rel_x
                o_msg.relative_y = rel_y
                o_msg.radius = obs_meas['radius']
                msg.obstacles.append(o_msg)

        # 3. Landmark centres (for the EKF; not used for collision avoidance)
        for lm_id, lx, ly in landmarks_in_range(self.pos_x, self.pos_y, self.sensing_range):
            lm_meas = compute_relative_obstacle(
                self_pos=self_pos,
                self_yaw=self.yaw,
                obs_center=(lx, ly),
                obs_radius=0.0,
                sensing_range=self.sensing_range,
                range_noise_std=self.range_noise_std,
                bearing_noise_std=self.bearing_noise_std,
                rng=self.rng,
            )
            if lm_meas is None:
                continue
            dist, bearing = lm_meas['distance'], lm_meas['bearing']
            if range_bias != 0.0 or bearing_bias != 0.0:
                dist, bearing = apply_sensor_bias(dist, bearing, range_bias, bearing_bias)
            l_msg = ObstacleMeasurement()
            l_msg.obstacle_id = lm_id
            l_msg.distance = dist
            l_msg.bearing = bearing
            l_msg.relative_x = dist * math.cos(bearing)
            l_msg.relative_y = dist * math.sin(bearing)
            l_msg.radius = 0.0
            msg.landmarks.append(l_msg)

        self.pub_neighbors.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    """Entry point for NeighborSensorNode."""
    run_node(NeighborSensorNode, args)


if __name__ == '__main__':
    main()
