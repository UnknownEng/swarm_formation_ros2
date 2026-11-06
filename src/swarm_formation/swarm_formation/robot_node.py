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

"""ROS 2 control node interfacing physical Gazebo robots with decentralized consensus & APF."""

import math
import random
from typing import Dict, List, Optional, Tuple

from geometry_msgs.msg import Pose, Twist
import rclpy
import rclpy.executors
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from .consensus import (
    compute_baseline_consensus,
    compute_huber_consensus,
    compute_target_offsets,
)
from .kinematics import compute_unicycle_twist
from .potential_field import (
    compute_obstacle_apf,
    compute_peer_apf,
    compute_peer_safety_scale,
)


class GazeboSwarmNode(Node):
    """Controls a physical mobile robot in Gazebo Harmonic using consensus and APF safety."""

    def __init__(self) -> None:
        """Initialize parameters, publishers, subscribers, and control loop."""
        super().__init__('gazebo_swarm_node')

        # Core parameters
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('total_robots', 5)
        self.declare_parameter('enable_faults', False)
        self.declare_parameter('use_resilient_method', False)
        self.declare_parameter('formation_radius', 2.0)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('max_linear_vel', 1.4)
        self.declare_parameter('max_angular_vel', 3.0)

        # Control & Consensus gains
        self.declare_parameter('linear_gain_kp', 1.8)
        self.declare_parameter('damping_gain_kd', 0.5)
        self.declare_parameter('yaw_gain_kyaw', 4.0)

        # Robust Consensus & Filtering
        self.declare_parameter('huber_delta', 2.0)
        self.declare_parameter('ema_alpha', 0.35)
        self.declare_parameter('heartbeat_timeout', 1.2)

        # APF Layer 1: Peer safety barrier
        self.declare_parameter('apf_collision_dist', 0.50)
        self.declare_parameter('apf_safe_dist', 1.10)
        self.declare_parameter('apf_influence_dist', 1.80)
        self.declare_parameter('apf_repulsion_gain', 1.2)
        self.declare_parameter('apf_barrier_gain', 2.0)
        self.declare_parameter('apf_damping_gain', 1.5)
        self.declare_parameter('apf_lateral_ratio', 0.6)

        # APF Layer 2: Obstacle safety barrier
        self.declare_parameter('obs_influence_dist', 0.45)
        self.declare_parameter('obs_safe_dist', 0.30)
        self.declare_parameter('obs_repulsion_gain', 0.6)
        self.declare_parameter('obs_barrier_gain', 0.8)
        self.declare_parameter('obs_damping_gain', 0.8)
        self.declare_parameter('obs_tangent_ratio', 1.0)

        # Configurable Fault Injection Parameters
        self.declare_parameter('fault_packet_loss_rate', 0.15)
        self.declare_parameter('fault_noise_std', 0.18)
        self.declare_parameter('fault_delay_min', 0.05)
        self.declare_parameter('fault_delay_max', 0.15)
        self.declare_parameter('fault_dropout_robot_id', 4)
        self.declare_parameter('fault_dropout_start_sec', 5.0)
        self.declare_parameter('fault_dropout_end_sec', 10.0)

        # Retrieve parameter values
        self.robot_id = int(self.get_parameter('robot_id').value)
        self.total_robots = int(self.get_parameter('total_robots').value)
        self.enable_faults = bool(self.get_parameter('enable_faults').value)
        self.use_resilient_method = bool(self.get_parameter('use_resilient_method').value)
        self.radius = float(self.get_parameter('formation_radius').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.max_lin_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_ang_vel = float(self.get_parameter('max_angular_vel').value)

        self.k_p = float(self.get_parameter('linear_gain_kp').value)
        self.k_d = float(self.get_parameter('damping_gain_kd').value)
        self.k_yaw = float(self.get_parameter('yaw_gain_kyaw').value)

        self.huber_delta = float(self.get_parameter('huber_delta').value)
        self.ema_alpha = float(self.get_parameter('ema_alpha').value)
        self.heartbeat_timeout = float(self.get_parameter('heartbeat_timeout').value)

        self.d_collision = float(self.get_parameter('apf_collision_dist').value)
        self.d_safe = float(self.get_parameter('apf_safe_dist').value)
        self.d_infl = float(self.get_parameter('apf_influence_dist').value)
        self.k_rep = float(self.get_parameter('apf_repulsion_gain').value)
        self.k_barrier = float(self.get_parameter('apf_barrier_gain').value)
        self.k_damp_apf = float(self.get_parameter('apf_damping_gain').value)
        self.apf_lateral_ratio = float(self.get_parameter('apf_lateral_ratio').value)

        self.d_obs_infl = float(self.get_parameter('obs_influence_dist').value)
        self.d_obs_safe = float(self.get_parameter('obs_safe_dist').value)
        self.k_obs_rep = float(self.get_parameter('obs_repulsion_gain').value)
        self.k_obs_barrier = float(self.get_parameter('obs_barrier_gain').value)
        self.k_obs_damp = float(self.get_parameter('obs_damping_gain').value)
        self.obs_tangent_ratio = float(self.get_parameter('obs_tangent_ratio').value)

        self.fault_loss_rate = float(self.get_parameter('fault_packet_loss_rate').value)
        self.fault_noise_std = float(self.get_parameter('fault_noise_std').value)
        self.fault_delay_min = float(self.get_parameter('fault_delay_min').value)
        self.fault_delay_max = float(self.get_parameter('fault_delay_max').value)
        self.fault_dropout_id = int(self.get_parameter('fault_dropout_robot_id').value)
        self.fault_dropout_start = float(self.get_parameter('fault_dropout_start_sec').value)
        self.fault_dropout_end = float(self.get_parameter('fault_dropout_end_sec').value)

        self.dt = 1.0 / self.control_rate

        # Physical state from Gazebo
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.last_pose_time: Optional[float] = None
        self.has_received_pose = False

        # Dual-Layer APF Layer 2: Environmental Obstacles (Disaster Zone)
        self.disaster_obstacles: List[Tuple[float, float, float]] = [
            (-1.66, 1.40, 0.15),  # Structural Steel Pillar 1
            (1.92, 1.87, 0.15),   # Structural Steel Pillar 2
            (0.25, 1.96, 0.22),   # Fallen Concrete Slab 1
            (0.95, -0.33, 0.22),  # Fallen Concrete Slab 2
            (-1.80, -1.09, 0.16),  # Broken Rubble Block 1
            (-0.12, -1.89, 0.16),  # Broken Rubble Block 2
            (1.87, -1.24, 0.16),  # Broken Rubble Block 3
        ]

        # Neighbor state tracking
        self.neighbor_states: Dict[int, Tuple[float, float, float, float, float]] = {}
        self.filtered_states: Dict[int, Tuple[float, float, float, float]] = {}
        self.last_valid_beacon_times: Dict[int, float] = {}
        self.packet_delay_queue: List[Tuple[float, Tuple[int, float, float, float, float]]] = []

        # Fault tracking
        self.is_dropped_out = False
        self.start_time: Optional[float] = None

        # Precompute target formation offsets
        self.target_offsets = compute_target_offsets(
            self.robot_id, self.total_robots, self.radius
        )

        # ROS 2 Topics
        self.pub_cmd_vel = self.create_publisher(
            Twist, f'/robot_{self.robot_id}/cmd_vel', 10
        )
        self.pub_beacon = self.create_publisher(Float32MultiArray, '/robot_all_poses', 20)
        self.sub_beacon = self.create_subscription(
            Float32MultiArray, '/robot_all_poses', self.beacon_callback, 20
        )
        self.sub_pose = self.create_subscription(
            Pose, f'/robot_{self.robot_id}/pose', self.pose_callback, 10
        )

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(
            f'GazeboSwarmNode {self.robot_id}/{self.total_robots} ready | '
            f'Faults: {self.enable_faults} | Resilient: {self.use_resilient_method}'
        )

    def pose_callback(self, msg: Pose) -> None:
        """Update robot physical pose and yaw from Gazebo physics simulation."""
        now = self.get_clock().now().nanoseconds * 1e-9
        new_x = msg.position.x
        new_y = msg.position.y

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.last_pose_time is not None:
            dt_pose = max(now - self.last_pose_time, 1e-4)
            self.vel_x = (new_x - self.pos_x) / dt_pose
            self.vel_y = (new_y - self.pos_y) / dt_pose

        self.pos_x = new_x
        self.pos_y = new_y
        self.last_pose_time = now
        self.has_received_pose = True

    def beacon_callback(self, msg: Float32MultiArray) -> None:
        """Handle peer beacon broadcast with real-world packet drop, delay, and noise."""
        data = msg.data
        if len(data) < 3:
            return

        sender_id = int(data[0])
        if sender_id == self.robot_id:
            return

        curr_time = self.get_clock().now().nanoseconds * 1e-9
        raw_x = float(data[1])
        raw_y = float(data[2])
        raw_vx = float(data[3]) if len(data) >= 5 else 0.0
        raw_vy = float(data[4]) if len(data) >= 5 else 0.0

        if self.enable_faults:
            if random.random() < self.fault_loss_rate:
                return

            noise_x = random.gauss(0.0, self.fault_noise_std)
            noise_y = random.gauss(0.0, self.fault_noise_std)
            noisy_x = raw_x + noise_x
            noisy_y = raw_y + noise_y

            delay = random.uniform(self.fault_delay_min, self.fault_delay_max)
            delivery_time = curr_time + delay
            self.packet_delay_queue.append(
                (delivery_time, (sender_id, noisy_x, noisy_y, raw_vx, raw_vy))
            )
        else:
            self.neighbor_states[sender_id] = (raw_x, raw_y, raw_vx, raw_vy, curr_time)

    def _process_delayed_packets(self, curr_time: float) -> None:
        """Process buffered packets whose transmission delay has passed."""
        ready_packets = []
        remaining = []
        for delivery_time, payload in self.packet_delay_queue:
            if curr_time >= delivery_time:
                ready_packets.append(payload)
            else:
                remaining.append((delivery_time, payload))
        self.packet_delay_queue = remaining

        for sender_id, px, py, vx, vy in ready_packets:
            self.neighbor_states[sender_id] = (px, py, vx, vy, curr_time)

    def _check_agent_dropout(self, elapsed: float) -> bool:
        """Simulate agent dropout fault for configured robot during dropout window."""
        if self.enable_faults and self.robot_id == self.fault_dropout_id:
            if self.fault_dropout_start <= elapsed <= self.fault_dropout_end:
                if not self.is_dropped_out:
                    self.get_logger().warn(
                        f'Physical Robot {self.robot_id} entered FAULT DROPOUT!'
                    )
                    self.is_dropped_out = True
                return True
            if self.is_dropped_out:
                self.get_logger().info(
                    f'Physical Robot {self.robot_id} RECOVERED from fault dropout.'
                )
                self.is_dropped_out = False
        return False

    def control_loop(self) -> None:
        """Compute consensus force, convert to unicycle velocity, and publish to Gazebo."""
        if not self.has_received_pose:
            return

        curr_time = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = curr_time

        elapsed = curr_time - self.start_time
        dropped_out = self._check_agent_dropout(elapsed)

        # Broadcast state beacon if active
        if not dropped_out:
            beacon_msg = Float32MultiArray()
            beacon_msg.data = [
                float(self.robot_id),
                self.pos_x,
                self.pos_y,
                self.vel_x,
                self.vel_y,
                curr_time,
            ]
            self.pub_beacon.publish(beacon_msg)

        if self.enable_faults:
            self._process_delayed_packets(curr_time)

        if dropped_out:
            self.pub_cmd_vel.publish(Twist())
            return

        if self.use_resilient_method:
            fx, fy = self._compute_resilient_control(curr_time)
        else:
            fx, fy = compute_baseline_consensus(
                self.pos_x, self.pos_y, self.target_offsets, self.neighbor_states, self.k_p
            )

        # Velocity damping
        fx -= self.k_d * self.vel_x
        fy -= self.k_d * self.vel_y

        v_cmd, w_cmd = compute_unicycle_twist(
            fx, fy, self.yaw, self.k_yaw, self.max_lin_vel, self.max_ang_vel
        )

        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.pub_cmd_vel.publish(cmd)

    def _compute_resilient_control(self, curr_time: float) -> Tuple[float, float]:
        """Compute robust Huber consensus with dual-layer APF safety barrier."""
        active_mask: Dict[int, bool] = {}

        for n_id in self.target_offsets.keys():
            if n_id not in self.neighbor_states:
                continue

            raw_nx, raw_ny, n_vx, n_vy, last_seen = self.neighbor_states[n_id]
            is_active = (curr_time - last_seen) <= self.heartbeat_timeout
            active_mask[n_id] = is_active

            time_gap = curr_time - self.last_valid_beacon_times.get(n_id, 0.0)

            if time_gap > 0.8:
                filt_x, filt_y = raw_nx, raw_ny
                filt_vx, filt_vy = n_vx, n_vy
                self.filtered_states[n_id] = (filt_x, filt_y, filt_vx, filt_vy)
                self.last_valid_beacon_times[n_id] = curr_time
            elif n_id in self.filtered_states:
                prev_x, prev_y, prev_vx, prev_vy = self.filtered_states[n_id]
                jump = math.hypot(raw_nx - prev_x, raw_ny - prev_y)
                if jump > 1.2:
                    filt_x, filt_y = prev_x, prev_y
                    filt_vx, filt_vy = prev_vx, prev_vy
                else:
                    alpha = self.ema_alpha
                    filt_x = (1.0 - alpha) * prev_x + alpha * raw_nx
                    filt_y = (1.0 - alpha) * prev_y + alpha * raw_ny
                    filt_vx = 0.5 * prev_vx + 0.5 * n_vx
                    filt_vy = 0.5 * prev_vy + 0.5 * n_vy
                    self.last_valid_beacon_times[n_id] = curr_time
                self.filtered_states[n_id] = (filt_x, filt_y, filt_vx, filt_vy)
            else:
                filt_x, filt_y = raw_nx, raw_ny
                filt_vx, filt_vy = n_vx, n_vy
                self.filtered_states[n_id] = (filt_x, filt_y, filt_vx, filt_vy)
                self.last_valid_beacon_times[n_id] = curr_time

        # 1. Huber robust consensus
        fx_form, fy_form, _ = compute_huber_consensus(
            self.pos_x,
            self.pos_y,
            self.target_offsets,
            self.filtered_states,
            active_mask,
            self.k_p,
            self.huber_delta,
        )

        # 2. Layer 1: Peer APF Safety Barrier
        fx_apf, fy_apf, min_peer_dist = compute_peer_apf(
            self.pos_x,
            self.pos_y,
            self.vel_x,
            self.vel_y,
            self.filtered_states,
            self.d_infl,
            self.d_safe,
            self.d_collision,
            self.k_rep,
            self.k_barrier,
            self.k_damp_apf,
            self.apf_lateral_ratio,
        )

        # 3. Layer 2: Environmental Obstacle APF
        fx_obs, fy_obs = compute_obstacle_apf(
            self.pos_x,
            self.pos_y,
            self.vel_x,
            self.vel_y,
            fx_form,
            fy_form,
            self.disaster_obstacles,
            self.d_obs_infl,
            self.d_obs_safe,
            self.k_obs_rep,
            self.k_obs_barrier,
            self.k_obs_damp,
            self.obs_tangent_ratio,
        )

        # 4. Peer proximity priority attenuation
        safety_scale = compute_peer_safety_scale(
            min_peer_dist, self.d_safe, self.d_collision
        )
        fx_form *= safety_scale
        fy_form *= safety_scale

        return fx_form + fx_apf + fx_obs, fy_form + fy_apf + fy_obs


def main(args=None) -> None:
    """Entry point for Gazebo swarm node."""
    rclpy.init(args=args)
    node = GazeboSwarmNode()
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
