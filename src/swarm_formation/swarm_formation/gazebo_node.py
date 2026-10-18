# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
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


class GazeboSwarmNode(Node):
    """Controls a physical mobile robot in Gazebo Harmonic using consensus and APF safety."""

    def __init__(self) -> None:
        """Initialize parameters, publishers, subscribers, and control loop."""
        super().__init__('gazebo_swarm_node')

        self.declare_parameter('robot_id', 1)
        self.declare_parameter('total_robots', 5)
        self.declare_parameter('enable_faults', False)
        self.declare_parameter('use_resilient_method', False)
        self.declare_parameter('formation_radius', 2.0)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('max_linear_vel', 1.4)
        self.declare_parameter('max_angular_vel', 3.0)

        self.robot_id = int(self.get_parameter('robot_id').value)
        self.total_robots = int(self.get_parameter('total_robots').value)
        self.enable_faults = bool(self.get_parameter('enable_faults').value)
        self.use_resilient_method = bool(self.get_parameter('use_resilient_method').value)
        self.radius = float(self.get_parameter('formation_radius').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.max_lin_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_ang_vel = float(self.get_parameter('max_angular_vel').value)
        self.dt = 1.0 / self.control_rate

        # Physical state from Gazebo
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.last_pose_time: Optional[float] = None
        self.has_received_pose = False

        # Consensus & Control gains
        self.k_p = 1.8
        self.k_d = 0.5
        self.k_yaw = 4.0

        # APF Safety Barrier parameters (Physical robot radius 0.20m -> collision 0.50m)
        self.d_collision = 0.50
        self.d_safe = 1.10
        self.d_infl = 1.80
        self.k_rep = 1.2
        self.k_barrier = 2.0
        self.k_damp_apf = 1.5

        # Dual-Layer APF Layer 2: Environmental Obstacles (Disaster Zone)
        # Matches worlds/collapsed_building.sdf: (x, y, radius)
        self.disaster_obstacles: List[Tuple[float, float, float]] = [
            (-1.66, 1.40, 0.15),  # Structural Steel Pillar 1
            (1.92, 1.87, 0.15),   # Structural Steel Pillar 2
            (0.25, 1.96, 0.22),   # Fallen Concrete Slab 1
            (0.95, -0.33, 0.22),  # Fallen Concrete Slab 2
            (-1.80, -1.09, 0.16),  # Broken Rubble Block 1
            (-0.12, -1.89, 0.16),  # Broken Rubble Block 2
            (1.87, -1.24, 0.16),  # Broken Rubble Block 3
        ]
        self.d_obs_infl = 0.45
        self.d_obs_safe = 0.30
        self.k_obs_rep = 0.6
        self.k_obs_barrier = 0.8
        self.k_obs_damp = 0.8

        # State storage for neighbors: id -> (x, y, vx, vy, last_received_time)
        self.neighbor_states: Dict[int, Tuple[float, float, float, float, float]] = {}
        self.filtered_states: Dict[int, Tuple[float, float, float, float]] = {}
        self.last_valid_beacon_times: Dict[int, float] = {}

        # Fault injection queue: list of (deliver_time, (id, x, y, vx, vy))
        self.packet_delay_queue: List[Tuple[float, Tuple[int, float, float, float, float]]] = []

        # Fault tracking
        self.is_dropped_out = False
        self.start_time: Optional[float] = None

        # Precompute target formation offsets
        self.target_offsets = self._compute_target_offsets()

        # ROS 2 Topics
        # Physical velocity control to Gazebo DiffDrive
        self.pub_cmd_vel = self.create_publisher(
            Twist, f'/robot_{self.robot_id}/cmd_vel', 10
        )
        # Swarm ad-hoc beacon network
        self.pub_beacon = self.create_publisher(Float32MultiArray, '/robot_all_poses', 20)
        self.sub_beacon = self.create_subscription(
            Float32MultiArray, '/robot_all_poses', self.beacon_callback, 20
        )
        # Ground-truth pose from Gazebo PosePublisher
        self.sub_pose = self.create_subscription(
            Pose, f'/robot_{self.robot_id}/pose', self.pose_callback, 10
        )

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(
            f'GazeboSwarmNode {self.robot_id}/{self.total_robots} ready | '
            f'Faults: {self.enable_faults} | Resilient: {self.use_resilient_method}'
        )

    def _compute_target_offsets(self) -> Dict[int, Tuple[float, float]]:
        """Compute relative polygon offsets for regular pentagon."""
        offsets: Dict[int, Tuple[float, float]] = {}
        my_idx = (self.robot_id - 1) % self.total_robots
        my_angle = 2.0 * math.pi * my_idx / self.total_robots
        my_x = self.radius * math.cos(my_angle)
        my_y = self.radius * math.sin(my_angle)

        for other_id in range(1, self.total_robots + 1):
            if other_id == self.robot_id:
                continue
            other_idx = (other_id - 1) % self.total_robots
            other_angle = 2.0 * math.pi * other_idx / self.total_robots
            other_x = self.radius * math.cos(other_angle)
            other_y = self.radius * math.sin(other_angle)
            offsets[other_id] = (other_x - my_x, other_y - my_y)

        return offsets

    def pose_callback(self, msg: Pose) -> None:
        """Update robot physical pose and yaw from Gazebo physics simulation."""
        now = self.get_clock().now().nanoseconds * 1e-9
        new_x = msg.position.x
        new_y = msg.position.y

        # Quaternion to yaw
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
            # Fault 1: Packet drop ~15%
            if random.random() < 0.15:
                return

            # Fault 2: Gaussian sensor noise (std dev = 0.18m)
            noise_x = random.gauss(0.0, 0.18)
            noise_y = random.gauss(0.0, 0.18)
            noisy_x = raw_x + noise_x
            noisy_y = raw_y + noise_y

            # Fault 3: Communication delay (50ms to 150ms)
            delay = random.uniform(0.05, 0.15)
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
        """Simulate agent dropout fault for robot 4 between t=5s and t=10s."""
        if self.enable_faults and self.robot_id == 4:
            if 5.0 <= elapsed <= 10.0:
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

        # If dropped out, command zero velocity and hold position
        if dropped_out:
            stop_twist = Twist()
            self.pub_cmd_vel.publish(stop_twist)
            return

        # Compute planar control acceleration
        if self.use_resilient_method:
            fx, fy = self._compute_resilient_control(curr_time)
        else:
            fx, fy = self._compute_baseline_control()

        # Add damping
        fx -= self.k_d * self.vel_x
        fy -= self.k_d * self.vel_y

        # Convert planar force vector to physical unicycle cmd_vel
        self._publish_unicycle_twist(fx, fy)

    def _compute_baseline_control(self) -> Tuple[float, float]:
        """Compute standard linear consensus without APF or filtering."""
        fx, fy = 0.0, 0.0
        for n_id, target_offset in self.target_offsets.items():
            if n_id not in self.neighbor_states:
                continue

            nx, ny, _, _, _ = self.neighbor_states[n_id]
            dx = nx - self.pos_x
            dy = ny - self.pos_y

            err_x = dx - target_offset[0]
            err_y = dy - target_offset[1]

            fx += self.k_p * err_x
            fy += self.k_p * err_y

        return fx, fy

    def _compute_resilient_control(self, curr_time: float) -> Tuple[float, float]:
        """Compute robust consensus with APF safety barrier and heartbeat detection."""
        fx_form, fy_form = 0.0, 0.0
        fx_apf, fy_apf = 0.0, 0.0

        heartbeat_timeout = 1.2
        active_neighbors = 0

        for n_id, target_offset in self.target_offsets.items():
            if n_id not in self.neighbor_states:
                continue

            raw_nx, raw_ny, n_vx, n_vy, last_seen = self.neighbor_states[n_id]
            is_active = (curr_time - last_seen) <= heartbeat_timeout

            # Outlier rejection and EMA filtering
            time_gap = curr_time - self.last_valid_beacon_times.get(n_id, 0.0)

            if time_gap > 0.8:
                # Reconnection after dropout: reset filter directly to current measurement
                filt_x, filt_y = raw_nx, raw_ny
                filt_vx, filt_vy = n_vx, n_vy
                self.filtered_states[n_id] = (filt_x, filt_y, filt_vx, filt_vy)
                self.last_valid_beacon_times[n_id] = curr_time
            elif n_id in self.filtered_states:
                prev_x, prev_y, prev_vx, prev_vy = self.filtered_states[n_id]
                jump = math.hypot(raw_nx - prev_x, raw_ny - prev_y)
                # Max plausible movement in 20ms + 3-sigma noise is ~1.2m
                if jump > 1.2:
                    filt_x, filt_y = prev_x, prev_y
                    filt_vx, filt_vy = prev_vx, prev_vy
                else:
                    alpha = 0.35
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

            # Consensus force over active neighbors with Huber loss & deadband
            if is_active:
                active_neighbors += 1
                dx = filt_x - self.pos_x
                dy = filt_y - self.pos_y
                err_x = dx - target_offset[0]
                err_y = dy - target_offset[1]
                err_norm = math.hypot(err_x, err_y)

                if err_norm > 0.02:
                    delta = 2.0
                    if err_norm > delta:
                        scale = delta / err_norm
                        err_x *= scale
                        err_y *= scale
                    fx_form += self.k_p * err_x
                    fy_form += self.k_p * err_y

            # APF Safety Barrier Certificate
            dist = math.hypot(self.pos_x - filt_x, self.pos_y - filt_y)
            if 0.001 < dist < self.d_infl:
                nx_hat = (self.pos_x - filt_x) / dist
                ny_hat = (self.pos_y - filt_y) / dist

                # 1. Repulsive potential field force
                f_rep_mag = self.k_rep * (1.0 / dist - 1.0 / self.d_infl) * (1.0 / (dist * dist))

                # 2. Hard barrier certificate when entering dangerous proximity
                f_barrier_mag = 0.0
                if dist < self.d_safe:
                    denom = max(dist - 0.50, 0.01)
                    f_barrier_mag = self.k_barrier * (1.0 / (denom * denom))

                # 3. Dynamic closing-velocity damping
                rel_vx = self.vel_x - filt_vx
                rel_vy = self.vel_y - filt_vy
                closing_vel = -(rel_vx * nx_hat + rel_vy * ny_hat)
                f_damp_mag = 0.0
                if closing_vel > 0.0:
                    f_damp_mag = self.k_damp_apf * (closing_vel / dist)

                total_rep = min(f_rep_mag + f_barrier_mag + f_damp_mag, 50.0)
                tx_peer, ty_peer = -ny_hat, nx_hat
                f_peer_lateral = 0.6 * total_rep

                fx_apf += total_rep * nx_hat + f_peer_lateral * tx_peer
                fy_apf += total_rep * ny_hat + f_peer_lateral * ty_peer

        # Layer 2: Environmental Obstacle APF Repulsion (Disaster Site Debris)
        fx_obs, fy_obs = 0.0, 0.0

        for ox, oy, orad in self.disaster_obstacles:
            dist_center = math.hypot(self.pos_x - ox, self.pos_y - oy)
            clearance = max(dist_center - orad, 1e-4)

            if clearance < self.d_obs_infl and dist_center > 1e-4:
                obs_nx = (self.pos_x - ox) / dist_center
                obs_ny = (self.pos_y - oy) / dist_center

                # Tangent vector to glide smoothly around obstacle without local minima
                tx, ty = -obs_ny, obs_nx
                if (fx_form * tx + fy_form * ty) < 0.0:
                    tx, ty = -tx, -ty

                # 1. Repulsive obstacle potential field force
                inv_c = 1.0 / clearance
                inv_infl = 1.0 / self.d_obs_infl
                f_obs_rep = self.k_obs_rep * (inv_c - inv_infl) * (inv_c**2)

                # 2. Hard barrier certificate when approaching obstacle
                f_obs_barrier = 0.0
                if clearance < self.d_obs_safe:
                    denom = max(clearance - 0.20, 0.01)
                    f_obs_barrier = self.k_obs_barrier * (1.0 / (denom * denom))

                # 3. Dynamic closing velocity damping towards static obstacle
                closing_vel_obs = -(self.vel_x * obs_nx + self.vel_y * obs_ny)
                f_obs_damp = 0.0
                if closing_vel_obs > 0.0:
                    f_obs_damp = self.k_obs_damp * (closing_vel_obs / clearance)

                total_normal = min(f_obs_rep + f_obs_barrier + f_obs_damp, 12.0)
                f_tangent = 1.0 * total_normal

                fx_obs += total_normal * obs_nx + f_tangent * tx
                fy_obs += total_normal * obs_ny + f_tangent * ty

        # Peer safety-critical priority attenuation:
        min_neighbor_dist = float('inf')
        for n_id in self.target_offsets.keys():
            if n_id in self.filtered_states:
                fx_n, fy_n, _, _ = self.filtered_states[n_id]
                d_n = math.hypot(self.pos_x - fx_n, self.pos_y - fy_n)
                if d_n < min_neighbor_dist:
                    min_neighbor_dist = d_n

        safety_scale_peer = 1.0
        if min_neighbor_dist < self.d_safe:
            safety_scale_peer = max(
                0.08, (min_neighbor_dist - 0.50) / (self.d_safe - 0.50)
            )

        fx_form *= safety_scale_peer
        fy_form *= safety_scale_peer

        return fx_form + fx_apf + fx_obs, fy_form + fy_apf + fy_obs

    def _publish_unicycle_twist(self, fx: float, fy: float) -> None:
        """Map 2D planar force vector to physical unicycle (linear, angular) velocity."""
        cmd = Twist()
        force_mag = math.hypot(fx, fy)

        if force_mag < 0.03:
            # Settled inside tolerance deadband
            self.pub_cmd_vel.publish(cmd)
            return

        desired_yaw = math.atan2(fy, fx)
        yaw_err = math.atan2(math.sin(desired_yaw - self.yaw), math.cos(desired_yaw - self.yaw))

        # Angular velocity proportional to heading error
        w_cmd = self.k_yaw * yaw_err
        w_cmd = max(-self.max_ang_vel, min(self.max_ang_vel, w_cmd))

        # Smooth continuous velocity scaling based on heading alignment
        speed = min(force_mag, self.max_lin_vel) * max(0.0, math.cos(yaw_err))
        v_cmd = speed

        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.pub_cmd_vel.publish(cmd)


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
