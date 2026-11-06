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

"""ROS 2 control node using relative neighbor sensing and local heading orientation."""

import math
from typing import Dict, List, Optional, Tuple

from geometry_msgs.msg import Pose, Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from swarm_interfaces.msg import NeighborList

from .consensus import compute_target_offsets
from .controllers import create_controller
from .kinematics import compute_unicycle_twist
from .potential_field import (
    compute_peer_safety_scale,
    compute_relative_obstacle_apf,
    compute_relative_peer_apf,
)
from .relative_sensing import transform_body_to_reference


class GazeboSwarmNode(Node):
    """Controls a physical mobile robot using relative neighbor sensing and APF safety."""

    def __init__(self) -> None:
        """Initialize parameters, publishers, subscribers, and control loop."""
        super().__init__('gazebo_swarm_node')

        self.declare_parameter('robot_id', 1)
        self.declare_parameter('total_robots', 5)
        self.declare_parameter('enable_faults', False)
        self.declare_parameter('use_resilient_method', False)
        self.declare_parameter('controller_type', 'displacement')
        self.declare_parameter('formation_radius', 2.0)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('max_linear_vel', 1.4)
        self.declare_parameter('max_angular_vel', 3.0)

        self.declare_parameter('linear_gain_kp', 1.8)
        self.declare_parameter('damping_gain_kd', 0.5)
        self.declare_parameter('yaw_gain_kyaw', 4.0)

        self.declare_parameter('huber_delta', 2.0)
        self.declare_parameter('ema_alpha', 0.35)
        self.declare_parameter('heartbeat_timeout', 1.2)

        self.declare_parameter('apf_collision_dist', 0.50)
        self.declare_parameter('apf_safe_dist', 1.10)
        self.declare_parameter('apf_influence_dist', 1.80)
        self.declare_parameter('apf_repulsion_gain', 1.2)
        self.declare_parameter('apf_barrier_gain', 2.0)
        self.declare_parameter('apf_damping_gain', 1.5)
        self.declare_parameter('apf_lateral_ratio', 0.6)

        self.declare_parameter('obs_influence_dist', 0.45)
        self.declare_parameter('obs_safe_dist', 0.30)
        self.declare_parameter('obs_repulsion_gain', 0.6)
        self.declare_parameter('obs_barrier_gain', 0.8)
        self.declare_parameter('obs_damping_gain', 0.8)
        self.declare_parameter('obs_tangent_ratio', 1.0)

        self.robot_id = int(self.get_parameter('robot_id').value)
        self.total_robots = int(self.get_parameter('total_robots').value)
        self.enable_faults = bool(self.get_parameter('enable_faults').value)
        self.use_resilient_method = bool(self.get_parameter('use_resilient_method').value)
        self.controller_type = str(self.get_parameter('controller_type').value)
        self.controller = create_controller(self.controller_type)
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

        self.dt = 1.0 / self.control_rate

        # Onboard orientation & local odometry (no global x, y used)
        self.yaw = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.current_v = 0.0
        self.has_received_yaw = False

        # Relative measurements storage: neighbor_id -> (dx, dy, rel_vx, rel_vy, timestamp)
        self.relative_neighbors: Dict[int, Tuple[float, float, float, float, float]] = {}
        self.filtered_relative: Dict[int, Tuple[float, float, float, float]] = {}
        self.last_valid_sensor_times: Dict[int, float] = {}

        # Relative obstacle storage: list of (rel_ox, rel_oy, radius)
        self.relative_obstacles: List[Tuple[float, float, float]] = []

        # Fault tracking
        self.is_motor_killed = False
        self.start_time: Optional[float] = None

        # Precompute target formation offsets
        self.target_offsets = compute_target_offsets(
            self.robot_id, self.total_robots, self.radius
        )

        # ROS 2 Topics
        self.pub_cmd_vel = self.create_publisher(
            Twist, f'/robot_{self.robot_id}/cmd_vel', 10
        )
        self.sub_motor_kill = self.create_subscription(
            Bool, f'/robot_{self.robot_id}/motor_kill', self._motor_kill_callback, 10
        )
        self.sub_neighbors = self.create_subscription(
            NeighborList, f'/robot_{self.robot_id}/neighbors', self.neighbor_callback, 10
        )
        # Orientation emulation from onboard IMU / compass
        self.sub_pose = self.create_subscription(
            Pose, f'/robot_{self.robot_id}/pose', self.pose_callback, 10
        )

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(
            f'GazeboSwarmNode {self.robot_id}/{self.total_robots} ready | '
            f'Relative Sensing: Active | Resilient: {self.use_resilient_method}'
        )

    def pose_callback(self, msg: Pose) -> None:
        """
        Update robot physical orientation (IMU/compass emulation).

        Note: Strictly extracts orientation (yaw); never reads global (x, y) position.
        """
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.has_received_yaw = True

    def neighbor_callback(self, msg: NeighborList) -> None:
        """Handle onboard relative range-bearing measurements of peers and obstacles."""
        now = self.get_clock().now().nanoseconds * 1e-9

        # Process peer relative measurements
        for n in msg.neighbors:
            peer_id = n.neighbor_id
            # Transform body-frame relative displacement to reference frame
            dx, dy = transform_body_to_reference(n.relative_x, n.relative_y, self.yaw)

            # Estimate relative velocity via finite difference if previously observed
            rel_vx, rel_vy = 0.0, 0.0
            if peer_id in self.relative_neighbors:
                prev_dx, prev_dy, _, _, prev_time = self.relative_neighbors[peer_id]
                dt_meas = max(now - prev_time, 1e-4)
                rel_vx = (dx - prev_dx) / dt_meas
                rel_vy = (dy - prev_dy) / dt_meas

            self.relative_neighbors[peer_id] = (dx, dy, rel_vx, rel_vy, now)

        # Process obstacle relative detections
        new_obs: List[Tuple[float, float, float]] = []
        for o in msg.obstacles:
            ox, oy = transform_body_to_reference(o.relative_x, o.relative_y, self.yaw)
            new_obs.append((ox, oy, o.radius))
        self.relative_obstacles = new_obs

    def _motor_kill_callback(self, msg: Bool) -> None:
        """Handle motor kill signal from external fault injector node."""
        if msg.data != self.is_motor_killed:
            self.is_motor_killed = msg.data
            if self.is_motor_killed:
                self.get_logger().warn(f'Robot {self.robot_id} MOTOR KILLED by fault injector!')
            else:
                self.get_logger().info(f'Robot {self.robot_id} MOTOR RESTORED.')

    def control_loop(self) -> None:
        """Compute relative consensus force, convert to unicycle velocity, and publish."""
        if not self.has_received_yaw:
            return

        if self.is_motor_killed:
            self.pub_cmd_vel.publish(Twist())
            return

        curr_time = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = curr_time

        if self.use_resilient_method:
            fx, fy = self._compute_resilient_control(curr_time)
        else:
            rel_displacements = {
                pid: (data[0], data[1]) for pid, data in self.relative_neighbors.items()
            }
            fx, fy = self.controller.compute_control_force(
                self.robot_id,
                self.total_robots,
                self.radius,
                rel_displacements,
                self.k_p,
                huber_delta=100.0,
            )

        # Local velocity damping
        fx -= self.k_d * self.vel_x
        fy -= self.k_d * self.vel_y

        v_cmd, w_cmd = compute_unicycle_twist(
            fx, fy, self.yaw, self.k_yaw, self.max_lin_vel, self.max_ang_vel
        )

        self.current_v = v_cmd
        self.vel_x = v_cmd * math.cos(self.yaw)
        self.vel_y = v_cmd * math.sin(self.yaw)

        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.pub_cmd_vel.publish(cmd)

    def _compute_resilient_control(self, curr_time: float) -> Tuple[float, float]:
        """Compute robust Huber consensus with dual-layer APF safety barrier."""
        active_mask: Dict[int, bool] = {}

        for n_id in self.target_offsets.keys():
            if n_id not in self.relative_neighbors:
                continue

            raw_dx, raw_dy, n_vx, n_vy, last_seen = self.relative_neighbors[n_id]
            is_active = (curr_time - last_seen) <= self.heartbeat_timeout
            active_mask[n_id] = is_active

            time_gap = curr_time - self.last_valid_sensor_times.get(n_id, 0.0)

            if time_gap > 0.8:
                filt_dx, filt_dy = raw_dx, raw_dy
                filt_vx, filt_vy = n_vx, n_vy
                self.filtered_relative[n_id] = (filt_dx, filt_dy, filt_vx, filt_vy)
                self.last_valid_sensor_times[n_id] = curr_time
            elif n_id in self.filtered_relative:
                prev_dx, prev_dy, prev_vx, prev_vy = self.filtered_relative[n_id]
                jump = math.hypot(raw_dx - prev_dx, raw_dy - prev_dy)

                # Fault injection rejection: reject discontinuous teleports
                if jump < 2.5:
                    filt_dx = (
                        self.ema_alpha * raw_dx + (1.0 - self.ema_alpha) * prev_dx
                    )
                    filt_dy = (
                        self.ema_alpha * raw_dy + (1.0 - self.ema_alpha) * prev_dy
                    )
                    filt_vx = (
                        self.ema_alpha * n_vx + (1.0 - self.ema_alpha) * prev_vx
                    )
                    filt_vy = (
                        self.ema_alpha * n_vy + (1.0 - self.ema_alpha) * prev_vy
                    )
                    self.filtered_relative[n_id] = (filt_dx, filt_dy, filt_vx, filt_vy)
                    self.last_valid_sensor_times[n_id] = curr_time
            else:
                self.filtered_relative[n_id] = (raw_dx, raw_dy, n_vx, n_vy)
                self.last_valid_sensor_times[n_id] = curr_time

        # 1. Formation Control Force using selected Controller Family
        rel_pos_filtered = {
            pid: (d[0], d[1]) for pid, d in self.filtered_relative.items()
        }
        fx_form, fy_form = self.controller.compute_control_force(
            self.robot_id,
            self.total_robots,
            self.radius,
            rel_pos_filtered,
            self.k_p,
            self.huber_delta,
            active_mask,
        )

        # 2. Dual-Layer APF Layer 1: Peer Repulsive Force
        fx_apf, fy_apf, min_peer_dist = compute_relative_peer_apf(
            self.filtered_relative,
            d_infl=self.d_infl,
            d_safe=self.d_safe,
            d_collision=self.d_collision,
            k_rep=self.k_rep,
            k_barrier=self.k_barrier,
            k_damp=self.k_damp_apf,
            lateral_ratio=self.apf_lateral_ratio,
        )

        # 3. Dual-Layer APF Layer 2: Obstacle Repulsive Force
        fx_obs, fy_obs = compute_relative_obstacle_apf(
            self.relative_obstacles,
            vel_x=self.vel_x,
            vel_y=self.vel_y,
            fx_guide=fx_form,
            fy_guide=fy_form,
            d_obs_infl=self.d_obs_infl,
            d_obs_safe=self.d_obs_safe,
            k_obs_rep=self.k_obs_rep,
            k_obs_barrier=self.k_obs_barrier,
            k_obs_damp=self.k_obs_damp,
            tangent_ratio=self.obs_tangent_ratio,
        )

        # 4. Peer Safety Priority Attenuation
        safety_scale = compute_peer_safety_scale(
            min_peer_dist, self.d_safe, self.d_collision
        )
        fx_form *= safety_scale
        fy_form *= safety_scale

        return fx_form + fx_apf + fx_obs, fy_form + fy_apf + fy_obs


def main(args: Optional[List[str]] = None) -> None:
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
