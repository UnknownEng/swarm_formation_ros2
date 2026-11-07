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

"""ROS 2 control node using relative neighbor sensing and local heading orientation."""

import math
from typing import Dict, List, Optional, Tuple

from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
from swarm_interfaces.msg import NeighborList

from .consensus import compute_anchor_force, compute_target_offsets
from .controllers import create_controller
from .kinematics import compute_unicycle_twist, UnicycleMapper
from .navigation import (
    anchor_slot_position,
    fit_template_rotation,
    formation_target,
    navigation_force,
    NavigationGuide,
    OccupancyGrid,
    required_clearance,
)
from .node_runner import run_node
from .potential_field import (
    compute_peer_safety_scale,
    compute_relative_obstacle_apf,
    compute_relative_peer_apf,
)
from .relative_sensing import (
    compensate_delay,
    compute_heartbeat_mask,
    PoseHistory,
    transform_body_to_reference,
    update_relative_store,
    YawHistory,
)
from .world_model import DEFAULT_FORMATION_CENTER


NAV_DEBUG_FIELDS = (
    't', 'nav_active', 'steer_x', 'steer_y', 'target_x', 'target_y',
    'f_form_x', 'f_form_y', 'f_nav_x', 'f_nav_y', 'f_anchor_x', 'f_anchor_y',
    'f_total_x', 'f_total_y', 'v_cmd', 'w_cmd', 'est_x', 'est_y', 'est_yaw',
)


class GazeboSwarmNode(Node):
    """Controls a physical mobile robot using relative neighbor sensing and APF safety."""

    def __init__(self) -> None:
        """Initialize parameters, publishers, subscribers, and control loop."""
        super().__init__('gazebo_swarm_node')

        self.declare_parameter('robot_id', 1)
        self.declare_parameter('total_robots', 5)
        self.declare_parameter('enable_faults', False)
        self.declare_parameter('use_resilient_method', False)
        self.declare_parameter('use_safety_filter', True)
        self.declare_parameter('controller_type', 'displacement')
        self.declare_parameter('formation_radius', 2.0)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('max_linear_vel', 1.4)
        self.declare_parameter('max_angular_vel', 3.0)

        self.declare_parameter('linear_gain_kp', 1.8)
        self.declare_parameter('damping_gain_kd', 0.5)
        self.declare_parameter('yaw_gain_kyaw', 4.0)

        # Formation anchor (0 disables): pulls each robot toward its slot around a fixed
        # centre using its EKF position estimate, which stops the formation drifting.
        self.declare_parameter('anchor_gain', 0.0)
        self.declare_parameter('formation_center_x', DEFAULT_FORMATION_CENTER[0])
        self.declare_parameter('formation_center_y', DEFAULT_FORMATION_CENTER[1])

        # Navigation layer: route around static obstacles when the straight line to the
        # formation target is blocked (map-based, uses the EKF position estimate).
        self.declare_parameter('use_navigation', False)
        # < 0: derive from the CBF parameters below (recommended)
        self.declare_parameter('nav_inflation', -1.0)
        self.declare_parameter('safety_dist_obs', 0.30)
        self.declare_parameter('delay_bound_sec', 0.05)
        self.declare_parameter('cbf_lookahead', 0.15)
        self.declare_parameter('nav_carrot_dist', 0.6)
        self.declare_parameter('nav_replan_period', 0.5)
        self.declare_parameter('nav_min_force', 0.4)

        # Force -> (v, w) mapping. 'calm': force-scaled turning plus a stop band with
        # hysteresis (robots no longer spin in place at rest); 'legacy': turn toward any
        # force above 0.03 at up to k_yaw * heading error.
        self.declare_parameter('twist_mapper', 'calm')
        self.declare_parameter('turn_full_force', 1.0)
        self.declare_parameter('stop_force', 0.5)
        self.declare_parameter('resume_force', 0.9)

        # Delay compensation: subtract this robot's own motion since each (delayed)
        # measurement was taken; peer_velocity_gain > 0 also extrapolates the neighbour
        # (off by default: the finite-difference velocity is too noisy).
        self.declare_parameter('delay_compensation', True)
        self.declare_parameter('peer_velocity_gain', 0.0)

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
        self.use_safety_filter = bool(self.get_parameter('use_safety_filter').value)
        self.controller_type = str(self.get_parameter('controller_type').value)
        self.controller = create_controller(self.controller_type)
        self.radius = float(self.get_parameter('formation_radius').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.max_lin_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_ang_vel = float(self.get_parameter('max_angular_vel').value)

        self.k_p = float(self.get_parameter('linear_gain_kp').value)
        self.k_d = float(self.get_parameter('damping_gain_kd').value)
        self.k_yaw = float(self.get_parameter('yaw_gain_kyaw').value)

        self.anchor_gain = float(self.get_parameter('anchor_gain').value)
        self.formation_center = (
            float(self.get_parameter('formation_center_x').value),
            float(self.get_parameter('formation_center_y').value),
        )

        self.nav_guide: Optional[NavigationGuide] = None
        self.nav_min_force = float(self.get_parameter('nav_min_force').value)
        if bool(self.get_parameter('use_navigation').value):
            inflation = float(self.get_parameter('nav_inflation').value)
            if inflation < 0.0:
                inflation = required_clearance(
                    safety_dist_obs=float(self.get_parameter('safety_dist_obs').value),
                    lookahead=float(self.get_parameter('cbf_lookahead').value),
                    max_linear_vel=self.max_lin_vel,
                    delay_bound_sec=float(self.get_parameter('delay_bound_sec').value),
                )
            self.get_logger().info(f'Navigation grid inflation: {inflation:.3f} m')
            grid = OccupancyGrid(inflation=inflation)
            self.nav_guide = NavigationGuide(
                grid,
                carrot_dist=float(self.get_parameter('nav_carrot_dist').value),
                replan_period=float(self.get_parameter('nav_replan_period').value),
            )

        self.twist_mapper = str(self.get_parameter('twist_mapper').value).lower()
        self.mapper = UnicycleMapper(
            k_yaw=self.k_yaw,
            max_lin_vel=self.max_lin_vel,
            max_ang_vel=self.max_ang_vel,
            turn_full_force=float(self.get_parameter('turn_full_force').value),
            stop_force=float(self.get_parameter('stop_force').value),
            resume_force=float(self.get_parameter('resume_force').value),
        )

        self.delay_compensation = bool(self.get_parameter('delay_compensation').value)
        self.peer_velocity_gain = float(self.get_parameter('peer_velocity_gain').value)

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
        self.est_x = 0.0
        self.est_y = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.current_v = 0.0
        self.has_received_yaw = False
        self.yaw_history = YawHistory()
        self.pose_history = PoseHistory()

        # Current neighbor table: neighbor_id -> (dx, dy, rel_vx, rel_vy, measurement_time)
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
        cmd_topic = (
            f'/robot_{self.robot_id}/cmd_vel_raw'
            if self.use_safety_filter
            else f'/robot_{self.robot_id}/cmd_vel'
        )
        self.pub_cmd_vel = self.create_publisher(Twist, cmd_topic, 10)
        # Diagnostics, one message per control step; layout in NAV_DEBUG_FIELDS
        self.pub_nav_debug = self.create_publisher(
            Float32MultiArray, f'/robot_{self.robot_id}/nav_debug', 10
        )
        self.sub_motor_kill = self.create_subscription(
            Bool, f'/robot_{self.robot_id}/motor_kill', self._motor_kill_callback, 10
        )
        self.sub_neighbors = self.create_subscription(
            NeighborList, f'/robot_{self.robot_id}/neighbors', self.neighbor_callback, 10
        )
        # Estimated pose from onboard localization node (never reads ground truth)
        self.sub_estimated_pose = self.create_subscription(
            PoseStamped,
            f'/robot_{self.robot_id}/estimated_pose',
            self._estimated_pose_callback,
            10,
        )

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(
            f'GazeboSwarmNode {self.robot_id}/{self.total_robots} ready | '
            f'Relative Sensing: Active | Resilient: {self.use_resilient_method}'
        )

    def _estimated_pose_callback(self, msg: PoseStamped) -> None:
        """
        Update heading (and, for the optional anchor, position) from the onboard EKF.

        Never reads ground truth.
        """
        self.est_x = msg.pose.position.x
        self.est_y = msg.pose.position.y
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.has_received_yaw = True
        now_est = self.get_clock().now().nanoseconds * 1e-9
        self.yaw_history.add(now_est, self.yaw)
        self.pose_history.add(now_est, self.est_x, self.est_y)

    def neighbor_callback(self, msg: NeighborList) -> None:
        """Handle onboard relative range-bearing measurements of peers and obstacles."""
        now = self.get_clock().now().nanoseconds * 1e-9

        # Peer measurements: rotate with the heading at measurement time, keep only the
        # peers in this list (stale neighbors stop influencing control).
        measurements = []
        for n in msg.neighbors:
            t_meas = n.stamp.sec + n.stamp.nanosec * 1e-9
            if t_meas <= 0.0:
                t_meas = now  # unstamped (legacy) measurement
            measurements.append((n.neighbor_id, n.relative_x, n.relative_y, t_meas))
        self.relative_neighbors = update_relative_store(
            self.relative_neighbors, measurements, self._yaw_at
        )

        # Process obstacle relative detections
        new_obs: List[Tuple[float, float, float]] = []
        for o in msg.obstacles:
            ox, oy = transform_body_to_reference(o.relative_x, o.relative_y, self.yaw)
            new_obs.append((ox, oy, o.radius))
        self.relative_obstacles = new_obs

    def _current_displacements(self, now: float) -> Dict[int, Tuple[float, float]]:
        """Neighbor displacements, brought to the current time if compensation is on."""
        if not self.delay_compensation:
            return {pid: (d[0], d[1]) for pid, d in self.relative_neighbors.items()}
        return compensate_delay(
            self.relative_neighbors, now, self.pose_history.lookup, self.peer_velocity_gain
        )

    def _delay_corrections(self, now: float) -> Dict[int, Tuple[float, float]]:
        """Per-neighbor correction (compensated minus raw displacement)."""
        if not self.delay_compensation:
            return {}
        comp = self._current_displacements(now)
        return {
            pid: (cx - self.relative_neighbors[pid][0], cy - self.relative_neighbors[pid][1])
            for pid, (cx, cy) in comp.items()
        }

    def _yaw_at(self, t: float) -> float:
        """Return the estimated heading at time t (current heading if unknown)."""
        yaw = self.yaw_history.lookup(t)
        return self.yaw if yaw is None else yaw

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
            rel_displacements = self._current_displacements(curr_time)
            fx, fy = self.controller.compute_control_force(
                self.robot_id,
                self.total_robots,
                self.radius,
                rel_displacements,
                self.k_p,
                huber_delta=100.0,
            )

        f_form = (fx, fy)
        # Distance control leaves the formation's orientation free: the anchor and the
        # navigation target then follow the formation's current best-fit orientation.
        free_rotation = self.controller.name == 'distance'
        pattern_rotation = 0.0
        if free_rotation and self.anchor_gain > 0.0:
            pattern_rotation = fit_template_rotation(
                self._current_displacements(curr_time), self.target_offsets
            )
        nav_active = 0.0
        steer_point = None
        target = None

        # Navigation layer: if the straight line to the formation target is blocked by
        # static obstacles, steer along a planned route instead (CBF still filters it).
        if self.nav_guide is not None:
            pos = (self.est_x, self.est_y)
            rel = self._current_displacements(curr_time)
            anchor_slot = anchor_slot_position(
                self.robot_id, self.total_robots, self.radius, self.formation_center,
                rotation=pattern_rotation,
            )
            target = formation_target(
                pos, rel, self.target_offsets, consensus_gain=self.k_p,
                anchor_slot=anchor_slot, anchor_gain=self.anchor_gain,
                fit_rotation=free_rotation,
            )
            steer_point = self.nav_guide.steer(curr_time, pos, target)
            if steer_point is not None:
                nav_active = 1.0
                fx, fy = navigation_force((fx, fy), pos, steer_point, self.nav_min_force)
        f_nav = (fx, fy)

        # Optional formation anchor (virtual leader around a fixed centre)
        fax, fay = compute_anchor_force(
            self.robot_id, self.total_robots, self.radius, self.formation_center,
            (self.est_x, self.est_y), self.anchor_gain, rotation=pattern_rotation,
        )
        fx += fax
        fy += fay

        # Local velocity damping
        fx -= self.k_d * self.vel_x
        fy -= self.k_d * self.vel_y

        if self.twist_mapper == 'legacy':
            v_cmd, w_cmd = compute_unicycle_twist(
                fx, fy, self.yaw, self.k_yaw, self.max_lin_vel, self.max_ang_vel
            )
        else:
            v_cmd, w_cmd = self.mapper.to_twist(fx, fy, self.yaw)

        dbg = Float32MultiArray()
        dbg.data = [float(v) for v in (
            curr_time, nav_active,
            steer_point[0] if steer_point else float('nan'),
            steer_point[1] if steer_point else float('nan'),
            target[0] if target else float('nan'),
            target[1] if target else float('nan'),
            f_form[0], f_form[1], f_nav[0], f_nav[1], fax, fay, fx, fy,
            v_cmd, w_cmd, self.est_x, self.est_y, self.yaw,
        )]
        self.pub_nav_debug.publish(dbg)

        self.current_v = v_cmd
        self.vel_x = v_cmd * math.cos(self.yaw)
        self.vel_y = v_cmd * math.sin(self.yaw)

        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.pub_cmd_vel.publish(cmd)

    def _compute_resilient_control(self, curr_time: float) -> Tuple[float, float]:
        """Compute robust Huber consensus with dual-layer APF safety barrier."""
        # Heartbeat on measurement age, not reception time: the network node re-publishes
        # the last delivered measurement for up to 1.5 s, which used to hide lost robots.
        active_mask = compute_heartbeat_mask(
            self.relative_neighbors, list(self.target_offsets.keys()), curr_time,
            self.heartbeat_timeout,
        )

        for n_id in self.target_offsets.keys():
            if n_id not in self.relative_neighbors:
                continue

            raw_dx, raw_dy, n_vx, n_vy, _ = self.relative_neighbors[n_id]

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
        # Same delay correction on top of the EMA-filtered positions
        delta = self._delay_corrections(curr_time)
        rel_pos_filtered = {
            pid: (d[0] + delta.get(pid, (0.0, 0.0))[0], d[1] + delta.get(pid, (0.0, 0.0))[1])
            for pid, d in self.filtered_relative.items()
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
        active_filtered = {
            pid: st for pid, st in self.filtered_relative.items() if active_mask.get(pid, False)
        }
        fx_apf, fy_apf, min_peer_dist = compute_relative_peer_apf(
            active_filtered,
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
    run_node(GazeboSwarmNode, args)


if __name__ == '__main__':
    main()
