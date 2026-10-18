# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
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

"""Metrics logging utility for decentralized swarm shape formation."""

import csv
import math
import os
from typing import Dict, Optional, Tuple

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


class SwarmMetricsLogger(Node):
    """Monitors, computes, and logs quantitative performance metrics for swarm formation."""

    def __init__(self) -> None:
        """Initialize parameters, CSV file, subscribers, and timers."""
        super().__init__('swarm_metrics_logger')

        self.declare_parameter('total_robots', 5)
        self.declare_parameter('formation_radius', 2.0)
        self.declare_parameter('convergence_threshold', 0.20)
        self.declare_parameter('settle_duration', 2.0)
        self.declare_parameter('csv_path', 'swarm_metrics.csv')
        self.declare_parameter('auto_shutdown_sec', 0.0)
        self.declare_parameter('log_rate', 20.0)
        self.declare_parameter('enable_faults', False)
        self.declare_parameter('use_resilient_method', False)

        self.total_robots = int(self.get_parameter('total_robots').value)
        self.radius = float(self.get_parameter('formation_radius').value)
        self.convergence_threshold = float(self.get_parameter('convergence_threshold').value)
        self.settle_duration = float(self.get_parameter('settle_duration').value)
        self.csv_path = str(self.get_parameter('csv_path').value)
        self.auto_shutdown_sec = float(self.get_parameter('auto_shutdown_sec').value)
        self.log_rate = float(self.get_parameter('log_rate').value)
        self.enable_faults = bool(self.get_parameter('enable_faults').value)
        self.use_resilient_method = bool(self.get_parameter('use_resilient_method').value)
        self.dt = 1.0 / self.log_rate

        self.robot_poses: Dict[int, Tuple[float, float, float]] = {}
        self.start_time: Optional[float] = None
        self.last_log_print_time = 0.0

        # Metrics tracking
        self.global_min_distance = float('inf')
        self.total_collision_events = 0
        self.consecutive_settled_time = 0.0
        self.convergence_time: Optional[float] = None
        self.is_converged = False
        self.last_shape_error = 0.0
        self.summary_printed = False

        # Initialize CSV file
        self._init_csv()

        # Publishers & Subscribers
        self.sub_poses = self.create_subscription(
            Float32MultiArray, '/robot_all_poses', self.pose_callback, 50
        )
        self.pub_markers = self.create_publisher(MarkerArray, '/swarm/visualization', 10)

        self.timer = self.create_timer(self.dt, self.metrics_loop)
        self.get_logger().info(
            f'SwarmMetricsLogger started | Exporting to: {self.csv_path} | '
            f'Total Robots: {self.total_robots} | Threshold: {self.convergence_threshold}m'
        )

    def _init_csv(self) -> None:
        """Create and initialize the CSV file with headers."""
        csv_dir = os.path.dirname(self.csv_path)
        if csv_dir and not os.path.exists(csv_dir):
            os.makedirs(csv_dir, exist_ok=True)

        header = [
            'time_sec',
            'shape_error_rms',
            'min_distance',
            'global_min_distance',
            'collisions_count',
            'is_converged',
            'convergence_time',
        ]
        for i in range(1, self.total_robots + 1):
            header.extend([f'r{i}_x', f'r{i}_y'])

        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)

    def pose_callback(self, msg: Float32MultiArray) -> None:
        """Update robot positions from broadcast beacon messages."""
        data = msg.data
        if len(data) >= 3:
            r_id = int(data[0])
            px = float(data[1])
            py = float(data[2])
            curr_time = self.get_clock().now().nanoseconds * 1e-9
            self.robot_poses[r_id] = (px, py, curr_time)

    def _compute_metrics(
        self,
    ) -> Optional[Tuple[float, float, int, float, float]]:
        """Compute Shape Error (RMS), min distance, collisions, and centroid."""
        if len(self.robot_poses) < self.total_robots:
            return None

        # 1. Swarm Centroid
        sum_x = sum(self.robot_poses[i][0] for i in range(1, self.total_robots + 1))
        sum_y = sum(self.robot_poses[i][1] for i in range(1, self.total_robots + 1))
        cx = sum_x / self.total_robots
        cy = sum_y / self.total_robots

        # 2. Shape Error (RMS distance from target polygon vertices)
        sq_err_sum = 0.0
        for i in range(1, self.total_robots + 1):
            idx = (i - 1) % self.total_robots
            angle = 2.0 * math.pi * idx / self.total_robots
            target_x = cx + self.radius * math.cos(angle)
            target_y = cy + self.radius * math.sin(angle)

            rx, ry, _ = self.robot_poses[i]
            sq_err_sum += (rx - target_x) ** 2 + (ry - target_y) ** 2

        shape_error = math.sqrt(sq_err_sum / self.total_robots)

        # 3. Minimum Inter-Robot Distance and Collision Check
        min_dist = float('inf')
        collisions = 0
        collision_threshold = 0.50  # 2 * robot radius (0.25m)

        for i in range(1, self.total_robots + 1):
            for j in range(i + 1, self.total_robots + 1):
                p1 = self.robot_poses[i]
                p2 = self.robot_poses[j]
                d = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
                if d < min_dist:
                    min_dist = d
                if d < collision_threshold:
                    collisions += 1

        return shape_error, min_dist, collisions, cx, cy

    def metrics_loop(self) -> None:
        """Periodic metrics calculation, logging to CSV, and visualization."""
        curr_time = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = curr_time

        elapsed = curr_time - self.start_time

        metrics = self._compute_metrics()
        if metrics is None:
            return

        shape_error, min_dist, collisions, cx, cy = metrics
        self.last_shape_error = shape_error

        if min_dist < self.global_min_distance:
            self.global_min_distance = min_dist

        if collisions > 0:
            self.total_collision_events += collisions

        # Check convergence: must stay below threshold for settle_duration
        if shape_error < self.convergence_threshold:
            self.consecutive_settled_time += self.dt
            if self.consecutive_settled_time >= self.settle_duration and not self.is_converged:
                self.is_converged = True
                self.convergence_time = elapsed - self.settle_duration
                self.get_logger().info(
                    f'>>> SWARM CONVERGED! Settle Time: {self.convergence_time:.2f}s | '
                    f'RMS Error: {shape_error:.3f}m <<<'
                )
        else:
            self.consecutive_settled_time = 0.0

        # Log row to CSV
        conv_val = f'{self.convergence_time:.3f}' if self.convergence_time is not None else 'NaN'
        row = [
            f'{elapsed:.2f}',
            f'{shape_error:.4f}',
            f'{min_dist:.4f}',
            f'{self.global_min_distance:.4f}',
            str(collisions),
            str(self.is_converged),
            conv_val,
        ]
        for i in range(1, self.total_robots + 1):
            rx, ry, _ = self.robot_poses[i]
            row.extend([f'{rx:.4f}', f'{ry:.4f}'])

        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)

        # Periodic console print every 2.0s
        if elapsed - self.last_log_print_time >= 2.0:
            self.last_log_print_time = elapsed
            conv_str = (
                f'YES ({self.convergence_time:.2f}s)' if self.is_converged else 'CONVERGING...'
            )
            self.get_logger().info(
                f'[t={elapsed:5.1f}s] Shape Err: {shape_error:6.3f}m | '
                f'Min Dist: {min_dist:5.3f}m (Global: {self.global_min_distance:5.3f}m) | '
                f'Collisions: {self.total_collision_events} | Converged: {conv_str}'
            )

        # Publish RViz markers
        self._publish_visualization(cx, cy)

        # Auto-shutdown if configured
        if self.auto_shutdown_sec > 0.0 and elapsed >= self.auto_shutdown_sec:
            self._print_final_summary()
            self.destroy_node()
            if rclpy.ok():
                try:
                    rclpy.shutdown()
                except Exception:
                    pass
            return

    def _publish_visualization(self, cx: float, cy: float) -> None:
        """Publish RViz markers for robots, target pentagon, and formation mesh."""
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Robot sphere markers
        for i in range(1, self.total_robots + 1):
            rx, ry, _ = self.robot_poses[i]
            m = Marker()
            m.header.frame_id = 'swarm_frame'
            m.header.stamp = now
            m.ns = 'robots'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = rx
            m.pose.position.y = ry
            m.pose.position.z = 0.1
            m.scale.x = 0.4
            m.scale.y = 0.4
            m.scale.z = 0.4

            # Green if converged, Blue if resilient, Cyan otherwise
            m.color = ColorRGBA(r=0.1, g=0.8, b=0.2, a=0.9) if self.is_converged else (
                ColorRGBA(r=0.2, g=0.5, b=1.0, a=0.9) if self.use_resilient_method else
                ColorRGBA(r=0.9, g=0.3, b=0.2, a=0.9)
            )
            marker_array.markers.append(m)

        # Pentagon line strip connecting consecutive robots
        poly_marker = Marker()
        poly_marker.header.frame_id = 'swarm_frame'
        poly_marker.header.stamp = now
        poly_marker.ns = 'formation_polygon'
        poly_marker.id = 100
        poly_marker.type = Marker.LINE_STRIP
        poly_marker.action = Marker.ADD
        poly_marker.scale.x = 0.05
        poly_marker.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.8)

        for i in range(1, self.total_robots + 1):
            p = Point()
            p.x = self.robot_poses[i][0]
            p.y = self.robot_poses[i][1]
            p.z = 0.05
            poly_marker.points.append(p)
        # Close the loop
        p0 = Point()
        p0.x = self.robot_poses[1][0]
        p0.y = self.robot_poses[1][1]
        p0.z = 0.05
        poly_marker.points.append(p0)

        marker_array.markers.append(poly_marker)
        self.pub_markers.publish(marker_array)

    def _print_final_summary(self) -> None:
        """Print experiment summary report to console."""
        if self.summary_printed:
            return
        self.summary_printed = True

        mode_str = 'Part 1: Baseline Consensus'
        if self.enable_faults and not self.use_resilient_method:
            mode_str = 'Part 2: Real-World Faults Injection (Degraded Baseline)'
        elif self.enable_faults and self.use_resilient_method:
            mode_str = 'Part 3: Resilient Method + APF Safety Barrier'

        conv_str = (
            f'CONVERGED at t = {self.convergence_time:.2f} s'
            if self.is_converged
            else 'FAILED TO CONVERGE within timeout'
        )

        summary = (
            '\n======================================================================\n'
            f'           SWARM FORMATION EXPERIMENT SUMMARY REPORT\n'
            '======================================================================\n'
            f' Configuration:       {mode_str}\n'
            f' Total Swarm Agents:  {self.total_robots}\n'
            f' Final Shape Error:   {self.last_shape_error:.4f} m (RMS)\n'
            f' Convergence Status:  {conv_str}\n'
            f' Global Min Distance: {self.global_min_distance:.4f} m (Safety Limit: 0.50 m)\n'
            f' Total Collisions:    {self.total_collision_events} crash events detected\n'
            f' Metrics Saved:       {self.csv_path}\n'
            '======================================================================\n'
        )
        self.get_logger().info(summary)


def main(args=None) -> None:
    """Entry point for swarm metrics logger."""
    rclpy.init(args=args)
    node = SwarmMetricsLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node._print_final_summary()
        except Exception:
            pass
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
