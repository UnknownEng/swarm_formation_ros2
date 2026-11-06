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

"""Rosbag2 offline processing and quantitative metrics computation engine."""

import argparse
import csv
import json
import math
import os
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rclpy.serialization
from rosidl_runtime_py.utilities import get_message
import yaml

try:
    import rosbag2_py
except ImportError:
    rosbag2_py = None


class RosbagProcessor:
    """Extracts trajectories and computes quantitative swarm formation metrics from rosbag2."""

    def __init__(
        self,
        bag_path: str,
        total_robots: int = 5,
        formation_radius: float = 2.0,
        convergence_threshold: float = 0.25,
        settle_duration: float = 2.0,
        collision_threshold: float = 0.50,
    ) -> None:
        """Initialize processor configuration and internal data structures."""
        self.bag_path = bag_path
        self.total_robots = total_robots
        self.radius = formation_radius
        self.convergence_threshold = convergence_threshold
        self.settle_duration = settle_duration
        self.collision_threshold = collision_threshold

        # Raw extracted time series: robot_id -> List of (time_sec, x, y, yaw)
        self.raw_poses: Dict[int, List[Tuple[float, float, float, float]]] = {
            i: [] for i in range(1, total_robots + 1)
        }
        self.raw_est_poses: Dict[int, List[Tuple[float, float, float, float]]] = {
            i: [] for i in range(1, total_robots + 1)
        }
        self.raw_cmd_vel: Dict[int, List[Tuple[float, float, float]]] = {
            i: [] for i in range(1, total_robots + 1)
        }
        self.raw_cmd_vel_raw: Dict[int, List[Tuple[float, float, float]]] = {
            i: [] for i in range(1, total_robots + 1)
        }
        self.raw_est_error: Dict[int, List[Tuple[float, float]]] = {
            i: [] for i in range(1, total_robots + 1)
        }
        self.raw_interventions: Dict[int, List[Tuple[float, float]]] = {
            i: [] for i in range(1, total_robots + 1)
        }

        self.storage_identifier: str = ''
        self.start_time_ns: Optional[int] = None
        self.end_time_ns: Optional[int] = None
        self.cached_metrics: Optional[Dict[str, Any]] = None

    def _discover_storage_identifier(self) -> str:
        """Inspect metadata.yaml to detect the storage plugin (mcap or sqlite3)."""
        meta_file = os.path.join(self.bag_path, 'metadata.yaml')
        if os.path.exists(meta_file):
            with open(meta_file, 'r', encoding='utf-8') as f:
                meta = yaml.safe_load(f)
                bag_info = meta.get('rosbag2_bagfile_information', {})
                return bag_info.get('storage_identifier', 'mcap')
        return 'mcap'

    def read_bag(self) -> None:
        """Read and deserialize all relevant messages from the rosbag2 directory."""
        if rosbag2_py is None:
            raise RuntimeError('rosbag2_py is not installed or available in this environment.')

        if not os.path.exists(self.bag_path):
            raise FileNotFoundError(f'Bag directory does not exist: {self.bag_path}')

        self.storage_identifier = self._discover_storage_identifier()
        storage_options = rosbag2_py.StorageOptions(
            uri=self.bag_path,
            storage_id=self.storage_identifier,
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        )

        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        topics_and_types = {
            t.name: t.type for t in reader.get_all_topics_and_types()
        }

        type_map: Dict[str, Any] = {}
        for topic_name, type_str in topics_and_types.items():
            try:
                type_map[topic_name] = get_message(type_str)
            except Exception:
                type_map[topic_name] = None

        first_ts_ns: Optional[int] = None

        while reader.has_next():
            topic, data, ts_ns = reader.read_next()
            if first_ts_ns is None:
                first_ts_ns = ts_ns
            self.end_time_ns = ts_ns

            msg_cls = type_map.get(topic)
            if msg_cls is None:
                continue

            try:
                msg = rclpy.serialization.deserialize_message(data, msg_cls)
            except Exception:
                continue

            t_sec = (ts_ns - first_ts_ns) * 1e-9

            # Check robot-specific topics
            for i in range(1, self.total_robots + 1):
                if topic == f'/robot_{i}/pose':
                    p_obj = msg.pose if hasattr(msg, 'pose') else msg
                    yaw = self._quaternion_to_yaw(
                        p_obj.orientation.x,
                        p_obj.orientation.y,
                        p_obj.orientation.z,
                        p_obj.orientation.w,
                    )
                    self.raw_poses[i].append((t_sec, p_obj.position.x, p_obj.position.y, yaw))
                elif topic == f'/robot_{i}/estimated_pose':
                    p_obj = msg.pose if hasattr(msg, 'pose') else msg
                    yaw = self._quaternion_to_yaw(
                        p_obj.orientation.x,
                        p_obj.orientation.y,
                        p_obj.orientation.z,
                        p_obj.orientation.w,
                    )
                    self.raw_est_poses[i].append((t_sec, p_obj.position.x, p_obj.position.y, yaw))
                elif topic == f'/robot_{i}/cmd_vel':
                    self.raw_cmd_vel[i].append((t_sec, msg.linear.x, msg.angular.z))
                elif topic == f'/robot_{i}/cmd_vel_raw':
                    self.raw_cmd_vel_raw[i].append((t_sec, msg.linear.x, msg.angular.z))
                elif topic == f'/robot_{i}/estimation_error':
                    self.raw_est_error[i].append((t_sec, msg.x))
                elif topic == f'/robot_{i}/safety_intervention':
                    self.raw_interventions[i].append((t_sec, float(msg.data)))

        self.start_time_ns = first_ts_ns

    @staticmethod
    def _quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        """Convert quaternion components to yaw rotation in radians."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def compute_procrustes_error(
        positions: List[Tuple[float, float]],
        radius: float,
        total_robots: int = 5,
    ) -> float:
        """Calculate Procrustes RMS shape error aligned to regular polygon of given radius."""
        if len(positions) < total_robots:
            return float('nan')

        sum_x = sum(p[0] for p in positions[:total_robots])
        sum_y = sum(p[1] for p in positions[:total_robots])
        cx = sum_x / total_robots
        cy = sum_y / total_robots

        target_pts = [
            (
                radius * math.cos(2.0 * math.pi * i / total_robots),
                radius * math.sin(2.0 * math.pi * i / total_robots),
            )
            for i in range(total_robots)
        ]

        # Calculate optimal rotation alignment angle (Procrustes)
        num = sum(
            (positions[i][0] - cx) * target_pts[i][1]
            - (positions[i][1] - cy) * target_pts[i][0]
            for i in range(total_robots)
        )
        den = sum(
            (positions[i][0] - cx) * target_pts[i][0]
            + (positions[i][1] - cy) * target_pts[i][1]
            for i in range(total_robots)
        )
        theta_align = math.atan2(num, den)

        sq_err_sum = 0.0
        for i in range(total_robots):
            u = positions[i][0] - cx
            v = positions[i][1] - cy
            rot_u = u * math.cos(theta_align) - v * math.sin(theta_align)
            rot_v = u * math.sin(theta_align) + v * math.cos(theta_align)
            tx, ty = target_pts[i]
            sq_err_sum += (rot_u - tx) ** 2 + (rot_v - ty) ** 2

        return math.sqrt(sq_err_sum / total_robots)

    def compute_metrics(self, dt: float = 0.05) -> Dict[str, Any]:
        """Resample time-series data at regular intervals dt and compute all swarm metrics."""
        # Check if we have valid pose data
        max_times = []
        for i in range(1, self.total_robots + 1):
            if self.raw_poses[i]:
                max_times.append(self.raw_poses[i][-1][0])
            else:
                max_times.append(0.0)

        t_end = max(max_times) if max_times else 0.0
        if t_end <= 0.0:
            return {
                'time_sec': np.array([]),
                'shape_error_rms': np.array([]),
                'min_distance': np.array([]),
                'global_min_distance': float('inf'),
                'total_collisions': 0,
                'is_converged': False,
                'convergence_time': None,
                'mean_est_error': 0.0,
                'empirically_collision_free': True,
                'total_control_energy': 0.0,
                'total_safety_interventions': 0.0,
            }

        num_steps = max(1, int(math.ceil(t_end / dt)) + 1)
        grid_t = np.linspace(0.0, t_end, num_steps)

        # Interpolate robot ground-truth poses onto uniform grid
        interp_x: Dict[int, np.ndarray] = {}
        interp_y: Dict[int, np.ndarray] = {}
        for i in range(1, self.total_robots + 1):
            records = self.raw_poses[i]
            if len(records) >= 2:
                ts = [r[0] for r in records]
                xs = [r[1] for r in records]
                ys = [r[2] for r in records]
                interp_x[i] = np.interp(grid_t, ts, xs)
                interp_y[i] = np.interp(grid_t, ts, ys)
            elif len(records) == 1:
                interp_x[i] = np.full_like(grid_t, records[0][1])
                interp_y[i] = np.full_like(grid_t, records[0][2])
            else:
                interp_x[i] = np.zeros_like(grid_t)
                interp_y[i] = np.zeros_like(grid_t)

        # Interpolate estimated poses if available
        interp_est_x: Dict[int, np.ndarray] = {}
        interp_est_y: Dict[int, np.ndarray] = {}
        has_est = any(len(self.raw_est_poses[i]) > 0 for i in range(1, self.total_robots + 1))
        if has_est:
            for i in range(1, self.total_robots + 1):
                records = self.raw_est_poses[i]
                if len(records) >= 2:
                    ts = [r[0] for r in records]
                    xs = [r[1] for r in records]
                    ys = [r[2] for r in records]
                    interp_est_x[i] = np.interp(grid_t, ts, xs)
                    interp_est_y[i] = np.interp(grid_t, ts, ys)
                elif len(records) == 1:
                    interp_est_x[i] = np.full_like(grid_t, records[0][1])
                    interp_est_y[i] = np.full_like(grid_t, records[0][2])
                else:
                    interp_est_x[i] = interp_x[i].copy()
                    interp_est_y[i] = interp_y[i].copy()

        # Compute metrics step-by-step
        shape_errors = np.zeros(num_steps)
        min_distances = np.zeros(num_steps)
        global_min_dists = np.zeros(num_steps)
        step_collisions = np.zeros(num_steps, dtype=int)
        mean_est_errors = np.zeros(num_steps)

        running_global_min = float('inf')
        total_collisions = 0

        # Convergence state tracking
        consecutive_settled_time = 0.0
        is_converged_flag = False
        convergence_time: Optional[float] = None
        is_converged_series = np.zeros(num_steps, dtype=bool)

        for k in range(num_steps):
            pts = [(interp_x[i][k], interp_y[i][k]) for i in range(1, self.total_robots + 1)]
            se = self.compute_procrustes_error(pts, self.radius, self.total_robots)
            shape_errors[k] = se

            # Inter-robot distances
            step_min = float('inf')
            colls = 0
            for i in range(self.total_robots):
                for j in range(i + 1, self.total_robots):
                    d = math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1])
                    if d < step_min:
                        step_min = d
                    if d < self.collision_threshold:
                        colls += 1

            min_distances[k] = step_min
            if step_min < running_global_min:
                running_global_min = step_min
            global_min_dists[k] = running_global_min

            if colls > 0:
                total_collisions += colls
            step_collisions[k] = colls

            # Convergence tracking
            if se < self.convergence_threshold:
                consecutive_settled_time += dt
                if consecutive_settled_time >= self.settle_duration and not is_converged_flag:
                    is_converged_flag = True
                    convergence_time = grid_t[k] - self.settle_duration
            else:
                consecutive_settled_time = 0.0

            is_converged_series[k] = is_converged_flag

            # Estimation error calculation
            if has_est:
                err_sum = sum(
                    math.hypot(interp_est_x[i][k] - interp_x[i][k],
                               interp_est_y[i][k] - interp_y[i][k])
                    for i in range(1, self.total_robots + 1)
                )
                mean_est_errors[k] = err_sum / self.total_robots
            else:
                mean_est_errors[k] = 0.0

        # Compute control energy: sum integral (v^2 + w^2) dt
        total_energy = 0.0
        for i in range(1, self.total_robots + 1):
            cmd_recs = self.raw_cmd_vel[i]
            if len(cmd_recs) >= 2:
                ts = [r[0] for r in cmd_recs]
                vs = [r[1] for r in cmd_recs]
                ws = [r[2] for r in cmd_recs]
                interp_v = np.interp(grid_t, ts, vs)
                interp_w = np.interp(grid_t, ts, ws)
                total_energy += float(np.sum(interp_v ** 2 + interp_w ** 2) * dt)

        # Compute total safety interventions
        total_interventions = 0.0
        for i in range(1, self.total_robots + 1):
            int_recs = self.raw_interventions[i]
            if int_recs:
                total_interventions += sum(r[1] for r in int_recs)

        results: Dict[str, Any] = {
            'time_sec': grid_t,
            'shape_error_rms': shape_errors,
            'min_distance': min_distances,
            'global_min_distance': running_global_min,
            'global_min_distance_series': global_min_dists,
            'step_collisions': step_collisions,
            'total_collisions': total_collisions,
            'is_converged': is_converged_flag,
            'is_converged_series': is_converged_series,
            'convergence_time': convergence_time,
            'mean_est_error': float(np.mean(mean_est_errors)),
            'mean_est_error_series': mean_est_errors,
            'max_est_error': float(np.max(mean_est_errors)) if len(mean_est_errors) > 0 else 0.0,
            'final_shape_error': float(shape_errors[-1]) if len(shape_errors) > 0 else 0.0,
            'min_shape_error': float(np.min(shape_errors)) if len(shape_errors) > 0 else 0.0,
            'empirically_collision_free': bool(running_global_min >= self.collision_threshold),
            'total_control_energy': total_energy,
            'total_safety_interventions': total_interventions,
            'interp_x': interp_x,
            'interp_y': interp_y,
            'interp_est_x': interp_est_x,
            'interp_est_y': interp_est_y,
            'total_duration': float(t_end),
            'total_robots': self.total_robots,
        }
        self.cached_metrics = results
        return results

    def export_csv(self, output_csv: str, dt: float = 0.05) -> None:
        """Export computed time-series metrics into a standardized CSV file."""
        if self.cached_metrics is None:
            self.compute_metrics(dt=dt)

        m = self.cached_metrics
        assert m is not None

        csv_dir = os.path.dirname(output_csv)
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
            'mean_est_error',
        ]
        for i in range(1, self.total_robots + 1):
            header.extend([f'r{i}_x', f'r{i}_y'])

        with open(output_csv, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(header)

            times = m['time_sec']
            shape_err = m['shape_error_rms']
            min_dist = m['min_distance']
            g_min_dist = m['global_min_distance_series']
            colls = m['step_collisions']
            conv_series = m['is_converged_series']
            conv_t = m['convergence_time']
            conv_val = f'{conv_t:.3f}' if conv_t is not None else 'NaN'
            est_err = m['mean_est_error_series']

            for k in range(len(times)):
                row = [
                    f'{times[k]:.2f}',
                    f'{shape_err[k]:.4f}',
                    f'{min_dist[k]:.4f}',
                    f'{g_min_dist[k]:.4f}',
                    str(colls[k]),
                    str(conv_series[k]),
                    conv_val,
                    f'{est_err[k]:.4f}',
                ]
                for i in range(1, self.total_robots + 1):
                    rx = m['interp_x'][i][k]
                    ry = m['interp_y'][i][k]
                    row.extend([f'{rx:.4f}', f'{ry:.4f}'])
                writer.writerow(row)

    def export_summary_json(self, output_json: str, dt: float = 0.05) -> None:
        """Export high-level quantitative summary metrics to JSON."""
        if self.cached_metrics is None:
            self.compute_metrics(dt=dt)

        m = self.cached_metrics
        assert m is not None

        json_dir = os.path.dirname(output_json)
        if json_dir and not os.path.exists(json_dir):
            os.makedirs(json_dir, exist_ok=True)

        summary = {
            'bag_path': self.bag_path,
            'total_robots': self.total_robots,
            'formation_radius': self.radius,
            'convergence_threshold': self.convergence_threshold,
            'settle_duration': self.settle_duration,
            'collision_threshold': self.collision_threshold,
            'total_duration_sec': m['total_duration'],
            'final_shape_error_rms': m['final_shape_error'],
            'min_shape_error_rms': m['min_shape_error'],
            'global_min_distance': m['global_min_distance'],
            'total_collisions': m['total_collisions'],
            'is_converged': m['is_converged'],
            'convergence_time_sec': m['convergence_time'],
            'mean_est_error': m['mean_est_error'],
            'max_est_error': m['max_est_error'],
            'empirically_collision_free': m['empirically_collision_free'],
            'total_control_energy': m['total_control_energy'],
            'total_safety_interventions': m['total_safety_interventions'],
        }

        with open(output_json, 'w', encoding='utf-8') as f:
            json.dump(summary, f, indent=2)


def main() -> None:
    """Entry point for standalone rosbag processing CLI."""
    parser = argparse.ArgumentParser(
        description='Extract trajectories and compute offline metrics from rosbag2.'
    )
    parser.add_argument('bag_path', type=str, help='Path to rosbag2 directory.')
    parser.add_argument(
        '--output-csv',
        type=str,
        default='offline_metrics.csv',
        help='Output path for metrics CSV (default: offline_metrics.csv).',
    )
    parser.add_argument(
        '--output-json',
        type=str,
        default='offline_summary.json',
        help='Output path for summary JSON (default: offline_summary.json).',
    )
    parser.add_argument(
        '--total-robots',
        type=int,
        default=5,
        help='Number of robots in the swarm (default: 5).',
    )
    parser.add_argument(
        '--radius',
        type=float,
        default=2.0,
        help='Target formation polygon radius in meters (default: 2.0).',
    )
    parser.add_argument(
        '--threshold',
        type=float,
        default=0.25,
        help='Convergence threshold in meters (default: 0.25).',
    )
    parser.add_argument(
        '--dt',
        type=float,
        default=0.05,
        help='Resampling timestep in seconds (default: 0.05 = 20Hz).',
    )

    args = parser.parse_args()

    processor = RosbagProcessor(
        bag_path=args.bag_path,
        total_robots=args.total_robots,
        formation_radius=args.radius,
        convergence_threshold=args.threshold,
    )

    print(f'Reading bag: {args.bag_path}...')
    processor.read_bag()
    print('Computing metrics...')
    metrics = processor.compute_metrics(dt=args.dt)

    processor.export_csv(args.output_csv, dt=args.dt)
    processor.export_summary_json(args.output_json, dt=args.dt)

    print('\n========================================')
    print('       OFFLINE ROSBAG METRICS SUMMARY   ')
    print('========================================')
    print(f'Total Duration        : {metrics["total_duration"]:.2f} s')
    print(f'Final Shape Error     : {metrics["final_shape_error"]:.4f} m')
    print(f'Min Shape Error       : {metrics["min_shape_error"]:.4f} m')
    print(f'Global Min Distance   : {metrics["global_min_distance"]:.4f} m')
    print(f'Total Collisions      : {metrics["total_collisions"]}')
    print(f'Converged             : {metrics["is_converged"]}')
    if metrics['convergence_time'] is not None:
        print(f'Convergence Time      : {metrics["convergence_time"]:.2f} s')
    else:
        print('Convergence Time      : Not Converged')
    print(f'Mean Estimation Error : {metrics["mean_est_error"]:.4f} m')
    print(f'Collision-Free Status : {metrics["empirically_collision_free"]}')
    print(f'Total Control Energy  : {metrics["total_control_energy"]:.4f}')
    print('========================================')
    print(f'Saved CSV : {args.output_csv}')
    print(f'Saved JSON: {args.output_json}\n')


if __name__ == '__main__':
    main()
