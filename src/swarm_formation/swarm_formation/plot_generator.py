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

"""Offline plot generation pipeline from rosbag2 data alone."""

import argparse
import os
from typing import Any, Dict, List

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from swarm_formation.rosbag_processor import RosbagProcessor

matplotlib.use('Agg')


class SwarmPlotGenerator:
    """Generates publication-quality diagnostic and trajectory figures from swarm run data."""

    ROBOT_COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2']

    def __init__(
        self,
        metrics: Dict[str, Any],
        output_dir: str = 'plots',
        prefix: str = '',
    ) -> None:
        """Initialize generator with precomputed metrics dictionary."""
        self.metrics = metrics
        self.output_dir = output_dir
        self.prefix = prefix
        if not os.path.exists(output_dir):
            os.makedirs(output_dir, exist_ok=True)

    @classmethod
    def from_bag(
        cls,
        bag_path: str,
        output_dir: str = 'plots',
        prefix: str = '',
    ) -> 'SwarmPlotGenerator':
        """Load and compute metrics directly from a rosbag2 folder."""
        processor = RosbagProcessor(bag_path=bag_path)
        processor.read_bag()
        metrics = processor.compute_metrics()
        return cls(metrics=metrics, output_dir=output_dir, prefix=prefix)

    @classmethod
    def from_csv(
        cls,
        csv_path: str,
        output_dir: str = 'plots',
        prefix: str = '',
    ) -> 'SwarmPlotGenerator':
        """Load metrics from a previously generated CSV file."""
        data = np.genfromtxt(csv_path, delimiter=',', names=True, dtype=None, encoding='utf-8')
        total_robots = 5
        interp_x = {}
        interp_y = {}
        for i in range(1, total_robots + 1):
            xk = f'r{i}_x'
            yk = f'r{i}_y'
            if xk in data.dtype.names and yk in data.dtype.names:
                interp_x[i] = data[xk]
                interp_y[i] = data[yk]

        metrics = {
            'time_sec': data['time_sec'],
            'shape_error_rms': data['shape_error_rms'],
            'min_distance': data['min_distance'],
            'global_min_distance': float(np.min(data['min_distance'])),
            'global_min_distance_series': data['global_min_distance'],
            'step_collisions': data['collisions_count'],
            'total_collisions': int(np.sum(data['collisions_count'])),
            'is_converged': bool(data['is_converged'][-1]),
            'is_converged_series': data['is_converged'].astype(bool),
            'convergence_time': None,
            'mean_est_error': float(np.mean(data['mean_est_error'])),
            'mean_est_error_series': data['mean_est_error'],
            'final_shape_error': float(data['shape_error_rms'][-1]),
            'min_shape_error': float(np.min(data['shape_error_rms'])),
            'empirically_collision_free': bool(np.min(data['min_distance']) >= 0.50),
            'interp_x': interp_x,
            'interp_y': interp_y,
            'total_duration': float(data['time_sec'][-1]),
            'total_robots': len(interp_x),
        }
        return cls(metrics=metrics, output_dir=output_dir, prefix=prefix)

    def _get_path(self, filename: str) -> str:
        """Construct full file path with optional prefix."""
        full_name = f'{self.prefix}_{filename}' if self.prefix else filename
        return os.path.join(self.output_dir, full_name)

    def plot_trajectories(self, filename: str = 'formation_trajectories.png') -> str:
        """Generate 2D robot swarm trajectories plot."""
        fig, ax = plt.subplots(figsize=(8, 8))
        m = self.metrics
        total_robots = m.get('total_robots', 5)

        for i in range(1, total_robots + 1):
            if i in m['interp_x'] and i in m['interp_y']:
                rx = m['interp_x'][i]
                ry = m['interp_y'][i]
                c = self.ROBOT_COLORS[(i - 1) % len(self.ROBOT_COLORS)]
                ax.plot(rx, ry, color=c, alpha=0.7, linewidth=2, label=f'Robot {i}')
                ax.scatter(rx[0], ry[0], color=c, marker='o', s=80, zorder=5)
                ax.scatter(rx[-1], ry[-1], color=c, marker='*', s=150, zorder=5)

        # Plot polygon connecting final positions
        final_x = [m['interp_x'][i][-1] for i in range(1, total_robots + 1) if i in m['interp_x']]
        final_y = [m['interp_y'][i][-1] for i in range(1, total_robots + 1) if i in m['interp_y']]
        if len(final_x) == total_robots:
            final_x.append(final_x[0])
            final_y.append(final_y[0])
            ax.plot(final_x, final_y, 'k--', linewidth=1.8, label='Final Polygon')

        # Plot swarm centroid trajectory
        cx = np.mean([m['interp_x'][i] for i in range(1, total_robots + 1)], axis=0)
        cy = np.mean([m['interp_y'][i] for i in range(1, total_robots + 1)], axis=0)
        ax.plot(cx, cy, 'm:', linewidth=1.5, label='Centroid Trajectory')

        ax.set_title('2D Swarm Trajectories (o=Start, *=End)', fontsize=14, fontweight='bold')
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_aspect('equal', 'box')
        ax.grid(True, linestyle=':', alpha=0.6)
        ax.legend(loc='upper right', fontsize='small')

        out_path = self._get_path(filename)
        fig.tight_layout()
        fig.savefig(out_path, dpi=300)
        plt.close(fig)
        return out_path

    def plot_shape_and_safety(self, filename: str = 'shape_error_and_safety.png') -> str:
        """Generate 2-panel figure: Shape Error (RMS) and Minimum Distance vs Time."""
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        m = self.metrics
        time = m['time_sec']

        # Panel 1: RMS Shape Error
        ax1.plot(time, m['shape_error_rms'], 'b-', linewidth=2, label='Shape Error (RMS)')
        ax1.axhline(0.25, color='r', linestyle='--', linewidth=1.5,
                    label='Convergence Threshold (0.25m)')
        if m.get('convergence_time') is not None:
            ax1.axvline(m['convergence_time'], color='g', linestyle=':', linewidth=2,
                        label=f'Converged ({m["convergence_time"]:.2f}s)')
        ax1.set_title('Swarm Shape Error (RMS) vs Time', fontsize=12, fontweight='bold')
        ax1.set_ylabel('RMS Error (m)', fontsize=11)
        ax1.grid(True, linestyle=':', alpha=0.6)
        ax1.legend(loc='upper right')

        # Panel 2: Minimum Distance
        ax2.plot(time, m['min_distance'], 'g-', linewidth=2, label='Min Inter-Robot Distance')
        ax2.axhline(0.70, color='orange', linestyle='--', linewidth=1.5,
                    label='Safety Barrier Zone (0.7m)')
        ax2.axhline(0.50, color='r', linestyle='-', linewidth=2,
                    label='Collision Danger Limit (0.5m)')
        ax2.set_title('Minimum Inter-Robot Distance vs Time', fontsize=12, fontweight='bold')
        ax2.set_xlabel('Time (s)', fontsize=11)
        ax2.set_ylabel('Distance (m)', fontsize=11)
        ax2.grid(True, linestyle=':', alpha=0.6)
        ax2.legend(loc='lower right')

        out_path = self._get_path(filename)
        fig.tight_layout()
        fig.savefig(out_path, dpi=300)
        plt.close(fig)
        return out_path

    def plot_estimation_error(self, filename: str = 'estimation_error.png') -> str:
        """Generate localization estimation error plot."""
        fig, ax = plt.subplots(figsize=(10, 5))
        m = self.metrics
        time = m['time_sec']

        ax.plot(time, m['mean_est_error_series'], 'k-', linewidth=2.5, label='Swarm Mean Error')
        ax.set_title('Localization Estimation Error vs Time', fontsize=12, fontweight='bold')
        ax.set_xlabel('Time (s)', fontsize=11)
        ax.set_ylabel('Estimation Error (m)', fontsize=11)
        ax.grid(True, linestyle=':', alpha=0.6)
        ax.legend(loc='upper right')

        out_path = self._get_path(filename)
        fig.tight_layout()
        fig.savefig(out_path, dpi=300)
        plt.close(fig)
        return out_path

    def plot_diagnostic_summary(self, filename: str = 'diagnostic_summary.png') -> str:
        """Generate 4-panel comprehensive diagnostic summary plot."""
        fig, axs = plt.subplots(2, 2, figsize=(14, 10))
        m = self.metrics
        time = m['time_sec']
        total_robots = m.get('total_robots', 5)

        fig.suptitle('Swarm Formation Diagnostics (Offline Rosbag Analysis)',
                     fontsize=16, fontweight='bold')

        # 1. Shape Error
        axs[0, 0].plot(time, m['shape_error_rms'], 'b-', linewidth=2, label='Shape Error (RMS)')
        axs[0, 0].axhline(0.25, color='r', linestyle='--', label='Threshold (0.25m)')
        if m.get('convergence_time') is not None:
            conv_lbl = f'Conv: {m["convergence_time"]:.1f}s'
            axs[0, 0].axvline(m['convergence_time'], color='g', linestyle=':', label=conv_lbl)
        axs[0, 0].set_title('Formation Shape Error (RMS) vs Time')
        axs[0, 0].set_xlabel('Time (s)')
        axs[0, 0].set_ylabel('RMS Error (m)')
        axs[0, 0].grid(True, linestyle=':')
        axs[0, 0].legend()

        # 2. Min Distance
        axs[0, 1].plot(time, m['min_distance'], 'g-', linewidth=2, label='Min Distance')
        axs[0, 1].axhline(0.70, color='orange', linestyle='--', label='Safety Zone (0.7m)')
        axs[0, 1].axhline(0.50, color='r', linestyle='-', label='Collision Limit (0.5m)')
        axs[0, 1].set_title('Minimum Inter-Robot Distance vs Time')
        axs[0, 1].set_xlabel('Time (s)')
        axs[0, 1].set_ylabel('Distance (m)')
        axs[0, 1].grid(True, linestyle=':')
        axs[0, 1].legend()

        # 3. Trajectories
        for i in range(1, total_robots + 1):
            if i in m['interp_x'] and i in m['interp_y']:
                rx = m['interp_x'][i]
                ry = m['interp_y'][i]
                c = self.ROBOT_COLORS[(i - 1) % len(self.ROBOT_COLORS)]
                axs[1, 0].plot(rx, ry, color=c, alpha=0.7, label=f'Robot {i}')
                axs[1, 0].scatter(rx[0], ry[0], color=c, marker='o', s=50)
                axs[1, 0].scatter(rx[-1], ry[-1], color=c, marker='*', s=100)

        final_x = [m['interp_x'][i][-1] for i in range(1, total_robots + 1) if i in m['interp_x']]
        final_y = [m['interp_y'][i][-1] for i in range(1, total_robots + 1) if i in m['interp_y']]
        if len(final_x) == total_robots:
            final_x.append(final_x[0])
            final_y.append(final_y[0])
            axs[1, 0].plot(final_x, final_y, 'k--', linewidth=1.5, label='Final Polygon')

        axs[1, 0].set_title('2D Trajectories (o=Start, *=End)')
        axs[1, 0].set_xlabel('X (m)')
        axs[1, 0].set_ylabel('Y (m)')
        axs[1, 0].set_aspect('equal', 'box')
        axs[1, 0].grid(True, linestyle=':')
        axs[1, 0].legend(loc='upper right', fontsize='x-small')

        # 4. Quantitative Summary Box
        axs[1, 1].axis('off')
        if m.get('convergence_time') is not None:
            conv_text = f'{m["convergence_time"]:.2f} s'
        else:
            conv_text = 'Not Converged'
        safe_str = 'COLLISION-FREE' if m['empirically_collision_free'] else 'COLLISION DETECTED'
        summary_text = (
            '=== Quantitative Rosbag Summary ===\n\n'
            f'Total Simulation Time    : {m["total_duration"]:.2f} s\n'
            f'Final Shape Error (RMS)  : {m["final_shape_error"]:.4f} m\n'
            f'Min Shape Error (RMS)    : {m["min_shape_error"]:.4f} m\n'
            f'Convergence Status       : {m["is_converged"]}\n'
            f'Convergence Settle Time  : {conv_text}\n'
            f'Global Min Distance      : {m["global_min_distance"]:.4f} m\n'
            f'Total Collisions Recorded: {m["total_collisions"]}\n'
            f'Empirical Safety Status  : {safe_str}\n'
            f'Mean Localization Error  : {m["mean_est_error"]:.4f} m\n'
        )
        box_props = {
            'boxstyle': 'round,pad=0.8',
            'facecolor': '#f0f4f8',
            'edgecolor': '#3b82f6',
            'alpha': 0.9,
        }
        axs[1, 1].text(0.08, 0.5, summary_text, fontsize=12, family='monospace',
                       verticalalignment='center', bbox=box_props)

        out_path = self._get_path(filename)
        fig.tight_layout()
        fig.savefig(out_path, dpi=300)
        plt.close(fig)
        return out_path

    def generate_all_plots(self) -> List[str]:
        """Generate all figures in the pipeline and return list of saved file paths."""
        return [
            self.plot_trajectories(),
            self.plot_shape_and_safety(),
            self.plot_estimation_error(),
            self.plot_diagnostic_summary(),
        ]


def main() -> None:
    """Entry point for standalone plot generator CLI."""
    parser = argparse.ArgumentParser(description='Generate plots from rosbag2 or metrics CSV.')
    parser.add_argument('input_path', type=str, help='Path to rosbag2 folder or metrics CSV.')
    parser.add_argument(
        '--output-dir', type=str, default='plots', help='Output directory for figures.'
    )
    parser.add_argument(
        '--prefix', type=str, default='', help='Prefix for generated image filenames.'
    )

    args = parser.parse_args()

    if os.path.isdir(args.input_path):
        print(f'Loading data from rosbag2: {args.input_path}...')
        generator = SwarmPlotGenerator.from_bag(
            args.input_path, output_dir=args.output_dir, prefix=args.prefix
        )
    elif os.path.isfile(args.input_path) and args.input_path.endswith('.csv'):
        print(f'Loading data from CSV: {args.input_path}...')
        generator = SwarmPlotGenerator.from_csv(
            args.input_path, output_dir=args.output_dir, prefix=args.prefix
        )
    else:
        print(f'Error: Unsupported input format: {args.input_path}')
        return

    saved_files = generator.generate_all_plots()
    print('Generated plots:')
    for p in saved_files:
        print(f'  - {p}')


if __name__ == '__main__':
    main()
