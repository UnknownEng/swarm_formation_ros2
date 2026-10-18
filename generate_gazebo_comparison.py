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

"""Generate individual and comparative diagnostic plots across Gazebo Harmonic Parts."""

import os
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

matplotlib.use('Agg')


def plot_individual_run(csv_file: str, output_image: str, title_suffix: str = '') -> None:
    """Read a single metrics CSV and generate a 4-panel diagnostic plot."""
    if not os.path.exists(csv_file):
        print(f'Warning: CSV file {csv_file} not found. Skipping {output_image}.')
        return

    data = np.genfromtxt(csv_file, delimiter=',', names=True, dtype=None, encoding='utf-8')
    if data.size == 0:
        print(f'Warning: CSV file {csv_file} is empty. Skipping.')
        return

    time = data['time_sec']
    shape_error = data['shape_error_rms']
    min_dist = data['min_distance']

    fig, axs = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f'Gazebo Multi-Robot Formation Diagnostics {title_suffix}', fontsize=16, fontweight='bold')

    # 1. Shape Error vs Time
    axs[0, 0].plot(time, shape_error, 'b-', linewidth=2, label='Shape Error (RMS)')
    axs[0, 0].axhline(0.20, color='r', linestyle='--', label='Convergence Threshold (0.2m)')
    axs[0, 0].set_title('Formation Shape Error (RMS) vs Time')
    axs[0, 0].set_xlabel('Time (s)')
    axs[0, 0].set_ylabel('RMS Error (m)')
    axs[0, 0].grid(True, linestyle=':')
    axs[0, 0].legend()

    # 2. Minimum Inter-Robot Distance vs Time
    axs[0, 1].plot(time, min_dist, 'g-', linewidth=2, label='Min Inter-Robot Distance')
    axs[0, 1].axhline(0.70, color='orange', linestyle='--', label='APF Safety Zone (0.7m)')
    axs[0, 1].axhline(0.50, color='r', linestyle='-', label='Collision Danger Limit (0.5m)')
    axs[0, 1].set_title('Minimum Inter-Robot Distance vs Time')
    axs[0, 1].set_xlabel('Time (s)')
    axs[0, 1].set_ylabel('Distance (m)')
    axs[0, 1].grid(True, linestyle=':')
    axs[0, 1].legend()

    # 3. 2D Swarm Trajectories
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
    total_robots = 5
    for i in range(1, total_robots + 1):
        x_key = f'r{i}_x'
        y_key = f'r{i}_y'
        if x_key in data.dtype.names and y_key in data.dtype.names:
            rx = data[x_key]
            ry = data[y_key]
            c = colors[(i - 1) % len(colors)]
            axs[1, 0].plot(rx, ry, color=c, alpha=0.7, label=f'Robot {i}')
            axs[1, 0].scatter(rx[0], ry[0], color=c, marker='o', s=60)
            axs[1, 0].scatter(rx[-1], ry[-1], color=c, marker='*', s=120)

    final_x = [data[f'r{i}_x'][-1] for i in range(1, total_robots + 1) if f'r{i}_x' in data.dtype.names]
    final_y = [data[f'r{i}_y'][-1] for i in range(1, total_robots + 1) if f'r{i}_y' in data.dtype.names]
    if len(final_x) == total_robots:
        final_x.append(final_x[0])
        final_y.append(final_y[0])
        axs[1, 0].plot(final_x, final_y, 'k--', linewidth=1.5, label='Final Shape')

    axs[1, 0].set_title('2D Robot Swarm Trajectories (o=Start, *=End)')
    axs[1, 0].set_xlabel('X (m)')
    axs[1, 0].set_ylabel('Y (m)')
    axs[1, 0].set_aspect('equal', 'box')
    axs[1, 0].grid(True, linestyle=':')
    axs[1, 0].legend(loc='upper right', fontsize='small')

    # 4. Statistical Summary Box
    axs[1, 1].axis('off')
    final_err = shape_error[-1]
    global_min = np.min(min_dist)
    below_collision = np.sum(min_dist < 0.50)
    summary_text = (
        '=== Quantitative Summary ===\n\n'
        f'Total Simulation Time: {time[-1]:.2f} s\n'
        f'Final Shape Error (RMS): {final_err:.4f} m\n'
        f'Global Min Inter-Robot Dist: {global_min:.4f} m\n'
        f'APF Safety Barrier Active: {global_min >= 0.50}\n'
        f'Total Collision Events (d < 0.50m): {below_collision}\n'
    )
    axs[1, 1].text(
        0.1, 0.5, summary_text, fontsize=12, family='monospace',
        verticalalignment='center',
        bbox={'boxstyle': 'round', 'facecolor': 'wheat', 'alpha': 0.5}
    )

    plt.tight_layout()
    plt.savefig(output_image, dpi=200)
    plt.close(fig)
    print(f'Generated {output_image} successfully.')


def generate_comparison() -> None:
    """Load Gazebo benchmark CSV logs and generate multi-panel comparative figure."""
    f1 = 'gazebo_part1_baseline.csv'
    f2 = 'gazebo_part2_faults.csv'
    f3 = 'gazebo_part3_resilient.csv'

    if not (os.path.exists(f1) and os.path.exists(f2) and os.path.exists(f3)):
        print('Warning: One or more benchmark CSVs missing. Skipping comparative plot.')
        return

    d1 = np.genfromtxt(f1, delimiter=',', names=True, dtype=None, encoding='utf-8')
    d2 = np.genfromtxt(f2, delimiter=',', names=True, dtype=None, encoding='utf-8')
    d3 = np.genfromtxt(f3, delimiter=',', names=True, dtype=None, encoding='utf-8')

    fig = plt.figure(figsize=(16, 12))
    fig.suptitle('Gazebo Harmonic Multi-Robot Formation: Physics Benchmark Comparison',
                 fontsize=17, fontweight='bold', y=0.98)

    # 1. Shape Error (RMS) comparison
    ax1 = plt.subplot(2, 2, 1)
    ax1.plot(d1['time_sec'], d1['shape_error_rms'], 'b-', linewidth=2.0, label='Part 1: Baseline (Ideal Physics)')
    ax1.plot(d2['time_sec'], d2['shape_error_rms'], 'r--', linewidth=1.8, label='Part 2: Baseline + Real-World Faults')
    ax1.plot(d3['time_sec'], d3['shape_error_rms'], 'g-', linewidth=2.4, label='Part 3: Resilient Consensus + APF')
    ax1.axhline(0.20, color='gray', linestyle=':', linewidth=1.5, label='Convergence Threshold (0.20m)')
    ax1.axvspan(5.0, 10.0, color='orange', alpha=0.15, label='Agent 4 Dropout (5s - 10s)')
    ax1.set_title('Shape Error (RMS) vs Time', fontsize=13, fontweight='bold')
    ax1.set_xlabel('Time (s)', fontsize=11)
    ax1.set_ylabel('RMS Shape Error (m)', fontsize=11)
    ax1.grid(True, linestyle=':')
    ax1.legend(loc='upper right', fontsize='small')

    # 2. Minimum Inter-Robot Distance comparison
    ax2 = plt.subplot(2, 2, 2)
    ax2.plot(d1['time_sec'], d1['min_distance'], 'b-', linewidth=1.8, label='Part 1: Baseline (CRASHES)')
    ax2.plot(d2['time_sec'], d2['min_distance'], 'r--', linewidth=1.8, label='Part 2: Baseline + Faults (CRASHES)')
    ax2.plot(d3['time_sec'], d3['min_distance'], 'g-', linewidth=2.5, label='Part 3: Resilient + APF (ZERO CRASHES)')
    ax2.axhline(0.85, color='orange', linestyle='--', linewidth=1.5, label='APF Safety Zone (0.85m)')
    ax2.axhline(0.50, color='red', linestyle='-', linewidth=2.0, label='Collision Danger Limit (0.50m)')
    ax2.set_title('Minimum Inter-Robot Distance vs Time (Physics Safety)', fontsize=13, fontweight='bold')
    ax2.set_xlabel('Time (s)', fontsize=11)
    ax2.set_ylabel('Distance (m)', fontsize=11)
    ax2.grid(True, linestyle=':')
    ax2.legend(loc='upper right', fontsize='small')

    # 3. Bar Chart Comparison: Global Minimum Distance
    ax3 = plt.subplot(2, 2, 3)
    cases = ['Part 1: Baseline', 'Part 2: Faults', 'Part 3: Resilient APF']
    min_dists = [float(np.min(d1['min_distance'])),
                 float(np.min(d2['min_distance'])),
                 float(np.min(d3['min_distance']))]
    x = np.arange(len(cases))
    width = 0.45

    bars1 = ax3.bar(x, min_dists, width, color=['#337ab7', '#d9534f', '#5cb85c'])
    ax3.axhline(0.50, color='red', linestyle='--', linewidth=2.0, label='Crash Threshold (0.50m)')
    ax3.set_ylabel('Minimum Distance (m)', fontsize=11)
    ax3.set_title('Global Minimum Distance (Rigid Body Clearance)', fontsize=13, fontweight='bold')
    ax3.set_xticks(x)
    ax3.set_xticklabels(cases, fontsize=10, fontweight='bold')
    ax3.set_ylim(0, max(min_dists) * 1.25)
    ax3.grid(True, axis='y', linestyle=':')
    ax3.legend(loc='upper left')
    for bar in bars1:
        h = bar.get_height()
        ax3.annotate(f'{h:.3f}m',
                     xy=(bar.get_x() + bar.get_width() / 2, h),
                     xytext=(0, 4),
                     textcoords="offset points",
                     ha='center', va='bottom', fontweight='bold', fontsize=11)

    # 4. Bar Chart Comparison: Physical Collision Events
    ax4 = plt.subplot(2, 2, 4)
    collisions = [int(np.sum(d1['min_distance'] < 0.50)),
                  int(np.sum(d2['min_distance'] < 0.50)),
                  int(np.sum(d3['min_distance'] < 0.50))]
    bars2 = ax4.bar(x, collisions, width, color=['#d9534f', '#f0ad4e', '#5cb85c'])
    ax4.set_ylabel('Collision Events Detected (d < 0.50m)', fontsize=11)
    ax4.set_title('Physical Collisions Count (Crash Events)', fontsize=13, fontweight='bold')
    ax4.set_xticks(x)
    ax4.set_xticklabels(cases, fontsize=10, fontweight='bold')
    ax4.set_ylim(0, max(collisions) * 1.25 + 5)
    ax4.grid(True, axis='y', linestyle=':')
    for bar in bars2:
        h = bar.get_height()
        ax4.annotate(f'{h}',
                     xy=(bar.get_x() + bar.get_width() / 2, h),
                     xytext=(0, 4),
                     textcoords="offset points",
                     ha='center', va='bottom', fontweight='bold', fontsize=12)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig('gazebo_swarm_comparison.png', dpi=200)
    plt.close(fig)
    print('Generated gazebo_swarm_comparison.png successfully.')


def main() -> None:
    """Generate both individual run plots and overall benchmark comparison figure."""
    runs = [
        ('gazebo_part1_baseline.csv', 'gazebo_part1_baseline.png', '(Part 1: Baseline)'),
        ('gazebo_part2_faults.csv', 'gazebo_part2_faults.png', '(Part 2: Real-World Faults)'),
        ('gazebo_part3_resilient.csv', 'gazebo_part3_resilient.png', '(Part 3: Resilient + APF Barrier)'),
    ]
    for csv_file, out_png, title in runs:
        plot_individual_run(csv_file, out_png, title)

    generate_comparison()


if __name__ == '__main__':
    main()
