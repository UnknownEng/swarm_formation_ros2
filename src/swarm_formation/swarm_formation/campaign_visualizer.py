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

"""Publication-grade statistical visualization engine for multi-robot experiment campaigns."""

import argparse
import json
import os
from typing import Any, Dict, List

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

matplotlib.use('Agg')


class CampaignVisualizer:
    """Generates comparative statistical figures, ablation charts, and parameter sweeps."""

    def __init__(
        self,
        campaign_data: Dict[str, Any],
        output_dir: str = 'results/M8/plots',
    ) -> None:
        """Initialize visualizer with campaign data dictionary."""
        self.data = campaign_data
        self.output_dir = output_dir
        if not os.path.exists(output_dir):
            os.makedirs(output_dir, exist_ok=True)

    @classmethod
    def from_json(
        cls,
        json_path: str,
        output_dir: str = 'results/M8/plots',
    ) -> 'CampaignVisualizer':
        """Load campaign data directly from campaign_results.json."""
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        return cls(campaign_data=data, output_dir=output_dir)

    def plot_ablation_study(self, filename: str = 'ablation_study_barchart.png') -> str:
        """Generate comparative bar chart for ablation conditions with 95% CI error bars."""
        conditions = [
            ('baseline_unshielded', 'Baseline\n(Linear)'),
            ('ablation_huber_only', 'Huber\nOnly'),
            ('ablation_ema_only', 'EMA\nOnly'),
            ('ablation_heartbeat_only', 'Heartbeat\nOnly'),
            ('full_resilient_cbf', 'Full Stack\n(Resilient+CBF)'),
        ]

        labels = []
        shape_means = []
        shape_ci_errs = []
        safe_rates = []

        for key, name in conditions:
            if key in self.data:
                s = self.data[key]['summary']
                labels.append(name)
                shape_means.append(s['shape_error_mean'])
                ci_margin = max(0.0, s['shape_error_ci_upper'] - s['shape_error_mean'])
                shape_ci_errs.append(ci_margin)
                safe_rates.append(s['collision_free_rate_pct'])

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
        fig.suptitle('Ablation Study: Swarm Resiliency & Safety Components (25 Seeds)',
                     fontsize=14, fontweight='bold')

        x_pos = np.arange(len(labels))
        colors = ['#94a3b8', '#60a5fa', '#38bdf8', '#818cf8', '#22c55e']

        # Panel 1: Formation Shape Error (RMS)
        ax1.bar(x_pos, shape_means, yerr=shape_ci_errs, capsize=5, color=colors,
                edgecolor='black', alpha=0.85, width=0.6)
        ax1.axhline(0.25, color='r', linestyle='--', linewidth=1.5,
                    label='Convergence Threshold (0.25m)')
        ax1.set_xticks(x_pos)
        ax1.set_xticklabels(labels, fontsize=10)
        ax1.set_ylabel('Final Shape Error (RMS, m) [Mean +/- 95% CI]', fontsize=11)
        ax1.set_title('Formation Accuracy', fontsize=12, fontweight='bold')
        ax1.grid(True, linestyle=':', alpha=0.6, axis='y')
        ax1.legend(loc='upper right')

        # Panel 2: Empirical Collision-Free Rate
        safe_colors = ['#ef4444' if r < 100.0 else '#22c55e' for r in safe_rates]
        bars2 = ax2.bar(x_pos, safe_rates, color=safe_colors, edgecolor='black',
                        alpha=0.85, width=0.6)
        ax2.set_xticks(x_pos)
        ax2.set_xticklabels(labels, fontsize=10)
        ax2.set_ylabel('Empirical Collision-Free Rate (%)', fontsize=11)
        ax2.set_ylim(0, 110)
        ax2.set_title('Swarm Safety Preservation', fontsize=12, fontweight='bold')
        ax2.grid(True, linestyle=':', alpha=0.6, axis='y')

        # Add percentage labels above bars
        for bar, val in zip(bars2, safe_rates):
            ax2.text(bar.get_x() + bar.get_width() / 2.0, val + 2.0,
                     f'{val:.1f}%', ha='center', va='bottom', fontsize=10, fontweight='bold')

        out_path = os.path.join(self.output_dir, filename)
        fig.tight_layout()
        fig.savefig(out_path, dpi=300)
        plt.close(fig)
        return out_path

    def plot_safety_comparison(self, filename: str = 'safety_comparison_apf_vs_cbf.png') -> str:
        """Generate box plot distribution comparing Unshielded, APF-only, and CBF-QP safety."""
        conditions = [
            ('baseline_unshielded', 'Unshielded\n(No Avoidance)'),
            ('apf_only', 'APF-Only\n(Potential Field)'),
            ('cbf_qp_only', 'CBF-QP Filter\n(Control Barrier)'),
        ]

        data_boxes = []
        labels = []
        for key, name in conditions:
            if key in self.data:
                seeds = self.data[key]['seed_results']
                dists = [r['global_min_distance'] for r in seeds]
                data_boxes.append(dists)
                labels.append(name)

        fig, ax = plt.subplots(figsize=(8, 6))
        ax.set_title('Collision Avoidance Comparison under Crossing Trajectories (25 Seeds)',
                     fontsize=13, fontweight='bold')

        box_style = {'facecolor': '#dbeafe', 'color': '#1d4ed8'}
        median_style = {'color': '#b91c1c', 'linewidth': 2.0}
        bp = ax.boxplot(
            data_boxes,
            labels=labels,
            patch_artist=True,
            boxprops=box_style,
            medianprops=median_style,
        )

        # Color the CBF-QP box differently to highlight it
        if len(bp['boxes']) >= 3:
            bp['boxes'][2].set_facecolor('#dcfce7')
            bp['boxes'][2].set_edgecolor('#15803d')

        ax.axhline(0.50, color='r', linestyle='-', linewidth=2.0,
                   label='Collision Limit (0.50m - Danger Boundary)')
        ax.axhline(0.70, color='orange', linestyle='--', linewidth=1.5,
                   label='Safety Barrier Margin (0.70m)')

        ax.set_ylabel('Global Minimum Inter-Robot Distance (m)', fontsize=11)
        ax.grid(True, linestyle=':', alpha=0.6, axis='y')
        ax.legend(loc='upper left', fontsize=10)

        out_path = os.path.join(self.output_dir, filename)
        fig.tight_layout()
        fig.savefig(out_path, dpi=300)
        plt.close(fig)
        return out_path

    def plot_packet_drop_sweep(self, filename: str = 'difficulty_sweep_packet_drop.png') -> str:
        """Generate line plot with 95% CI bands across packet loss rates (0% to 50%)."""
        sweep_keys = [
            ('full_resilient_cbf', 0.0),
            ('sweep_drop_15pct', 0.15),
            ('sweep_drop_30pct', 0.30),
            ('sweep_drop_50pct', 0.50),
        ]

        drops = []
        shape_means = []
        shape_lowers = []
        shape_uppers = []
        safe_rates = []

        for key, drop in sweep_keys:
            if key in self.data:
                s = self.data[key]['summary']
                drops.append(drop * 100.0)
                shape_means.append(s['shape_error_mean'])
                shape_lowers.append(s['shape_error_ci_lower'])
                shape_uppers.append(s['shape_error_ci_upper'])
                safe_rates.append(s['collision_free_rate_pct'])

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        fig.suptitle('Environmental Robustness: Network Packet Loss Sweep (25 Seeds)',
                     fontsize=14, fontweight='bold')

        # Panel 1: Shape Error vs Packet Drop
        ax1.plot(drops, shape_means, 'b-o', linewidth=2, label='Mean Shape Error')
        ax1.fill_between(drops, shape_lowers, shape_uppers, color='b', alpha=0.2,
                         label='95% Confidence Band')
        ax1.axhline(0.25, color='r', linestyle='--', label='Convergence Threshold (0.25m)')
        ax1.set_xlabel('Packet Drop Rate (%)', fontsize=11)
        ax1.set_ylabel('Final Shape Error (RMS, m)', fontsize=11)
        ax1.set_title('Formation Accuracy Degradation', fontsize=12, fontweight='bold')
        ax1.grid(True, linestyle=':', alpha=0.6)
        ax1.legend()

        # Panel 2: Safety Rate vs Packet Drop
        ax2.plot(drops, safe_rates, 'g-s', linewidth=2.5, label='Collision-Free Rate')
        ax2.set_xlabel('Packet Drop Rate (%)', fontsize=11)
        ax2.set_ylabel('Empirical Collision-Free Rate (%)', fontsize=11)
        ax2.set_ylim(0, 110)
        ax2.set_title('Safety Resilience', fontsize=12, fontweight='bold')
        ax2.grid(True, linestyle=':', alpha=0.6)
        ax2.legend()

        out_path = os.path.join(self.output_dir, filename)
        fig.tight_layout()
        fig.savefig(out_path, dpi=300)
        plt.close(fig)
        return out_path

    def plot_latency_sweep(self, filename: str = 'difficulty_sweep_latency.png') -> str:
        """Generate line plot with 95% CI bands across communication latency (0 to 200ms)."""
        sweep_keys = [
            ('full_resilient_cbf', 0.0),
            ('sweep_delay_50ms', 50.0),
            ('sweep_delay_100ms', 100.0),
            ('sweep_delay_200ms', 200.0),
        ]

        delays = []
        shape_means = []
        shape_lowers = []
        shape_uppers = []
        dist_means = []

        for key, delay in sweep_keys:
            if key in self.data:
                s = self.data[key]['summary']
                delays.append(delay)
                shape_means.append(s['shape_error_mean'])
                shape_lowers.append(s['shape_error_ci_lower'])
                shape_uppers.append(s['shape_error_ci_upper'])
                dist_means.append(s['min_distance_mean'])

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        fig.suptitle('Environmental Robustness: Communication Latency Sweep (25 Seeds)',
                     fontsize=14, fontweight='bold')

        # Panel 1: Shape Error vs Delay
        ax1.plot(delays, shape_means, 'm-o', linewidth=2, label='Mean Shape Error')
        ax1.fill_between(delays, shape_lowers, shape_uppers, color='m', alpha=0.2,
                         label='95% Confidence Band')
        ax1.axhline(0.25, color='r', linestyle='--', label='Convergence Threshold (0.25m)')
        ax1.set_xlabel('Communication Delay (ms)', fontsize=11)
        ax1.set_ylabel('Final Shape Error (RMS, m)', fontsize=11)
        ax1.set_title('Formation Precision vs Latency', fontsize=12, fontweight='bold')
        ax1.grid(True, linestyle=':', alpha=0.6)
        ax1.legend()

        # Panel 2: Mean Min Distance vs Delay
        ax2.plot(delays, dist_means, 'c-^', linewidth=2, label='Mean Min Distance')
        ax2.axhline(0.50, color='r', linestyle='-', linewidth=2.0,
                    label='Collision Limit (0.50m)')
        ax2.axhline(0.70, color='orange', linestyle='--',
                    label='Nominal Barrier Zone (0.70m)')
        ax2.set_xlabel('Communication Delay (ms)', fontsize=11)
        ax2.set_ylabel('Mean Min Distance (m)', fontsize=11)
        ax2.set_title('Barrier Margin Maintenance', fontsize=12, fontweight='bold')
        ax2.grid(True, linestyle=':', alpha=0.6)
        ax2.legend()

        out_path = os.path.join(self.output_dir, filename)
        fig.tight_layout()
        fig.savefig(out_path, dpi=300)
        plt.close(fig)
        return out_path

    def generate_all_campaign_plots(self) -> List[str]:
        """Generate the complete set of campaign visualization artifacts."""
        return [
            self.plot_ablation_study(),
            self.plot_safety_comparison(),
            self.plot_packet_drop_sweep(),
            self.plot_latency_sweep(),
        ]


def main() -> None:
    """Entry point for standalone campaign visualizer CLI."""
    parser = argparse.ArgumentParser(
        description='Generate comparative plots from campaign_results.json.'
    )
    parser.add_argument(
        '--input-json',
        type=str,
        default='results/M8/campaign_results.json',
        help='Path to input campaign results JSON file.',
    )
    parser.add_argument(
        '--output-dir',
        type=str,
        default='results/M8/plots',
        help='Directory to store output plots.',
    )

    args = parser.parse_args()

    if not os.path.exists(args.input_json):
        print(f'Error: Results file not found: {args.input_json}')
        return

    visualizer = CampaignVisualizer.from_json(args.input_json, output_dir=args.output_dir)
    saved = visualizer.generate_all_campaign_plots()
    print('Generated campaign plots:')
    for p in saved:
        print(f'  - {p}')


if __name__ == '__main__':
    main()
