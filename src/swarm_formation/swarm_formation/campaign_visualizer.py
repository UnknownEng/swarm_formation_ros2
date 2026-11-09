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

"""Publication-grade statistical visualization engine for multi-robot experiment campaigns."""

import argparse
import json
import os
from typing import Any, Dict, List, Tuple

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

matplotlib.use('Agg')


class CampaignVisualizer:
    """Generates comparative statistical figures, ablation charts, and parameter sweeps."""

    def __init__(
        self,
        campaign_data: Dict[str, Any],
        output_dir: str = 'results/campaign/plots',
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
        output_dir: str = 'results/campaign/plots',
    ) -> 'CampaignVisualizer':
        """Load campaign data directly from campaign_results.json."""
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        return cls(campaign_data=data, output_dir=output_dir)

    def _num_seeds(self) -> int:
        for cdata in self.data.values():
            return int(cdata['summary'].get('num_seeds', len(cdata.get('seed_results', []))))
        return 0

    def plot_ablation_study(self, filename: str = 'ablation_study_barchart.png') -> str:
        """Generate comparative bar chart for ablation conditions with 95% CI error bars."""
        conditions = [
            ('deployed_impaired', 'Deployed\nstack'),
            ('impaired_no_navigation', 'without\nnavigation'),
            ('impaired_no_anchor', 'without\nanchor'),
            ('impaired_no_delay_comp', 'without delay\ncompensation'),
            ('impaired_legacy_mapper', 'legacy\ntwist mapper'),
            ('impaired_with_resilience', '+ Huber/EMA/\nheartbeat'),
        ]
        title = ('Leave-one-out ablation of the deployed stack under nominal impairment '
                 f'(15% drop, 80+/-40 ms delay, noise; {self._num_seeds()} seeds)')
        return self._plot_condition_bars(conditions, title, filename)

    def plot_asymmetric_links(self, filename: str = 'asymmetric_network_barchart.png') -> str:
        """Generate the bar chart for the asymmetric (per-direction) link conditions."""
        conditions = [
            ('deployed_clean', 'symmetric\nclean'),
            ('asym_robot1_mute', 'robot 1\nmute'),
            ('asym_robot1_deaf', 'robot 1\ndeaf'),
            ('asym_robot1_tx_degraded', 'robot 1 TX\n50% / 200 ms'),
            ('asym_one_way_ring', 'one-way\nring'),
            ('asym_random', 'random per-\ndirection links'),
        ]
        title = ('Asymmetric network impairments on the deployed stack '
                 f'({self._num_seeds()} seeds)')
        return self._plot_condition_bars(conditions, title, filename)

    def _plot_condition_bars(
        self, conditions: List[Tuple[str, str]], title: str, filename: str
    ) -> str:
        """Two panels: final shape error (mean, 95% CI) and convergence/safety rates."""
        labels = []
        shape_means = []
        shape_ci_errs = []
        safe_rates = []
        conv_rates = []

        for key, name in conditions:
            if key in self.data:
                s = self.data[key]['summary']
                labels.append(name)
                shape_means.append(s['shape_error_mean'])
                ci_margin = max(0.0, s['shape_error_ci_upper'] - s['shape_error_mean'])
                shape_ci_errs.append(ci_margin)
                safe_rates.append(min(s['collision_free_rate_pct'],
                                      s.get('obstacle_collision_free_rate_pct', 100.0)))
                conv_rates.append(s.get('convergence_rate_pct', float('nan')))

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5))
        fig.suptitle(title, fontsize=14, fontweight='bold')

        x_pos = np.arange(len(labels))
        colors = ['#22c55e', '#94a3b8', '#60a5fa', '#38bdf8', '#818cf8', '#f59e0b'][:len(labels)]

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

        # Panel 2: Convergence and collision-free rates side by side
        width = 0.38
        bars_conv = ax2.bar(x_pos - width / 2, conv_rates, width=width, color='#60a5fa',
                            edgecolor='black', alpha=0.85, label='Reached formation')
        bars_safe = ax2.bar(x_pos + width / 2, safe_rates, width=width, color='#22c55e',
                            edgecolor='black', alpha=0.85,
                            label='Collision-free (robots and obstacles)')
        ax2.set_xticks(x_pos)
        ax2.set_xticklabels(labels, fontsize=10)
        ax2.set_ylabel('Runs (%)', fontsize=11)
        ax2.set_ylim(0, 115)
        ax2.set_title('Convergence and Safety', fontsize=12, fontweight='bold')
        ax2.grid(True, linestyle=':', alpha=0.6, axis='y')
        ax2.legend(loc='lower right')

        for bar, val in list(zip(bars_conv, conv_rates)) + list(zip(bars_safe, safe_rates)):
            ax2.text(bar.get_x() + bar.get_width() / 2.0, val + 1.5,
                     f'{val:.0f}%', ha='center', va='bottom', fontsize=8)

        out_path = os.path.join(self.output_dir, filename)
        fig.tight_layout()
        fig.savefig(out_path, dpi=300)
        plt.close(fig)
        return out_path

    def plot_safety_comparison(self, filename: str = 'safety_comparison_apf_vs_cbf.png') -> str:
        """Generate box plot distribution comparing Unshielded, APF-only, and CBF-QP safety."""
        conditions = [
            ('arena_unshielded', 'No avoidance'),
            ('arena_apf', 'APF instead of CBF'),
            ('deployed_clean', 'Deployed (CBF-QP)'),
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
        ax.set_title('Collision Avoidance in the Arena, Crossing Starts '
                     f'({self._num_seeds()} seeds, clean channel)',
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
        ax.axhline(0.55, color='orange', linestyle='--', linewidth=1.5,
                   label='CBF safety distance D_s (0.55m)')

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
            ('deployed_clean', 0.0),
            ('sweep_drop_15pct', 0.15),
            ('sweep_drop_30pct', 0.30),
            ('sweep_drop_50pct', 0.50),
        ]

        drops = []
        shape_means = []
        shape_lowers = []
        shape_uppers = []
        safe_rates = []
        conv_rates = []

        for key, drop in sweep_keys:
            if key in self.data:
                s = self.data[key]['summary']
                drops.append(drop * 100.0)
                shape_means.append(s['shape_error_mean'])
                shape_lowers.append(s['shape_error_ci_lower'])
                shape_uppers.append(s['shape_error_ci_upper'])
                safe_rates.append(s['collision_free_rate_pct'])
                conv_rates.append(s.get('convergence_rate_pct', float('nan')))

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        fig.suptitle(f'Packet Loss Sweep, deployed stack ({self._num_seeds()} seeds)',
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
        ax2.plot(drops, safe_rates, 'g-s', linewidth=2.5, label='Collision-free')
        ax2.plot(drops, conv_rates, 'b-o', linewidth=2.0, label='Reached formation')
        ax2.set_xlabel('Packet Drop Rate (%)', fontsize=11)
        ax2.set_ylabel('Runs (%)', fontsize=11)
        ax2.set_ylim(0, 110)
        ax2.set_title('Convergence and Safety', fontsize=12, fontweight='bold')
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
            ('deployed_clean', 0.0),
            ('sweep_delay_50ms', 50.0),
            ('sweep_delay_100ms', 100.0),
            ('sweep_delay_200ms', 200.0),
        ]

        delays = []
        shape_means = []
        shape_lowers = []
        shape_uppers = []
        dist_means = []
        conv_rates = []

        for key, delay in sweep_keys:
            if key in self.data:
                s = self.data[key]['summary']
                delays.append(delay)
                shape_means.append(s['shape_error_mean'])
                shape_lowers.append(s['shape_error_ci_lower'])
                shape_uppers.append(s['shape_error_ci_upper'])
                dist_means.append(s['min_distance_mean'])
                conv_rates.append(s.get('convergence_rate_pct', float('nan')))

        matched = self.data.get('sweep_delay_200ms_matched', {}).get('summary')

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        fig.suptitle('Applied Latency Sweep, deployed stack (CBF delay bound 50 ms) '
                     f'({self._num_seeds()} seeds)',
                     fontsize=14, fontweight='bold')

        # Panel 1: Shape Error vs Delay
        ax1.plot(delays, shape_means, 'm-o', linewidth=2, label='Mean Shape Error')
        ax1.fill_between(delays, shape_lowers, shape_uppers, color='m', alpha=0.2,
                         label='95% Confidence Band')
        ax1.axhline(0.25, color='r', linestyle='--', label='Convergence Threshold (0.25m)')
        for d, c, m in zip(delays, conv_rates, shape_means):
            ax1.annotate(f'{c:.0f}% conv.', (d, m), textcoords='offset points',
                         xytext=(0, 8), ha='center', fontsize=8)
        ax1.set_xlabel('Applied Delay (ms)', fontsize=11)
        ax1.set_ylabel('Final Shape Error (RMS, m)', fontsize=11)
        ax1.set_title('Formation Precision vs Latency', fontsize=12, fontweight='bold')
        ax1.grid(True, linestyle=':', alpha=0.6)
        ax1.legend()

        # Panel 2: Mean Min Distance vs Delay
        ax2.plot(delays, dist_means, 'c-^', linewidth=2,
                 label='Mean min distance (50 ms bound)')
        if matched is not None:
            ax2.plot([200.0], [matched['min_distance_mean']], 'gx', markersize=10, mew=2,
                     label='200 ms with matched 200 ms bound')
        ax2.axhline(0.50, color='r', linestyle='-', linewidth=2.0,
                    label='Collision Limit (0.50m)')
        ax2.set_xlabel('Applied Delay (ms)', fontsize=11)
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
            self.plot_asymmetric_links(),
        ]


def main() -> None:
    """Entry point for standalone campaign visualizer CLI."""
    parser = argparse.ArgumentParser(
        description='Generate comparative plots from campaign_results.json.'
    )
    parser.add_argument(
        '--input-json',
        type=str,
        default='results/campaign/campaign_results.json',
        help='Path to input campaign results JSON file.',
    )
    parser.add_argument(
        '--output-dir',
        type=str,
        default='results/campaign/plots',
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
