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

"""Multi-seed campaign execution and statistical analysis engine."""

import argparse
import csv
import json
import math
import os
from typing import Any, Dict, List, Tuple

import numpy as np
try:
    from scipy import stats
except ImportError:
    stats = None

from swarm_formation.campaign_simulator import CampaignRunConfig, run_single_simulation


def compute_statistics(values: List[float], alpha: float = 0.05) -> Dict[str, float]:
    """
    Compute sample mean, standard deviation, and two-sided 1-alpha confidence interval.

    :param values: Sample data vector.
    :param alpha: Significance level (default: 0.05 for 95% confidence).
    :return: Dictionary containing mean, std, se, ci_lower, ci_upper.
    """
    n = len(values)
    if n == 0:
        return {'mean': 0.0, 'std': 0.0, 'se': 0.0, 'ci_lower': 0.0, 'ci_upper': 0.0}

    arr = np.array(values, dtype=float)
    mean_val = float(np.mean(arr))

    if n == 1:
        return {
            'mean': mean_val,
            'std': 0.0,
            'se': 0.0,
            'ci_lower': mean_val,
            'ci_upper': mean_val,
        }

    std_val = float(np.std(arr, ddof=1))
    se_val = std_val / math.sqrt(n)

    if stats is not None:
        t_crit = float(stats.t.ppf(1.0 - alpha / 2.0, df=n - 1))
    else:
        # Normal approximation fallback
        t_crit = 1.96

    ci_margin = t_crit * se_val
    return {
        'mean': mean_val,
        'std': std_val,
        'se': se_val,
        'ci_lower': mean_val - ci_margin,
        'ci_upper': mean_val + ci_margin,
    }


def define_campaign_conditions(duration_sec: float = 30.0) -> Dict[str, CampaignRunConfig]:
    """Define the complete matrix of experimental conditions, ablations, and sweeps."""
    conditions: Dict[str, CampaignRunConfig] = {}

    # 1. Baseline: Unshielded linear consensus
    conditions['baseline_unshielded'] = CampaignRunConfig(
        duration_sec=duration_sec,
        use_resilient=False,
        use_huber=False,
        use_ema=False,
        use_heartbeat=False,
        use_cbf=False,
        use_apf=False,
    )

    # 2. Collision avoidance comparison
    conditions['apf_only'] = CampaignRunConfig(
        duration_sec=duration_sec,
        use_resilient=False,
        use_huber=False,
        use_ema=False,
        use_heartbeat=False,
        use_cbf=False,
        use_apf=True,
    )

    conditions['cbf_qp_only'] = CampaignRunConfig(
        duration_sec=duration_sec,
        use_resilient=False,
        use_huber=False,
        use_ema=False,
        use_heartbeat=False,
        use_cbf=True,
        use_apf=False,
    )

    # 3. Component ablations
    conditions['ablation_huber_only'] = CampaignRunConfig(
        duration_sec=duration_sec,
        use_resilient=True,
        use_huber=True,
        use_ema=False,
        use_heartbeat=False,
        use_cbf=False,
        use_apf=True,
    )

    conditions['ablation_ema_only'] = CampaignRunConfig(
        duration_sec=duration_sec,
        use_resilient=False,
        use_huber=False,
        use_ema=True,
        use_heartbeat=False,
        use_cbf=False,
        use_apf=True,
    )

    conditions['ablation_heartbeat_only'] = CampaignRunConfig(
        duration_sec=duration_sec,
        use_resilient=False,
        use_huber=False,
        use_ema=False,
        use_heartbeat=True,
        use_cbf=False,
        use_apf=True,
    )

    # 4. Full Resilient Stack with CBF-QP Safety Filter
    conditions['full_resilient_cbf'] = CampaignRunConfig(
        duration_sec=duration_sec,
        use_resilient=True,
        use_huber=True,
        use_ema=True,
        use_heartbeat=True,
        use_cbf=True,
        use_apf=False,
    )

    # 5. Network Drop Sweep (on Full Stack)
    for drop in [0.15, 0.30, 0.50]:
        conditions[f'sweep_drop_{int(drop*100)}pct'] = CampaignRunConfig(
            duration_sec=duration_sec,
            use_resilient=True,
            use_huber=True,
            use_ema=True,
            use_heartbeat=True,
            use_cbf=True,
            use_apf=False,
            drop_rate=drop,
        )

    # 6. Latency Sweep (on Full Stack)
    for delay_ms in [50, 100, 200]:
        conditions[f'sweep_delay_{delay_ms}ms'] = CampaignRunConfig(
            duration_sec=duration_sec,
            use_resilient=True,
            use_huber=True,
            use_ema=True,
            use_heartbeat=True,
            use_cbf=True,
            use_apf=False,
            delay_sec=float(delay_ms) / 1000.0,
        )

    return conditions


def run_campaign(
    num_seeds: int = 25,
    duration_sec: float = 30.0,
    output_dir: str = 'results/M8',
) -> Tuple[Dict[str, Any], str, str]:
    """Execute complete campaign across all seeds and return aggregated summaries."""
    if not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)

    conditions = define_campaign_conditions(duration_sec=duration_sec)
    raw_campaign_data: Dict[str, Any] = {}
    summary_table_rows: List[Dict[str, Any]] = []

    print(
        f'Starting Campaign: {len(conditions)} Conditions | '
        f'{num_seeds} Seeds/Condition | Total Runs: {len(conditions) * num_seeds}'
    )

    for cond_name, cfg in conditions.items():
        print(f'  --> Running condition: {cond_name} ({num_seeds} seeds)...', end='', flush=True)
        seed_results = []
        for s in range(1, num_seeds + 1):
            seed = s * 1000 + 42
            res = run_single_simulation(cfg, seed)
            seed_results.append(res)

        # Aggregate metrics
        shape_errors = [r['final_shape_error'] for r in seed_results]
        min_dists = [r['global_min_distance'] for r in seed_results]
        collisions = [r['total_collisions'] for r in seed_results]
        safe_flags = [1.0 if r['empirically_collision_free'] else 0.0 for r in seed_results]
        conv_flags = [1.0 if r['is_converged'] else 0.0 for r in seed_results]
        conv_times = [
            r['convergence_time']
            for r in seed_results
            if r['convergence_time'] is not None
        ]
        est_errors = [r['mean_est_error'] for r in seed_results]
        energies = [r['total_control_energy'] for r in seed_results]

        stats_shape = compute_statistics(shape_errors)
        stats_min_dist = compute_statistics(min_dists)
        stats_colls = compute_statistics(collisions)
        stats_est = compute_statistics(est_errors)
        stats_energy = compute_statistics(energies)
        stats_conv_time = compute_statistics(conv_times) if conv_times else {
            'mean': float('nan'), 'std': 0.0, 'ci_lower': 0.0, 'ci_upper': 0.0
        }

        safe_rate = float(np.mean(safe_flags)) * 100.0
        conv_rate = float(np.mean(conv_flags)) * 100.0

        cond_summary = {
            'condition': cond_name,
            'num_seeds': num_seeds,
            'shape_error_mean': stats_shape['mean'],
            'shape_error_std': stats_shape['std'],
            'shape_error_ci_lower': stats_shape['ci_lower'],
            'shape_error_ci_upper': stats_shape['ci_upper'],
            'min_distance_mean': stats_min_dist['mean'],
            'min_distance_std': stats_min_dist['std'],
            'min_distance_ci_lower': stats_min_dist['ci_lower'],
            'min_distance_ci_upper': stats_min_dist['ci_upper'],
            'collisions_mean': stats_colls['mean'],
            'collision_free_rate_pct': safe_rate,
            'convergence_rate_pct': conv_rate,
            'convergence_time_mean': stats_conv_time['mean'],
            'mean_est_error': stats_est['mean'],
            'mean_control_energy': stats_energy['mean'],
        }
        summary_table_rows.append(cond_summary)

        raw_campaign_data[cond_name] = {
            'config': {
                'use_resilient': cfg.use_resilient,
                'use_huber': cfg.use_huber,
                'use_ema': cfg.use_ema,
                'use_heartbeat': cfg.use_heartbeat,
                'use_cbf': cfg.use_cbf,
                'use_apf': cfg.use_apf,
                'drop_rate': cfg.drop_rate,
                'delay_sec': cfg.delay_sec,
            },
            'summary': cond_summary,
            'seed_results': seed_results,
        }

        print(
            f' Done. Shape Err: {stats_shape["mean"]:.3f}m | '
            f'Min Dist: {stats_min_dist["mean"]:.3f}m | '
            f'Safety: {safe_rate:.1f}%'
        )

    # Export CSV summary
    csv_path = os.path.join(output_dir, 'campaign_summary.csv')
    fieldnames = list(summary_table_rows[0].keys())
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summary_table_rows)

    # Export JSON results
    json_path = os.path.join(output_dir, 'campaign_results.json')
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(raw_campaign_data, f, indent=2)

    return raw_campaign_data, csv_path, json_path


def main() -> None:
    """Entry point for standalone campaign runner CLI."""
    parser = argparse.ArgumentParser(
        description='Run swarm experiment campaign and statistical sweeps.'
    )
    parser.add_argument(
        '--seeds', type=int, default=25, help='Number of seeds per condition (default: 25).'
    )
    parser.add_argument(
        '--duration', type=float, default=30.0, help='Duration per run (default: 30s).'
    )
    parser.add_argument(
        '--output-dir', type=str, default='results/M8', help='Directory for output data.'
    )

    args = parser.parse_args()

    _, csv_path, json_path = run_campaign(
        num_seeds=args.seeds,
        duration_sec=args.duration,
        output_dir=args.output_dir,
    )

    print('\nCampaign completed successfully.')
    print(f'Summary CSV  : {csv_path}')
    print(f'Full Data JSON: {json_path}\n')


if __name__ == '__main__':
    main()
