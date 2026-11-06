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
from dataclasses import asdict
import json
import math
import os
from typing import Any, Dict, List, Optional, Tuple

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


# Channel impairment used by the launch file when enable_faults:=true
NOMINAL_IMPAIRMENT = {
    'drop_rate': 0.15,
    'delay_sec': 0.08,
    'jitter_sec': 0.04,
    'range_noise_std': 0.05,
    'bearing_noise_std': 0.02,
}

FULL_STACK = {
    'use_huber': True,
    'use_ema': True,
    'use_heartbeat': True,
    'use_cbf': True,
}


def define_campaign_conditions(duration_sec: float = 30.0) -> Dict[str, CampaignRunConfig]:
    """
    Define the experimental matrix.

    * Safety comparison (clean channel): unshielded, APF, CBF-QP, full stack.
    * Leave-one-out ablation under the nominal impairment of the launch file
      (15% drop, 80 ms delay, +/-40 ms jitter, sensor noise): full stack, the full
      stack minus each resilience component, and CBF without any resilience component.
    * Sweeps on the full stack: packet drop, and *applied* latency with a CBF margin
      matched to the latency, plus one 200 ms run with the ROS default 50 ms margin.
    """
    def cfg(**kw: Any) -> CampaignRunConfig:
        return CampaignRunConfig(duration_sec=duration_sec, **kw)

    conditions: Dict[str, CampaignRunConfig] = {}

    # 1. Safety comparison on a clean channel
    conditions['baseline_unshielded'] = cfg()
    conditions['apf_only'] = cfg(use_apf=True)
    conditions['cbf_qp_only'] = cfg(use_cbf=True)
    conditions['full_resilient_cbf'] = cfg(**FULL_STACK)

    # 2. Leave-one-out ablation under nominal impairment
    conditions['impaired_full_resilient_cbf'] = cfg(**FULL_STACK, **NOMINAL_IMPAIRMENT)
    for comp in ('huber', 'ema', 'heartbeat'):
        flags = dict(FULL_STACK)
        flags[f'use_{comp}'] = False
        conditions[f'impaired_no_{comp}'] = cfg(**flags, **NOMINAL_IMPAIRMENT)
    conditions['impaired_cbf_only'] = cfg(use_cbf=True, **NOMINAL_IMPAIRMENT)

    # 3. Packet drop sweep (full stack, no delay)
    for drop in [0.15, 0.30, 0.50]:
        conditions[f'sweep_drop_{int(round(drop * 100))}pct'] = cfg(**FULL_STACK, drop_rate=drop)

    # 4. Latency sweep: delay is actually applied to every peer measurement
    for delay_ms in [50, 100, 200]:
        conditions[f'sweep_delay_{delay_ms}ms'] = cfg(
            **FULL_STACK, delay_sec=delay_ms / 1000.0
        )
    conditions['sweep_delay_200ms_fixed_margin'] = cfg(
        **FULL_STACK, delay_sec=0.200, cbf_delay_bound_sec=0.05
    )

    return conditions


def run_campaign(
    num_seeds: int = 25,
    duration_sec: float = 30.0,
    output_dir: str = 'results/M8',
    condition_names: Optional[List[str]] = None,
) -> Tuple[Dict[str, Any], str, str]:
    """
    Execute the campaign across all seeds and return aggregated summaries.

    :param condition_names: Optional subset of condition names to run (default: all).
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)

    conditions = define_campaign_conditions(duration_sec=duration_sec)
    if condition_names is not None:
        unknown = set(condition_names) - set(conditions)
        if unknown:
            raise ValueError(f'Unknown campaign conditions: {sorted(unknown)}')
        conditions = {k: v for k, v in conditions.items() if k in condition_names}
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
        def col(key: str) -> List[float]:
            return [float(r[key]) for r in seed_results]

        stats_shape = compute_statistics(col('final_shape_error'))
        stats_min_dist = compute_statistics(col('global_min_distance'))
        stats_colls = compute_statistics(col('total_collisions'))
        stats_effort = compute_statistics(col('control_effort_lin'))
        stats_path = compute_statistics(col('path_length'))
        conv_times = [
            r['convergence_time'] for r in seed_results if r['convergence_time'] is not None
        ]
        stats_conv_time = compute_statistics(conv_times) if conv_times else {
            'mean': float('nan'), 'std': 0.0, 'ci_lower': 0.0, 'ci_upper': 0.0
        }
        safe_rate = 100.0 * float(np.mean([r['empirically_collision_free'] for r in seed_results]))
        conv_rate = 100.0 * float(np.mean([r['is_converged'] for r in seed_results]))

        cond_summary = {
            'condition': cond_name,
            'num_seeds': num_seeds,
            'convergence_rate_pct': conv_rate,
            'collision_free_rate_pct': safe_rate,
            'shape_error_mean': stats_shape['mean'],
            'shape_error_std': stats_shape['std'],
            'shape_error_ci_lower': stats_shape['ci_lower'],
            'shape_error_ci_upper': stats_shape['ci_upper'],
            'min_distance_mean': stats_min_dist['mean'],
            'min_distance_std': stats_min_dist['std'],
            'min_distance_ci_lower': stats_min_dist['ci_lower'],
            'min_distance_ci_upper': stats_min_dist['ci_upper'],
            'min_distance_worst': float(min(col('global_min_distance'))),
            'collision_steps_mean': stats_colls['mean'],
            'convergence_time_mean': stats_conv_time['mean'],
            'mean_est_error': float(np.mean(col('mean_est_error'))),
            'mean_yaw_error': float(np.mean(col('mean_yaw_error'))),
            'control_effort_lin_mean': stats_effort['mean'],
            'path_length_mean': stats_path['mean'],
            'packet_drop_fraction': float(np.mean(col('packet_drop_fraction'))),
            'applied_latency_ms': 1000.0 * float(np.mean(col('mean_applied_latency_sec'))),
            'cbf_delay_bound_ms': 1000.0 * cfg.effective_cbf_delay_bound() if cfg.use_cbf else 0.0,
        }
        summary_table_rows.append(cond_summary)

        raw_campaign_data[cond_name] = {
            'config': asdict(cfg),
            'summary': cond_summary,
            'seed_results': seed_results,
        }

        print(
            f' Done. Converged: {conv_rate:.0f}% | Shape Err: {stats_shape["mean"]:.3f}m | '
            f'Min Dist: {stats_min_dist["mean"]:.3f}m | Collision-free: {safe_rate:.0f}%'
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
