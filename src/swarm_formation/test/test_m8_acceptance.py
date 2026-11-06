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

"""Acceptance tests for Milestone M8: Experiment Campaign, Statistical Rigor & Ablations."""

import json
import os
import shutil
import tempfile

from swarm_formation.campaign_runner import run_campaign
from swarm_formation.campaign_visualizer import CampaignVisualizer


def test_m8_acceptance_statistical_campaign_execution():
    """Verify statistical campaign across 20+ seeds with confidence intervals and ablations."""
    tmp_dir = tempfile.mkdtemp(prefix='test_m8_acceptance_')
    plots_dir = os.path.join(tmp_dir, 'plots')

    try:
        # Run 20 seeds to meet acceptance criteria (20-30 seeds per condition)
        num_seeds = 20
        raw_data, csv_path, json_path = run_campaign(
            num_seeds=num_seeds,
            duration_sec=12.0,
            output_dir=tmp_dir,
        )

        assert os.path.exists(csv_path), 'campaign_summary.csv must be created'
        assert os.path.exists(json_path), 'campaign_results.json must be created'

        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        # 1. Verify 20 seeds executed for every condition
        for cond_name, cdata in data.items():
            assert len(cdata['seed_results']) == num_seeds, (
                f'Condition {cond_name} did not execute required {num_seeds} seeds'
            )
            # Verify 95% Confidence Intervals are present
            summary = cdata['summary']
            assert 'shape_error_ci_lower' in summary
            assert 'shape_error_ci_upper' in summary
            assert summary['shape_error_ci_lower'] <= summary['shape_error_mean']
            assert summary['shape_error_ci_upper'] >= summary['shape_error_mean']

        # 2. Verify Ablation Conditions Present
        required_ablations = [
            'baseline_unshielded',
            'apf_only',
            'cbf_qp_only',
            'ablation_huber_only',
            'ablation_ema_only',
            'ablation_heartbeat_only',
            'full_resilient_cbf',
        ]
        for cond in required_ablations:
            assert cond in data, f'Required ablation condition {cond} missing'

        # 3. Verify Safety Layer Comparison: CBF-QP must achieve 100% collision-free rate
        cbf_summary = data['cbf_qp_only']['summary']
        baseline_summary = data['baseline_unshielded']['summary']

        assert cbf_summary['collision_free_rate_pct'] == 100.0, (
            'CBF-QP filter must maintain 100% empirical collision-free rate'
        )
        assert baseline_summary['collision_free_rate_pct'] < 100.0, (
            'Baseline unshielded crossing must demonstrate collision vulnerability'
        )
        assert cbf_summary['min_distance_mean'] >= 0.50, (
            'CBF-QP mean minimum distance must strictly respect 0.50m safety limit'
        )

        # 4. Verify Difficulty Sweeps Present
        assert 'sweep_drop_50pct' in data
        assert 'sweep_delay_200ms' in data

        # 5. Verify Plotting Suite
        visualizer = CampaignVisualizer.from_json(json_path, output_dir=plots_dir)
        plots = visualizer.generate_all_campaign_plots()

        assert len(plots) == 4, 'Must generate exactly 4 campaign comparison plots'
        expected_plots = [
            'ablation_study_barchart.png',
            'safety_comparison_apf_vs_cbf.png',
            'difficulty_sweep_packet_drop.png',
            'difficulty_sweep_latency.png',
        ]
        for name in expected_plots:
            plot_file = os.path.join(plots_dir, name)
            assert os.path.exists(plot_file), f'Plot {name} missing'
            assert os.path.getsize(plot_file) > 10000, f'Plot {name} is corrupted or empty'

    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)
