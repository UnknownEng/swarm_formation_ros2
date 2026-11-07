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

"""Acceptance tests for Milestone M8: Experiment Campaign, Statistical Rigor & Ablations."""

import json
import os
import shutil
import tempfile

from swarm_formation.campaign_runner import run_campaign
from swarm_formation.campaign_visualizer import CampaignVisualizer

SAFETY_CONDITIONS = ['arena_unshielded', 'arena_apf', 'deployed_clean', 'deployed_impaired']


def test_m8_acceptance_statistical_campaign_execution():
    """20-seed arena campaign: CIs present, the deployed stack is safe and converges."""
    tmp_dir = tempfile.mkdtemp(prefix='test_m8_acceptance_')
    plots_dir = os.path.join(tmp_dir, 'plots')

    try:
        num_seeds = 20
        _, csv_path, json_path = run_campaign(
            num_seeds=num_seeds,
            duration_sec=25.0,
            output_dir=tmp_dir,
            condition_names=SAFETY_CONDITIONS,
        )

        assert os.path.exists(csv_path), 'campaign_summary.csv must be created'
        assert os.path.exists(json_path), 'campaign_results.json must be created'

        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        # 1. Seeds and 95% confidence intervals
        for cond_name in SAFETY_CONDITIONS:
            cdata = data[cond_name]
            assert len(cdata['seed_results']) == num_seeds, cond_name
            summary = cdata['summary']
            assert summary['shape_error_ci_lower'] <= summary['shape_error_mean']
            assert summary['shape_error_ci_upper'] >= summary['shape_error_mean']

        baseline = data['arena_unshielded']['summary']
        cbf = data['deployed_clean']['summary']
        full = data['deployed_impaired']['summary']

        # 2. The crossing scenario is genuinely dangerous without avoidance
        assert baseline['collision_free_rate_pct'] < 100.0

        # 3. Safety: every deployed run keeps robots apart and off every obstacle and wall
        for summary in (cbf, full):
            assert summary['collision_free_rate_pct'] == 100.0
            assert summary['min_distance_worst'] >= 0.50
            assert summary['obstacle_collision_free_rate_pct'] == 100.0

        # 4. Safety must not be bought by deadlock: the formation must still form
        for name, summary in (('deployed_clean', cbf), ('deployed_impaired', full)):
            assert summary['convergence_rate_pct'] >= 90.0, (
                f"{name} reached the formation in only {summary['convergence_rate_pct']:.0f}% "
                'of runs'
            )
            assert summary['shape_error_mean'] < 0.25, name

        # 5. Plotting suite tolerates a subset of conditions
        visualizer = CampaignVisualizer.from_json(json_path, output_dir=plots_dir)
        plots = visualizer.generate_all_campaign_plots()
        assert len(plots) == 4
        for plot_file in plots:
            assert os.path.exists(plot_file)

    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)
