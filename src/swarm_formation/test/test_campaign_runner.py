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

"""Unit tests for campaign execution engine and statistical calculations."""

import csv
import json
import os
import shutil
import tempfile

import numpy as np
import pytest

from swarm_formation.campaign_runner import (
    compute_statistics,
    define_campaign_conditions,
    run_campaign,
)
from swarm_formation.campaign_simulator import CampaignRunConfig, run_single_simulation
from swarm_formation.campaign_visualizer import CampaignVisualizer


def test_compute_statistics_known_values():
    """Verify statistical calculations for sample mean, standard deviation, and 95% CI."""
    data = [10.0, 12.0, 11.0, 13.0, 9.0]
    # mean = 11.0, variance = (1 + 1 + 0 + 4 + 4)/4 = 10/4 = 2.5, std = sqrt(2.5) ~ 1.5811
    stats_dict = compute_statistics(data)
    assert stats_dict['mean'] == pytest.approx(11.0, abs=1e-5)
    assert stats_dict['std'] == pytest.approx(np.sqrt(2.5), abs=1e-4)
    assert stats_dict['ci_lower'] < stats_dict['mean']
    assert stats_dict['ci_upper'] > stats_dict['mean']
    # Symmetry of confidence interval around sample mean
    margin = stats_dict['mean'] - stats_dict['ci_lower']
    assert stats_dict['ci_upper'] - stats_dict['mean'] == pytest.approx(margin, abs=1e-5)


def test_define_campaign_conditions_matrix():
    """Verify that all required conditions, ablations, and sweeps are defined."""
    conditions = define_campaign_conditions(duration_sec=20.0)
    required = [
        'baseline_unshielded',
        'apf_only',
        'cbf_qp_only',
        'ablation_huber_only',
        'ablation_ema_only',
        'ablation_heartbeat_only',
        'full_resilient_cbf',
        'sweep_drop_15pct',
        'sweep_drop_30pct',
        'sweep_drop_50pct',
        'sweep_delay_50ms',
        'sweep_delay_100ms',
        'sweep_delay_200ms',
    ]
    for req in required:
        assert req in conditions, f'Condition {req} missing from campaign matrix'


def test_single_simulation_baseline_vs_cbf_safety():
    """Verify that unshielded crossing results in collisions while CBF-QP prevents collisions."""
    seed = 42

    # 1. Unshielded crossing
    cfg_unshielded = CampaignRunConfig(
        duration_sec=15.0,
        use_resilient=False,
        use_cbf=False,
        use_apf=False,
        crossing_scenario=True,
    )
    res_unshielded = run_single_simulation(cfg_unshielded, seed)
    assert res_unshielded['global_min_distance'] < 0.50, 'Unshielded crossing must violate safety'
    assert res_unshielded['total_collisions'] > 0
    assert res_unshielded['empirically_collision_free'] is False

    # 2. CBF-QP shielded crossing on same seed
    cfg_cbf = CampaignRunConfig(
        duration_sec=15.0,
        use_resilient=False,
        use_cbf=True,
        use_apf=False,
        crossing_scenario=True,
    )
    res_cbf = run_single_simulation(cfg_cbf, seed)
    assert res_cbf['global_min_distance'] >= 0.50, 'CBF-QP must maintain safety'
    assert res_cbf['total_collisions'] == 0
    assert res_cbf['empirically_collision_free'] is True
    assert res_cbf['total_safety_interventions'] > 0.0


def test_mini_campaign_and_visualization_pipeline():
    """Verify end-to-end execution of batch campaign, statistical export, and plotting."""
    tmp_dir = tempfile.mkdtemp(prefix='test_campaign_pipe_')
    plots_dir = os.path.join(tmp_dir, 'plots')

    try:
        raw_data, csv_path, json_path = run_campaign(
            num_seeds=3,
            duration_sec=8.0,
            output_dir=tmp_dir,
        )

        assert os.path.exists(csv_path)
        assert os.path.exists(json_path)

        # Verify CSV summary content
        with open(csv_path, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            rows = list(reader)
            assert len(rows) >= 10
            for r in rows:
                assert 'condition' in r
                assert float(r['shape_error_mean']) > 0.0
                assert float(r['collision_free_rate_pct']) >= 0.0

        # Verify JSON results structure
        with open(json_path, 'r', encoding='utf-8') as f:
            loaded = json.load(f)
            assert 'baseline_unshielded' in loaded
            assert 'full_resilient_cbf' in loaded
            assert len(loaded['baseline_unshielded']['seed_results']) == 3

        # Test visualization pipeline
        visualizer = CampaignVisualizer.from_json(json_path, output_dir=plots_dir)
        plots = visualizer.generate_all_campaign_plots()

        assert len(plots) == 4
        for p in plots:
            assert os.path.exists(p)
            assert os.path.getsize(p) > 10000

    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)
