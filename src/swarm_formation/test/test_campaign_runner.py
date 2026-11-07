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
    stats_dict = compute_statistics(data)
    assert stats_dict['mean'] == pytest.approx(11.0, abs=1e-5)
    assert stats_dict['std'] == pytest.approx(np.sqrt(2.5), abs=1e-4)
    assert stats_dict['ci_lower'] < stats_dict['mean']
    assert stats_dict['ci_upper'] > stats_dict['mean']
    margin = stats_dict['mean'] - stats_dict['ci_lower']
    assert stats_dict['ci_upper'] - stats_dict['mean'] == pytest.approx(margin, abs=1e-5)


def test_define_campaign_conditions_matrix():
    """The matrix is built around the deployed stack, with one change per condition."""
    from swarm_formation.campaign_runner import DEPLOYED, NOMINAL_IMPAIRMENT

    conditions = define_campaign_conditions(duration_sec=20.0)
    required = [
        'arena_unshielded', 'arena_apf', 'deployed_clean', 'deployed_impaired',
        'impaired_no_navigation', 'impaired_no_anchor', 'impaired_no_delay_comp',
        'impaired_legacy_mapper', 'impaired_with_resilience',
        'sweep_drop_15pct', 'sweep_drop_30pct', 'sweep_drop_50pct',
        'sweep_delay_50ms', 'sweep_delay_100ms', 'sweep_delay_200ms',
        'sweep_delay_200ms_matched', 'robot4_dropout',
        'asym_robot1_mute', 'asym_robot1_deaf', 'asym_robot1_tx_degraded',
        'asym_one_way_ring', 'asym_random',
    ]
    for req in required:
        assert req in conditions, f'Condition {req} missing from campaign matrix'

    # Every condition runs in the arena with the deployed defaults unless it varies them
    deployed = conditions['deployed_clean']
    for key, value in DEPLOYED.items():
        assert getattr(deployed, key) == value, key
    for cfg in conditions.values():
        assert cfg.use_world_obstacles

    # Leave-one-out: each ablation differs from deployed_impaired in exactly one setting
    base = conditions['deployed_impaired']
    for key, value in NOMINAL_IMPAIRMENT.items():
        assert getattr(base, key) == value
    ablations = {
        'impaired_no_navigation': {'use_navigation'},
        'impaired_no_anchor': {'anchor_gain'},
        'impaired_no_delay_comp': {'delay_compensation'},
        'impaired_legacy_mapper': {'twist_mapper'},
        'impaired_with_resilience': {'use_huber', 'use_ema', 'use_heartbeat'},
    }
    for name, changed in ablations.items():
        diff = {k for k, v in vars(conditions[name]).items() if vars(base)[k] != v}
        assert diff == changed, (name, diff)

    # The latency sweep applies delay; the deployed bound stays 50 ms unless matched
    assert conditions['sweep_delay_200ms'].delay_sec == pytest.approx(0.2)
    assert conditions['sweep_delay_200ms'].effective_cbf_delay_bound() == pytest.approx(0.05)
    assert conditions['sweep_delay_200ms_matched'].effective_cbf_delay_bound() == (
        pytest.approx(0.2)
    )


def test_single_simulation_baseline_vs_cbf_safety_and_convergence():
    """Unshielded crossing collides; CBF-QP stays safe AND still reaches the formation."""
    seed = 42

    res_unshielded = run_single_simulation(CampaignRunConfig(duration_sec=20.0), seed)
    assert res_unshielded['global_min_distance'] < 0.50, 'Unshielded crossing must violate safety'
    assert res_unshielded['empirically_collision_free'] is False

    res_cbf = run_single_simulation(CampaignRunConfig(duration_sec=20.0, use_cbf=True), seed)
    assert res_cbf['empirically_collision_free'] is True
    assert res_cbf['total_safety_interventions'] > 0.0
    assert res_cbf['is_converged'] is True, (
        f"CBF-QP must not deadlock: final shape error {res_cbf['final_shape_error']:.3f} m"
    )


def test_delay_is_actually_applied_to_measurements():
    """The latency sweep must delay peer measurements, not only change the CBF margin."""
    cfg = CampaignRunConfig(duration_sec=3.0, use_cbf=True, delay_sec=0.2, jitter_sec=0.0)
    res = run_single_simulation(cfg, 7)
    assert res['mean_applied_latency_sec'] == pytest.approx(0.2, abs=1e-9)
    assert res['cbf_delay_bound_sec'] == pytest.approx(0.2, abs=1e-9)

    cfg_jitter = CampaignRunConfig(duration_sec=3.0, delay_sec=0.08, jitter_sec=0.04)
    res_j = run_single_simulation(cfg_jitter, 7)
    assert 0.06 < res_j['mean_applied_latency_sec'] < 0.10


def test_packet_drop_rate_is_applied():
    """Bernoulli drops on the peer measurement channel match the configured rate."""
    res = run_single_simulation(CampaignRunConfig(duration_sec=5.0, drop_rate=0.3), 11)
    assert res['packet_drop_fraction'] == pytest.approx(0.3, abs=0.04)


def test_estimator_is_not_ground_truth():
    """Localization uses the onboard EKF: small but non-zero error, growing with noise."""
    clean = run_single_simulation(CampaignRunConfig(duration_sec=8.0), 5)
    noisy = run_single_simulation(
        CampaignRunConfig(duration_sec=8.0, range_noise_std=0.1, bearing_noise_std=0.05,
                          odom_speed_noise_frac=0.1, odom_yaw_rate_noise_std=0.1),
        5,
    )
    assert 0.0 < clean['mean_est_error'] < 0.2
    assert noisy['mean_est_error'] > clean['mean_est_error']


def test_mini_campaign_and_visualization_pipeline():
    """Verify end-to-end execution of batch campaign, statistical export, and plotting."""
    tmp_dir = tempfile.mkdtemp(prefix='test_campaign_pipe_')
    plots_dir = os.path.join(tmp_dir, 'plots')

    try:
        _, csv_path, json_path = run_campaign(num_seeds=2, duration_sec=6.0, output_dir=tmp_dir)

        assert os.path.exists(csv_path)
        assert os.path.exists(json_path)

        with open(csv_path, 'r', encoding='utf-8') as f:
            rows = list(csv.DictReader(f))
            assert len(rows) == len(define_campaign_conditions())
            for r in rows:
                assert float(r['shape_error_mean']) > 0.0
                assert 0.0 <= float(r['convergence_rate_pct']) <= 100.0
                assert 0.0 <= float(r['collision_free_rate_pct']) <= 100.0

        with open(json_path, 'r', encoding='utf-8') as f:
            loaded = json.load(f)
            assert len(loaded['arena_unshielded']['seed_results']) == 2
            assert loaded['sweep_delay_200ms']['summary']['applied_latency_ms'] == (
                pytest.approx(200.0)
            )

        visualizer = CampaignVisualizer.from_json(json_path, output_dir=plots_dir)
        plots = visualizer.generate_all_campaign_plots()
        assert len(plots) == 5
        for p in plots:
            assert os.path.getsize(p) > 10000

    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)
