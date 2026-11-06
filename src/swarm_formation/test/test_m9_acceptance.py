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

"""Acceptance tests for Milestone M9: Documentation, Docker, and Reproduction Package."""

import os
import subprocess


def test_dockerfile_completeness():
    """Verify Dockerfile exists and conforms to ROS 2 Jazzy specification."""
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    dockerfile_path = os.path.join(repo_root, 'Dockerfile')
    assert os.path.isfile(dockerfile_path), f'Dockerfile missing at {dockerfile_path}'

    with open(dockerfile_path, 'r', encoding='utf-8') as f:
        content = f.read()

    assert 'FROM osrf/ros:jazzy-desktop' in content
    assert 'colcon build' in content
    assert 'ros-jazzy-ros-gz' in content
    assert 'ENTRYPOINT' in content
    assert 'reproduce_all.sh' in content


def test_readme_completeness():
    """Verify README.md contains complete system architecture, references, and guide."""
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    readme_path = os.path.join(repo_root, 'README.md')
    assert os.path.isfile(readme_path), f'README.md missing at {readme_path}'

    with open(readme_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Check for core sections and diagrams
    assert 'flowchart TD' in content or 'mermaid' in content
    assert 'neighbor_sensor' in content
    assert 'localization_node' in content
    assert 'safety_filter' in content
    assert 'network_impairment' in content
    assert 'fault_injector' in content
    assert 'ToggleLink' in content
    assert 'TriggerFault' in content
    assert 'Displacement-Based' in content or 'displacement' in content
    assert 'Distance-Based' in content or 'distance' in content
    assert 'Bearing-Based' in content or 'bearing' in content
    assert 'Control Barrier Function' in content or 'CBF-QP' in content
    assert 'reproduce_all.sh' in content
    assert '95% Confidence Interval' in content


def test_technical_report_completeness():
    """Verify reports/TECHNICAL_REPORT.md contains complete academic paper content."""
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    report_path = os.path.join(repo_root, 'reports', 'TECHNICAL_REPORT.md')
    assert os.path.isfile(report_path), f'TECHNICAL_REPORT.md missing at {report_path}'

    with open(report_path, 'r', encoding='utf-8') as f:
        content = f.read()

    assert 'Abstract' in content
    assert 'Ren & Beard' in content or 'Ren' in content
    assert 'Oh & Ahn' in content or 'Oh' in content
    assert 'Zhao & Zelazo' in content or 'Zhao' in content
    assert 'Extended Kalman Filter' in content or 'EKF' in content
    assert 'Control Barrier Function' in content
    assert 'D_s^{\\text{eff}}' in content or 'D_s' in content
    assert '325' in content
    assert '47.2%' in content or '208.8' in content
    assert 'References' in content


def test_reproduce_all_script_validity():
    """Verify reproduce_all.sh exists, is executable, and has valid bash syntax."""
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    script_path = os.path.join(repo_root, 'reproduce_all.sh')
    assert os.path.isfile(script_path), f'reproduce_all.sh missing at {script_path}'
    assert os.access(script_path, os.X_OK), 'reproduce_all.sh is not marked executable'

    # Run bash -n syntax check
    res = subprocess.run(['bash', '-n', script_path], capture_output=True, text=True)
    assert res.returncode == 0, f'bash syntax check failed: {res.stderr}'

    with open(script_path, 'r', encoding='utf-8') as f:
        content = f.read()

    assert 'colcon build' in content
    assert 'pytest-3' in content
    assert 'gazebo_swarm_launch.py' in content
    assert 'process_bag' in content
    assert 'plot_bag' in content
    assert 'run_campaign' in content
    assert 'plot_campaign' in content


def test_campaign_artifacts_presence():
    """Verify all statistical campaign results and publication figures exist."""
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    summary_csv = os.path.join(repo_root, 'results', 'M8', 'campaign_summary.csv')
    results_json = os.path.join(repo_root, 'results', 'M8', 'campaign_results.json')
    plots_dir = os.path.join(repo_root, 'results', 'M8', 'plots')

    assert os.path.isfile(summary_csv)
    assert os.path.isfile(results_json)
    for fig_name in [
        'ablation_study_barchart.png',
        'safety_comparison_apf_vs_cbf.png',
        'difficulty_sweep_packet_drop.png',
        'difficulty_sweep_latency.png',
    ]:
        fig_path = os.path.join(plots_dir, fig_name)
        assert os.path.isfile(fig_path), f'Missing campaign figure {fig_path}'
        assert os.path.getsize(fig_path) > 10000, f'Figure {fig_name} is unexpectedly small'
