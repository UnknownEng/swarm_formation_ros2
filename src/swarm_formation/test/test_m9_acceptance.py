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

"""
Acceptance tests for Milestone M9: documentation, Docker image and reproduction pipeline.

These check behaviour, not keywords: the earlier version only searched for words, and
passed while the Dockerfile requested a package that does not exist on Ubuntu 24.04,
the pipeline called the bag tools with flags they do not accept, and the tests were
run from a directory where the lint tests fail.
"""

import os
import re
import shlex
import shutil
import subprocess

import pytest

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
WS_DIR = os.path.abspath(os.path.join(PKG_DIR, '..', '..'))
DOCKERFILE = os.path.join(WS_DIR, 'Dockerfile')
SCRIPT = os.path.join(WS_DIR, 'reproduce_all.sh')
README = os.path.join(WS_DIR, 'README.md')


def _read(path: str) -> str:
    with open(path, 'r', encoding='utf-8') as f:
        return f.read()


def _dockerfile_apt_packages() -> list:
    text = _read(DOCKERFILE).replace('\\\n', ' ')
    match = re.search(r'apt-get install -y --no-install-recommends(.*?)&&', text, re.S)
    assert match, 'apt-get install line not found in Dockerfile'
    return [tok for tok in match.group(1).split() if not tok.startswith('-')]


def test_dockerfile_structure():
    """The image builds the workspace and runs reproduce_all.sh through an entrypoint."""
    content = _read(DOCKERFILE)
    assert 'FROM osrf/ros:jazzy-desktop' in content
    assert 'colcon build' in content
    assert 'COPY reproduce_all.sh' in content
    assert 'ENTRYPOINT' in content
    packages = _dockerfile_apt_packages()
    for required in ('ros-jazzy-ros-gz', 'ros-jazzy-rosbag2-storage-mcap', 'python3-pytest',
                     'ros-jazzy-ament-flake8', 'ros-jazzy-ament-pep257', 'procps'):
        assert required in packages, required
    assert 'python3-pep257' not in packages, 'python3-pep257 does not exist on Ubuntu 24.04'


@pytest.mark.skipif(shutil.which('apt-cache') is None, reason='needs apt (Ubuntu)')
def test_dockerfile_apt_packages_exist():
    """Every package the image installs must be available, or `docker build` fails."""
    missing = []
    for pkg in _dockerfile_apt_packages():
        out = subprocess.run(['apt-cache', 'policy', pkg], capture_output=True, text=True,
                             timeout=60).stdout
        cand = re.search(r'Candidate:\s*(\S+)', out)
        if not cand or cand.group(1) == '(none)':
            missing.append(pkg)
    assert not missing, f'apt packages not available: {missing}'


def test_reproduce_script_is_valid_and_executable():
    """The pipeline is executable bash and runs every stage."""
    assert os.access(SCRIPT, os.X_OK), 'reproduce_all.sh is not marked executable'
    res = subprocess.run(['bash', '-n', SCRIPT], capture_output=True, text=True, timeout=30)
    assert res.returncode == 0, res.stderr
    content = _read(SCRIPT)
    for stage in ('colcon build', 'gazebo_swarm_launch.py', 'process_bag', 'plot_bag',
                  'run_campaign', 'plot_campaign', 'STEP 7/7'):
        assert stage in content, stage


def test_reproduce_script_runs_tests_from_the_package_directory():
    """From the workspace root, the ament lint tests also scan build/ and install/."""
    content = _read(SCRIPT)
    assert 'cd "${SCRIPT_DIR}/src/swarm_formation" && timeout' in content
    assert 'pytest-3 "${SCRIPT_DIR}/src/swarm_formation/test"' not in content


def _tool_options(module: str) -> set:
    """Option flags accepted by a console tool's argparse parser (parsed from source)."""
    src = _read(os.path.join(PKG_DIR, 'swarm_formation', f'{module}.py'))
    return set(re.findall(r"add_argument\(\s*'(--[a-z0-9-]+)'", src))


@pytest.mark.parametrize('doc', [SCRIPT, README])
def test_bag_tool_invocations_match_their_parsers(doc):
    """Every flag passed to process_bag / plot_bag in the docs and script must exist."""
    accepted = {'process_bag': _tool_options('rosbag_processor'),
                'plot_bag': _tool_options('plot_generator'),
                'run_campaign': _tool_options('campaign_runner'),
                'plot_campaign': _tool_options('campaign_visualizer')}
    text = _read(doc).replace('\\\n', ' ')
    found = 0
    for tool, options in accepted.items():
        for match in re.finditer(rf'ros2 run swarm_formation {tool}([^\n]*)', text):
            found += 1
            for tok in shlex.split(match.group(1), posix=True):
                if tok.startswith('--'):
                    assert tok in options, f'{os.path.basename(doc)}: {tool} has no option {tok}'
    assert found > 0


def test_readme_documents_setup_and_results():
    """README explains how to install, build, run and test, and reports the results."""
    content = _read(README)
    for section in ('## Quick start', '## Results', '## Testing', './setup.sh',
                    'reproduce_all.sh', 'LICENSE'):
        assert section in content, section
    assert os.path.isfile(os.path.join(WS_DIR, 'LICENSE'))
    assert os.path.isfile(os.path.join(WS_DIR, 'setup.sh'))
