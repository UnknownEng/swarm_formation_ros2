#!/usr/bin/env bash
# ==============================================================================
# reproduce_all.sh: Master Reproduction Script
# Decentralized Swarm Formation Control (ROS 2 Jazzy + Gazebo Harmonic)
#
# Builds, tests, runs the default configuration in Gazebo with a rosbag, processes
# the bag offline, runs the 550-run kinematic campaign, regenerates all figures, and
# finally VERIFIES the reproduced results against the claims in README.md.
# Any failing step stops the pipeline with a non-zero exit code.
# ==============================================================================

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

echo "========================================================================"
echo "    SWARM FORMATION: ONE-COMMAND MASTER REPRODUCTION PIPELINE"
echo "========================================================================"
echo "Workspace Directory: ${SCRIPT_DIR}"
echo "Start Time: $(date -u)"

# Cleanup: stop only the process group started by this script (Gazebo, bridge and
# swarm nodes launched below), never unrelated ros2/gz/ruby processes on the machine.
LAUNCH_PGID=""
cleanup() {
    local own_pgid
    own_pgid=$(ps -o pgid= -p $$ | tr -d ' ')
    if [ -n "${LAUNCH_PGID}" ] && [ "${LAUNCH_PGID}" != "${own_pgid}" ] \
        && kill -0 -- "-${LAUNCH_PGID}" 2>/dev/null; then
        echo ""
        echo ">> [CLEANUP] Stopping simulation process group ${LAUNCH_PGID}..."
        kill -TERM -- "-${LAUNCH_PGID}" 2>/dev/null || true
        sleep 3
        kill -KILL -- "-${LAUNCH_PGID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

# Step 1: ROS 2 underlay
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    echo ">> [STEP 1/7] Sourcing ROS 2 Jazzy underlay..."
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
else
    echo "ERROR: /opt/ros/jazzy/setup.bash not found. Is ROS 2 Jazzy installed?" >&2
    exit 1
fi

# Step 2: Build
echo ">> [STEP 2/7] Building workspace with colcon..."
timeout 900 colcon build --symlink-install --packages-select swarm_interfaces swarm_formation
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/install/setup.bash"

# Step 3: Tests. Run from the package directory: from the workspace root the ament
# copyright/flake8 tests would also scan the generated build/ and install/ trees.
echo ">> [STEP 3/7] Running regression and acceptance test suite..."
(cd "${SCRIPT_DIR}/src/swarm_formation" && timeout 1800 pytest-3 test -q -p no:cacheprovider)

# Output directories
REPRO_DIR="${SCRIPT_DIR}/results/reproduction"
GAZEBO_BAG_DIR="${REPRO_DIR}/gazebo_bag"
CAMPAIGN_DIR="${REPRO_DIR}/campaign"
PLOTS_DIR="${REPRO_DIR}/plots"
rm -rf "${GAZEBO_BAG_DIR}" "${CAMPAIGN_DIR}" "${PLOTS_DIR}"
mkdir -p "${REPRO_DIR}" "${CAMPAIGN_DIR}" "${PLOTS_DIR}"

# Step 4: Default configuration in Gazebo (headless), with rosbag
echo ">> [STEP 4/7] Running the default configuration in Gazebo Harmonic (40 s simulated)..."
LAUNCH_LOG="${REPRO_DIR}/gazebo_launch.log"
# setsid gives the launch its own process group so cleanup can stop exactly it.
# Nodes run on the Gazebo clock (use_sim_time); on slow machines (e.g. WSL, RTF ~0.1)
# 40 s of simulated time can take several minutes of wall time.
setsid timeout 2400 ros2 launch swarm_formation gazebo_swarm_launch.py \
    headless:=true \
    sim_duration:=40.0 \
    enable_faults:=false \
    use_resilient_method:=false \
    use_safety_filter:=true \
    controller_type:=displacement \
    record_bag:=true \
    bag_path:="${GAZEBO_BAG_DIR}" \
    csv_path:="${REPRO_DIR}/repro_metrics.csv" > "${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!
# Without job control the background child is not a group leader, so setsid runs
# in-process and the new process group id equals the child's pid.
LAUNCH_PGID="${LAUNCH_PID}"
LAUNCH_STATUS=0
wait "${LAUNCH_PID}" || LAUNCH_STATUS=$?
LAUNCH_PGID_DONE="${LAUNCH_PGID}"
cleanup
LAUNCH_PGID=""
sleep 2

# Fail loudly: a crashed node, a timeout, or missing outputs invalidate the run
if [ "${LAUNCH_STATUS}" -ne 0 ]; then
    echo "ERROR: Gazebo launch exited with status ${LAUNCH_STATUS} (124 = timeout). See ${LAUNCH_LOG}" >&2
    exit 1
fi
if grep -q "Unhandled exception" "${LAUNCH_LOG}"; then
    echo "ERROR: a swarm node crashed during the Gazebo run (pgid ${LAUNCH_PGID_DONE}). See ${LAUNCH_LOG}" >&2
    exit 1
fi
if grep -q "No wheel odometry" "${LAUNCH_LOG}"; then
    echo "ERROR: wheel odometry never reached the EKF (check the gz bridge). See ${LAUNCH_LOG}" >&2
    exit 1
fi
if [ ! -s "${REPRO_DIR}/repro_metrics.csv" ]; then
    echo "ERROR: Gazebo run produced no metrics CSV. See ${LAUNCH_LOG}" >&2
    exit 1
fi
if [ ! -d "${GAZEBO_BAG_DIR}" ]; then
    echo "ERROR: Gazebo rosbag was not recorded at ${GAZEBO_BAG_DIR}. See ${LAUNCH_LOG}" >&2
    exit 1
fi

# Step 5: Offline rosbag processing and trajectory plots (bag path is positional)
echo ">> [STEP 5/7] Processing the recorded rosbag and generating plots..."
timeout 900 ros2 run swarm_formation process_bag "${GAZEBO_BAG_DIR}" \
    --output-csv "${REPRO_DIR}/offline_bag_metrics.csv" \
    --output-json "${REPRO_DIR}/offline_bag_summary.json"
timeout 900 ros2 run swarm_formation plot_bag "${GAZEBO_BAG_DIR}" \
    --output-dir "${PLOTS_DIR}/gazebo"

# Step 6: Statistical campaign (kinematic simulator) and figures
echo ">> [STEP 6/7] Running the campaign (22 conditions x 25 seeds = 550 runs)..."
timeout 3600 ros2 run swarm_formation run_campaign \
    --seeds 25 \
    --duration 30.0 \
    --workers "$(nproc)" \
    --output-dir "${CAMPAIGN_DIR}"
timeout 300 ros2 run swarm_formation plot_campaign \
    --input-json "${CAMPAIGN_DIR}/campaign_results.json" \
    --output-dir "${PLOTS_DIR}/campaign"

# Step 7: Verify the reproduced results against the documented claims
echo ">> [STEP 7/7] Verifying reproduced results against README claims..."
python3 - "${REPRO_DIR}" <<'PYEOF'
import csv
import json
import os
import sys

repro = sys.argv[1]
failures = []


def check(ok, msg):
    print(('  PASS  ' if ok else '  FAIL  ') + msg)
    if not ok:
        failures.append(msg)


rows = list(csv.DictReader(open(os.path.join(repro, 'repro_metrics.csv'), encoding='utf-8')))
last = rows[-1]
check(last['is_converged'] == 'True', 'Gazebo default run reached the formation')
check(float(last['shape_error_rms']) < 0.25,
      f"Gazebo final shape error {float(last['shape_error_rms']):.3f} m < 0.25 m")
check(float(last['global_min_distance']) >= 0.50,
      f"Gazebo closest robots {float(last['global_min_distance']):.2f} m >= 0.50 m")
summary = json.load(open(os.path.join(repro, 'offline_bag_summary.json'), encoding='utf-8'))
check(bool(summary.get('empirically_collision_free')), 'offline bag analysis: collision-free')

camp = {r['condition']: r for r in csv.DictReader(
    open(os.path.join(repro, 'campaign', 'campaign_summary.csv'), encoding='utf-8'))}
for name in ('deployed_clean', 'deployed_impaired'):
    c = camp[name]
    check(float(c['collision_free_rate_pct']) == 100.0, f'{name}: 100% robot-robot collision-free')
    check(float(c['obstacle_collision_free_rate_pct']) == 100.0,
          f'{name}: 100% robot-obstacle collision-free')
    check(float(c['convergence_rate_pct']) >= 95.0,
          f"{name}: converged in {float(c['convergence_rate_pct']):.0f}% of runs (>= 95%)")
check(float(camp['arena_unshielded']['collision_free_rate_pct']) < 100.0,
      'without avoidance the crossing scenario collides')
for name in sorted(k for k in camp if k.startswith('asym_')):
    c = camp[name]
    check(float(c['collision_free_rate_pct']) == 100.0
          and float(c['obstacle_collision_free_rate_pct']) == 100.0,
          f'{name} (asymmetric links): 100% collision-free')
check(len([k for k in camp if k.startswith('asym_')]) == 5, 'five asymmetric-link conditions')
for fig in ('ablation_study_barchart.png', 'safety_comparison_apf_vs_cbf.png',
            'difficulty_sweep_packet_drop.png', 'difficulty_sweep_latency.png',
            'asymmetric_network_barchart.png'):
    path = os.path.join(repro, 'plots', 'campaign', fig)
    check(os.path.isfile(path) and os.path.getsize(path) > 10000, f'figure {fig}')
if failures:
    print(f'{len(failures)} verification check(s) failed')
    sys.exit(1)
print('All verification checks passed')
PYEOF

echo ""
echo "========================================================================"
echo "    REPRODUCTION PIPELINE COMPLETE AND VERIFIED"
echo "========================================================================"
echo "Artifact Directory: ${REPRO_DIR}"
echo "Campaign Summary CSV:  ${CAMPAIGN_DIR}/campaign_summary.csv"
echo "Gazebo Metrics CSV:    ${REPRO_DIR}/repro_metrics.csv"
echo "End Time: $(date -u)"
