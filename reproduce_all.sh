#!/usr/bin/env bash
# ==============================================================================
# reproduce_all.sh: Master Reproduction Script
# Physics-Verified Decentralized Swarm Formation Control (ROS 2 Jazzy + Gazebo Harmonic)
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
    if [ -n "${LAUNCH_PGID}" ] && [ "${LAUNCH_PGID}" != "${own_pgid}" ]         && kill -0 -- "-${LAUNCH_PGID}" 2>/dev/null; then
        echo ""
        echo ">> [CLEANUP] Stopping simulation process group ${LAUNCH_PGID}..."
        kill -TERM -- "-${LAUNCH_PGID}" 2>/dev/null || true
        sleep 3
        kill -KILL -- "-${LAUNCH_PGID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

# Sourcing ROS 2 Underlay
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    echo ">> [STEP 1/6] Sourcing ROS 2 Jazzy underlay..."
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
else
    echo "ERROR: /opt/ros/jazzy/setup.bash not found. Is ROS 2 Jazzy installed?" >&2
    exit 1
fi

# Step 2: Colcon Build
echo ">> [STEP 2/6] Building workspace with colcon..."
timeout 180 colcon build --symlink-install --packages-select swarm_interfaces swarm_formation

# Source Workspace Overlay
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/install/setup.bash"

# Step 3: Regression Test Suite
echo ">> [STEP 3/6] Running regression and acceptance test suite..."
timeout 900 pytest-3 "${SCRIPT_DIR}/src/swarm_formation/test" -v

# Prepare Reproduction Output Directories
REPRO_DIR="${SCRIPT_DIR}/results/reproduction"
GAZEBO_BAG_DIR="${REPRO_DIR}/gazebo_bag"
CAMPAIGN_DIR="${REPRO_DIR}/campaign"
PLOTS_DIR="${REPRO_DIR}/plots"

rm -rf "${GAZEBO_BAG_DIR}" "${CAMPAIGN_DIR}" "${PLOTS_DIR}"
mkdir -p "${REPRO_DIR}" "${CAMPAIGN_DIR}" "${PLOTS_DIR}"

# Step 4: Headless Physics Simulation in Gazebo Harmonic with Rosbag Recording
echo ">> [STEP 4/6] Launching 5-robot physics simulation in Gazebo Harmonic (35s duration)..."
LAUNCH_LOG="${REPRO_DIR}/gazebo_launch.log"
# setsid gives the launch its own process group so cleanup can stop exactly it
setsid timeout 90 ros2 launch swarm_formation gazebo_swarm_launch.py \
    headless:=true \
    sim_duration:=35.0 \
    enable_faults:=true \
    use_resilient_method:=true \
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

# Wait 2 seconds for buffers to sync
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
if [ ! -s "${REPRO_DIR}/repro_metrics.csv" ]; then
    echo "ERROR: Gazebo run produced no metrics CSV. See ${LAUNCH_LOG}" >&2
    exit 1
fi

# Step 5: Process Rosbag & Generate Trajectory Plots
echo ">> [STEP 5/6] Processing recorded rosbag and generating publication plots..."
if [ -d "${GAZEBO_BAG_DIR}" ]; then
    timeout 60 ros2 run swarm_formation process_bag \
        --input-bag "${GAZEBO_BAG_DIR}" \
        --output-csv "${REPRO_DIR}/offline_bag_metrics.csv"

    timeout 60 ros2 run swarm_formation plot_bag \
        --input-bag "${GAZEBO_BAG_DIR}" \
        --output-dir "${PLOTS_DIR}/gazebo"
else
    echo "ERROR: Gazebo rosbag was not recorded at ${GAZEBO_BAG_DIR}. See ${LAUNCH_LOG}" >&2
    exit 1
fi

# Step 6: Execute Statistical Campaign & Generate All 4 Main Figures
echo ">> [STEP 6/6] Executing multi-seed campaign (16 conditions x 25 seeds = 400 runs)..."
timeout 1800 ros2 run swarm_formation run_campaign \
    --seeds 25 \
    --duration 30.0 \
    --output-dir "${CAMPAIGN_DIR}"

echo ">> Generating master publication figures from campaign results..."
timeout 60 ros2 run swarm_formation plot_campaign \
    --input-json "${CAMPAIGN_DIR}/campaign_results.json" \
    --output-dir "${PLOTS_DIR}/campaign"

echo ""
echo "========================================================================"
echo "    REPRODUCTION PIPELINE COMPLETE"
echo "========================================================================"
echo "Artifact Directory: ${REPRO_DIR}"
echo "Campaign Results JSON: ${CAMPAIGN_DIR}/campaign_results.json"
echo "Campaign Summary CSV:  ${CAMPAIGN_DIR}/campaign_summary.csv"
echo "Generated Plots:"
ls -la "${PLOTS_DIR}/campaign" || true
echo "End Time: $(date -u)"
