#!/usr/bin/env bash
# Install dependencies, build the workspace and optionally start the simulation.
#
# Usage: ./setup.sh [--run] [--test] [--skip-deps] [launch arguments...]
#   --run        start the simulation with the Gazebo GUI after building
#   --test       run the test suite after building
#   --skip-deps  do not check or install system packages
# Extra arguments are passed to the launch file, e.g.
#   ./setup.sh --run enable_faults:=true controller_type:=bearing

set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP=/opt/ros/jazzy/setup.bash

RUN=false
TEST=false
SKIP_DEPS=false
LAUNCH_ARGS=()
for arg in "$@"; do
    case "${arg}" in
        --run) RUN=true ;;
        --test) TEST=true ;;
        --skip-deps) SKIP_DEPS=true ;;
        -h|--help) sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *:=*) LAUNCH_ARGS+=("${arg}") ;;
        *) echo "Unknown option: ${arg} (see --help)" >&2; exit 2 ;;
    esac
done

info() { printf '\033[1;34m==>\033[0m %s\n' "$*"; }
fail() { printf '\033[1;31mError:\033[0m %s\n' "$*" >&2; exit 1; }

if [ ! -f "${ROS_SETUP}" ]; then
    fail "ROS 2 Jazzy was not found at /opt/ros/jazzy.
Install it first: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html"
fi

PACKAGES=(
    python3-colcon-common-extensions
    python3-pytest
    python3-numpy
    python3-scipy
    python3-matplotlib
    python3-yaml
    procps
    ros-jazzy-ros-gz
    ros-jazzy-ros-gz-sim
    ros-jazzy-ros-gz-bridge
    ros-jazzy-rosbag2
    ros-jazzy-rosbag2-py
    ros-jazzy-rosbag2-storage-mcap
    ros-jazzy-ament-copyright
    ros-jazzy-ament-flake8
    ros-jazzy-ament-pep257
)

if [ "${SKIP_DEPS}" = false ]; then
    info "Checking system packages"
    MISSING=()
    for pkg in "${PACKAGES[@]}"; do
        dpkg-query -W -f='${Status}' "${pkg}" 2>/dev/null | grep -q "install ok installed" \
            || MISSING+=("${pkg}")
    done
    if [ "${#MISSING[@]}" -gt 0 ]; then
        info "Installing: ${MISSING[*]}"
        SUDO=(sudo)
        [ "$(id -u)" -eq 0 ] && SUDO=()
        "${SUDO[@]}" apt-get update
        "${SUDO[@]}" env DEBIAN_FRONTEND=noninteractive \
            apt-get install -y --no-install-recommends "${MISSING[@]}"
    else
        info "All packages are installed"
    fi
fi

# shellcheck disable=SC1090
source "${ROS_SETUP}"
cd "${WS_DIR}"

info "Building the workspace"
colcon build --symlink-install --packages-select swarm_interfaces swarm_formation
# shellcheck disable=SC1091
source "${WS_DIR}/install/setup.bash"

if [ "${TEST}" = true ]; then
    info "Running tests"
    (cd "${WS_DIR}/src/swarm_formation" && pytest-3 test -q -p no:cacheprovider)
fi

if [ "${RUN}" = true ]; then
    info "Starting the simulation (Ctrl-C to stop)"
    exec ros2 launch swarm_formation gazebo_swarm_launch.py headless:=false "${LAUNCH_ARGS[@]}"
fi

info "Done. In a new terminal run:"
echo "    source ${WS_DIR}/install/setup.bash"
echo "    ros2 launch swarm_formation gazebo_swarm_launch.py headless:=false"
