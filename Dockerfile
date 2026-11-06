# ==============================================================================
# Dockerfile: Physics-Verified Decentralized Swarm Formation Control
# ROS 2 Jazzy Jalisco + Gazebo Harmonic (8.x) on Ubuntu 24.04 LTS (Noble Numbat)
# ==============================================================================

FROM osrf/ros:jazzy-desktop

# Set non-interactive debian frontend and configure locale/environment
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=jazzy
ENV GZ_VERSION=harmonic
ENV QT_QPA_PLATFORM=offscreen
ENV PYTHONUNBUFFERED=1

# Install system dependencies, Gazebo Harmonic ROS bridge, and Python tooling
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash \
    build-essential \
    cmake \
    git \
    pkg-config \
    python3-colcon-common-extensions \
    python3-pip \
    python3-pytest \
    python3-pytest-cov \
    python3-flake8 \
    python3-pep257 \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    python3-yaml \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-rosbag2 \
    ros-jazzy-rosbag2-py \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-rosbag2-storage-sqlite3 \
    && rm -rf /var/lib/apt/lists/*

# Create swarm workspace directory
WORKDIR /root/swarm_ws

# Copy ROS 2 package sources, launch files, configurations, and reproduction scripts
COPY src/ /root/swarm_ws/src/
COPY reproduce_all.sh /root/swarm_ws/reproduce_all.sh
COPY README.md /root/swarm_ws/README.md

# Ensure reproduction script is executable
RUN chmod +x /root/swarm_ws/reproduce_all.sh

# Source ROS 2 and build the swarm_formation workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Create an entrypoint script that sources both underlay and overlay
RUN printf '#!/bin/bash\nset -e\nsource /opt/ros/%s/setup.bash\nsource /root/swarm_ws/install/setup.bash\nexec "$@"\n' "${ROS_DISTRO}" > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
