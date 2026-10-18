#!/usr/bin/env bash
# Copyright 2026 Open Source Robotics Foundation, Inc.
set -e

echo "=========================================================================="
echo " GAZEBO HARMONIC DECENTRALIZED MULTI-ROBOT SHAPE FORMATION BENCHMARK RUNNER"
echo "=========================================================================="

# 1. Source ROS 2 environment and optimize PATH against WSL 9p RPC latency
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v '^/mnt/' | tr '\n' ':' | sed 's/:$//')
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo ""
echo ">>> [1/3] Running Gazebo Part 1: Baseline Formation (No APF / No Faults)..."
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f gazebo_node 2>/dev/null || true
pkill -9 -f metrics_logger 2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
killall -9 gz ruby gzserver gzclient 2>/dev/null || true
sleep 4
ros2 launch swarm_formation gazebo_swarm_launch.py \
    enable_faults:=false \
    use_resilient_method:=false \
    csv_path:=gazebo_part1_baseline.csv \
    sim_duration:=25.0

echo ""
echo ">>> [2/3] Running Gazebo Part 2: Real-World Faults (15% drop, noise, agent 4 dropout)..."
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f gazebo_node 2>/dev/null || true
pkill -9 -f metrics_logger 2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
killall -9 gz ruby gzserver gzclient 2>/dev/null || true
sleep 4
ros2 launch swarm_formation gazebo_swarm_launch.py \
    enable_faults:=true \
    use_resilient_method:=false \
    csv_path:=gazebo_part2_faults.csv \
    sim_duration:=25.0

echo ""
echo ">>> [3/3] Running Gazebo Part 3: Resilient Consensus + APF Safety Barrier..."
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f gazebo_node 2>/dev/null || true
pkill -9 -f metrics_logger 2>/dev/null || true
pkill -9 -f parameter_bridge 2>/dev/null || true
killall -9 gz ruby gzserver gzclient 2>/dev/null || true
sleep 4
ros2 launch swarm_formation gazebo_swarm_launch.py \
    enable_faults:=true \
    use_resilient_method:=true \
    csv_path:=gazebo_part3_resilient.csv \
    sim_duration:=45.0

cp gazebo_part3_resilient.csv swarm_metrics.csv

echo ""
echo ">>> Generating Individual Run Plots and Comparative Summary..."
python3 generate_gazebo_comparison.py

echo ""
echo "=========================================================================="
echo " ALL GAZEBO PHYSICS BENCHMARK EXPERIMENTS COMPLETED SUCCESSFULLY!"
echo " Generated CSV Files: gazebo_part1_baseline.csv, gazebo_part2_faults.csv, gazebo_part3_resilient.csv, swarm_metrics.csv"
echo " Generated Plots:     gazebo_part1_baseline.png, gazebo_part2_faults.png, gazebo_part3_resilient.png, gazebo_swarm_comparison.png"
echo "=========================================================================="
