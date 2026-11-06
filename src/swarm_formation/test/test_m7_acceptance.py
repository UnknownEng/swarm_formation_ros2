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

"""Acceptance tests for Milestone M7: Rosbag Pipeline & Offline Metrics/Plotting."""

import json
import math
import os
import shutil
import tempfile

from geometry_msgs.msg import Pose, Twist, Vector3
import pytest
import rclpy.serialization
import rosbag2_py
from std_msgs.msg import Float64

from swarm_formation.plot_generator import SwarmPlotGenerator
from swarm_formation.rosbag_processor import RosbagProcessor


def test_m7_launch_arguments_presence():
    """Verify that gazebo_swarm_launch declares record_bag and bag_path arguments."""
    import importlib.util

    launch_path = os.path.join(os.path.dirname(__file__), '..', 'launch', 'gazebo_swarm_launch.py')
    spec = importlib.util.spec_from_file_location('gazebo_swarm_launch', launch_path)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    ld = mod.generate_launch_description()
    declared_arg_names = [
        entity.name
        for entity in ld.entities
        if hasattr(entity, 'name')
    ]

    assert 'record_bag' in declared_arg_names, 'record_bag launch argument must be declared'
    assert 'bag_path' in declared_arg_names, 'bag_path launch argument must be declared'


def test_m7_offline_metrics_reproduction():
    """Verify that offline metrics computation reproduces analytical values accurately."""
    tmp_dir = tempfile.mkdtemp(prefix='test_m7_reproduction_')
    bag_path = os.path.join(tmp_dir, 'bag')
    csv_path = os.path.join(tmp_dir, 'offline_metrics.csv')
    json_path = os.path.join(tmp_dir, 'offline_summary.json')

    try:
        writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        )
        writer.open(storage_options, converter_options)

        # Register topics
        for i in range(1, 6):
            writer.create_topic(
                rosbag2_py.TopicMetadata(
                    i * 5 + 0, f'/robot_{i}/pose', 'geometry_msgs/msg/Pose', 'cdr'
                )
            )
            writer.create_topic(
                rosbag2_py.TopicMetadata(
                    i * 5 + 1, f'/robot_{i}/estimated_pose', 'geometry_msgs/msg/Pose', 'cdr'
                )
            )
            writer.create_topic(
                rosbag2_py.TopicMetadata(
                    i * 5 + 2, f'/robot_{i}/cmd_vel', 'geometry_msgs/msg/Twist', 'cdr'
                )
            )
            writer.create_topic(
                rosbag2_py.TopicMetadata(
                    i * 5 + 3, f'/robot_{i}/estimation_error', 'geometry_msgs/msg/Vector3', 'cdr'
                )
            )
            writer.create_topic(
                rosbag2_py.TopicMetadata(
                    i * 5 + 4, f'/robot_{i}/safety_intervention', 'std_msgs/msg/Float64', 'cdr'
                )
            )

        # Generate a known, converged 8-second trajectory
        total_robots = 5
        dt = 0.1
        total_steps = 80
        t0_ns = 5000000000

        for step in range(total_steps + 1):
            t_sec = step * dt
            ts_ns = t0_ns + int(t_sec * 1e9)
            # Interpolate from radius 3.5 to 2.0 over first 4 seconds, then hold steady
            r_cur = 3.5 - 1.5 * min(1.0, t_sec / 4.0)

            for i in range(1, total_robots + 1):
                angle = 2.0 * math.pi * (i - 1) / total_robots
                px = r_cur * math.cos(angle)
                py = r_cur * math.sin(angle)

                p = Pose()
                p.position.x = px
                p.position.y = py
                p.orientation.w = 1.0
                writer.write(f'/robot_{i}/pose', rclpy.serialization.serialize_message(p), ts_ns)

                p_est = Pose()
                p_est.position.x = px + 0.01
                p_est.position.y = py + 0.01
                p_est.orientation.w = 1.0
                writer.write(
                    f'/robot_{i}/estimated_pose',
                    rclpy.serialization.serialize_message(p_est),
                    ts_ns,
                )

                twist = Twist()
                twist.linear.x = 0.1 if t_sec < 4.0 else 0.0
                writer.write(
                    f'/robot_{i}/cmd_vel',
                    rclpy.serialization.serialize_message(twist),
                    ts_ns,
                )

                err_vec = Vector3()
                err_vec.x = math.hypot(0.01, 0.01)
                writer.write(
                    f'/robot_{i}/estimation_error',
                    rclpy.serialization.serialize_message(err_vec),
                    ts_ns,
                )

                interv = Float64()
                interv.data = 0.0
                writer.write(
                    f'/robot_{i}/safety_intervention',
                    rclpy.serialization.serialize_message(interv),
                    ts_ns,
                )

        del writer

        processor = RosbagProcessor(
            bag_path=bag_path,
            total_robots=5,
            formation_radius=2.0,
            convergence_threshold=0.25,
            settle_duration=2.0,
            collision_threshold=0.50,
        )
        processor.read_bag()
        metrics = processor.compute_metrics(dt=0.05)

        # Shape error at end must be ~ 0.0 (< 0.01m)
        assert metrics['final_shape_error'] < 0.01, 'Final shape error must converge to 0'
        # Error drops below 0.25m threshold at t = 3.33s (r_cur = 2.25m)
        assert metrics['is_converged'] is True
        assert metrics['convergence_time'] == pytest.approx(3.33, abs=0.5)

        # Min inter-robot distance is between adjacent robots on regular pentagon (radius 2.0):
        # d = 2 * R * sin(pi/5) = 2 * 2.0 * sin(36 deg) = 2.351m
        expected_min_dist = 2.0 * 2.0 * math.sin(math.pi / 5.0)
        assert metrics['min_distance'][-1] == pytest.approx(expected_min_dist, abs=0.05)
        assert metrics['global_min_distance'] >= 0.50
        assert metrics['empirically_collision_free'] is True
        assert metrics['total_collisions'] == 0

        # Export and verify files
        processor.export_csv(csv_path)
        processor.export_summary_json(json_path)

        assert os.path.exists(csv_path)
        assert os.path.exists(json_path)

        with open(json_path, 'r', encoding='utf-8') as f:
            summary = json.load(f)
            assert summary['empirically_collision_free'] is True
            assert summary['is_converged'] is True

        # Generate plots from rosbag alone
        plots_dir = os.path.join(tmp_dir, 'plots')
        generator = SwarmPlotGenerator.from_bag(bag_path, output_dir=plots_dir)
        saved_plots = generator.generate_all_plots()

        assert len(saved_plots) == 4
        expected_names = [
            'formation_trajectories.png',
            'shape_error_and_safety.png',
            'estimation_error.png',
            'diagnostic_summary.png',
        ]
        for name in expected_names:
            expected_file = os.path.join(plots_dir, name)
            assert os.path.exists(expected_file)
            assert os.path.getsize(expected_file) > 10000

    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)
