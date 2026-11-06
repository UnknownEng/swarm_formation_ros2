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

"""Unit tests for offline rosbag2 processor and plot generator pipeline."""

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


def test_procrustes_error_exact_pentagon():
    """Verify that an exact regular pentagon produces zero Procrustes RMS shape error."""
    radius = 2.0
    total_robots = 5
    pts = [
        (
            radius * math.cos(2.0 * math.pi * i / total_robots),
            radius * math.sin(2.0 * math.pi * i / total_robots),
        )
        for i in range(total_robots)
    ]
    err = RosbagProcessor.compute_procrustes_error(pts, radius, total_robots)
    assert err == pytest.approx(0.0, abs=1e-6)


def test_procrustes_error_rotated_translated_pentagon():
    """Verify that rigid translation and rotation invariant alignment gives zero error."""
    radius = 2.0
    total_robots = 5
    theta = 1.15  # Arbitrary rotation
    dx = 4.5
    dy = -3.2

    pts = []
    for i in range(total_robots):
        base_x = radius * math.cos(2.0 * math.pi * i / total_robots)
        base_y = radius * math.sin(2.0 * math.pi * i / total_robots)
        rx = base_x * math.cos(theta) - base_y * math.sin(theta) + dx
        ry = base_x * math.sin(theta) + base_y * math.cos(theta) + dy
        pts.append((rx, ry))

    err = RosbagProcessor.compute_procrustes_error(pts, radius, total_robots)
    assert err == pytest.approx(0.0, abs=1e-5)


def test_quaternion_to_yaw():
    """Verify conversion of planar quaternions to yaw radians."""
    # Yaw = 0.0
    assert RosbagProcessor._quaternion_to_yaw(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0, abs=1e-6)

    # Yaw = pi/2: z = sin(pi/4), w = cos(pi/4)
    s = math.sin(math.pi / 4.0)
    c = math.cos(math.pi / 4.0)
    yaw_pos = RosbagProcessor._quaternion_to_yaw(0.0, 0.0, s, c)
    assert yaw_pos == pytest.approx(math.pi / 2.0, abs=1e-6)

    # Yaw = -pi/2
    yaw_neg = RosbagProcessor._quaternion_to_yaw(0.0, 0.0, -s, c)
    assert yaw_neg == pytest.approx(-math.pi / 2.0, abs=1e-6)


def create_synthetic_rosbag(bag_dir: str, duration_sec: float = 6.0, dt: float = 0.1) -> None:
    """Create a temporary rosbag2 containing synthetic swarm messages."""
    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    writer.open(storage_options, converter_options)

    # Create topics for 5 robots
    topic_id = 0
    for i in range(1, 6):
        writer.create_topic(
            rosbag2_py.TopicMetadata(topic_id, f'/robot_{i}/pose', 'geometry_msgs/msg/Pose', 'cdr')
        )
        topic_id += 1
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                topic_id, f'/robot_{i}/estimated_pose', 'geometry_msgs/msg/Pose', 'cdr'
            )
        )
        topic_id += 1
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                topic_id, f'/robot_{i}/cmd_vel', 'geometry_msgs/msg/Twist', 'cdr'
            )
        )
        topic_id += 1
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                topic_id, f'/robot_{i}/estimation_error', 'geometry_msgs/msg/Vector3', 'cdr'
            )
        )
        topic_id += 1
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                topic_id, f'/robot_{i}/safety_intervention', 'std_msgs/msg/Float64', 'cdr'
            )
        )
        topic_id += 1

    # Synthetic non-crossing radial contraction trajectory
    r_start = 4.0
    radius = 2.0
    start_positions = [
        (
            r_start * math.cos(2.0 * math.pi * i / 5),
            r_start * math.sin(2.0 * math.pi * i / 5),
        )
        for i in range(5)
    ]

    target_positions = [
        (
            radius * math.cos(2.0 * math.pi * i / 5),
            radius * math.sin(2.0 * math.pi * i / 5),
        )
        for i in range(5)
    ]

    steps = int(duration_sec / dt)
    t0_ns = 1000000000

    for step in range(steps + 1):
        t_sec = step * dt
        ts_ns = t0_ns + int(t_sec * 1e9)
        alpha = min(1.0, t_sec / (duration_sec * 0.6))  # Converges at 60% of time

        for i in range(1, 6):
            sx, sy = start_positions[i - 1]
            tx, ty = target_positions[i - 1]
            cur_x = (1.0 - alpha) * sx + alpha * tx
            cur_y = (1.0 - alpha) * sy + alpha * ty

            # Pose
            p = Pose()
            p.position.x = cur_x
            p.position.y = cur_y
            p.orientation.w = 1.0
            writer.write(f'/robot_{i}/pose', rclpy.serialization.serialize_message(p), ts_ns)

            # Estimated pose (with small 0.03m offset)
            p_est = Pose()
            p_est.position.x = cur_x + 0.02
            p_est.position.y = cur_y - 0.02
            p_est.orientation.w = 1.0
            writer.write(
                f'/robot_{i}/estimated_pose',
                rclpy.serialization.serialize_message(p_est),
                ts_ns,
            )

            # Cmd vel
            twist = Twist()
            twist.linear.x = 0.2 * (1.0 - alpha)
            writer.write(
                f'/robot_{i}/cmd_vel',
                rclpy.serialization.serialize_message(twist),
                ts_ns,
            )

            # Estimation error
            err_vec = Vector3()
            err_vec.x = math.hypot(0.02, 0.02)
            writer.write(
                f'/robot_{i}/estimation_error',
                rclpy.serialization.serialize_message(err_vec),
                ts_ns,
            )

            # Safety intervention (zero intervention in nominal flight)
            interv = Float64()
            interv.data = 0.0
            writer.write(
                f'/robot_{i}/safety_intervention',
                rclpy.serialization.serialize_message(interv),
                ts_ns,
            )

    del writer


def test_rosbag_processor_full_pipeline():
    """Verify end-to-end rosbag2 ingestion, metrics computation, and export."""
    tmp_dir = tempfile.mkdtemp(prefix='test_bag_pipeline_')
    bag_path = os.path.join(tmp_dir, 'synthetic_swarm_bag')
    csv_path = os.path.join(tmp_dir, 'metrics.csv')
    json_path = os.path.join(tmp_dir, 'summary.json')

    try:
        create_synthetic_rosbag(bag_path, duration_sec=6.0, dt=0.1)

        processor = RosbagProcessor(
            bag_path=bag_path,
            total_robots=5,
            formation_radius=2.0,
            convergence_threshold=0.25,
            settle_duration=1.5,
            collision_threshold=0.50,
        )

        processor.read_bag()
        assert len(processor.raw_poses[1]) > 50
        assert len(processor.raw_est_poses[1]) > 50

        metrics = processor.compute_metrics(dt=0.05)

        # Verify quantitative assertions
        assert metrics['total_duration'] == pytest.approx(6.0, abs=0.2)
        assert metrics['final_shape_error'] < 0.05
        assert metrics['is_converged'] is True
        assert metrics['convergence_time'] is not None
        assert metrics['convergence_time'] < 5.0
        assert metrics['global_min_distance'] >= 0.50
        assert metrics['total_collisions'] == 0
        assert metrics['empirically_collision_free'] is True
        assert metrics['mean_est_error'] == pytest.approx(math.hypot(0.02, 0.02), abs=0.01)

        # Export and verify CSV
        processor.export_csv(csv_path, dt=0.05)
        assert os.path.exists(csv_path)
        with open(csv_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
            assert len(lines) > 100
            assert 'shape_error_rms' in lines[0]
            assert 'r1_x' in lines[0]
            assert 'r5_y' in lines[0]

        # Export and verify JSON
        processor.export_summary_json(json_path, dt=0.05)
        assert os.path.exists(json_path)
        with open(json_path, 'r', encoding='utf-8') as f:
            summary = json.load(f)
            assert summary['is_converged'] is True
            assert summary['final_shape_error_rms'] < 0.05
            assert summary['empirically_collision_free'] is True

        # Generate plots from the bag
        plots_dir = os.path.join(tmp_dir, 'plots')
        generator = SwarmPlotGenerator.from_bag(bag_path, output_dir=plots_dir)
        saved = generator.generate_all_plots()

        assert len(saved) == 4
        for p in saved:
            assert os.path.exists(p)
            assert os.path.getsize(p) > 10000  # Non-trivial image file

        # Generate plots from the exported CSV
        csv_plots_dir = os.path.join(tmp_dir, 'plots_from_csv')
        csv_gen = SwarmPlotGenerator.from_csv(csv_path, output_dir=csv_plots_dir)
        csv_saved = csv_gen.generate_all_plots()
        assert len(csv_saved) == 4
        for p in csv_saved:
            assert os.path.exists(p)
            assert os.path.getsize(p) > 10000

    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)
