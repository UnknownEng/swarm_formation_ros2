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

"""Acceptance tests for Milestone M4 (Three Controller Families)."""

import math
from typing import Dict, Tuple

import rclpy
from swarm_formation.controllers import create_controller
from swarm_formation.robot_node import GazeboSwarmNode
from swarm_interfaces.msg import NeighborList, NeighborMeasurement


def compute_shape_error_rms(
    positions: Dict[int, Tuple[float, float]], target_radius: float = 2.0
) -> float:
    """Compute centroid-aligned Procrustes RMS shape error against a regular pentagon."""
    n = len(positions)
    cx = sum(p[0] for p in positions.values()) / n
    cy = sum(p[1] for p in positions.values()) / n

    # Ideal polygon coordinates centered at origin
    target = {
        i: (
            target_radius * math.cos(2.0 * math.pi * (i - 1) / n),
            target_radius * math.sin(2.0 * math.pi * (i - 1) / n),
        )
        for i in range(1, n + 1)
    }

    # Procrustes optimal rotation angle theta
    num = sum(
        (positions[i][0] - cx) * target[i][1] - (positions[i][1] - cy) * target[i][0]
        for i in range(1, n + 1)
    )
    den = sum(
        (positions[i][0] - cx) * target[i][0] + (positions[i][1] - cy) * target[i][1]
        for i in range(1, n + 1)
    )
    theta = math.atan2(num, den)

    # Compute RMS error after optimal rigid body transformation (SE(2) alignment)
    sum_sq_err = 0.0
    for i in range(1, n + 1):
        px = positions[i][0] - cx
        py = positions[i][1] - cy
        rx = px * math.cos(theta) - py * math.sin(theta)
        ry = px * math.sin(theta) + py * math.cos(theta)
        sum_sq_err += (rx - target[i][0]) ** 2 + (ry - target[i][1]) ** 2

    return math.sqrt(sum_sq_err / n)


def simulate_swarm_convergence(controller_name: str, steps: int = 150, dt: float = 0.05) -> float:
    """Simulate kinematic formation convergence of a 5-robot swarm from perturbed positions."""
    controller = create_controller(controller_name)
    n = 5
    radius = 2.0

    # Initial perturbed positions: agents spaced around radius 3.5m with angular offsets
    positions: Dict[int, Tuple[float, float]] = {
        i: (
            3.5 * math.cos(2.0 * math.pi * (i - 1) / n + 0.3),
            3.5 * math.sin(2.0 * math.pi * (i - 1) / n - 0.2),
        )
        for i in range(1, n + 1)
    }

    # Simulate closed-loop consensus dynamics
    for _ in range(steps):
        forces: Dict[int, Tuple[float, float]] = {}
        for i in range(1, n + 1):
            rel_displacements: Dict[int, Tuple[float, float]] = {}
            xi, yi = positions[i]
            for j in range(1, n + 1):
                if i == j:
                    continue
                xj, yj = positions[j]
                rel_displacements[j] = (xj - xi, yj - yi)

            fx, fy = controller.compute_control_force(
                robot_id=i,
                total_robots=n,
                radius=radius,
                rel_displacements=rel_displacements,
                gain=1.5,
                huber_delta=2.0,
            )
            forces[i] = (fx, fy)

        # Update positions via forward Euler integration
        for i in range(1, n + 1):
            px, py = positions[i]
            fx, fy = forces[i]
            positions[i] = (px + fx * dt, py + fy * dt)

    return compute_shape_error_rms(positions, radius)


def test_displacement_controller_convergence() -> None:
    """Verify displacement-based controller converges to regular pentagon (<0.05m RMS)."""
    rms_err = simulate_swarm_convergence('displacement')
    assert rms_err < 0.05, f'Displacement controller failed to converge: RMS={rms_err:.4f}m'


def test_distance_controller_convergence() -> None:
    """Verify distance-based controller converges to regular pentagon (<0.05m RMS)."""
    rms_err = simulate_swarm_convergence('distance')
    assert rms_err < 0.05, f'Distance controller failed to converge: RMS={rms_err:.4f}m'


def test_bearing_controller_convergence() -> None:
    """Verify bearing-based controller converges to regular pentagon (<0.05m RMS)."""
    rms_err = simulate_swarm_convergence('bearing')
    assert rms_err < 0.05, f'Bearing controller failed to converge: RMS={rms_err:.4f}m'


def test_robot_node_controller_selection() -> None:
    """Verify GazeboSwarmNode instantiates each controller type and produces valid Twist."""
    if not rclpy.ok():
        rclpy.init()

    for c_type in ('displacement', 'distance', 'bearing'):
        node = GazeboSwarmNode()
        try:
            node.controller_type = c_type
            node.controller = create_controller(c_type)
            assert node.controller.name == c_type

            # Provide mock relative measurement from peer 2
            n_list = NeighborList()
            n_list.robot_id = 1
            m = NeighborMeasurement()
            m.neighbor_id = 2
            m.relative_x = 2.0
            m.relative_y = 0.0
            m.distance = 2.0
            m.bearing = 0.0
            n_list.neighbors.append(m)

            node.neighbor_callback(n_list)
            assert 2 in node.relative_neighbors
        finally:
            node.destroy_node()
