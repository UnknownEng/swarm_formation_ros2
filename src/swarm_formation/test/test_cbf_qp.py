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

"""Unit tests for Control Barrier Function Quadratic Programming (CBF-QP) library."""

import math

from swarm_formation.cbf_qp import (
    build_obstacle_cbf_constraints,
    build_peer_cbf_constraints,
    compute_cbf_safe_twist,
    solve_2d_cbf_qp,
)


def test_cbf_peer_constraints_delay_expansion() -> None:
    """Verify that communication latency bound expands effective barrier safety radius."""
    relative_peers = {2: (1.0, 0.0)}

    # Without delay: d_eff = 0.55m
    c_no_delay = build_peer_cbf_constraints(
        relative_peers=relative_peers,
        safety_distance=0.55,
        delay_bound_sec=0.0,
        max_linear_vel=1.4,
    )
    # With delay 50ms: d_eff = 0.55 + 2 * 1.4 * 0.05 = 0.69m
    c_with_delay = build_peer_cbf_constraints(
        relative_peers=relative_peers,
        safety_distance=0.55,
        delay_bound_sec=0.05,
        max_linear_vel=1.4,
    )

    assert len(c_no_delay) == 1
    assert len(c_with_delay) == 1

    # With delay, constraint bound b is smaller (tighter constraint on approach velocity)
    assert c_with_delay[0].b < c_no_delay[0].b


def test_cbf_obstacle_constraints() -> None:
    """Verify obstacle barrier constraints for static obstacles."""
    relative_obstacles = [(2.0, 0.0, 0.20)]
    constraints = build_obstacle_cbf_constraints(
        relative_obstacles=relative_obstacles,
        robot_radius=0.20,
        safety_margin=0.15,
        delay_bound_sec=0.0,
        max_linear_vel=1.4,
    )
    assert len(constraints) == 1
    c = constraints[0]
    assert c.entity_type == 'obstacle'
    assert math.isclose(c.a_x, 2.0, abs_tol=1e-5)
    assert math.isclose(c.a_y, 0.0, abs_tol=1e-5)
    assert c.b > 0.0  # Outside obstacle, barrier value is positive


def test_cbf_qp_unconstrained_pass_through() -> None:
    """Verify that when no constraints are violated, desired velocity is preserved unmodified."""
    u_des = (0.5, 0.3)
    # Peer is far away at (5.0, 5.0)
    constraints = build_peer_cbf_constraints(
        relative_peers={2: (5.0, 5.0)},
        safety_distance=0.55,
        delay_bound_sec=0.0,
    )

    safe_ux, safe_uy, slack = solve_2d_cbf_qp(u_des, constraints, max_speed=1.4)
    assert math.isclose(safe_ux, 0.5, abs_tol=1e-4)
    assert math.isclose(safe_uy, 0.3, abs_tol=1e-4)
    assert math.isclose(slack, 0.0, abs_tol=1e-6)


def test_cbf_qp_head_on_deflection() -> None:
    """Verify that commanding robot directly toward a nearby peer is safely deflected."""
    # Peer directly ahead along +X at 0.70m (safety distance 0.55m)
    # Commanded speed u_des = (+1.0, 0.0) directly into peer
    u_des = (1.0, 0.0)
    constraints = build_peer_cbf_constraints(
        relative_peers={2: (0.70, 0.0)},
        safety_distance=0.55,
        gamma=1.5,
        delay_bound_sec=0.0,
    )

    safe_ux, safe_uy, _ = solve_2d_cbf_qp(u_des, constraints, max_speed=1.4)

    # Approach component along +X must be reduced below barrier limit b / a_x
    limit_x = constraints[0].b / constraints[0].a_x
    assert safe_ux <= limit_x + 1e-4
    # Total speed cannot exceed requested speed
    assert math.hypot(safe_ux, safe_uy) <= 1.0 + 1e-4


def test_cbf_qp_infeasible_slack_relaxation() -> None:
    """Verify QP solves smoothly via positive slack when initialized inside safety radius."""
    # Robot starts inside safety distance at 0.40m (< 0.55m safety threshold)
    u_des = (0.5, 0.0)
    constraints = build_peer_cbf_constraints(
        relative_peers={2: (0.40, 0.0)},
        safety_distance=0.55,
        gamma=1.5,
        delay_bound_sec=0.0,
    )

    safe_ux, safe_uy, slack = solve_2d_cbf_qp(u_des, constraints, max_speed=1.4)

    # Must find a bounded solution without exception
    assert isinstance(safe_ux, float)
    assert isinstance(safe_uy, float)
    # Velocity directed toward peer must not push deeper
    assert safe_ux <= 0.01


def test_compute_cbf_safe_twist() -> None:
    """Verify complete pipeline converting raw unicycle twist to safe filtered unicycle twist."""
    v_raw = 1.0
    w_raw = 0.0
    robot_yaw = 0.0  # Heading directly along +X

    # Peer at (0.65, 0.0) directly in front of robot
    relative_peers = {2: (0.65, 0.0)}
    relative_obstacles = []

    safe_v, safe_w, intervention = compute_cbf_safe_twist(
        v_raw=v_raw,
        w_raw=w_raw,
        robot_yaw=robot_yaw,
        relative_peers=relative_peers,
        relative_obstacles=relative_obstacles,
        safety_dist_peer=0.55,
        gamma_peer=1.5,
        delay_bound_sec=0.02,
        max_linear_vel=1.4,
        max_angular_vel=3.0,
    )

    assert intervention > 0.0, 'Safety filter should have intervened'
    assert safe_v < v_raw, 'Linear velocity should have been restricted'
