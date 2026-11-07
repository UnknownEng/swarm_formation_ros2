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

"""Unit tests for Control Barrier Function Quadratic Programming (CBF-QP) library."""

import math
import random

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


def test_cbf_qp_returns_feasible_point_whenever_one_exists() -> None:
    """Randomized check: the exact solver never violates constraints that are satisfiable."""
    rng = random.Random(0)
    v_max = 1.4
    grid = [
        (x * v_max / 40.0, y * v_max / 40.0)
        for x in range(-40, 41)
        for y in range(-40, 41)
        if (x * x + y * y) <= 1600
    ]
    for _ in range(400):
        peers = {
            k: (rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5))
            for k in range(rng.randint(2, 4))
        }
        constraints = build_peer_cbf_constraints(peers)
        u_des = (rng.uniform(-1.4, 1.4), rng.uniform(-1.4, 1.4))
        feasible = [
            g for g in grid
            if all(c.a_x * g[0] + c.a_y * g[1] <= c.b for c in constraints)
        ]
        if not feasible:
            continue
        ux, uy, slack = solve_2d_cbf_qp(u_des, constraints, max_speed=v_max)
        worst = max(c.a_x * ux + c.a_y * uy - c.b for c in constraints)
        assert worst <= 1e-5, f'violation {worst} although a feasible point exists'
        assert slack == 0.0
        # Optimality: no feasible grid point is meaningfully closer to u_des
        u0 = u_des
        sp = math.hypot(*u0)
        if sp > v_max:
            u0 = (u0[0] * v_max / sp, u0[1] * v_max / sp)
        best_grid = min((g[0] - u0[0]) ** 2 + (g[1] - u0[1]) ** 2 for g in feasible)
        cost = (ux - u0[0]) ** 2 + (uy - u0[1]) ** 2
        assert cost <= best_grid + 1e-9


def test_lookahead_twist_satisfies_barrier_for_executed_motion() -> None:
    """The (v, w) actually executed must satisfy the barrier on the look-ahead point."""
    rng = random.Random(1)
    ell = 0.15
    for _ in range(300):
        yaw = rng.uniform(-math.pi, math.pi)
        peers = {k: (rng.uniform(-1.8, 1.8), rng.uniform(-1.8, 1.8)) for k in range(3)}
        if any(math.hypot(*p) < 0.95 for p in peers.values()):
            continue  # start outside every barrier
        v, w, _ = compute_cbf_safe_twist(
            v_raw=rng.uniform(0.0, 1.4), w_raw=rng.uniform(-3.0, 3.0), robot_yaw=yaw,
            relative_peers=peers, relative_obstacles=[], delay_bound_sec=0.0,
            lookahead_dist=ell, deadlock_bias=0.0,
        )
        qdot = (v * math.cos(yaw) - ell * w * math.sin(yaw),
                v * math.sin(yaw) + ell * w * math.cos(yaw))
        q_from = {
            k: (dx - ell * math.cos(yaw), dy - ell * math.sin(yaw))
            for k, (dx, dy) in peers.items()
        }
        for c in build_peer_cbf_constraints(q_from, safety_distance=0.55 + ell,
                                            delay_bound_sec=0.0):
            assert c.a_x * qdot[0] + c.a_y * qdot[1] <= c.b + 1e-6
        assert abs(v) <= 1.4 + 1e-9 and abs(w) <= 3.0 + 1e-9


def test_symmetric_head_on_does_not_deadlock() -> None:
    """Two robots swapping places head-on must pass each other safely."""
    p = [[-1.5, 0.0, 0.0], [1.5, 0.0, math.pi]]
    goal = [(1.5, 0.0), (-1.5, 0.0)]
    dt = 0.02
    min_dist = float('inf')
    for _ in range(1000):
        cmds = []
        for i in range(2):
            j = 1 - i
            gx, gy = goal[i][0] - p[i][0], goal[i][1] - p[i][1]
            heading_err = math.atan2(gy, gx) - p[i][2]
            heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))
            v_des = min(1.0, math.hypot(gx, gy)) * max(0.0, math.cos(heading_err))
            w_des = max(-3.0, min(3.0, 4.0 * heading_err))
            cmds.append(compute_cbf_safe_twist(
                v_raw=v_des, w_raw=w_des, robot_yaw=p[i][2],
                relative_peers={j + 1: (p[j][0] - p[i][0], p[j][1] - p[i][1])},
                relative_obstacles=[], delay_bound_sec=0.0,
            ))
        for i in range(2):
            v, w, _ = cmds[i]
            p[i][0] += v * math.cos(p[i][2]) * dt
            p[i][1] += v * math.sin(p[i][2]) * dt
            p[i][2] += w * dt
        min_dist = min(min_dist, math.hypot(p[0][0] - p[1][0], p[0][1] - p[1][1]))
    assert min_dist >= 0.50
    for i in range(2):
        assert math.hypot(goal[i][0] - p[i][0], goal[i][1] - p[i][1]) < 0.2, 'deadlocked'


def test_pure_rotation_passes_even_next_to_an_obstacle() -> None:
    """Turning in place cannot reduce any distance, so the filter must not block it."""
    v, w, interv = compute_cbf_safe_twist(
        v_raw=0.0, w_raw=3.0, robot_yaw=0.0,
        relative_peers={2: (0.0, 0.70)},           # peer close beside the robot
        relative_obstacles=[(0.45, 0.0, 0.0)],     # obstacle surface just ahead
    )
    assert v == 0.0 and w == 3.0 and interv == 0.0
    # Forward motion in the same situation is still filtered
    v_f, _, interv_f = compute_cbf_safe_twist(
        v_raw=1.0, w_raw=0.0, robot_yaw=0.0,
        relative_peers={2: (0.0, 0.70)}, relative_obstacles=[(0.45, 0.0, 0.0)],
    )
    assert interv_f > 0.0 and v_f < 1.0
