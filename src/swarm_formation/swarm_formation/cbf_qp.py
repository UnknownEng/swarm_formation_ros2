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

"""
Control Barrier Function (CBF) with Quadratic Program (QP) minimal-correction safety filter.

Provides active safety filtering for multi-robot swarms under finite sensing range,
kinematic speed limits, and communication delay bounds.

Assumptions:
1. Relative sensing range is bounded (e.g. R_sense = 6.0m).
2. Robot translational speed is bounded (||u|| <= v_max).
3. Network delay is bounded by tau_max (compensated via augmented safety margins).
4. System is empirically collision-free; not an unconditional theoretical guarantee
   against arbitrary unmodeled dynamic disturbances or sensor dropouts.
"""

from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Tuple

from .kinematics import compute_unicycle_twist


@dataclass
class CBFConstraint:
    """Represents a linear inequality barrier constraint: a_x * u_x + a_y * u_y <= b."""

    a_x: float
    a_y: float
    b: float
    barrier_value: float
    entity_id: int
    entity_type: str  # 'peer' or 'obstacle'


def build_peer_cbf_constraints(
    relative_peers: Dict[int, Tuple[float, float]],
    safety_distance: float = 0.55,
    gamma: float = 1.5,
    delay_bound_sec: float = 0.05,
    max_linear_vel: float = 1.4,
) -> List[CBFConstraint]:
    """
    Construct reciprocal Control Barrier Function constraints for inter-robot collision avoidance.

    Barrier function: h_ij = ||p_ij||^2 - (D_s^eff)^2 >= 0
    Time derivative under reciprocal avoidance:
    dot{h}_ij = 2 p_ij^T (u_j - u_i) >= -gamma * h_ij
    Assuming symmetric reciprocal effort: p_ij^T u_i <= (gamma / 4) * (||p_ij||^2 - (D_s^eff)^2)

    :param relative_peers: Dict of peer_id -> (dx, dy) in reference frame.
    :param safety_distance: Minimum allowable inter-center distance in meters.
    :param gamma: Extended class-K gain parameter controlling barrier decay rate.
    :param delay_bound_sec: Maximum latency bound tau_max for margin expansion.
    :param max_linear_vel: Maximum robot translation speed for delay margin expansion.
    :return: List of CBFConstraint instances.
    """
    constraints: List[CBFConstraint] = []
    # Delay compensation: augment safety radius by worst-case mutual displacement during delay
    d_eff = safety_distance + 2.0 * max_linear_vel * delay_bound_sec
    d_eff_sq = d_eff * d_eff

    for peer_id, (dx, dy) in relative_peers.items():
        dist_sq = dx * dx + dy * dy
        dist = math.sqrt(dist_sq)

        if dist < 1e-4:
            continue

        h = dist_sq - d_eff_sq
        # p_ij^T u_i <= (gamma / 4) * h
        a_x = dx
        a_y = dy
        b = (gamma * 0.25) * h

        constraints.append(
            CBFConstraint(
                a_x=a_x,
                a_y=a_y,
                b=b,
                barrier_value=h,
                entity_id=peer_id,
                entity_type='peer',
            )
        )

    return constraints


def build_obstacle_cbf_constraints(
    relative_obstacles: List[Tuple[float, float, float]],
    robot_radius: float = 0.20,
    safety_margin: float = 0.15,
    gamma_obs: float = 2.0,
    delay_bound_sec: float = 0.05,
    max_linear_vel: float = 1.4,
) -> List[CBFConstraint]:
    """
    Construct Control Barrier Function constraints for static obstacle collision avoidance.

    Barrier function: h_ik = ||p_ik||^2 - (D_obs^eff)^2 >= 0
    Static obstacle derivative: dot{h}_ik = -2 p_ik^T u_i >= -gamma_obs * h_ik
    Constraint: p_ik^T u_i <= (gamma_obs / 2) * (||p_ik||^2 - (D_obs^eff)^2)

    :param relative_obstacles: List of (ox, oy, radius) in reference frame.
    :param robot_radius: Physical bounding radius of robot in meters.
    :param safety_margin: Buffer distance in meters.
    :param gamma_obs: Class-K gain for obstacle barrier decay rate.
    :param delay_bound_sec: Maximum latency bound tau_max.
    :param max_linear_vel: Maximum translation speed.
    :return: List of CBFConstraint instances.
    """
    constraints: List[CBFConstraint] = []

    for obs_idx, (ox, oy, orad) in enumerate(relative_obstacles, start=1):
        dist_sq = ox * ox + oy * oy
        dist = math.sqrt(dist_sq)

        if dist < 1e-4:
            continue

        d_eff = (orad + robot_radius + safety_margin) + max_linear_vel * delay_bound_sec
        d_eff_sq = d_eff * d_eff

        h = dist_sq - d_eff_sq
        a_x = ox
        a_y = oy
        b = (gamma_obs * 0.5) * h

        constraints.append(
            CBFConstraint(
                a_x=a_x,
                a_y=a_y,
                b=b,
                barrier_value=h,
                entity_id=obs_idx,
                entity_type='obstacle',
            )
        )

    return constraints


def solve_2d_cbf_qp(
    u_des: Tuple[float, float],
    constraints: List[CBFConstraint],
    max_speed: float = 1.4,
) -> Tuple[float, float, float]:
    """
    Solve 2D Quadratic Program finding minimal-correction velocity subject to CBF constraints.

    min_{u in R^2, delta >= 0} 1/2 ||u - u_des||^2 + (W_slack / 2) * delta^2
    subject to:
      a_k^T u <= b_k + delta,  forall k
      ||u|| <= max_speed

    :param u_des: Desired nominal velocity vector (u_x, u_y).
    :param constraints: List of active linear CBF constraints.
    :param max_speed: Kinematic velocity saturation limit.
    :return: Tuple of (safe_u_x, safe_u_y, slack_delta).
    """
    u0_x, u0_y = u_des

    # Clip desired input to maximum speed
    speed0 = math.hypot(u0_x, u0_y)
    if speed0 > max_speed:
        u0_x = (u0_x / speed0) * max_speed
        u0_y = (u0_y / speed0) * max_speed

    if not constraints:
        return u0_x, u0_y, 0.0

    # 1. Check if unconstrained candidate u0 satisfies all constraints
    is_u0_feasible = True
    for c in constraints:
        val = c.a_x * u0_x + c.a_y * u0_y
        if val > c.b + 1e-5:
            is_u0_feasible = False
            break

    if is_u0_feasible:
        return u0_x, u0_y, 0.0

    # 2. Candidate collection: solutions on constraint boundary lines
    # Candidate point = (u_x, u_y, cost, slack_violation)
    candidates: List[Tuple[float, float, float, float]] = []

    # Projection of u0 onto single constraint line a_i^T u = b_i
    for c in constraints:
        norm_a_sq = c.a_x * c.a_x + c.a_y * c.a_y
        if norm_a_sq < 1e-8:
            continue

        dot = c.a_x * u0_x + c.a_y * u0_y
        diff = dot - c.b
        # Orthogonal projection onto hyperplane
        proj_x = u0_x - (diff / norm_a_sq) * c.a_x
        proj_y = u0_y - (diff / norm_a_sq) * c.a_y

        # Clip speed if exceeds maximum
        sp = math.hypot(proj_x, proj_y)
        if sp > max_speed:
            proj_x = (proj_x / sp) * max_speed
            proj_y = (proj_y / sp) * max_speed

        # Check maximum violation across all constraints
        max_viol = 0.0
        for other in constraints:
            viol = other.a_x * proj_x + other.a_y * proj_y - other.b
            if viol > max_viol:
                max_viol = viol

        cost = (proj_x - u0_x) ** 2 + (proj_y - u0_y) ** 2
        candidates.append((proj_x, proj_y, cost, max_viol))

    # Intersection of pairs of constraint lines: a_i^T u = b_i and a_j^T u = b_j
    m = len(constraints)
    for i in range(m):
        ci = constraints[i]
        for j in range(i + 1, m):
            cj = constraints[j]
            det = ci.a_x * cj.a_y - ci.a_y * cj.a_x
            if abs(det) < 1e-6:
                continue

            ix_x = (ci.b * cj.a_y - ci.a_y * cj.b) / det
            ix_y = (ci.a_x * cj.b - ci.b * cj.a_x) / det

            sp = math.hypot(ix_x, ix_y)
            if sp > max_speed:
                ix_x = (ix_x / sp) * max_speed
                ix_y = (ix_y / sp) * max_speed

            max_viol = 0.0
            for other in constraints:
                viol = other.a_x * ix_x + other.a_y * ix_y - other.b
                if viol > max_viol:
                    max_viol = viol

            cost = (ix_x - u0_x) ** 2 + (ix_y - u0_y) ** 2
            candidates.append((ix_x, ix_y, cost, max_viol))

    # Zero velocity fallback (stopping)
    max_viol_zero = 0.0
    for other in constraints:
        viol = -other.b
        if viol > max_viol_zero:
            max_viol_zero = viol
    cost_zero = u0_x * u0_x + u0_y * u0_y
    candidates.append((0.0, 0.0, cost_zero, max_viol_zero))

    # 3. Select optimal candidate:
    # First priority: minimize constraint violation (slack delta)
    # Second priority: minimize deviation from desired control
    w_slack = 1000.0
    best_candidate: Optional[Tuple[float, float, float]] = None
    best_total_cost = float('inf')

    for cand_x, cand_y, cost, viol in candidates:
        slack = max(0.0, viol)
        total_cost = cost + w_slack * (slack ** 2)
        if total_cost < best_total_cost:
            best_total_cost = total_cost
            best_candidate = (cand_x, cand_y, slack)

    if best_candidate is not None:
        return best_candidate

    return 0.0, 0.0, 0.0


def compute_cbf_safe_twist(
    v_raw: float,
    w_raw: float,
    robot_yaw: float,
    relative_peers: Dict[int, Tuple[float, float]],
    relative_obstacles: List[Tuple[float, float, float]],
    safety_dist_peer: float = 0.55,
    safety_dist_obs: float = 0.35,
    gamma_peer: float = 1.5,
    gamma_obs: float = 2.0,
    delay_bound_sec: float = 0.05,
    max_linear_vel: float = 1.4,
    max_angular_vel: float = 3.0,
    yaw_gain_kyaw: float = 4.0,
) -> Tuple[float, float, float]:
    """
    Filter unicycle velocity command using Control Barrier Function Quadratic Program.

    :param v_raw: Raw commanded forward linear speed from controller.
    :param w_raw: Raw commanded angular yaw velocity from controller.
    :param robot_yaw: Current estimated robot heading in radians.
    :param relative_peers: Dict of peer_id -> (dx, dy) relative displacements.
    :param relative_obstacles: List of (ox, oy, radius) obstacle detections.
    :param safety_dist_peer: Minimum peer distance threshold in meters.
    :param safety_dist_obs: Minimum obstacle distance threshold in meters.
    :param gamma_peer: Barrier convergence rate for peers.
    :param gamma_obs: Barrier convergence rate for obstacles.
    :param delay_bound_sec: Communication delay bound for margin expansion.
    :param max_linear_vel: Actuator forward speed limit.
    :param max_angular_vel: Actuator yaw velocity limit.
    :param yaw_gain_kyaw: Kinematic unicycle orientation tracking gain.
    :return: Tuple of (safe_v, safe_w, intervention_norm).
    """
    # 1. Convert unicycle twist to 2D reference frame translation vector
    u0_x = v_raw * math.cos(robot_yaw)
    u0_y = v_raw * math.sin(robot_yaw)

    # 2. Build barrier constraints
    peer_constraints = build_peer_cbf_constraints(
        relative_peers=relative_peers,
        safety_distance=safety_dist_peer,
        gamma=gamma_peer,
        delay_bound_sec=delay_bound_sec,
        max_linear_vel=max_linear_vel,
    )
    obs_constraints = build_obstacle_cbf_constraints(
        relative_obstacles=relative_obstacles,
        robot_radius=0.20,
        safety_margin=max(0.0, safety_dist_obs - 0.20),
        gamma_obs=gamma_obs,
        delay_bound_sec=delay_bound_sec,
        max_linear_vel=max_linear_vel,
    )
    all_constraints = peer_constraints + obs_constraints

    # 3. Solve 2D CBF-QP
    safe_ux, safe_uy, _ = solve_2d_cbf_qp(
        u_des=(u0_x, u0_y),
        constraints=all_constraints,
        max_speed=max_linear_vel,
    )

    # 4. Measure filter intervention magnitude
    intervention_norm = math.hypot(safe_ux - u0_x, safe_uy - u0_y)

    # 5. Map filtered translation vector back to unicycle twist
    # If no intervention occurred and raw inputs were valid, preserve raw inputs
    if intervention_norm < 1e-4:
        return v_raw, w_raw, 0.0

    safe_v, safe_w = compute_unicycle_twist(
        fx=safe_ux,
        fy=safe_uy,
        current_yaw=robot_yaw,
        k_yaw=yaw_gain_kyaw,
        max_lin_vel=max_linear_vel,
        max_ang_vel=max_angular_vel,
    )

    return safe_v, safe_w, intervention_norm
