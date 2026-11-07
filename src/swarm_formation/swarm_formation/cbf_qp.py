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
    responsibility: float = 0.5,
) -> List[CBFConstraint]:
    """
    Construct reciprocal Control Barrier Function constraints for inter-robot collision avoidance.

    Barrier function: h_ij = ||p_ij||^2 - (D_s^eff)^2 >= 0
    Time derivative: dot{h}_ij = 2 p_ij^T (u_j - u_i) >= -gamma * h_ij.
    Robot i takes a share r of the required effort: p_ij^T u_i <= (r * gamma / 2) * h_ij.
    With r = 0.5 (reciprocal, default) the two constraints add up to the barrier
    condition; against a peer that does not move, i's half alone is already stricter than
    needed. r = 1 uses the whole budget and is only valid against a passive peer. Neither
    covers a peer that cannot see i (one-way link) and drives toward it; that would need
    i to know whether j hears it. Measured on asymmetric links: README Section 6.4.

    :param relative_peers: Dict of peer_id -> (dx, dy) in reference frame.
    :param safety_distance: Minimum allowable inter-center distance in meters.
    :param gamma: Extended class-K gain parameter controlling barrier decay rate.
    :param delay_bound_sec: Maximum latency bound tau_max for margin expansion.
    :param max_linear_vel: Maximum robot translation speed for delay margin expansion.
    :param responsibility: Share r of the barrier effort this robot takes (0.5 or 1).
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
        # p_ij^T u_i <= (r * gamma / 2) * h
        a_x = dx
        a_y = dy
        b = (gamma * 0.5 * responsibility) * h

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


def _max_violation(u_x: float, u_y: float, constraints: List[CBFConstraint]) -> float:
    """Return the largest constraint violation a^T u - b (0.0 if all are satisfied)."""
    worst = 0.0
    for c in constraints:
        viol = c.a_x * u_x + c.a_y * u_y - c.b
        if viol > worst:
            worst = viol
    return worst


def solve_2d_cbf_qp(
    u_des: Tuple[float, float],
    constraints: List[CBFConstraint],
    max_speed: float = 1.4,
    feas_tol: float = 1e-6,
) -> Tuple[float, float, float]:
    """
    Solve the 2D minimal-correction QP exactly by active-set enumeration.

    min_u 1/2 ||u - u_des||^2
    subject to a_k^T u <= b_k for all k, and ||u|| <= max_speed.

    In 2D the projection onto (polygon intersected with a disk) lies at u_des itself,
    on one constraint line, at a vertex of two lines, or where a line meets the speed
    circle. All such candidates are enumerated and the cheapest feasible one is
    returned. Only when no feasible point exists (e.g. the robot already starts inside
    a barrier, or crowding robots press it against a wall) is the least-violating
    candidate returned, with its violation as slack. Obstacle constraints take priority:
    a static obstacle cannot make room, a peer can, so the candidate with the smallest
    obstacle violation wins and peer violation only breaks ties.

    :param u_des: Desired nominal velocity vector (u_x, u_y).
    :param constraints: List of linear CBF constraints.
    :param max_speed: Kinematic velocity saturation limit.
    :param feas_tol: Tolerance used to accept a candidate as feasible.
    :return: Tuple of (safe_u_x, safe_u_y, slack) where slack is the residual violation.
    """
    u0_x, u0_y = u_des
    speed0 = math.hypot(u0_x, u0_y)
    if speed0 > max_speed:
        u0_x = (u0_x / speed0) * max_speed
        u0_y = (u0_y / speed0) * max_speed

    if not constraints or _max_violation(u0_x, u0_y, constraints) <= feas_tol:
        return u0_x, u0_y, 0.0

    r_sq = max_speed * max_speed + 1e-9
    candidates: List[Tuple[float, float]] = [(0.0, 0.0)]

    for c in constraints:
        norm_a_sq = c.a_x * c.a_x + c.a_y * c.a_y
        if norm_a_sq < 1e-12:
            continue
        # Orthogonal projection of u0 onto the line a^T u = b
        diff = (c.a_x * u0_x + c.a_y * u0_y - c.b) / norm_a_sq
        px, py = u0_x - diff * c.a_x, u0_y - diff * c.a_y
        if px * px + py * py <= r_sq:
            candidates.append((px, py))
        # Intersections of the line with the speed circle ||u|| = max_speed
        foot_x, foot_y = c.b * c.a_x / norm_a_sq, c.b * c.a_y / norm_a_sq
        foot_sq = foot_x * foot_x + foot_y * foot_y
        if foot_sq <= r_sq:
            half_chord = math.sqrt(max(0.0, max_speed * max_speed - foot_sq))
            inv_norm = 1.0 / math.sqrt(norm_a_sq)
            tx, ty = -c.a_y * inv_norm, c.a_x * inv_norm
            candidates.append((foot_x + half_chord * tx, foot_y + half_chord * ty))
            candidates.append((foot_x - half_chord * tx, foot_y - half_chord * ty))

    m = len(constraints)
    for i in range(m):
        ci = constraints[i]
        for j in range(i + 1, m):
            cj = constraints[j]
            det = ci.a_x * cj.a_y - ci.a_y * cj.a_x
            if abs(det) < 1e-9:
                continue
            ix = (ci.b * cj.a_y - ci.a_y * cj.b) / det
            iy = (ci.a_x * cj.b - ci.b * cj.a_x) / det
            if ix * ix + iy * iy <= r_sq:
                candidates.append((ix, iy))

    best_feasible: Optional[Tuple[float, float]] = None
    best_cost = float('inf')
    obstacle_cons = [c for c in constraints if c.entity_type == 'obstacle']
    other_cons = [c for c in constraints if c.entity_type != 'obstacle']
    least_viol_pt = (0.0, 0.0)
    least_viol = float('inf')
    least_key = (float('inf'), float('inf'), float('inf'))

    for cx, cy in candidates:
        viol = _max_violation(cx, cy, constraints)
        cost = (cx - u0_x) ** 2 + (cy - u0_y) ** 2
        if viol <= feas_tol:
            if cost < best_cost:
                best_cost = cost
                best_feasible = (cx, cy)
            continue
        # Lexicographic: obstacle violation first, then peer violation, then cost
        obs_v = _max_violation(cx, cy, obstacle_cons)
        key = (obs_v if obs_v > feas_tol else 0.0, _max_violation(cx, cy, other_cons), cost)
        if key < least_key:
            least_key = key
            least_viol = viol
            least_viol_pt = (cx, cy)

    if best_feasible is not None:
        return best_feasible[0], best_feasible[1], 0.0
    return least_viol_pt[0], least_viol_pt[1], least_viol


def compute_cbf_safe_twist(
    v_raw: float,
    w_raw: float,
    robot_yaw: float,
    relative_peers: Dict[int, Tuple[float, float]],
    relative_obstacles: List[Tuple[float, float, float]],
    safety_dist_peer: float = 0.55,
    safety_dist_obs: float = 0.30,
    gamma_peer: float = 1.5,
    gamma_obs: float = 2.0,
    delay_bound_sec: float = 0.05,
    max_linear_vel: float = 1.4,
    max_angular_vel: float = 3.0,
    yaw_gain_kyaw: float = 4.0,
    lookahead_dist: float = 0.15,
    deadlock_bias: float = 0.5,
    deadlock_band: float = 0.5,
    peer_responsibility: float = 0.5,
) -> Tuple[float, float, float]:
    """
    Filter a unicycle velocity command with a CBF-QP on a look-ahead point.

    A unicycle cannot move sideways, so a barrier written for a single integrator no
    longer holds once its output is converted back to (v, w). Instead the barrier is
    placed on the look-ahead point q = p + l [cos(yaw), sin(yaw)], whose velocity
    q_dot = R(yaw) diag(1, l) [v, w]^T is fully actuated. The QP result for q_dot is
    mapped back to (v, w) exactly, so the inequality the QP enforces is the one the
    robot executes. Keeping ||p_j - q_i|| >= D + l implies ||p_j - p_i|| >= D.

    :param v_raw: Raw commanded forward linear speed from controller.
    :param w_raw: Raw commanded angular yaw velocity from controller.
    :param robot_yaw: Current estimated robot heading in radians.
    :param relative_peers: Dict of peer_id -> (dx, dy) center-to-center displacements.
    :param relative_obstacles: List of (ox, oy, radius) obstacle detections.
    :param safety_dist_peer: Minimum peer distance threshold in meters.
    :param safety_dist_obs: Minimum obstacle distance threshold in meters.
    :param gamma_peer: Barrier convergence rate for peers.
    :param gamma_obs: Barrier convergence rate for obstacles.
    :param delay_bound_sec: Communication delay bound for margin expansion.
    :param max_linear_vel: Actuator forward speed limit.
    :param max_angular_vel: Actuator yaw velocity limit.
    :param yaw_gain_kyaw: Unused; kept for backward-compatible call signatures.
    :param lookahead_dist: Look-ahead offset l in meters (> 0).
    :param deadlock_bias: Gain of the right-hand-rule tangential bias added to the nominal
        command when it violates a barrier (0 disables). Symmetric multi-robot CBF filters
        are known to deadlock; a common turning direction breaks the symmetry. The bias
        only changes the *nominal* input, so the barrier guarantee is unaffected.
    :param deadlock_band: Distance beyond the effective barrier radius over which the
        bias fades out, in meters.
    :param peer_responsibility: Share of each peer barrier this robot enforces (0.5 =
        reciprocal; 1 = whole budget, valid only against a passive peer).
    :return: Tuple of (safe_v, safe_w, intervention_norm).
    """
    del yaw_gain_kyaw

    # 0. Pure rotation in place cannot move the robot centre, so it cannot reduce any
    #    centre-to-centre or centre-to-obstacle distance. The look-ahead barrier would still
    #    block it (rotating swings the look-ahead point sideways), which in Gazebo left a
    #    robot unable to turn toward a force behind it and stuck for 8+ s. Pass it through.
    if abs(v_raw) < 1e-6:
        return 0.0, max(-max_angular_vel, min(max_angular_vel, w_raw)), 0.0

    ell = max(1e-3, lookahead_dist)
    cos_h = math.cos(robot_yaw)
    sin_h = math.sin(robot_yaw)

    # 1. Look-ahead point velocity produced by the raw command
    u0_x = v_raw * cos_h - ell * w_raw * sin_h
    u0_y = v_raw * sin_h + ell * w_raw * cos_h

    # 2. Barrier constraints written relative to the look-ahead point
    peers_from_q = {
        pid: (dx - ell * cos_h, dy - ell * sin_h) for pid, (dx, dy) in relative_peers.items()
    }
    obstacles_from_q = [
        (ox - ell * cos_h, oy - ell * sin_h, orad) for ox, oy, orad in relative_obstacles
    ]
    peer_constraints = build_peer_cbf_constraints(
        relative_peers=peers_from_q,
        safety_distance=safety_dist_peer + ell,
        gamma=gamma_peer,
        delay_bound_sec=delay_bound_sec,
        max_linear_vel=max_linear_vel,
        responsibility=peer_responsibility,
    )
    obs_constraints = build_obstacle_cbf_constraints(
        relative_obstacles=obstacles_from_q,
        robot_radius=0.20,
        safety_margin=max(0.0, safety_dist_obs - 0.20) + ell,
        gamma_obs=gamma_obs,
        delay_bound_sec=delay_bound_sec,
        max_linear_vel=max_linear_vel,
    )
    all_constraints = peer_constraints + obs_constraints

    # 3. Deadlock breaking: if the nominal command pushes into any barrier, add a
    #    tangential component around nearby *peers* (all robots turn the same way, like a
    #    roundabout). Biasing around static obstacles too was tested and made jams in the
    #    rubble field worse (0/4 vs 3/4 runs converging), so obstacles get no bias.
    nom_x, nom_y = u0_x, u0_y
    speed_nom = math.hypot(u0_x, u0_y)
    if (
        deadlock_bias > 0.0
        and speed_nom > 1e-6
        and all_constraints
        and _max_violation(u0_x, u0_y, all_constraints) > 0.0
    ):
        d_peer = safety_dist_peer + ell + 2.0 * max_linear_vel * delay_bound_sec
        for qx, qy in peers_from_q.values():
            d_eff = d_peer
            dist = math.hypot(qx, qy)
            if dist < 1e-6:
                continue
            weight = (d_eff + deadlock_band - dist) / max(deadlock_band, 1e-6)
            weight = min(1.0, max(0.0, weight))
            if weight <= 0.0:
                continue
            # Unit vector toward the peer rotated -90 deg: every robot veers to its right
            nom_x += deadlock_bias * speed_nom * weight * (qy / dist)
            nom_y += deadlock_bias * speed_nom * weight * (-qx / dist)

    # 4. Exact 2D QP on the look-ahead velocity
    safe_ux, safe_uy, _ = solve_2d_cbf_qp(
        u_des=(nom_x, nom_y),
        constraints=all_constraints,
        max_speed=max_linear_vel,
    )

    intervention_norm = math.hypot(safe_ux - u0_x, safe_uy - u0_y)
    if intervention_norm < 1e-6:
        safe_v, safe_w = v_raw, w_raw
    else:
        # 5. Exact inverse map q_dot -> (v, w); uniform scaling keeps a^T u <= b when b >= 0
        safe_v = cos_h * safe_ux + sin_h * safe_uy
        safe_w = (-sin_h * safe_ux + cos_h * safe_uy) / ell
        scale = 1.0
        if abs(safe_v) > max_linear_vel:
            scale = min(scale, max_linear_vel / abs(safe_v))
        if abs(safe_w) > max_angular_vel:
            scale = min(scale, max_angular_vel / abs(safe_w))
        safe_v *= scale
        safe_w *= scale

    # 6. Centre barrier. The look-ahead barrier alone can be ratcheted: unfiltered
    #    rotations (step 0) swing q into a barrier, and the QP then lets the centre move
    #    inward while q moves out. Measured in the campaign simulator: a robot pressed
    #    against a wall crept ~7 mm per rotate/drive cycle until it crossed it. The centre
    #    moves only along the heading (p_dot = v h), so its barriers bound v alone:
    #    v (a . h) <= b. Clamping v can only slow the robot, never turn it.
    #    It is a backstop: for peers it keeps only the safety distance itself (the delay
    #    margin is the look-ahead barrier's job; with it, robots squeezing past a stopped
    #    robot stalled: 16/25 instead of 21/25 converged in the robot-dropout condition).
    centre_constraints = build_peer_cbf_constraints(
        relative_peers=relative_peers,
        safety_distance=safety_dist_peer,
        gamma=gamma_peer,
        delay_bound_sec=0.0,
        max_linear_vel=max_linear_vel,
        responsibility=peer_responsibility,
    ) + build_obstacle_cbf_constraints(
        relative_obstacles=relative_obstacles,
        robot_radius=0.20,
        safety_margin=max(0.0, safety_dist_obs - 0.20),
        gamma_obs=gamma_obs,
        delay_bound_sec=delay_bound_sec,
        max_linear_vel=max_linear_vel,
    )
    v_hi, v_lo = max_linear_vel, -max_linear_vel
    for c in centre_constraints:
        slope = c.a_x * cos_h + c.a_y * sin_h
        if slope > 1e-9:
            v_hi = min(v_hi, c.b / slope)
        elif slope < -1e-9:
            v_lo = max(v_lo, c.b / slope)
    clamped_v = min(max(safe_v, v_lo), v_hi) if v_lo <= v_hi else 0.0
    if abs(clamped_v - safe_v) > 1e-9:
        intervention_norm += abs(clamped_v - safe_v)
        safe_v = clamped_v

    if intervention_norm < 1e-6:
        return v_raw, w_raw, 0.0
    return safe_v, safe_w, intervention_norm
