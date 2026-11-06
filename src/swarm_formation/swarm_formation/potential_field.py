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

"""Artificial Potential Field (APF) safety barrier modules for collision avoidance."""

import math
from typing import Dict, List, Tuple


def compute_peer_apf(
    pos_x: float,
    pos_y: float,
    vel_x: float,
    vel_y: float,
    filtered_states: Dict[int, Tuple[float, float, float, float]],
    d_infl: float = 1.80,
    d_safe: float = 1.10,
    d_collision: float = 0.50,
    k_rep: float = 1.2,
    k_barrier: float = 2.0,
    k_damp: float = 1.5,
    lateral_ratio: float = 0.6,
) -> Tuple[float, float, float]:
    """
    Compute Layer 1 inter-robot repulsive force with anti-symmetric lateral deflection.

    :param pos_x: Robot current x position.
    :param pos_y: Robot current y position.
    :param vel_x: Robot current x velocity.
    :param vel_y: Robot current y velocity.
    :param filtered_states: Filtered peer states (x, y, vx, vy).
    :param d_infl: Proximity sphere of influence distance (m).
    :param d_safe: Critical safe distance barrier threshold (m).
    :param d_collision: Hard physical collision contact limit (m).
    :param k_rep: Potential field repulsive gain.
    :param k_barrier: Hard barrier gain for dangerous proximity.
    :param k_damp: Dynamic closing velocity damping gain.
    :param lateral_ratio: Anti-symmetric rule-of-the-road deflection ratio.
    :return: Tuple of (fx_apf, fy_apf, min_distance_to_peers).
    """
    fx_apf, fy_apf = 0.0, 0.0
    min_dist = float('inf')

    for n_id, (filt_x, filt_y, filt_vx, filt_vy) in filtered_states.items():
        dist = math.hypot(pos_x - filt_x, pos_y - filt_y)
        if dist < min_dist:
            min_dist = dist

        if 0.001 < dist < d_infl:
            nx_hat = (pos_x - filt_x) / dist
            ny_hat = (pos_y - filt_y) / dist

            # 1. Repulsive potential field force
            f_rep_mag = k_rep * (1.0 / dist - 1.0 / d_infl) * (1.0 / (dist * dist))

            # 2. Hard barrier certificate when entering dangerous proximity
            f_barrier_mag = 0.0
            if dist < d_safe:
                denom = max(dist - d_collision, 0.01)
                f_barrier_mag = k_barrier * (1.0 / (denom * denom))

            # 3. Dynamic closing-velocity damping
            rel_vx = vel_x - filt_vx
            rel_vy = vel_y - filt_vy
            closing_vel = -(rel_vx * nx_hat + rel_vy * ny_hat)
            f_damp_mag = 0.0
            if closing_vel > 0.0:
                f_damp_mag = k_damp * (closing_vel / dist)

            total_rep = min(f_rep_mag + f_barrier_mag + f_damp_mag, 50.0)
            tx_peer, ty_peer = -ny_hat, nx_hat
            f_peer_lateral = lateral_ratio * total_rep

            fx_apf += total_rep * nx_hat + f_peer_lateral * tx_peer
            fy_apf += total_rep * ny_hat + f_peer_lateral * ty_peer

    return fx_apf, fy_apf, min_dist


def compute_obstacle_apf(
    pos_x: float,
    pos_y: float,
    vel_x: float,
    vel_y: float,
    fx_guide: float,
    fy_guide: float,
    obstacles: List[Tuple[float, float, float]],
    d_obs_infl: float = 0.45,
    d_obs_safe: float = 0.30,
    k_obs_rep: float = 0.6,
    k_obs_barrier: float = 0.8,
    k_obs_damp: float = 0.8,
    tangent_ratio: float = 1.0,
) -> Tuple[float, float]:
    """
    Compute Layer 2 environmental obstacle repulsive and tangential circulation forces.

    :param pos_x: Robot current x position.
    :param pos_y: Robot current y position.
    :param vel_x: Robot current x velocity.
    :param vel_y: Robot current y velocity.
    :param fx_guide: Formation driving force in x used to resolve circulation direction.
    :param fy_guide: Formation driving force in y used to resolve circulation direction.
    :param obstacles: List of obstacles as (x, y, radius).
    :param d_obs_infl: Obstacle influence clearance (m).
    :param d_obs_safe: Obstacle safe barrier clearance (m).
    :param k_obs_rep: Obstacle repulsive potential gain.
    :param k_obs_barrier: Obstacle hard barrier gain.
    :param k_obs_damp: Dynamic closing velocity damping towards obstacle.
    :param tangent_ratio: Tangential circulation force ratio.
    :return: Planar obstacle avoidance force vector (fx_obs, fy_obs).
    """
    fx_obs, fy_obs = 0.0, 0.0

    for ox, oy, orad in obstacles:
        dist_center = math.hypot(pos_x - ox, pos_y - oy)
        clearance = max(dist_center - orad, 1e-4)

        if clearance < d_obs_infl and dist_center > 1e-4:
            obs_nx = (pos_x - ox) / dist_center
            obs_ny = (pos_y - oy) / dist_center

            # Tangent vector to glide smoothly around obstacle without local minima traps
            tx, ty = -obs_ny, obs_nx
            if (fx_guide * tx + fy_guide * ty) < 0.0:
                tx, ty = -tx, -ty

            # 1. Repulsive obstacle potential field force
            inv_c = 1.0 / clearance
            inv_infl = 1.0 / d_obs_infl
            f_obs_rep = k_obs_rep * (inv_c - inv_infl) * (inv_c**2)

            # 2. Hard barrier certificate when approaching obstacle
            f_obs_barrier = 0.0
            if clearance < d_obs_safe:
                denom = max(clearance - 0.20, 0.01)
                f_obs_barrier = k_obs_barrier * (1.0 / (denom * denom))

            # 3. Dynamic closing velocity damping towards static obstacle
            closing_vel_obs = -(vel_x * obs_nx + vel_y * obs_ny)
            f_obs_damp = 0.0
            if closing_vel_obs > 0.0:
                f_obs_damp = k_obs_damp * (closing_vel_obs / clearance)

            total_normal = min(f_obs_rep + f_obs_barrier + f_obs_damp, 12.0)
            f_tangent = tangent_ratio * total_normal

            fx_obs += total_normal * obs_nx + f_tangent * tx
            fy_obs += total_normal * obs_ny + f_tangent * ty

    return fx_obs, fy_obs


def compute_peer_safety_scale(
    min_dist: float, d_safe: float = 1.10, d_collision: float = 0.50
) -> float:
    """
    Compute priority attenuation scale for formation force when peers are close.

    :param min_dist: Minimum distance to any peer robot.
    :param d_safe: Critical safe distance barrier threshold.
    :param d_collision: Contact collision limit.
    :return: Attenuation factor in [0.08, 1.0].
    """
    if min_dist < d_safe:
        return max(0.08, (min_dist - d_collision) / (d_safe - d_collision))
    return 1.0
