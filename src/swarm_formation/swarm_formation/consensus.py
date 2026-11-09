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

"""Consensus algorithms for decentralized polygon formation control."""

import math
from typing import Dict, Tuple


def compute_target_offsets(
    robot_id: int, total_robots: int, radius: float
) -> Dict[int, Tuple[float, float]]:
    """
    Compute relative displacement offsets for a regular polygon formation.

    :param robot_id: 1-based index of this robot.
    :param total_robots: Total number of robots in the polygon.
    :param radius: Circumscribed radius of the regular polygon in meters.
    :return: Mapping from neighbor robot_id to (delta_x, delta_y).
    """
    offsets: Dict[int, Tuple[float, float]] = {}
    my_idx = (robot_id - 1) % total_robots
    my_angle = 2.0 * math.pi * my_idx / total_robots
    my_x = radius * math.cos(my_angle)
    my_y = radius * math.sin(my_angle)

    for other_id in range(1, total_robots + 1):
        if other_id == robot_id:
            continue
        other_idx = (other_id - 1) % total_robots
        other_angle = 2.0 * math.pi * other_idx / total_robots
        other_x = radius * math.cos(other_angle)
        other_y = radius * math.sin(other_angle)
        offsets[other_id] = (other_x - my_x, other_y - my_y)

    return offsets


def compute_baseline_consensus(
    pos_x: float,
    pos_y: float,
    target_offsets: Dict[int, Tuple[float, float]],
    neighbor_states: Dict[int, Tuple[float, float, float, float, float]],
    k_p: float,
) -> Tuple[float, float]:
    """
    Compute standard linear displacement-based consensus drive force.

    :param pos_x: Robot current x position.
    :param pos_y: Robot current y position.
    :param target_offsets: Relative formation offsets to peers.
    :param neighbor_states: States (x, y, vx, vy, time) of peers.
    :param k_p: Proportional consensus gain.
    :return: Planar formation force vector (fx, fy).
    """
    fx, fy = 0.0, 0.0
    for n_id, target_offset in target_offsets.items():
        if n_id not in neighbor_states:
            continue

        nx, ny, _, _, _ = neighbor_states[n_id]
        dx = nx - pos_x
        dy = ny - pos_y

        err_x = dx - target_offset[0]
        err_y = dy - target_offset[1]

        fx += k_p * err_x
        fy += k_p * err_y

    return fx, fy


def compute_huber_consensus(
    pos_x: float,
    pos_y: float,
    target_offsets: Dict[int, Tuple[float, float]],
    filtered_states: Dict[int, Tuple[float, float, float, float]],
    active_mask: Dict[int, bool],
    k_p: float,
    huber_delta: float = 2.0,
    deadband: float = 0.02,
) -> Tuple[float, float, int]:
    """
    Compute robust consensus using Huber loss influence function.

    :param pos_x: Robot current x position.
    :param pos_y: Robot current y position.
    :param target_offsets: Relative formation offsets to peers.
    :param filtered_states: Filtered peer states (x, y, vx, vy).
    :param active_mask: Boolean indicator whether each peer is active.
    :param k_p: Proportional consensus gain.
    :param huber_delta: Huber threshold bounding gradient influence.
    :param deadband: Minimum tracking error threshold.
    :return: Tuple of (fx, fy, active_neighbor_count).
    """
    fx_form, fy_form = 0.0, 0.0
    active_neighbors = 0

    for n_id, target_offset in target_offsets.items():
        if not active_mask.get(n_id, False):
            continue
        if n_id not in filtered_states:
            continue

        active_neighbors += 1
        filt_x, filt_y, _, _ = filtered_states[n_id]
        dx = filt_x - pos_x
        dy = filt_y - pos_y
        err_x = dx - target_offset[0]
        err_y = dy - target_offset[1]
        err_norm = math.hypot(err_x, err_y)

        if err_norm > deadband:
            if err_norm > huber_delta:
                scale = huber_delta / err_norm
                err_x *= scale
                err_y *= scale
            fx_form += k_p * err_x
            fy_form += k_p * err_y

    return fx_form, fy_form, active_neighbors


def compute_relative_consensus(
    target_offsets: Dict[int, Tuple[float, float]],
    relative_displacements: Dict[int, Tuple[float, float]],
    k_p: float,
) -> Tuple[float, float]:
    """
    Compute standard linear displacement-based consensus from relative measurements.

    :param target_offsets: Desired relative formation offsets to peers in reference frame.
    :param relative_displacements: Measured relative displacements (dx, dy) to peers.
    :param k_p: Proportional consensus gain.
    :return: Planar formation force vector (fx, fy).
    """
    fx, fy = 0.0, 0.0
    for n_id, target_offset in target_offsets.items():
        if n_id not in relative_displacements:
            continue
        dx, dy = relative_displacements[n_id]
        err_x = dx - target_offset[0]
        err_y = dy - target_offset[1]
        fx += k_p * err_x
        fy += k_p * err_y
    return fx, fy


def compute_relative_huber_consensus(
    target_offsets: Dict[int, Tuple[float, float]],
    relative_displacements: Dict[int, Tuple[float, float]],
    active_mask: Dict[int, bool],
    k_p: float,
    huber_delta: float = 2.0,
    deadband: float = 0.02,
) -> Tuple[float, float, int]:
    """
    Compute robust consensus using Huber loss directly on relative measurements.

    :param target_offsets: Desired relative formation offsets to peers.
    :param relative_displacements: Measured relative displacements (dx, dy) to peers.
    :param active_mask: Boolean indicator whether each peer is active.
    :param k_p: Proportional consensus gain.
    :param huber_delta: Huber threshold bounding gradient influence.
    :param deadband: Minimum tracking error threshold.
    :return: Tuple of (fx, fy, active_neighbor_count).
    """
    fx_form, fy_form = 0.0, 0.0
    active_neighbors = 0

    for n_id, target_offset in target_offsets.items():
        if not active_mask.get(n_id, False):
            continue
        if n_id not in relative_displacements:
            continue

        active_neighbors += 1
        dx, dy = relative_displacements[n_id]
        err_x = dx - target_offset[0]
        err_y = dy - target_offset[1]
        err_norm = math.hypot(err_x, err_y)

        if err_norm > deadband:
            if err_norm > huber_delta:
                scale = huber_delta / err_norm
                err_x *= scale
                err_y *= scale
            fx_form += k_p * err_x
            fy_form += k_p * err_y

    return fx_form, fy_form, active_neighbors


def compute_anchor_force(
    robot_id: int,
    total_robots: int,
    radius: float,
    center: Tuple[float, float],
    est_pos: Tuple[float, float],
    gain: float,
    saturation: float = 1.0,
    rotation: float = 0.0,
) -> Tuple[float, float]:
    """
    Weak virtual-leader term pulling a robot toward its slot around a fixed centre.

    Displacement consensus only controls relative positions, so with unicycle robots the
    whole formation drifts (measured 1.2-1.7 m in 40 s in the kinematic campaign, more in
    Gazebo, where it pinned robots against the boundary wall). This term fixes the
    formation's position. It uses the robot's *estimated* world position from its EKF,
    i.e. it relies on the landmark-based global localization the stack already runs.

    :param robot_id: 1-based robot index (slot k sits at angle 2*pi*(k-1)/N).
    :param total_robots: Number of robots in the formation.
    :param radius: Formation radius in meters.
    :param center: Agreed formation centre (x, y) in the world frame.
    :param est_pos: Robot's estimated world position (x, y).
    :param gain: Anchor gain (0 disables the term).
    :param saturation: Error norm above which the pull is capped, in meters.
    :param rotation: Orientation of the slot pattern (rad). 0 for controllers that fix the
        orientation; for distance control, which leaves it free, the current best-fit
        orientation, so that the anchor fixes only the formation's position.
    :return: Planar force (fx, fy).
    """
    if gain <= 0.0:
        return 0.0, 0.0
    angle = 2.0 * math.pi * ((robot_id - 1) % total_robots) / total_robots + rotation
    ex = center[0] + radius * math.cos(angle) - est_pos[0]
    ey = center[1] + radius * math.sin(angle) - est_pos[1]
    norm = math.hypot(ex, ey)
    if norm > saturation > 0.0:
        ex, ey = ex * saturation / norm, ey * saturation / norm
    return gain * ex, gain * ey
