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

"""Pure-Python formation metrics shared by the rosbag processor and the campaign simulator."""

import math
from typing import List, Tuple


def compute_procrustes_error(
    positions: List[Tuple[float, float]],
    radius: float,
    total_robots: int = 5,
    allow_reflection: bool = False,
) -> float:
    """
    Calculate Procrustes RMS shape error aligned to a regular polygon of given radius.

    Translation and rotation are factored out; the robot-to-vertex assignment is fixed
    (robot i must occupy vertex i). With ``allow_reflection`` the mirrored polygon also
    counts as correct (the right metric for distance-only control, which cannot tell a
    formation from its mirror image).
    """
    err = _procrustes_rotation_only(positions, radius, total_robots)
    if allow_reflection and len(positions) >= total_robots:
        mirrored = [(x, -y) for x, y in positions[:total_robots]]
        err = min(err, _procrustes_rotation_only(mirrored, radius, total_robots))
    return err


def _procrustes_rotation_only(
    positions: List[Tuple[float, float]],
    radius: float,
    total_robots: int,
) -> float:
    """Return the RMS error after optimal translation and rotation (no reflection)."""
    if len(positions) < total_robots:
        return float('nan')

    cx = sum(p[0] for p in positions[:total_robots]) / total_robots
    cy = sum(p[1] for p in positions[:total_robots]) / total_robots

    target_pts = [
        (
            radius * math.cos(2.0 * math.pi * i / total_robots),
            radius * math.sin(2.0 * math.pi * i / total_robots),
        )
        for i in range(total_robots)
    ]

    num = sum(
        (positions[i][0] - cx) * target_pts[i][1]
        - (positions[i][1] - cy) * target_pts[i][0]
        for i in range(total_robots)
    )
    den = sum(
        (positions[i][0] - cx) * target_pts[i][0]
        + (positions[i][1] - cy) * target_pts[i][1]
        for i in range(total_robots)
    )
    theta_align = math.atan2(num, den)

    sq_err_sum = 0.0
    for i in range(total_robots):
        u = positions[i][0] - cx
        v = positions[i][1] - cy
        rot_u = u * math.cos(theta_align) - v * math.sin(theta_align)
        rot_v = u * math.sin(theta_align) + v * math.cos(theta_align)
        tx, ty = target_pts[i]
        sq_err_sum += (rot_u - tx) ** 2 + (rot_v - ty) ** 2

    return math.sqrt(sq_err_sum / total_robots)
