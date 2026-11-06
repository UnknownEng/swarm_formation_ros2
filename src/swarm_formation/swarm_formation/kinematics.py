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

"""Kinematic mapping routines for differential-drive unicycle robots."""

import math
from typing import Tuple


def wrap_angle(angle: float) -> float:
    """Wrap angle to range [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def compute_unicycle_twist(
    fx: float,
    fy: float,
    current_yaw: float,
    k_yaw: float = 4.0,
    max_lin_vel: float = 1.4,
    max_ang_vel: float = 3.0,
    deadband: float = 0.03,
) -> Tuple[float, float]:
    """
    Map planar 2D force vector to unicycle (linear, angular) velocity commands.

    :param fx: Planar force in x direction.
    :param fy: Planar force in y direction.
    :param current_yaw: Current heading angle of the robot in radians.
    :param k_yaw: Proportional heading control gain.
    :param max_lin_vel: Maximum forward linear velocity limit (m/s).
    :param max_ang_vel: Maximum angular velocity limit (rad/s).
    :param deadband: Force magnitude threshold below which velocity is zero.
    :return: Tuple of (linear_velocity, angular_velocity).
    """
    force_mag = math.hypot(fx, fy)
    if force_mag < deadband:
        return 0.0, 0.0

    desired_yaw = math.atan2(fy, fx)
    yaw_err = wrap_angle(desired_yaw - current_yaw)

    w_cmd = k_yaw * yaw_err
    w_cmd = max(-max_ang_vel, min(max_ang_vel, w_cmd))

    # Smooth continuous scaling: zero forward velocity when heading error >= pi/2
    speed = min(force_mag, max_lin_vel) * max(0.0, math.cos(yaw_err))
    return speed, w_cmd
