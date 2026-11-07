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


class UnicycleMapper:
    """
    Stateful force -> (v, w) mapping that does not spin in place at rest.

    ``compute_unicycle_twist`` turns at up to ``k_yaw * heading_error`` toward any force
    above a 0.03 deadband. Once the formation has settled, the residual forces are small
    and change direction constantly, so the robots kept turning at up to 3 rad/s in
    Gazebo while barely moving. This mapper adds three standard remedies:

    * **Reverse driving** (optional, off by default): if the force points behind the
      robot, drive backwards. In the kinematic sim this stopped the swarm converging (0/4),
      because the CBF's look-ahead point always sits in front of the robot.
    * **Force-scaled turning**: the turn rate is scaled by min(1, |f| / turn_full_force),
      so a small force gives a gentle correction, not a full-rate spin.
    * **Stop band with hysteresis**: below ``stop_force`` the robot stops and stays
      stopped until the force exceeds ``resume_force``.
    """

    def __init__(
        self,
        k_yaw: float = 4.0,
        max_lin_vel: float = 1.4,
        max_ang_vel: float = 3.0,
        allow_reverse: bool = False,
        turn_full_force: float = 1.0,
        stop_force: float = 0.5,
        resume_force: float = 0.9,
    ) -> None:
        """Create a mapper; see the class docstring for the parameters' roles."""
        self.k_yaw = k_yaw
        self.max_lin_vel = max_lin_vel
        self.max_ang_vel = max_ang_vel
        self.allow_reverse = allow_reverse
        self.turn_full_force = turn_full_force
        self.stop_force = stop_force
        self.resume_force = max(resume_force, stop_force)
        self.stopped = True

    def to_twist(self, fx: float, fy: float, current_yaw: float) -> Tuple[float, float]:
        """Return (linear_velocity, angular_velocity) for the planar force (fx, fy)."""
        force_mag = math.hypot(fx, fy)
        if self.stopped:
            if force_mag < self.resume_force:
                return 0.0, 0.0
            self.stopped = False
        elif force_mag < self.stop_force:
            self.stopped = True
            return 0.0, 0.0

        desired_yaw = math.atan2(fy, fx)
        yaw_err = wrap_angle(desired_yaw - current_yaw)
        direction = 1.0
        if self.allow_reverse and abs(yaw_err) > math.pi / 2.0:
            yaw_err = wrap_angle(yaw_err - math.pi)
            direction = -1.0

        turn_scale = 1.0
        if self.turn_full_force > 0.0:
            turn_scale = min(1.0, force_mag / self.turn_full_force)
        w_cmd = self.k_yaw * yaw_err * turn_scale
        w_cmd = max(-self.max_ang_vel, min(self.max_ang_vel, w_cmd))

        speed = min(force_mag, self.max_lin_vel) * max(0.0, math.cos(yaw_err))
        return direction * speed, w_cmd
