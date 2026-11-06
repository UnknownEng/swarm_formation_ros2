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

"""Relative sensing geometry and measurement model for peer robots and obstacles."""

import math
import random
from typing import Dict, Optional, Tuple

from .kinematics import wrap_angle


def compute_relative_bearing(dx: float, dy: float, heading: float) -> float:
    """
    Compute relative bearing in robot body frame.

    :param dx: X displacement in reference frame (target - self).
    :param dy: Y displacement in reference frame (target - self).
    :param heading: Robot heading angle in reference frame.
    :return: Relative bearing in radians [-pi, pi].
    """
    world_bearing = math.atan2(dy, dx)
    return wrap_angle(world_bearing - heading)


def compute_relative_measurement(
    self_pos: Tuple[float, float],
    self_yaw: float,
    target_pos: Tuple[float, float],
    sensing_range: float,
    range_noise_std: float = 0.0,
    bearing_noise_std: float = 0.0,
    rng: Optional[random.Random] = None,
) -> Optional[Dict[str, float]]:
    """
    Compute relative range and bearing measurement to a target robot.

    :param self_pos: (x, y) coordinates of observing robot.
    :param self_yaw: Heading of observing robot in radians.
    :param target_pos: (x, y) coordinates of target robot.
    :param sensing_range: Maximum detection distance in meters.
    :param range_noise_std: Standard deviation of range Gaussian noise in meters.
    :param bearing_noise_std: Standard deviation of bearing noise in radians.
    :param rng: Random number generator instance.
    :return: Measurement dict with keys (distance, bearing, relative_x, relative_y,
             range_variance, bearing_variance), or None if outside sensing range.
    """
    dx = target_pos[0] - self_pos[0]
    dy = target_pos[1] - self_pos[1]
    dist = math.hypot(dx, dy)

    if dist > sensing_range or dist < 1e-6:
        return None

    bearing = compute_relative_bearing(dx, dy, self_yaw)

    d_meas = dist
    b_meas = bearing

    if rng is not None:
        if range_noise_std > 0.0:
            d_meas = max(0.01, dist + rng.gauss(0.0, range_noise_std))
        if bearing_noise_std > 0.0:
            b_meas = wrap_angle(bearing + rng.gauss(0.0, bearing_noise_std))

    rel_x = d_meas * math.cos(b_meas)
    rel_y = d_meas * math.sin(b_meas)

    return {
        'distance': float(d_meas),
        'bearing': float(b_meas),
        'relative_x': float(rel_x),
        'relative_y': float(rel_y),
        'range_variance': float(range_noise_std ** 2),
        'bearing_variance': float(bearing_noise_std ** 2),
    }


def compute_relative_obstacle(
    self_pos: Tuple[float, float],
    self_yaw: float,
    obs_center: Tuple[float, float],
    obs_radius: float,
    sensing_range: float,
    range_noise_std: float = 0.0,
    bearing_noise_std: float = 0.0,
    rng: Optional[random.Random] = None,
) -> Optional[Dict[str, float]]:
    """
    Compute relative detection to an environmental obstacle.

    :param self_pos: (x, y) coordinates of observing robot.
    :param self_yaw: Heading of observing robot in radians.
    :param obs_center: (x, y) center coordinates of obstacle.
    :param obs_radius: Physical radius of obstacle in meters.
    :param sensing_range: Maximum sensor detection range in meters.
    :param range_noise_std: Range noise standard deviation.
    :param bearing_noise_std: Bearing noise standard deviation.
    :param rng: Random number generator instance.
    :return: Measurement dict or None if out of range.
    """
    dx = obs_center[0] - self_pos[0]
    dy = obs_center[1] - self_pos[1]
    dist_center = math.hypot(dx, dy)

    if (dist_center - obs_radius) > sensing_range or dist_center < 1e-6:
        return None

    bearing = compute_relative_bearing(dx, dy, self_yaw)

    d_meas = dist_center
    b_meas = bearing

    if rng is not None:
        if range_noise_std > 0.0:
            d_meas = max(obs_radius + 0.01, dist_center + rng.gauss(0.0, range_noise_std))
        if bearing_noise_std > 0.0:
            b_meas = wrap_angle(bearing + rng.gauss(0.0, bearing_noise_std))

    rel_x = d_meas * math.cos(b_meas)
    rel_y = d_meas * math.sin(b_meas)

    return {
        'distance': float(d_meas),
        'bearing': float(b_meas),
        'relative_x': float(rel_x),
        'relative_y': float(rel_y),
        'radius': float(obs_radius),
        'range_variance': float(range_noise_std ** 2),
        'bearing_variance': float(bearing_noise_std ** 2),
    }


def transform_body_to_reference(
    rel_x_body: float, rel_y_body: float, heading: float
) -> Tuple[float, float]:
    """
    Transform a 2D vector from robot body frame to orientation-aligned reference frame.

    :param rel_x_body: Longitudinal coordinate in body frame (forward).
    :param rel_y_body: Lateral coordinate in body frame (left).
    :param heading: Robot heading angle in reference frame.
    :return: (dx, dy) vector in reference frame.
    """
    cos_h = math.cos(heading)
    sin_h = math.sin(heading)
    dx = rel_x_body * cos_h - rel_y_body * sin_h
    dy = rel_x_body * sin_h + rel_y_body * cos_h
    return dx, dy
