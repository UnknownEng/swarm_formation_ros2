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

"""Relative sensing geometry and measurement model for peer robots and obstacles."""

import math
import random
from typing import Callable, Dict, List, Optional, Tuple

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


class YawHistory:
    """
    Short buffer of (time, yaw) samples for rotating delayed body-frame measurements.

    A range/bearing measurement is expressed in the body frame at the time it was taken.
    Rotating it with the heading at *reception* time adds an error equal to the rotation
    that happened during the delay; this buffer returns the heading at measurement time.
    """

    def __init__(self, horizon_sec: float = 3.0) -> None:
        """Create an empty history that keeps samples for ``horizon_sec`` seconds."""
        self.horizon_sec = horizon_sec
        self._samples: List[Tuple[float, float]] = []

    def add(self, t: float, yaw: float) -> None:
        """Append a heading sample; samples must arrive in non-decreasing time order."""
        if self._samples and t < self._samples[-1][0]:
            return
        self._samples.append((t, yaw))
        cutoff = t - self.horizon_sec
        drop = 0
        while drop < len(self._samples) - 1 and self._samples[drop][0] < cutoff:
            drop += 1
        if drop:
            del self._samples[:drop]

    def lookup(self, t: float) -> Optional[float]:
        """Return the heading at time ``t`` (interpolated), or None if the buffer is empty."""
        if not self._samples:
            return None
        if t <= self._samples[0][0]:
            return self._samples[0][1]
        if t >= self._samples[-1][0]:
            return self._samples[-1][1]
        lo, hi = 0, len(self._samples) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self._samples[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, y0 = self._samples[lo]
        t1, y1 = self._samples[hi]
        frac = (t - t0) / max(t1 - t0, 1e-9)
        return wrap_angle(y0 + frac * wrap_angle(y1 - y0))


def update_relative_store(
    prev_store: Dict[int, Tuple[float, float, float, float, float]],
    measurements: List[Tuple[int, float, float, float]],
    yaw_at: Callable[[float], float],
) -> Dict[int, Tuple[float, float, float, float, float]]:
    """
    Build the current neighbor table from one NeighborList.

    Only peers in the current list are kept, so a neighbor that is no longer delivered
    stops influencing control. Each body-frame vector is rotated with the heading at its
    *measurement* time. Relative velocity is a finite difference between measurements
    with different timestamps; a re-published (unchanged) measurement keeps the previous
    velocity instead of producing a spurious zero.

    :param prev_store: Previous table, peer_id -> (dx, dy, vx, vy, t_meas).
    :param measurements: Current list of (peer_id, rel_x_body, rel_y_body, t_meas).
    :param yaw_at: Function returning the robot heading at a given time.
    :return: New table, peer_id -> (dx, dy, vx, vy, t_meas).
    """
    store: Dict[int, Tuple[float, float, float, float, float]] = {}
    for peer_id, rel_x, rel_y, t_meas in measurements:
        dx, dy = transform_body_to_reference(rel_x, rel_y, yaw_at(t_meas))
        vx, vy = 0.0, 0.0
        prev = prev_store.get(peer_id)
        if prev is not None:
            pdx, pdy, pvx, pvy, pt = prev
            if t_meas > pt + 1e-6:
                vx, vy = (dx - pdx) / (t_meas - pt), (dy - pdy) / (t_meas - pt)
            else:
                vx, vy = pvx, pvy
        store[peer_id] = (dx, dy, vx, vy, t_meas)
    return store


def compute_heartbeat_mask(
    store: Dict[int, Tuple[float, float, float, float, float]],
    known_peers: List[int],
    now: float,
    timeout: float,
) -> Dict[int, bool]:
    """
    Mark a peer active only if it is in the current table and its measurement is fresh.

    Freshness uses the measurement timestamp, so a stale measurement that the network
    keeps re-publishing still times out.
    """
    return {
        pid: pid in store and (now - store[pid][4]) <= timeout
        for pid in known_peers
    }


class PoseHistory:
    """Short buffer of (time, x, y) estimated positions for delay compensation."""

    def __init__(self, horizon_sec: float = 3.0) -> None:
        """Create an empty history that keeps samples for ``horizon_sec`` seconds."""
        self.horizon_sec = horizon_sec
        self._samples: List[Tuple[float, float, float]] = []

    def add(self, t: float, x: float, y: float) -> None:
        """Append a position sample; samples must arrive in non-decreasing time order."""
        if self._samples and t < self._samples[-1][0]:
            return
        self._samples.append((t, x, y))
        cutoff = t - self.horizon_sec
        drop = 0
        while drop < len(self._samples) - 1 and self._samples[drop][0] < cutoff:
            drop += 1
        if drop:
            del self._samples[:drop]

    def lookup(self, t: float) -> Optional[Tuple[float, float]]:
        """Return the position at time ``t`` (interpolated), or None if empty."""
        if not self._samples:
            return None
        if t <= self._samples[0][0]:
            return self._samples[0][1], self._samples[0][2]
        if t >= self._samples[-1][0]:
            return self._samples[-1][1], self._samples[-1][2]
        lo, hi = 0, len(self._samples) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self._samples[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, x0, y0 = self._samples[lo]
        t1, x1, y1 = self._samples[hi]
        frac = (t - t0) / max(t1 - t0, 1e-9)
        return x0 + frac * (x1 - x0), y0 + frac * (y1 - y0)


def compensate_delay(
    store: Dict[int, Tuple[float, float, float, float, float]],
    now: float,
    own_pos_at: Callable[[float], Optional[Tuple[float, float]]],
    peer_velocity_gain: float = 0.0,
    max_age: float = 0.5,
) -> Dict[int, Tuple[float, float]]:
    """
    Bring delayed relative measurements to the current time.

    A measurement taken at t_meas is stale by age = now - t_meas. Over that time this robot
    moved by p_i(now) - p_i(t_meas), which is known exactly from its own pose history and is
    subtracted (no added noise). Optionally the neighbour's motion is extrapolated too:
    its velocity is the relative velocity plus this robot's own velocity, scaled by
    ``peer_velocity_gain`` (0 disables; the finite-difference estimate is noisy).

    :param store: Neighbor table peer_id -> (dx, dy, rel_vx, rel_vy, t_meas).
    :param now: Current time.
    :param own_pos_at: Function returning this robot's estimated position at a time.
    :param peer_velocity_gain: Weight of the neighbour-motion extrapolation.
    :param max_age: Ages are clipped to this (s) so a stale entry cannot be pushed far.
    :return: peer_id -> compensated (dx, dy).
    """
    p_now = own_pos_at(now)
    out: Dict[int, Tuple[float, float]] = {}
    for pid, (dx, dy, rvx, rvy, t_meas) in store.items():
        age = min(max(0.0, now - t_meas), max_age)
        p_then = own_pos_at(now - age)
        if p_now is None or p_then is None or age <= 0.0:
            out[pid] = (dx, dy)
            continue
        own_dx = p_now[0] - p_then[0]
        own_dy = p_now[1] - p_then[1]
        cx, cy = dx - own_dx, dy - own_dy
        if peer_velocity_gain > 0.0:
            own_vx, own_vy = own_dx / age, own_dy / age
            cx += peer_velocity_gain * (rvx + own_vx) * age
            cy += peer_velocity_gain * (rvy + own_vy) * age
        out[pid] = (cx, cy)
    return out
