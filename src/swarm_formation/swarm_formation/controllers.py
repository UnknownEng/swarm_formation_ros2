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
Swarm formation controllers behind a common interface.

Supports:
1. Displacement-based consensus (Ren & Beard 2008, Oh et al. 2015).
2. Distance-based rigidity control (Krick et al. 2009).
3. Bearing-based rigidity control with scale anchoring (Zhao & Zelazo 2016).
"""

from abc import ABC, abstractmethod
import math
from typing import Dict, Optional, Tuple

from .consensus import compute_target_offsets


def compute_target_distances(
    robot_id: int, total_robots: int, radius: float
) -> Dict[int, float]:
    """
    Compute desired Euclidean inter-agent distances for a regular polygon formation.

    :param robot_id: 1-based index of observing robot.
    :param total_robots: Total number of robots in the polygon.
    :param radius: Radius of circumscribed circle in meters.
    :return: Mapping of peer_id -> desired distance in meters.
    """
    offsets = compute_target_offsets(robot_id, total_robots, radius)
    distances: Dict[int, float] = {}
    for peer_id, (dx, dy) in offsets.items():
        distances[peer_id] = math.hypot(dx, dy)
    return distances


def compute_target_bearings(
    robot_id: int, total_robots: int, radius: float
) -> Dict[int, Tuple[float, float, float]]:
    """
    Compute desired unit bearing vector and distance to each peer in the formation.

    :param robot_id: 1-based index of observing robot.
    :param total_robots: Total number of robots in the polygon.
    :param radius: Radius of circumscribed circle in meters.
    :return: Mapping of peer_id -> (gx_star, gy_star, d_star).
    """
    offsets = compute_target_offsets(robot_id, total_robots, radius)
    bearings: Dict[int, Tuple[float, float, float]] = {}
    for peer_id, (dx, dy) in offsets.items():
        d = math.hypot(dx, dy)
        if d > 1e-6:
            bearings[peer_id] = (dx / d, dy / d, d)
        else:
            bearings[peer_id] = (1.0, 0.0, 0.0)
    return bearings


class SwarmControllerBase(ABC):
    """Abstract base class defining common interface for swarm formation controllers."""

    def __init__(self, name: str) -> None:
        """Initialize controller with descriptive identifier."""
        self.name = name

    @abstractmethod
    def compute_control_force(
        self,
        robot_id: int,
        total_robots: int,
        radius: float,
        rel_displacements: Dict[int, Tuple[float, float]],
        gain: float,
        huber_delta: float = 1.0,
        active_mask: Optional[Dict[int, bool]] = None,
    ) -> Tuple[float, float]:
        """
        Compute reference-frame formation control force vector (fx, fy).

        :param robot_id: 1-based index of controlling robot.
        :param total_robots: Total number of robots in swarm.
        :param radius: Desired polygon formation radius in meters.
        :param rel_displacements: Mapping peer_id -> (dx, dy) in reference frame.
        :param gain: Primary proportional controller gain.
        :param huber_delta: Saturation/Huber threshold for robust clipping.
        :param active_mask: Optional health mask mapping peer_id -> bool.
        :return: Tuple of (fx, fy) control forces in Newtons.
        """
        pass


class DisplacementController(SwarmControllerBase):
    """
    Relative displacement-based consensus controller.

    Reference:
    Ren, W. and Beard, R.W., 'Distributed Consensus in Multi-vehicle
    Cooperative Control', Springer, 2008.
    """

    def __init__(self) -> None:
        """Initialize DisplacementController."""
        super().__init__('displacement')

    def compute_control_force(
        self,
        robot_id: int,
        total_robots: int,
        radius: float,
        rel_displacements: Dict[int, Tuple[float, float]],
        gain: float,
        huber_delta: float = 1.0,
        active_mask: Optional[Dict[int, bool]] = None,
    ) -> Tuple[float, float]:
        """Compute relative displacement consensus control force with Huber saturation."""
        target_offsets = compute_target_offsets(robot_id, total_robots, radius)
        f_x = 0.0
        f_y = 0.0

        for peer_id, (actual_dx, actual_dy) in rel_displacements.items():
            if active_mask is not None and not active_mask.get(peer_id, True):
                continue
            if peer_id not in target_offsets:
                continue

            target_dx, target_dy = target_offsets[peer_id]
            err_x = actual_dx - target_dx
            err_y = actual_dy - target_dy

            err_norm = math.hypot(err_x, err_y)
            if err_norm > huber_delta and err_norm > 1e-6:
                scale = huber_delta / err_norm
                f_x += gain * scale * err_x
                f_y += gain * scale * err_y
            else:
                f_x += gain * err_x
                f_y += gain * err_y

        return f_x, f_y


class DistanceController(SwarmControllerBase):
    """
    Distance-based formation controller based on graph rigidity.

    Incorporates distance gradient (Krick et al. 2009) and almost-global
    topological guidance (Oh & Ahn 2014).
    """

    def __init__(self, orientation_gain_ratio: float = 0.25) -> None:
        """
        Initialize DistanceController.

        :param orientation_gain_ratio: Guidance gain ratio resolving topological saddle points.
        """
        super().__init__('distance')
        self.orientation_gain_ratio = orientation_gain_ratio

    def compute_control_force(
        self,
        robot_id: int,
        total_robots: int,
        radius: float,
        rel_displacements: Dict[int, Tuple[float, float]],
        gain: float,
        huber_delta: float = 1.0,
        active_mask: Optional[Dict[int, bool]] = None,
    ) -> Tuple[float, float]:
        """Compute distance gradient rigidity control force with saddle-point untangling."""
        target_distances = compute_target_distances(robot_id, total_robots, radius)
        target_offsets = compute_target_offsets(robot_id, total_robots, radius)
        f_x = 0.0
        f_y = 0.0

        for peer_id, (actual_dx, actual_dy) in rel_displacements.items():
            if active_mask is not None and not active_mask.get(peer_id, True):
                continue
            if peer_id not in target_distances:
                continue

            d_measured = math.hypot(actual_dx, actual_dy)
            if d_measured < 1e-4:
                continue

            d_star = target_distances[peer_id]
            dist_err = d_measured - d_star

            # Huber saturation on distance error magnitude
            if abs(dist_err) > huber_delta:
                dist_err = math.copysign(huber_delta, dist_err)

            # Gradient direction: unit vector pointing from robot to peer
            unit_x = actual_dx / d_measured
            unit_y = actual_dy / d_measured

            # Positive error (too far) pulls robot towards peer (+unit)
            # Negative error (too close) pushes robot away from peer (-unit)
            f_x += gain * dist_err * unit_x
            f_y += gain * dist_err * unit_y

            # Orientation guidance to resolve non-planar/inverted local minima
            if self.orientation_gain_ratio > 0.0 and peer_id in target_offsets:
                tgt_x, tgt_y = target_offsets[peer_id]
                og_x = actual_dx - tgt_x
                og_y = actual_dy - tgt_y
                og_norm = math.hypot(og_x, og_y)
                if og_norm > huber_delta and og_norm > 1e-6:
                    scale = huber_delta / og_norm
                    f_x += gain * self.orientation_gain_ratio * scale * og_x
                    f_y += gain * self.orientation_gain_ratio * scale * og_y
                else:
                    f_x += gain * self.orientation_gain_ratio * og_x
                    f_y += gain * self.orientation_gain_ratio * og_y

        return f_x, f_y


class BearingController(SwarmControllerBase):
    """
    Bearing-based formation controller with orthogonal projection and scale anchor.

    Reference:
    Zhao, S. and Zelazo, D., 'Bearing rigidity and almost global bearing-only
    formation stabilization', IEEE Transactions on Automatic Control 61(5), 2016.
    """

    def __init__(self, scale_gain_ratio: float = 0.8) -> None:
        """
        Initialize BearingController.

        :param scale_gain_ratio: Gain multiplier for radial distance scaling.
        """
        super().__init__('bearing')
        self.scale_gain_ratio = scale_gain_ratio

    def compute_control_force(
        self,
        robot_id: int,
        total_robots: int,
        radius: float,
        rel_displacements: Dict[int, Tuple[float, float]],
        gain: float,
        huber_delta: float = 1.0,
        active_mask: Optional[Dict[int, bool]] = None,
    ) -> Tuple[float, float]:
        """Compute bearing projection and scale-anchored control force."""
        target_bearings = compute_target_bearings(robot_id, total_robots, radius)
        f_x = 0.0
        f_y = 0.0

        for peer_id, (dx, dy) in rel_displacements.items():
            if active_mask is not None and not active_mask.get(peer_id, True):
                continue
            if peer_id not in target_bearings:
                continue

            gx_star, gy_star, d_star = target_bearings[peer_id]

            # Orthogonal projection operator P_g* = I - g* g*^T
            # P_g* * p = p - (g*^T p) * g*
            proj_along = dx * gx_star + dy * gy_star
            ortho_x = dx - proj_along * gx_star
            ortho_y = dy - proj_along * gy_star

            # Radial distance mismatch along desired bearing line
            dist_mismatch = proj_along - d_star

            # Apply Huber saturation to both components
            ortho_norm = math.hypot(ortho_x, ortho_y)
            if ortho_norm > huber_delta and ortho_norm > 1e-6:
                scale_ortho = huber_delta / ortho_norm
                ortho_x *= scale_ortho
                ortho_y *= scale_ortho

            if abs(dist_mismatch) > huber_delta:
                dist_mismatch = math.copysign(huber_delta, dist_mismatch)

            # Combined bearing orthogonal force + scale alignment force
            f_x += gain * (ortho_x + self.scale_gain_ratio * dist_mismatch * gx_star)
            f_y += gain * (ortho_y + self.scale_gain_ratio * dist_mismatch * gy_star)

        return f_x, f_y


def create_controller(controller_type: str) -> SwarmControllerBase:
    """
    Create and return a swarm formation controller instance.

    :param controller_type: One of 'displacement', 'distance', 'bearing'.
    :return: Instance of SwarmControllerBase.
    :raises ValueError: If unknown controller_type is requested.
    """
    ctl_lower = controller_type.strip().lower()
    if ctl_lower in ('displacement', 'disp'):
        return DisplacementController()
    elif ctl_lower in ('distance', 'dist', 'rigidity'):
        return DistanceController()
    elif ctl_lower in ('bearing', 'bear', 'bearing_only'):
        return BearingController()
    else:
        raise ValueError(
            f"Unknown controller_type '{controller_type}'. "
            f"Valid options: 'displacement', 'distance', 'bearing'."
        )
