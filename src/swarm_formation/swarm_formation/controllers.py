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
Swarm formation controllers behind a common interface.

Supports:
1. Displacement-based consensus (Ren & Beard 2008, Oh et al. 2015).
2. Distance-based gradient control with signed-area constraints (Krick et al. 2009;
   Anderson et al. 2017).
3. Bearing-only control (Zhao & Zelazo 2016) with a weak distance term for the scale.
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
    Distance-based formation controller with signed-area constraints.

    u_i = k * [ sum_j sat(||p_ij|| - d*_ij) p_ij / ||p_ij||
                - a * sum_{j<k} sat(A_ijk - A*_ijk) dA_ijk/dp_i ]

    with A_ijk = 1/2 (p_ij x p_ik) the signed area of triangle (i, j, k). Both terms are
    invariant to rotations of the robot's measurement frame, so, unlike displacement
    consensus, no common heading is needed. The first term is the gradient law of Krick,
    Broucke and Francis (IJC 2009); on its own it only converges locally (a scrambled
    start stalls in an undesired equilibrium) and cannot tell the formation from its
    mirror image. The signed areas fix the orientation and remove those equilibria
    (Anderson, Sun, Sugie et al., 'Formation shape control with distance and area
    constraints', IFAC J. Syst. Control 1, 2017). ``area_gain = 0`` gives the pure law.

    ``orientation_gain_ratio`` optionally adds the displacement-consensus term; it is 0 by
    default because a non-zero value turns the law into a displacement controller.
    """

    def __init__(self, area_gain: float = 0.5, orientation_gain_ratio: float = 0.0) -> None:
        """
        Initialize DistanceController.

        :param area_gain: Weight of the signed-area terms (0 = pure distance gradient).
        :param orientation_gain_ratio: Optional displacement-term weight (0 = none).
        """
        super().__init__('distance')
        self.area_gain = area_gain
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
        """Compute the distance-gradient force (optionally plus a displacement term)."""
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

            dist_err = d_measured - target_distances[peer_id]
            if abs(dist_err) > huber_delta:
                dist_err = math.copysign(huber_delta, dist_err)

            # Too far: pulled toward the peer (+unit); too close: pushed away (-unit)
            f_x += gain * dist_err * actual_dx / d_measured
            f_y += gain * dist_err * actual_dy / d_measured

            if self.orientation_gain_ratio > 0.0 and peer_id in target_offsets:
                tgt_x, tgt_y = target_offsets[peer_id]
                og_x, og_y = actual_dx - tgt_x, actual_dy - tgt_y
                og_norm = math.hypot(og_x, og_y)
                if og_norm > huber_delta and og_norm > 1e-6:
                    og_x, og_y = og_x * huber_delta / og_norm, og_y * huber_delta / og_norm
                f_x += gain * self.orientation_gain_ratio * og_x
                f_y += gain * self.orientation_gain_ratio * og_y

        if self.area_gain > 0.0:
            a_x, a_y = self._area_force(rel_displacements, target_offsets, huber_delta,
                                        active_mask)
            f_x += gain * self.area_gain * a_x
            f_y += gain * self.area_gain * a_y

        return f_x, f_y

    @staticmethod
    def _area_force(
        rel_displacements: Dict[int, Tuple[float, float]],
        target_offsets: Dict[int, Tuple[float, float]],
        huber_delta: float,
        active_mask: Optional[Dict[int, bool]],
    ) -> Tuple[float, float]:
        """Return -sum_{j<k} sat(A_ijk - A*_ijk) * dA_ijk/dp_i over measured peer pairs."""
        ids = sorted(
            pid for pid in rel_displacements
            if pid in target_offsets
            and (active_mask is None or active_mask.get(pid, True))
        )
        f_x = 0.0
        f_y = 0.0
        for a, j in enumerate(ids):
            rjx, rjy = rel_displacements[j]
            tjx, tjy = target_offsets[j]
            for k in ids[a + 1:]:
                rkx, rky = rel_displacements[k]
                tkx, tky = target_offsets[k]
                area_err = 0.5 * (rjx * rky - rjy * rkx) - 0.5 * (tjx * tky - tjy * tkx)
                if abs(area_err) > huber_delta:
                    area_err = math.copysign(huber_delta, area_err)
                # dA/dp_i for A = 1/2 (p_j - p_i) x (p_k - p_i)
                f_x -= area_err * 0.5 * (rjy - rky)
                f_y -= area_err * 0.5 * (rkx - rjx)
        return f_x, f_y


class BearingController(SwarmControllerBase):
    """
    Bearing-only formation controller with a weak distance term for the scale.

    u_i = k * sum_j [ -d*_ij * P(g_ij) g*_ij  +  s * sat(||p_ij|| - d*_ij) * g_ij ]

    where g_ij = p_ij / ||p_ij|| is the *measured* bearing, g*_ij the desired bearing and
    P(g) = I - g g^T. The first term is the bearing-only law of Zhao and Zelazo (2016); it
    reaches the target bearings up to translation and scale (scaled by d*_ij so that it
    matches a displacement error near the target). Bearing-only laws preserve the initial
    formation scale, so the second, weak distance term (``scale_gain_ratio``) fixes the
    size. Bearings are world-frame directions, so the formation's orientation is fixed.

    Reference: Zhao, S. and Zelazo, D., 'Bearing rigidity and almost global bearing-only
    formation stabilization', IEEE TAC 61(5), 2016.
    """

    def __init__(self, scale_gain_ratio: float = 0.5) -> None:
        """
        Initialize BearingController.

        :param scale_gain_ratio: Weight of the distance (scale) term relative to bearing.
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
        """Compute the bearing-only force plus the weak scale term."""
        target_bearings = compute_target_bearings(robot_id, total_robots, radius)
        f_x = 0.0
        f_y = 0.0

        for peer_id, (dx, dy) in rel_displacements.items():
            if active_mask is not None and not active_mask.get(peer_id, True):
                continue
            if peer_id not in target_bearings:
                continue
            dist = math.hypot(dx, dy)
            if dist < 1e-4:
                continue

            gx, gy = dx / dist, dy / dist                 # measured bearing
            gx_star, gy_star, d_star = target_bearings[peer_id]

            # -P(g) g* = -(g* - (g . g*) g)
            dot = gx * gx_star + gy * gy_star
            bx = -(gx_star - dot * gx)
            by = -(gy_star - dot * gy)
            f_x += gain * d_star * bx
            f_y += gain * d_star * by

            dist_err = dist - d_star
            if abs(dist_err) > huber_delta:
                dist_err = math.copysign(huber_delta, dist_err)
            f_x += gain * self.scale_gain_ratio * dist_err * gx
            f_y += gain * self.scale_gain_ratio * dist_err * gy

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
