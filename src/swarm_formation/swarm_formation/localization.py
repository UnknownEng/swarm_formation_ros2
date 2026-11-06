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
Cooperative Extended Kalman Filter (EKF) localization library for swarm robots.

Fuses wheel odometry dead-reckoning with relative range-bearing neighbor
and landmark observations without accessing global ground truth.
"""

import math
from typing import List, Optional, Tuple

from .kinematics import wrap_angle


class RobotEKF:
    """
    Extended Kalman Filter estimating 2D planar robot pose (x, y, yaw).

    State vector: x = [p_x, p_y, psi]^T in reference coordinate frame.
    """

    def __init__(
        self,
        init_x: float = 0.0,
        init_y: float = 0.0,
        init_yaw: float = 0.0,
        process_var_pos: float = 0.04,
        process_var_yaw: float = 0.02,
        meas_var_range: float = 0.05,
        meas_var_bearing: float = 0.03,
    ) -> None:
        """
        Initialize state vector and covariance matrices.

        :param init_x: Initial estimated x position.
        :param init_y: Initial estimated y position.
        :param init_yaw: Initial estimated heading in radians.
        :param process_var_pos: Process noise variance for linear translation (m^2/s).
        :param process_var_yaw: Process noise variance for angular rotation (rad^2/s).
        :param meas_var_range: Default relative range measurement variance (m^2).
        :param meas_var_bearing: Default relative bearing measurement variance (rad^2).
        """
        self.x = init_x
        self.y = init_y
        self.yaw = wrap_angle(init_yaw)

        # State covariance P (3x3 symmetric matrix)
        self.P = [
            [0.1, 0.0, 0.0],
            [0.0, 0.1, 0.0],
            [0.0, 0.0, 0.05],
        ]

        self.q_pos = process_var_pos
        self.q_yaw = process_var_yaw
        self.r_range = meas_var_range
        self.r_bearing = meas_var_bearing

    def predict(self, linear_vel: float, angular_vel: float, dt: float) -> None:
        """
        Execute EKF prediction step using unicycle kinematics from odometry.

        :param linear_vel: Forward linear speed in m/s.
        :param angular_vel: Yaw rotational rate in rad/s.
        :param dt: Time interval in seconds.
        """
        if dt <= 0.0:
            return

        delta_s = linear_vel * dt
        delta_psi = angular_vel * dt
        mid_yaw = self.yaw + delta_psi * 0.5

        # 1. State prediction
        self.x += delta_s * math.cos(mid_yaw)
        self.y += delta_s * math.sin(mid_yaw)
        self.yaw = wrap_angle(self.yaw + delta_psi)

        # 2. State transition Jacobian F
        f02 = -delta_s * math.sin(mid_yaw)
        f12 = delta_s * math.cos(mid_yaw)

        # F * P
        fp00 = self.P[0][0] + f02 * self.P[2][0]
        fp01 = self.P[0][1] + f02 * self.P[2][1]
        fp02 = self.P[0][2] + f02 * self.P[2][2]

        fp10 = self.P[1][0] + f12 * self.P[2][0]
        fp11 = self.P[1][1] + f12 * self.P[2][1]
        fp12 = self.P[1][2] + f12 * self.P[2][2]

        fp20 = self.P[2][0]
        fp21 = self.P[2][1]
        fp22 = self.P[2][2]

        # F * P * F^T
        p00 = fp00 + fp02 * f02
        p01 = fp01 + fp02 * f12
        p02 = fp02

        p10 = fp10 + fp12 * f02
        p11 = fp11 + fp12 * f12
        p12 = fp12

        p20 = fp20 + fp22 * f02
        p21 = fp21 + fp22 * f12
        p22 = fp22

        # Add process noise Q
        q_pos_val = self.q_pos * dt
        q_yaw_val = self.q_yaw * dt

        self.P[0][0] = p00 + q_pos_val
        self.P[0][1] = 0.5 * (p01 + p10)
        self.P[0][2] = 0.5 * (p02 + p20)

        self.P[1][0] = self.P[0][1]
        self.P[1][1] = p11 + q_pos_val
        self.P[1][2] = 0.5 * (p12 + p21)

        self.P[2][0] = self.P[0][2]
        self.P[2][1] = self.P[1][2]
        self.P[2][2] = p22 + q_yaw_val

    def update_range_bearing(
        self,
        target_x: float,
        target_y: float,
        measured_dist: float,
        measured_bearing: float,
        var_range: Optional[float] = None,
        var_bearing: Optional[float] = None,
    ) -> None:
        """
        Execute EKF correction step using relative range and bearing observation.

        :param target_x: Estimated or known target reference position X.
        :param target_y: Estimated or known target reference position Y.
        :param measured_dist: Observed range in meters.
        :param measured_bearing: Observed bearing in radians relative to body frame.
        :param var_range: Measurement range variance (m^2).
        :param var_bearing: Measurement bearing variance (rad^2).
        """
        dx = target_x - self.x
        dy = target_y - self.y
        dist_sq = dx * dx + dy * dy
        dist_pred = math.sqrt(dist_sq)

        if dist_pred < 1e-4:
            return

        # Predicted bearing relative to robot body heading
        bearing_pred = wrap_angle(math.atan2(dy, dx) - self.yaw)

        # Innovation (residual)
        res_dist = measured_dist - dist_pred
        res_bearing = wrap_angle(measured_bearing - bearing_pred)

        # Measurement Jacobian H (2x3)
        h00 = -dx / dist_pred
        h01 = -dy / dist_pred

        h10 = dy / dist_sq
        h11 = -dx / dist_sq
        h12 = -1.0

        # Measurement noise R (2x2)
        r0 = var_range if (var_range is not None and var_range > 0.0) else self.r_range
        r1 = var_bearing if (var_bearing is not None and var_bearing > 0.0) else self.r_bearing

        # H * P (2x3)
        hp00 = h00 * self.P[0][0] + h01 * self.P[1][0]
        hp01 = h00 * self.P[0][1] + h01 * self.P[1][1]
        hp02 = h00 * self.P[0][2] + h01 * self.P[1][2]

        hp10 = h10 * self.P[0][0] + h11 * self.P[1][0] + h12 * self.P[2][0]
        hp11 = h10 * self.P[0][1] + h11 * self.P[1][1] + h12 * self.P[2][1]
        hp12 = h10 * self.P[0][2] + h11 * self.P[1][2] + h12 * self.P[2][2]

        # Innovation covariance S = H * P * H^T + R (2x2)
        s00 = hp00 * h00 + hp01 * h01 + r0
        s01 = hp00 * h10 + hp01 * h11 + hp02 * h12
        s10 = hp10 * h00 + hp11 * h01
        s11 = hp10 * h10 + hp11 * h11 + hp12 * h12 + r1

        det_s = s00 * s11 - s01 * s10
        if abs(det_s) < 1e-9:
            return

        inv_s00 = s11 / det_s
        inv_s01 = -s01 / det_s
        inv_s10 = -s10 / det_s
        inv_s11 = s00 / det_s

        # Kalman gain K = P * H^T * S^-1 (3x2)
        # Note: (H * P)^T = P * H^T
        pht00 = hp00
        pht01 = hp10
        pht10 = hp01
        pht11 = hp11
        pht20 = hp02
        pht21 = hp12

        k00 = pht00 * inv_s00 + pht01 * inv_s10
        k01 = pht00 * inv_s01 + pht01 * inv_s11

        k10 = pht10 * inv_s00 + pht11 * inv_s10
        k11 = pht10 * inv_s01 + pht11 * inv_s11

        k20 = pht20 * inv_s00 + pht21 * inv_s10
        k21 = pht20 * inv_s01 + pht21 * inv_s11

        # State update
        dx_val = k00 * res_dist + k01 * res_bearing
        dy_val = k10 * res_dist + k11 * res_bearing
        dyaw_val = k20 * res_dist + k21 * res_bearing

        self.x += dx_val
        self.y += dy_val
        self.yaw = wrap_angle(self.yaw + dyaw_val)

        # Covariance update: P = (I - K * H) * P
        kh00 = k00 * h00 + k01 * h10
        kh01 = k00 * h01 + k01 * h11
        kh02 = k01 * h12

        kh10 = k10 * h00 + k11 * h10
        kh11 = k10 * h01 + k11 * h11
        kh12 = k11 * h12

        kh20 = k20 * h00 + k21 * h10
        kh21 = k20 * h01 + k21 * h11
        kh22 = k21 * h12

        i_kh00 = 1.0 - kh00
        i_kh01 = -kh01
        i_kh02 = -kh02

        i_kh10 = -kh10
        i_kh11 = 1.0 - kh11
        i_kh12 = -kh12

        i_kh20 = -kh20
        i_kh21 = -kh21
        i_kh22 = 1.0 - kh22

        new_p00 = i_kh00 * self.P[0][0] + i_kh01 * self.P[1][0] + i_kh02 * self.P[2][0]
        new_p01 = i_kh00 * self.P[0][1] + i_kh01 * self.P[1][1] + i_kh02 * self.P[2][1]
        new_p02 = i_kh00 * self.P[0][2] + i_kh01 * self.P[1][2] + i_kh02 * self.P[2][2]

        new_p10 = i_kh10 * self.P[0][0] + i_kh11 * self.P[1][0] + i_kh12 * self.P[2][0]
        new_p11 = i_kh10 * self.P[0][1] + i_kh11 * self.P[1][1] + i_kh12 * self.P[2][1]
        new_p12 = i_kh10 * self.P[0][2] + i_kh11 * self.P[1][2] + i_kh12 * self.P[2][2]

        new_p20 = i_kh20 * self.P[0][0] + i_kh21 * self.P[1][0] + i_kh22 * self.P[2][0]
        new_p21 = i_kh20 * self.P[0][1] + i_kh21 * self.P[1][1] + i_kh22 * self.P[2][1]
        new_p22 = i_kh20 * self.P[0][2] + i_kh21 * self.P[1][2] + i_kh22 * self.P[2][2]

        # Enforce covariance symmetry and positive definiteness
        self.P[0][0] = max(1e-6, new_p00)
        self.P[0][1] = 0.5 * (new_p01 + new_p10)
        self.P[0][2] = 0.5 * (new_p02 + new_p20)

        self.P[1][0] = self.P[0][1]
        self.P[1][1] = max(1e-6, new_p11)
        self.P[1][2] = 0.5 * (new_p12 + new_p21)

        self.P[2][0] = self.P[0][2]
        self.P[2][1] = self.P[1][2]
        self.P[2][2] = max(1e-6, new_p22)

    def update_heading(self, measured_yaw: float, var_yaw: Optional[float] = None) -> None:
        """
        Execute EKF correction step using orientation observation (e.g. odometry/IMU).

        :param measured_yaw: Measured heading angle in radians.
        :param var_yaw: Heading measurement variance in rad^2.
        """
        res_yaw = wrap_angle(measured_yaw - self.yaw)
        r = var_yaw if (var_yaw is not None and var_yaw > 0.0) else self.r_bearing

        s = self.P[2][2] + r
        if s < 1e-9:
            return

        k0 = self.P[0][2] / s
        k1 = self.P[1][2] / s
        k2 = self.P[2][2] / s

        self.x += k0 * res_yaw
        self.y += k1 * res_yaw
        self.yaw = wrap_angle(self.yaw + k2 * res_yaw)

        new_p00 = self.P[0][0] - k0 * self.P[2][0]
        new_p01 = self.P[0][1] - k0 * self.P[2][1]
        new_p02 = self.P[0][2] - k0 * self.P[2][2]

        new_p10 = self.P[1][0] - k1 * self.P[2][0]
        new_p11 = self.P[1][1] - k1 * self.P[2][1]
        new_p12 = self.P[1][2] - k1 * self.P[2][2]

        new_p20 = (1.0 - k2) * self.P[2][0]
        new_p21 = (1.0 - k2) * self.P[2][1]
        new_p22 = (1.0 - k2) * self.P[2][2]

        self.P[0][0] = max(1e-6, new_p00)
        self.P[0][1] = 0.5 * (new_p01 + new_p10)
        self.P[0][2] = 0.5 * (new_p02 + new_p20)

        self.P[1][0] = self.P[0][1]
        self.P[1][1] = max(1e-6, new_p11)
        self.P[1][2] = 0.5 * (new_p12 + new_p21)

        self.P[2][0] = self.P[0][2]
        self.P[2][1] = self.P[1][2]
        self.P[2][2] = max(1e-6, new_p22)

    def get_pose(self) -> Tuple[float, float, float]:
        """Return current estimated state tuple (x, y, yaw)."""
        return self.x, self.y, self.yaw

    def get_covariance(self) -> List[List[float]]:
        """Return 3x3 state covariance matrix P."""
        return [row[:] for row in self.P]


def compute_estimation_error(
    est_x: float, est_y: float, est_yaw: float, gt_x: float, gt_y: float, gt_yaw: float
) -> Tuple[float, float]:
    """
    Compute Euclidean position error and absolute angular heading error.

    :param est_x: Estimated x coordinate in meters.
    :param est_y: Estimated y coordinate in meters.
    :param est_yaw: Estimated yaw angle in radians.
    :param gt_x: Ground truth x coordinate in meters.
    :param gt_y: Ground truth y coordinate in meters.
    :param gt_yaw: Ground truth yaw angle in radians.
    :return: Tuple of (position_error_m, yaw_error_rad).
    """
    pos_err = math.hypot(est_x - gt_x, est_y - gt_y)
    yaw_err = abs(wrap_angle(est_yaw - gt_yaw))
    return pos_err, yaw_err
