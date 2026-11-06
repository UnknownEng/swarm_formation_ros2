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

"""Unit tests for differential-drive unicycle kinematics."""

import math

from swarm_formation.kinematics import compute_unicycle_twist, wrap_angle


def test_wrap_angle():
    """Verify angle wrapping to [-pi, pi]."""
    assert math.isclose(wrap_angle(0.0), 0.0)
    assert math.isclose(wrap_angle(math.pi), math.pi)
    assert math.isclose(wrap_angle(-math.pi), -math.pi)
    assert math.isclose(wrap_angle(3.0 * math.pi), math.pi, abs_tol=1e-6)
    assert math.isclose(wrap_angle(-3.0 * math.pi), -math.pi, abs_tol=1e-6)


def test_unicycle_deadband():
    """Verify that forces below deadband result in zero velocities."""
    v, w = compute_unicycle_twist(0.01, 0.01, current_yaw=0.0, deadband=0.03)
    assert v == 0.0
    assert w == 0.0


def test_unicycle_aligned_motion():
    """Verify forward acceleration when robot heading is aligned with force vector."""
    # Force strictly in +x, robot facing +x (yaw=0)
    v, w = compute_unicycle_twist(1.0, 0.0, current_yaw=0.0)
    assert math.isclose(v, 1.0, abs_tol=1e-4)
    assert math.isclose(w, 0.0, abs_tol=1e-4)


def test_unicycle_perpendicular_heading():
    """Verify zero forward velocity and pure rotational pivot when heading error is pi/2."""
    # Force in +y (desired yaw pi/2), robot facing +x (yaw=0)
    v, w = compute_unicycle_twist(0.0, 1.0, current_yaw=0.0)
    assert math.isclose(v, 0.0, abs_tol=1e-4)
    assert w > 0.0  # Counter-clockwise rotation


def test_unicycle_velocity_limits():
    """Verify saturation at max_linear_vel and max_angular_vel."""
    v, w = compute_unicycle_twist(
        10.0, 0.0, current_yaw=0.0, max_lin_vel=1.4, max_ang_vel=3.0
    )
    assert math.isclose(v, 1.4, abs_tol=1e-4)
    assert math.isclose(w, 0.0, abs_tol=1e-4)

    v, w = compute_unicycle_twist(
        0.0, 10.0, current_yaw=-math.pi, max_lin_vel=1.4, max_ang_vel=3.0
    )
    assert abs(w) <= 3.0
