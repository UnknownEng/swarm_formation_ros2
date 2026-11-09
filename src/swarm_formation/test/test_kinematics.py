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


def test_unicycle_mapper_stays_still_for_small_residual_forces() -> None:
    """Below the stop band the robot neither drives nor turns (no spinning at rest)."""
    from swarm_formation.kinematics import UnicycleMapper

    mapper = UnicycleMapper(stop_force=0.15, resume_force=0.30)
    assert mapper.to_twist(0.2, 0.2, 0.0) == (0.0, 0.0)      # below resume force: stays stopped
    v, w = mapper.to_twist(1.0, 0.0, 0.0)                     # clearly above: moves
    assert v > 0.0 and w == 0.0
    assert mapper.to_twist(0.05, 0.1, 0.0) == (0.0, 0.0)     # dropped below stop force: stops
    assert mapper.to_twist(0.25, 0.0, 0.0) == (0.0, 0.0)     # hysteresis: 0.25 < resume 0.30


def test_unicycle_mapper_scales_turn_rate_with_force() -> None:
    """A small sideways force turns gently; a large one turns at the full rate."""
    from swarm_formation.kinematics import UnicycleMapper

    small = UnicycleMapper(stop_force=0.15, resume_force=0.30)
    small.to_twist(1.0, 0.0, 0.0)                              # leave the stopped state
    _, w_small = small.to_twist(0.0, 0.4, 0.0)                 # 90 deg error, |f| = 0.4
    big = UnicycleMapper(stop_force=0.15, resume_force=0.30)
    _, w_big = big.to_twist(0.0, 2.0, 0.0)                     # 90 deg error, |f| = 2.0
    assert 0.0 < w_small < w_big
    assert math.isclose(w_big, 3.0)                       # saturated at max_ang_vel
    assert math.isclose(w_small, 4.0 * (math.pi / 2) * 0.4)


def test_unicycle_mapper_reverse_is_opt_in() -> None:
    """Reverse driving is off by default (it fought the CBF look-ahead point)."""
    from swarm_formation.kinematics import UnicycleMapper

    v_fwd, _ = UnicycleMapper().to_twist(-1.0, 0.0, 0.0)
    assert v_fwd == 0.0                                   # force behind: turn, no reverse
    v_rev, _ = UnicycleMapper(allow_reverse=True).to_twist(-1.0, 0.0, 0.0)
    assert v_rev < 0.0


def test_default_stop_band_ignores_residual_forces_of_a_settled_formation() -> None:
    """Gazebo: a ~7 cm settled error gives forces of 0.4-0.9; the default band ignores them."""
    from swarm_formation.kinematics import UnicycleMapper

    mapper = UnicycleMapper()
    for fx, fy in ((0.4, 0.1), (-0.3, 0.6), (0.0, -0.85)):
        assert mapper.to_twist(fx, fy, 0.0) == (0.0, 0.0)
    v, _ = mapper.to_twist(1.5, 0.0, 0.0)      # a real disturbance still gets corrected
    assert v > 0.0
