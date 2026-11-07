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

"""Unit tests for Cooperative Extended Kalman Filter (EKF) localization library."""

import math

import pytest
from swarm_formation.localization import compute_estimation_error, RobotEKF


def test_ekf_initialization() -> None:
    """Verify default and customized initialization of RobotEKF state and covariance."""
    ekf = RobotEKF(init_x=1.5, init_y=-2.0, init_yaw=math.pi / 2)
    x, y, yaw = ekf.get_pose()
    assert math.isclose(x, 1.5, abs_tol=1e-5)
    assert math.isclose(y, -2.0, abs_tol=1e-5)
    assert math.isclose(yaw, math.pi / 2, abs_tol=1e-5)

    cov = ekf.get_covariance()
    assert len(cov) == 3 and len(cov[0]) == 3
    assert cov[0][0] > 0.0
    assert cov[1][1] > 0.0
    assert cov[2][2] > 0.0
    # Symmetric check
    assert math.isclose(cov[0][1], cov[1][0], abs_tol=1e-6)


def test_ekf_prediction_straight_line() -> None:
    """Verify prediction step correctly propagates position along heading."""
    ekf = RobotEKF(init_x=0.0, init_y=0.0, init_yaw=0.0)
    initial_cov_trace = ekf.P[0][0] + ekf.P[1][1]

    # Move forward at 1.0 m/s for 1.0 second
    ekf.predict(linear_vel=1.0, angular_vel=0.0, dt=1.0)
    x, y, yaw = ekf.get_pose()

    assert math.isclose(x, 1.0, abs_tol=1e-4)
    assert math.isclose(y, 0.0, abs_tol=1e-4)
    assert math.isclose(yaw, 0.0, abs_tol=1e-4)

    # Prediction must increase state uncertainty
    new_cov_trace = ekf.P[0][0] + ekf.P[1][1]
    assert new_cov_trace > initial_cov_trace


def test_ekf_prediction_turn() -> None:
    """Verify prediction step correctly integrates angular rotational velocity."""
    ekf = RobotEKF(init_x=0.0, init_y=0.0, init_yaw=0.0)

    # Pure turn of pi/2 rad/s for 1.0 second
    ekf.predict(linear_vel=0.0, angular_vel=math.pi / 2, dt=1.0)
    x, y, yaw = ekf.get_pose()

    assert math.isclose(x, 0.0, abs_tol=1e-4)
    assert math.isclose(y, 0.0, abs_tol=1e-4)
    assert math.isclose(yaw, math.pi / 2, abs_tol=1e-4)


def test_ekf_heading_update() -> None:
    """Verify heading correction pulls estimated yaw toward measurement."""
    ekf = RobotEKF(init_x=0.0, init_y=0.0, init_yaw=0.0)
    prior_yaw_var = ekf.P[2][2]

    # Measured heading is 0.3 rad
    ekf.update_heading(0.3, var_yaw=0.01)
    _, _, post_yaw = ekf.get_pose()
    post_yaw_var = ekf.P[2][2]

    assert 0.0 < post_yaw <= 0.3
    assert post_yaw_var < prior_yaw_var


def test_ekf_landmark_correction() -> None:
    """Verify range-bearing observation to known landmark pulls estimate and shrinks covariance."""
    # Robot is at (0.0, 0.0) with yaw=0, but belief has an offset (0.2, -0.1)
    ekf = RobotEKF(init_x=0.2, init_y=-0.1, init_yaw=0.0)
    prior_cov_trace = ekf.P[0][0] + ekf.P[1][1]

    # Known landmark at (2.0, 0.0). True measurement from (0,0) is range=2.0, bearing=0.0
    ekf.update_range_bearing(
        target_x=2.0,
        target_y=0.0,
        measured_dist=2.0,
        measured_bearing=0.0,
    )
    post_x, post_y, _ = ekf.get_pose()
    post_cov_trace = ekf.P[0][0] + ekf.P[1][1]

    # Correction should pull estimated position closer to true origin (0.0, 0.0)
    assert abs(post_x) < 0.2
    assert abs(post_y) < 0.1
    # Measurement update must reduce covariance trace
    assert post_cov_trace < prior_cov_trace


def test_ekf_cooperative_peer_correction() -> None:
    """Verify relative range-bearing measurement to a peer robot updates state."""
    # Robot 1 at (0.0, 0.0), Peer 2 at (1.0, 1.0)
    ekf = RobotEKF(init_x=0.1, init_y=-0.1, init_yaw=0.0)
    peer_x, peer_y = 1.0, 1.0
    true_range = math.hypot(1.0, 1.0)
    true_bearing = math.atan2(1.0, 1.0)

    ekf.update_range_bearing(
        target_x=peer_x,
        target_y=peer_y,
        measured_dist=true_range,
        measured_bearing=true_bearing,
        var_range=0.02,
        var_bearing=0.01,
    )
    post_x, post_y, _ = ekf.get_pose()
    assert math.hypot(post_x, post_y) < math.hypot(0.1, -0.1)


def test_compute_estimation_error() -> None:
    """Verify Euclidean position and angular heading error computation."""
    pos_err, yaw_err = compute_estimation_error(
        est_x=1.0,
        est_y=2.0,
        est_yaw=math.pi,
        gt_x=1.3,
        gt_y=2.4,
        gt_yaw=-math.pi + 0.1,
    )
    assert math.isclose(pos_err, 0.5, abs_tol=1e-5)
    assert math.isclose(yaw_err, 0.1, abs_tol=1e-5)


def test_ekf_bounded_drift_with_landmarks() -> None:
    """Simulate a noisy odometry trajectory, verifying landmark updates bound drift."""
    ekf = RobotEKF(init_x=0.0, init_y=0.0, init_yaw=0.0)
    gt_x, gt_y, gt_yaw = 0.0, 0.0, 0.0

    landmark_x, landmark_y = 3.0, 3.0
    dt = 0.1
    v = 0.5
    w = 0.1

    # Drift over 50 steps
    for _ in range(50):
        # Update ground truth
        gt_x += v * dt * math.cos(gt_yaw + w * dt * 0.5)
        gt_y += v * dt * math.sin(gt_yaw + w * dt * 0.5)
        gt_yaw += w * dt

        # Noisy odometry measurement (simulated bias/noise)
        v_noisy = v + 0.05
        w_noisy = w + 0.02
        ekf.predict(v_noisy, w_noisy, dt)

        # Periodic landmark range-bearing correction every 5 steps
        true_dist = math.hypot(landmark_x - gt_x, landmark_y - gt_y)
        true_bearing = math.atan2(landmark_y - gt_y, landmark_x - gt_x) - gt_yaw
        ekf.update_range_bearing(
            target_x=landmark_x,
            target_y=landmark_y,
            measured_dist=true_dist,
            measured_bearing=true_bearing,
        )

    final_x, final_y, final_yaw = ekf.get_pose()
    pos_err, _ = compute_estimation_error(
        final_x, final_y, final_yaw, gt_x, gt_y, gt_yaw
    )
    # Bounded error due to landmark corrections
    assert pos_err < 0.25, f'Estimation error {pos_err}m exceeded bound 0.25m'


def _peer_update(target_cov, seed: int = 0, trials: int = 2000):
    """Monte Carlo NEES of one peer range/bearing update; return (mean NEES, mean |dx|)."""
    import random
    rng = random.Random(seed)
    p0 = 0.01                      # own prior position variance (m^2)
    sigma = 0.25                   # peer estimate position variance (m^2)
    r_rng, r_brg = 0.0025, 0.0004  # measurement noise
    nees = 0.0
    shift = 0.0
    for _ in range(trials):
        ekf = RobotEKF(
            init_x=rng.gauss(0.0, math.sqrt(p0)),
            init_y=rng.gauss(0.0, math.sqrt(p0)),
            init_yaw=0.0,
        )
        ekf.P = [[p0, 0.0, 0.0], [0.0, p0, 0.0], [0.0, 0.0, 1e-6]]
        x0 = ekf.x
        peer_x = 2.0 + rng.gauss(0.0, math.sqrt(sigma))
        peer_y = rng.gauss(0.0, math.sqrt(sigma))
        dist = 2.0 + rng.gauss(0.0, math.sqrt(r_rng))
        brg = rng.gauss(0.0, math.sqrt(r_brg))
        ekf.update_range_bearing(
            peer_x, peer_y, dist, brg, var_range=r_rng, var_bearing=r_brg,
            target_cov=target_cov(sigma),
        )
        ex, ey = ekf.x, ekf.y
        a, b, d = ekf.P[0][0], ekf.P[0][1], ekf.P[1][1]
        det = a * d - b * b
        nees += (d * ex * ex - 2 * b * ex * ey + a * ey * ey) / det
        shift += abs(ekf.x - x0)
    return nees / trials, shift / trials


def test_peer_covariance_zero_matches_exact_target() -> None:
    """A zero target covariance gives exactly the old (exact-beacon) update."""
    a = RobotEKF(init_x=0.1, init_y=-0.2, init_yaw=0.05)
    b = RobotEKF(init_x=0.1, init_y=-0.2, init_yaw=0.05)
    a.update_range_bearing(2.0, 1.0, 2.3, 0.4, 0.01, 0.002)
    b.update_range_bearing(2.0, 1.0, 2.3, 0.4, 0.01, 0.002,
                           target_cov=[[0.0, 0.0], [0.0, 0.0]])
    assert a.get_pose() == pytest.approx(b.get_pose())
    flat_a = [v for row in a.get_covariance() for v in row]
    flat_b = [v for row in b.get_covariance() for v in row]
    assert flat_a == pytest.approx(flat_b)


def test_peer_covariance_keeps_filter_consistent() -> None:
    """
    An uncertain peer treated as exact makes the EKF overconfident (NEES >> 2).

    Propagating the peer covariance restores consistency (2-DoF NEES close to 2) and
    stops the uncertain peer from dragging the estimate.
    """
    nees_exact, shift_exact = _peer_update(lambda s: None)
    nees_cov, shift_cov = _peer_update(lambda s: [[s, 0.0], [0.0, s]])
    assert nees_exact > 10.0
    assert 1.6 < nees_cov < 2.6
    assert shift_cov < 0.5 * shift_exact
