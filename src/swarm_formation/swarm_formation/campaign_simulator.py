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
Headless kinematic campaign simulator mirroring the ROS 2 node pipeline.

This is NOT a physics simulator. Robots are ideal unicycles integrated with forward
Euler. What it does reproduce, step for step, is the information flow of the ROS
stack so that statistical claims about that stack are meaningful:

* neighbor_sensor_node: range/bearing to peers in the observer's true body frame,
  with optional Gaussian noise, limited to the sensing range;
* network_impairment_node: every peer measurement passes through a directed channel
  with Bernoulli drop, base delay and uniform jitter; the latest delivered measurement
  is republished every step until it is older than ``stale_purge_sec``;
* localization_node: a real ``RobotEKF`` per robot, driven by noisy odometry, an
  odometry-integrated heading, known landmark beacons and peer range/bearing;
* robot_node: body-frame measurements rotated with the estimated yaw at *measurement*
  time, only currently delivered peers kept, finite-difference velocities between
  distinct measurements, heartbeat on measurement age, EMA filtering, Huber saturation
  and APF, then mapped to (v, w) (shared code in relative_sensing.py);
* safety_filter_node: the look-ahead CBF-QP on the impaired neighbor list.

Landmark beacons are the obstacle centres of ``collapsed_building.sdf``; they are used
for localization only and are not simulated as physical obstacles.
"""

from dataclasses import dataclass
import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from swarm_formation.cbf_qp import compute_cbf_safe_twist
from swarm_formation.consensus import compute_target_offsets
from swarm_formation.controllers import create_controller
from swarm_formation.formation_metrics import compute_procrustes_error
from swarm_formation.kinematics import compute_unicycle_twist, wrap_angle
from swarm_formation.localization import RobotEKF
from swarm_formation.potential_field import (
    compute_peer_safety_scale,
    compute_relative_peer_apf,
)
from swarm_formation.relative_sensing import compute_heartbeat_mask, update_relative_store

# Obstacle centres of collapsed_building.sdf, used as known localization beacons.
LANDMARKS: List[Tuple[float, float]] = [
    (-1.66, 1.40),
    (1.92, 1.87),
    (0.25, 1.96),
    (0.95, -0.33),
    (-1.80, -1.09),
    (-0.12, -1.89),
    (1.87, -1.24),
]


@dataclass
class CampaignRunConfig:
    """Configuration parameters for a single swarm campaign run."""

    total_robots: int = 5
    formation_radius: float = 2.0
    duration_sec: float = 30.0
    dt: float = 0.05

    # Gains and limits mirror config/swarm_params.yaml (the deployed ROS stack)
    k_p: float = 1.8
    k_d: float = 0.5
    k_yaw: float = 4.0
    v_max: float = 1.4
    omega_max: float = 3.0
    sensing_range: float = 6.0

    # Evaluation thresholds
    convergence_threshold: float = 0.25
    settle_duration: float = 2.0
    collision_threshold: float = 0.50

    # Controller and resilience components
    controller_type: str = 'displacement'
    use_huber: bool = False
    huber_delta: float = 2.0
    use_ema: bool = False
    ema_alpha: float = 0.35
    use_heartbeat: bool = False
    heartbeat_timeout: float = 1.2
    use_apf: bool = False

    # Safety filter
    use_cbf: bool = False
    cbf_safety_dist: float = 0.55
    cbf_gamma: float = 1.5
    cbf_lookahead: float = 0.15
    # None = margin matched to the applied worst-case latency (delay_sec + jitter_sec)
    cbf_delay_bound_sec: Optional[float] = None

    # Channel impairments on the peer measurement stream (network_impairment_node)
    drop_rate: float = 0.0
    delay_sec: float = 0.0
    jitter_sec: float = 0.0
    stale_purge_sec: float = 1.5

    # Sensing and odometry noise
    range_noise_std: float = 0.0
    bearing_noise_std: float = 0.0
    odom_speed_noise_frac: float = 0.02
    odom_yaw_rate_noise_std: float = 0.02
    use_landmarks: bool = True

    # Agent fault: robot stops (motor kill) from dropout_start_sec onward
    dropout_robot_id: int = 0
    dropout_start_sec: float = 0.0

    # Crossing trajectories scenario (challenging initial conditions)
    crossing_scenario: bool = True

    def effective_cbf_delay_bound(self) -> float:
        """Return the latency bound the CBF margin is computed for."""
        if self.cbf_delay_bound_sec is not None:
            return self.cbf_delay_bound_sec
        return max(0.0, self.delay_sec) + max(0.0, self.jitter_sec)


def generate_initial_states(
    total_robots: int,
    crossing: bool,
    rng: np.random.RandomState,
) -> List[Tuple[float, float, float]]:
    """Generate initial poses (x, y, yaw) for swarm robots with optional seed jitter."""
    if crossing:
        # Same spawn points as launch/gazebo_swarm_launch.py
        base_positions = [
            (-3.0, 0.0),
            (3.0, 0.0),
            (0.0, -3.0),
            (0.0, 3.0),
            (2.2, -2.2),
        ]
    else:
        radius = 4.0
        base_positions = [
            (
                radius * math.cos(2.0 * math.pi * i / total_robots),
                radius * math.sin(2.0 * math.pi * i / total_robots),
            )
            for i in range(total_robots)
        ]

    states = []
    for i in range(total_robots):
        if i < len(base_positions):
            bx, by = base_positions[i]
        else:
            bx = float(i) * 1.5 - 4.0
            by = 0.0
        jx = float(rng.uniform(-0.15, 0.15))
        jy = float(rng.uniform(-0.15, 0.15))
        jyaw = float(rng.uniform(-0.2, 0.2))
        states.append((bx + jx, by + jy, jyaw))

    return states


def run_single_simulation(
    config: CampaignRunConfig,
    seed: int,
) -> Dict[str, Any]:
    """Execute one deterministic headless run and return performance metrics."""
    rng = np.random.RandomState(seed)
    n = config.total_robots
    dt = config.dt
    num_steps = int(math.ceil(config.duration_sec / dt))
    cbf_delay_bound = config.effective_cbf_delay_bound()

    states = generate_initial_states(n, config.crossing_scenario, rng)
    cur_x = [float(s[0]) for s in states]
    cur_y = [float(s[1]) for s in states]
    cur_yaw = [float(s[2]) for s in states]

    # Onboard estimators: known (slightly noisy) initial pose, as in the launch file
    ekfs: List[RobotEKF] = []
    init_yaw_param = [0.0] * n
    for i in range(n):
        init_yaw_param[i] = cur_yaw[i] + float(rng.normal(0.0, 0.02))
        ekfs.append(RobotEKF(
            init_x=cur_x[i] + float(rng.normal(0.0, 0.03)),
            init_y=cur_y[i] + float(rng.normal(0.0, 0.03)),
            init_yaw=init_yaw_param[i],
        ))
    odom_yaw = [0.0] * n  # integrated wheel-odometry heading relative to start

    controller = create_controller(config.controller_type)
    offsets_by_robot = {
        i: compute_target_offsets(i, n, config.formation_radius) for i in range(1, n + 1)
    }

    # Channel state. Packet: (deliver_time, src_j, range, bearing, measurement_time)
    in_flight: List[List[Tuple[float, int, float, float, float]]] = [[] for _ in range(n)]
    delivered: List[Dict[int, Tuple[float, float, float, float]]] = [{} for _ in range(n)]
    # Estimated heading per control step (robot_node / safety_filter_node YawHistory)
    est_yaw_hist: List[List[float]] = [[] for _ in range(n)]
    packets_sent = 0
    packets_dropped = 0
    latency_sum = 0.0
    latency_count = 0

    # robot_node state (per observer): pid -> (dx, dy, vx, vy, measurement_time)
    rel_store: List[Dict[int, Tuple[float, float, float, float, float]]] = [
        {} for _ in range(n)
    ]
    ema_store: List[Dict[int, Tuple[float, float, float, float]]] = [{} for _ in range(n)]
    ema_valid_time: List[Dict[int, float]] = [{} for _ in range(n)]
    # robot_node's own commanded velocity (used for damping)
    cmd_vel_xy = [[0.0, 0.0] for _ in range(n)]

    shape_errors = np.zeros(num_steps)
    est_errors = np.zeros(num_steps)
    yaw_errors = np.zeros(num_steps)
    running_min_dist = float('inf')
    total_collision_events = 0
    effort_lin = 0.0
    effort_ang = 0.0
    path_length = 0.0
    total_interventions = 0.0
    consecutive_settled_time = 0.0
    is_converged = False
    convergence_time: Optional[float] = None

    for k in range(num_steps):
        t = k * dt

        # ---------------- 1. Ground-truth evaluation ----------------
        pts = [(cur_x[i], cur_y[i]) for i in range(n)]
        se = compute_procrustes_error(pts, config.formation_radius, n)
        shape_errors[k] = se
        for i in range(n):
            for j in range(i + 1, n):
                d = math.hypot(cur_x[i] - cur_x[j], cur_y[i] - cur_y[j])
                running_min_dist = min(running_min_dist, d)
                if d < config.collision_threshold:
                    total_collision_events += 1

        if se < config.convergence_threshold:
            consecutive_settled_time += dt
            if consecutive_settled_time >= config.settle_duration and not is_converged:
                is_converged = True
                convergence_time = t - config.settle_duration
        else:
            consecutive_settled_time = 0.0

        est_pos_err = 0.0
        est_yaw_err = 0.0
        for i in range(n):
            ex, ey, eyaw = ekfs[i].get_pose()
            est_pos_err += math.hypot(ex - cur_x[i], ey - cur_y[i])
            est_yaw_err += abs(wrap_angle(eyaw - cur_yaw[i]))
        est_errors[k] = est_pos_err / n
        yaw_errors[k] = est_yaw_err / n

        # ---------------- 2. Sensing + channel transmission ----------------
        for i in range(n):
            for j in range(n):
                if i == j:
                    continue
                dx = cur_x[j] - cur_x[i]
                dy = cur_y[j] - cur_y[i]
                dist = math.hypot(dx, dy)
                if dist > config.sensing_range or dist < 1e-6:
                    continue
                bearing = wrap_angle(math.atan2(dy, dx) - cur_yaw[i])
                if config.range_noise_std > 0.0:
                    dist = max(0.01, dist + rng.normal(0.0, config.range_noise_std))
                if config.bearing_noise_std > 0.0:
                    bearing = wrap_angle(bearing + rng.normal(0.0, config.bearing_noise_std))

                packets_sent += 1
                if config.drop_rate > 0.0 and rng.uniform() < config.drop_rate:
                    packets_dropped += 1
                    continue
                latency = max(0.0, config.delay_sec)
                if config.jitter_sec > 0.0:
                    jitter = rng.uniform(-config.jitter_sec, config.jitter_sec)
                    latency = max(0.0, latency + jitter)
                latency_sum += latency
                latency_count += 1
                in_flight[i].append((t + latency, j + 1, dist, bearing, t))

        # Deliver matured packets (in arrival order; reordering under jitter is kept)
        for i in range(n):
            remaining = []
            matured = []
            for pkt in in_flight[i]:
                if pkt[0] <= t + 1e-9:
                    matured.append(pkt)
                else:
                    remaining.append(pkt)
            in_flight[i] = remaining
            matured.sort(key=lambda p: p[0])
            for deliv_t, src, rng_m, brg_m, t_meas in matured:
                delivered[i][src] = (rng_m, brg_m, deliv_t, t_meas)
                # localization_node: cooperative update with the peer's current estimate
                px, py, _ = ekfs[src - 1].get_pose()
                ekfs[i].update_range_bearing(
                    px, py, rng_m, brg_m,
                    var_range=max(config.range_noise_std ** 2, 0.05),
                    var_bearing=max(config.bearing_noise_std ** 2, 0.03),
                )

        # localization_node: landmark updates (onboard sensing, not networked)
        if config.use_landmarks:
            for i in range(n):
                for lx, ly in LANDMARKS:
                    dx = lx - cur_x[i]
                    dy = ly - cur_y[i]
                    dist = math.hypot(dx, dy)
                    if dist > config.sensing_range or dist < 1e-6:
                        continue
                    bearing = wrap_angle(math.atan2(dy, dx) - cur_yaw[i])
                    if config.range_noise_std > 0.0:
                        dist = max(0.01, dist + rng.normal(0.0, config.range_noise_std))
                    if config.bearing_noise_std > 0.0:
                        bearing = wrap_angle(
                            bearing + rng.normal(0.0, config.bearing_noise_std)
                        )
                    ekfs[i].update_range_bearing(
                        lx, ly, dist, bearing, var_range=0.04, var_bearing=0.02
                    )

        # ---------------- 3. Control (robot_node + safety_filter_node) ----------------
        cmd_v = [0.0] * n
        cmd_w = [0.0] * n
        for i in range(n):
            robot_id = i + 1
            if config.dropout_robot_id == robot_id and t >= config.dropout_start_sec:
                cmd_vel_xy[i] = [0.0, 0.0]
                continue

            _, _, est_yaw = ekfs[i].get_pose()
            est_yaw_hist[i].append(est_yaw)
            hist = est_yaw_hist[i]

            def yaw_at(t_meas: float, hist: List[float] = hist) -> float:
                return hist[min(len(hist) - 1, max(0, int(round(t_meas / dt))))]

            # Impaired NeighborList republished this step (stale entries purged by the
            # network node on delivery age), handled exactly as robot_node does.
            measurements = []
            for src, (rng_m, brg_m, deliv_t, t_meas) in delivered[i].items():
                if t - deliv_t <= config.stale_purge_sec:
                    measurements.append(
                        (src, rng_m * math.cos(brg_m), rng_m * math.sin(brg_m), t_meas)
                    )
            rel_store[i] = update_relative_store(rel_store[i], measurements, yaw_at)
            current_list = {pid: (e[0], e[1]) for pid, e in rel_store[i].items()}

            other_ids = [pid for pid in range(1, n + 1) if pid != robot_id]
            if config.use_heartbeat:
                active_mask: Dict[int, bool] = compute_heartbeat_mask(
                    rel_store[i], other_ids, t, config.heartbeat_timeout
                )
            else:
                active_mask = {pid: pid in rel_store[i] for pid in other_ids}

            # Optional EMA with jump rejection (robot_node._compute_resilient_control)
            if config.use_ema:
                for pid, (rdx, rdy, rvx, rvy, _) in rel_store[i].items():
                    gap = t - ema_valid_time[i].get(pid, -1e9)
                    if gap > 0.8 or pid not in ema_store[i]:
                        ema_store[i][pid] = (rdx, rdy, rvx, rvy)
                        ema_valid_time[i][pid] = t
                    else:
                        pdx, pdy, pvx, pvy = ema_store[i][pid]
                        if math.hypot(rdx - pdx, rdy - pdy) < 2.5:
                            a = config.ema_alpha
                            ema_store[i][pid] = (
                                a * rdx + (1 - a) * pdx,
                                a * rdy + (1 - a) * pdy,
                                a * rvx + (1 - a) * pvx,
                                a * rvy + (1 - a) * pvy,
                            )
                            ema_valid_time[i][pid] = t
                peer_states = dict(ema_store[i])
            else:
                peer_states = {pid: e[:4] for pid, e in rel_store[i].items()}

            rel_disp = {pid: (s[0], s[1]) for pid, s in peer_states.items()}
            fx, fy = controller.compute_control_force(
                robot_id, n, config.formation_radius, rel_disp, config.k_p,
                huber_delta=config.huber_delta if config.use_huber else 100.0,
                active_mask=active_mask,
            )

            if config.use_apf:
                apf_peers = {
                    p: st for p, st in peer_states.items() if active_mask.get(p, False)
                }
                fx_apf, fy_apf, min_peer = compute_relative_peer_apf(apf_peers)
                scale = compute_peer_safety_scale(min_peer)
                fx = fx * scale + fx_apf
                fy = fy * scale + fy_apf

            fx -= config.k_d * cmd_vel_xy[i][0]
            fy -= config.k_d * cmd_vel_xy[i][1]

            raw_v, raw_w = compute_unicycle_twist(
                fx, fy, est_yaw, config.k_yaw, config.v_max, config.omega_max
            )
            cmd_vel_xy[i] = [raw_v * math.cos(est_yaw), raw_v * math.sin(est_yaw)]

            if config.use_cbf:
                safe_v, safe_w, interv = compute_cbf_safe_twist(
                    v_raw=raw_v,
                    w_raw=raw_w,
                    robot_yaw=est_yaw,
                    relative_peers=current_list,
                    relative_obstacles=[],
                    safety_dist_peer=config.cbf_safety_dist,
                    gamma_peer=config.cbf_gamma,
                    delay_bound_sec=cbf_delay_bound,
                    max_linear_vel=config.v_max,
                    max_angular_vel=config.omega_max,
                    lookahead_dist=config.cbf_lookahead,
                )
                cmd_v[i], cmd_w[i] = safe_v, safe_w
                total_interventions += interv * dt
            else:
                cmd_v[i], cmd_w[i] = raw_v, raw_w

        # ---------------- 4. Plant integration + odometry-driven EKF prediction ----------
        for i in range(n):
            v, w = cmd_v[i], cmd_w[i]
            effort_lin += v * v * dt
            effort_ang += w * w * dt
            path_length += abs(v) * dt
            cur_x[i] += v * math.cos(cur_yaw[i]) * dt
            cur_y[i] += v * math.sin(cur_yaw[i]) * dt
            cur_yaw[i] = wrap_angle(cur_yaw[i] + w * dt)

            v_odom = v * (1.0 + float(rng.normal(0.0, config.odom_speed_noise_frac)))
            w_odom = w + float(rng.normal(0.0, config.odom_yaw_rate_noise_std))
            odom_yaw[i] = wrap_angle(odom_yaw[i] + w_odom * dt)
            ekfs[i].predict(v_odom, w_odom, dt)
            ekfs[i].update_heading(wrap_angle(init_yaw_param[i] + odom_yaw[i]), var_yaw=0.01)

    return {
        'seed': seed,
        'final_shape_error': float(shape_errors[-1]),
        'min_shape_error': float(np.min(shape_errors)),
        'global_min_distance': float(running_min_dist),
        'total_collisions': int(total_collision_events),
        'empirically_collision_free': bool(running_min_dist >= config.collision_threshold),
        'is_converged': bool(is_converged),
        'convergence_time': convergence_time,
        'mean_est_error': float(np.mean(est_errors)),
        'mean_yaw_error': float(np.mean(yaw_errors)),
        'control_effort_lin': float(effort_lin),
        'control_effort_ang': float(effort_ang),
        'path_length': float(path_length),
        'total_safety_interventions': float(total_interventions),
        'packet_drop_fraction': float(packets_dropped / packets_sent) if packets_sent else 0.0,
        'mean_applied_latency_sec': float(latency_sum / latency_count) if latency_count else 0.0,
        'cbf_delay_bound_sec': float(cbf_delay_bound if config.use_cbf else 0.0),
    }
