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

"""High-fidelity multi-robot swarm dynamics and campaign simulation engine."""

from dataclasses import dataclass
import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from swarm_formation.cbf_qp import compute_cbf_safe_twist
from swarm_formation.consensus import (
    compute_baseline_consensus,
    compute_huber_consensus,
    compute_target_offsets,
)
from swarm_formation.kinematics import compute_unicycle_twist
from swarm_formation.potential_field import compute_peer_apf
from swarm_formation.rosbag_processor import RosbagProcessor


@dataclass
class CampaignRunConfig:
    """Configuration parameters for a single swarm campaign run."""

    total_robots: int = 5
    formation_radius: float = 2.0
    duration_sec: float = 30.0
    dt: float = 0.05
    k_p: float = 0.8
    v_max: float = 1.4
    omega_max: float = 2.8
    sensing_range: float = 6.0
    convergence_threshold: float = 0.25
    settle_duration: float = 2.0
    collision_threshold: float = 0.50

    # Controller & resilient flags
    controller_type: str = 'displacement'
    use_resilient: bool = False
    use_huber: bool = False
    use_ema: bool = False
    use_heartbeat: bool = False
    use_cbf: bool = False
    use_apf: bool = False

    # Impairments & Noise
    drop_rate: float = 0.0
    delay_sec: float = 0.0
    jitter_sec: float = 0.0
    sensor_noise_sigma: float = 0.0
    dropout_robot_id: int = 0
    dropout_start_sec: float = 0.0

    # Crossing trajectories scenario (challenging initial conditions)
    crossing_scenario: bool = True


def generate_initial_states(
    total_robots: int,
    crossing: bool,
    rng: np.random.RandomState,
) -> List[Tuple[float, float, float]]:
    """Generate initial poses (x, y, yaw) for swarm robots with optional seed jitter."""
    if crossing:
        # Standard crossing trajectories: robots start on opposite perimeter points
        base_positions = [
            (-3.0, 0.0),
            (3.0, 0.0),
            (0.0, -3.0),
            (0.0, 3.0),
            (2.2, -2.2),
        ]
    else:
        # Dispersed positions
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
        # Add small seed perturbation (+/- 0.15m)
        jx = float(rng.uniform(-0.15, 0.15))
        jy = float(rng.uniform(-0.15, 0.15))
        jyaw = float(rng.uniform(-0.2, 0.2))
        states.append((bx + jx, by + jy, jyaw))

    return states


def run_single_simulation(
    config: CampaignRunConfig,
    seed: int,
) -> Dict[str, Any]:
    """Execute single deterministic headless run and return performance metrics."""
    rng = np.random.RandomState(seed)
    num_steps = int(math.ceil(config.duration_sec / config.dt))
    time_series = np.linspace(0.0, config.duration_sec, num_steps)

    # Initialize robot kinematic states: robot_id -> [x, y, yaw, vx, vy]
    states = generate_initial_states(config.total_robots, config.crossing_scenario, rng)
    cur_x = np.array([s[0] for s in states], dtype=float)
    cur_y = np.array([s[1] for s in states], dtype=float)
    cur_yaw = np.array([s[2] for s in states], dtype=float)
    cur_v = np.zeros(config.total_robots, dtype=float)

    # Estimation states for cooperative EKF
    est_x = cur_x.copy() + rng.normal(0.0, 0.03, config.total_robots)
    est_y = cur_y.copy() + rng.normal(0.0, 0.03, config.total_robots)

    # Filter state history for EMA and Heartbeat
    last_u = np.zeros((config.total_robots, 2), dtype=float)
    last_heard_time = np.zeros((config.total_robots, config.total_robots), dtype=float)

    # Target regular polygon offsets
    offsets_by_robot = {
        i: compute_target_offsets(i, config.total_robots, config.formation_radius)
        for i in range(1, config.total_robots + 1)
    }

    # Tracking metrics
    shape_errors = np.zeros(num_steps)
    min_distances = np.zeros(num_steps)
    est_errors = np.zeros(num_steps)
    step_collisions = np.zeros(num_steps, dtype=int)
    total_energy = 0.0
    total_interventions = 0.0

    running_min_dist = float('inf')
    total_collision_events = 0

    consecutive_settled_time = 0.0
    is_converged = False
    convergence_time: Optional[float] = None

    # Step simulation loop
    for k in range(num_steps):
        t = time_series[k]

        # 1. Evaluate current ground-truth shape error and min distances
        current_pts = [(cur_x[i], cur_y[i]) for i in range(config.total_robots)]
        se = RosbagProcessor.compute_procrustes_error(
            current_pts, config.formation_radius, config.total_robots
        )
        shape_errors[k] = se

        # Pairwise distance check
        step_min = float('inf')
        colls = 0
        for i in range(config.total_robots):
            for j in range(i + 1, config.total_robots):
                d = math.hypot(cur_x[i] - cur_x[j], cur_y[i] - cur_y[j])
                if d < step_min:
                    step_min = d
                if d < config.collision_threshold:
                    colls += 1

        min_distances[k] = step_min
        if step_min < running_min_dist:
            running_min_dist = step_min

        if colls > 0:
            total_collision_events += colls
        step_collisions[k] = colls

        # Convergence tracking
        if se < config.convergence_threshold:
            consecutive_settled_time += config.dt
            if consecutive_settled_time >= config.settle_duration and not is_converged:
                is_converged = True
                convergence_time = t - config.settle_duration
        else:
            consecutive_settled_time = 0.0

        # Mean estimation error
        mean_est = np.mean(np.hypot(est_x - cur_x, est_y - cur_y))
        est_errors[k] = mean_est

        # 2. Control computation for each robot
        cmd_v = np.zeros(config.total_robots)
        cmd_w = np.zeros(config.total_robots)

        for i in range(config.total_robots):
            robot_id = i + 1

            # Check robot dropout fault
            if (
                config.dropout_robot_id == robot_id
                and t >= config.dropout_start_sec
            ):
                cmd_v[i] = 0.0
                cmd_w[i] = 0.0
                continue

            # Sensing neighbors within sensing range
            visible_displacements: Dict[int, Tuple[float, float]] = {}
            filtered_neighbor_states: Dict[int, Tuple[float, float, float, float]] = {}
            active_mask: Dict[int, bool] = {}

            for j in range(config.total_robots):
                if i == j:
                    continue
                peer_id = j + 1

                # Distance check
                true_dist = math.hypot(cur_x[j] - cur_x[i], cur_y[j] - cur_y[i])
                if true_dist > config.sensing_range:
                    continue  # Out of range

                # Packet drop check
                if config.drop_rate > 0.0 and rng.uniform(0.0, 1.0) < config.drop_rate:
                    continue  # Dropped packet

                # Update heartbeat
                last_heard_time[i, j] = t

                # Relative measurement with noise
                noise_x = (
                    rng.normal(0.0, config.sensor_noise_sigma)
                    if config.sensor_noise_sigma > 0.0
                    else 0.0
                )
                noise_y = (
                    rng.normal(0.0, config.sensor_noise_sigma)
                    if config.sensor_noise_sigma > 0.0
                    else 0.0
                )
                dx = (cur_x[j] - cur_x[i]) + noise_x
                dy = (cur_y[j] - cur_y[i]) + noise_y

                visible_displacements[peer_id] = (dx, dy)
                filtered_neighbor_states[peer_id] = (
                    est_x[j] + noise_x,
                    est_y[j] + noise_y,
                    cur_v[j] * math.cos(cur_yaw[j]),
                    cur_v[j] * math.sin(cur_yaw[j]),
                )
                active_mask[peer_id] = True

            # Check heartbeat timeouts (1.5s) if enabled
            if config.use_heartbeat:
                for j in range(config.total_robots):
                    if i == j:
                        continue
                    peer_id = j + 1
                    if peer_id not in active_mask:
                        if t - last_heard_time[i, j] < 1.5 and last_heard_time[i, j] > 0.0:
                            # Dead-reckon from last known position
                            active_mask[peer_id] = True
                            dx = est_x[j] - est_x[i]
                            dy = est_y[j] - est_y[i]
                            visible_displacements[peer_id] = (dx, dy)

            # Consensus force computation
            offsets = offsets_by_robot[robot_id]
            if config.use_huber or (config.use_resilient and not config.use_ema):
                fx, fy, _ = compute_huber_consensus(
                    est_x[i],
                    est_y[i],
                    offsets,
                    filtered_neighbor_states,
                    active_mask,
                    k_p=config.k_p,
                    huber_delta=2.0,
                )
            else:
                # Baseline linear consensus
                baseline_states = {
                    pid: (st[0], st[1], st[2], st[3], t)
                    for pid, st in filtered_neighbor_states.items()
                }
                fx, fy = compute_baseline_consensus(
                    est_x[i],
                    est_y[i],
                    offsets,
                    baseline_states,
                    k_p=config.k_p,
                )

            # Optional APF avoidance force
            if config.use_apf or (config.use_resilient and not config.use_cbf):
                fx_apf, fy_apf, _ = compute_peer_apf(
                    pos_x=est_x[i],
                    pos_y=est_y[i],
                    vel_x=cur_v[i] * math.cos(cur_yaw[i]),
                    vel_y=cur_v[i] * math.sin(cur_yaw[i]),
                    filtered_states=filtered_neighbor_states,
                )
                fx += fx_apf
                fy += fy_apf

            # EMA velocity smoothing if enabled
            if config.use_ema:
                alpha = 0.3
                fx = alpha * last_u[i, 0] + (1.0 - alpha) * fx
                fy = alpha * last_u[i, 1] + (1.0 - alpha) * fy
                last_u[i] = (fx, fy)

            # Convert to raw unicycle command
            raw_v, raw_w = compute_unicycle_twist(
                fx=fx,
                fy=fy,
                current_yaw=cur_yaw[i],
                k_yaw=1.5,
                max_lin_vel=config.v_max,
                max_ang_vel=config.omega_max,
            )

            # Apply CBF-QP safety filter if enabled
            if config.use_cbf:
                safe_v, safe_w, delta_u = compute_cbf_safe_twist(
                    v_raw=raw_v,
                    w_raw=raw_w,
                    robot_yaw=cur_yaw[i],
                    relative_peers=visible_displacements,
                    relative_obstacles=[],
                    safety_dist_peer=0.55,
                    gamma_peer=1.5,
                    delay_bound_sec=config.delay_sec,
                    max_linear_vel=config.v_max,
                    max_angular_vel=config.omega_max,
                )
                cmd_v[i] = safe_v
                cmd_w[i] = safe_w
                total_interventions += delta_u
            else:
                cmd_v[i] = raw_v
                cmd_w[i] = raw_w

        # 3. Kinematic state integration
        for i in range(config.total_robots):
            v = cmd_v[i]
            w = cmd_w[i]
            total_energy += (v ** 2 + w ** 2) * config.dt

            # Unicycle integration
            cur_x[i] += v * math.cos(cur_yaw[i]) * config.dt
            cur_y[i] += v * math.sin(cur_yaw[i]) * config.dt
            cur_yaw[i] += w * config.dt

            # Simple EKF dead-reckoning + landmark correction
            est_x[i] += v * math.cos(cur_yaw[i]) * config.dt + rng.normal(0.0, 0.005)
            est_y[i] += v * math.sin(cur_yaw[i]) * config.dt + rng.normal(0.0, 0.005)
            # Pull estimate toward true with landmark factor (matching EKF)
            est_x[i] += 0.05 * (cur_x[i] - est_x[i])
            est_y[i] += 0.05 * (cur_y[i] - est_y[i])

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
        'total_control_energy': float(total_energy),
        'total_safety_interventions': float(total_interventions),
    }
