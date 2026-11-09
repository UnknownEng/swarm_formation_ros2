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
from swarm_formation.consensus import compute_anchor_force, compute_target_offsets
from swarm_formation.controllers import create_controller
from swarm_formation.formation_metrics import compute_procrustes_error
from swarm_formation.kinematics import compute_unicycle_twist, UnicycleMapper, wrap_angle
from swarm_formation.localization import RobotEKF
from swarm_formation.navigation import (
    anchor_slot_position,
    fit_template_rotation,
    formation_target,
    navigation_force,
    NavigationGuide,
    OccupancyGrid,
    required_clearance,
)
from swarm_formation.potential_field import (
    compute_peer_safety_scale,
    compute_relative_peer_apf,
)
from swarm_formation.relative_sensing import (
    compensate_delay,
    compute_heartbeat_mask,
    update_relative_store,
)
from swarm_formation.world_model import (
    DEFAULT_FORMATION_CENTER,
    landmark_positions,
    static_obstacles_in_range,
)

# Obstacle centres of collapsed_building.sdf, used as known localization beacons.
LANDMARKS: List[Tuple[float, float]] = list(landmark_positions().values())


_GRID_CACHE: Dict[float, OccupancyGrid] = {}


def _grid_for(inflation: float) -> OccupancyGrid:
    if inflation not in _GRID_CACHE:
        _GRID_CACHE[inflation] = OccupancyGrid(inflation=inflation)
    return _GRID_CACHE[inflation]


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

    # Delay compensation (relative_sensing.compensate_delay): subtract own motion since
    # each measurement; optionally extrapolate the neighbour's motion too
    delay_compensation: bool = False
    peer_velocity_gain: float = 0.0

    # Force -> (v, w): 'calm' = UnicycleMapper (reverse, force-scaled turning, stop band),
    # 'legacy' = compute_unicycle_twist (turn toward any force above 0.03)
    twist_mapper: str = 'legacy'
    allow_reverse: bool = False
    turn_full_force: float = 1.0
    stop_force: float = 0.5
    resume_force: float = 0.9

    # Formation anchor (virtual leader around a fixed centre, uses the EKF estimate)
    anchor_gain: float = 0.0
    # Anchor gain while a robot hears no neighbour at all (isolation fallback)
    isolated_anchor_gain: float = 0.0
    formation_center: Tuple[float, float] = DEFAULT_FORMATION_CENTER

    # Safety filter
    use_cbf: bool = False
    cbf_safety_dist: float = 0.55
    cbf_gamma: float = 1.5
    cbf_lookahead: float = 0.15
    cbf_safety_dist_obs: float = 0.30
    # None = margin matched to the applied worst-case latency (delay_sec + jitter_sec)
    cbf_delay_bound_sec: Optional[float] = None

    # Channel impairments on the peer measurement stream (network_impairment_node)
    drop_rate: float = 0.0
    delay_sec: float = 0.0
    jitter_sec: float = 0.0
    stale_purge_sec: float = 1.5
    # Asymmetric links: (src, dst, drop, delay, jitter) overrides for the directed link
    # src -> dst (information about src reaching dst); 0 = any robot, as in the
    # /network/set_link service. Later entries win.
    link_overrides: Tuple[Tuple[int, int, float, float, float], ...] = ()
    # Random asymmetry: every directed link draws its own drop in [0, max] and delay in
    # [0, max] once per run (both directions of a pair independently)
    random_link_max_drop: float = 0.0
    random_link_max_delay: float = 0.0
    # Share of each robot-robot barrier a robot enforces (0.5 reciprocal, 1 full)
    cbf_peer_responsibility: float = 0.5

    # Sensing and odometry noise
    range_noise_std: float = 0.0
    bearing_noise_std: float = 0.0
    odom_speed_noise_frac: float = 0.02
    odom_yaw_rate_noise_std: float = 0.02
    use_landmarks: bool = True
    # localization_node use_peer_covariance: propagate the peer's estimate uncertainty
    use_peer_covariance: bool = True

    # Static obstacles and walls of collapsed_building.sdf (sensed and fed to the CBF;
    # contact is not simulated, clearance violations are only measured)
    use_world_obstacles: bool = False

    # Navigation layer (navigation.py), same defaults as config/swarm_params.yaml
    use_navigation: bool = False
    nav_inflation: Optional[float] = None  # None = derived from the CBF settings
    nav_carrot_dist: float = 0.6
    nav_replan_period: float = 0.5
    nav_min_force: float = 0.4
    robot_radius: float = 0.20

    # Agent fault: robot stops (motor kill) from dropout_start_sec onward
    dropout_robot_id: int = 0
    dropout_start_sec: float = 0.0

    # Crossing trajectories scenario (challenging initial conditions)
    crossing_scenario: bool = True

    def effective_cbf_delay_bound(self) -> float:
        """Return the latency bound the CBF margin is computed for."""
        if self.cbf_delay_bound_sec is not None:
            return self.cbf_delay_bound_sec
        worst = max(0.0, self.delay_sec) + max(0.0, self.jitter_sec)
        for _, _, _, delay, jitter in self.link_overrides:
            worst = max(worst, max(0.0, delay) + max(0.0, jitter))
        return worst + max(0.0, self.random_link_max_delay)

    def link_table(
        self, rng: Optional[np.random.RandomState] = None
    ) -> Dict[Tuple[int, int], Tuple[float, float, float]]:
        """Return (drop, delay, jitter) for every directed link (src, dst), 1-based."""
        n = self.total_robots
        table = {
            (s, d): (self.drop_rate, self.delay_sec, self.jitter_sec)
            for s in range(1, n + 1) for d in range(1, n + 1) if s != d
        }
        if rng is not None and (self.random_link_max_drop > 0.0
                                or self.random_link_max_delay > 0.0):
            for key in sorted(table):
                drop, delay, jitter = table[key]
                drop = max(drop, float(rng.uniform(0.0, self.random_link_max_drop)))
                delay = delay + float(rng.uniform(0.0, self.random_link_max_delay))
                table[key] = (drop, delay, jitter)
        for src, dst, drop, delay, jitter in self.link_overrides:
            for key in table:
                if (src == 0 or key[0] == src) and (dst == 0 or key[1] == dst):
                    table[key] = (drop, delay, jitter)
        return table


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
    # Directed link table; random asymmetry uses its own generator so that the other
    # random streams (and every symmetric condition) are unchanged
    links = config.link_table(np.random.RandomState(seed + 7919))
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
    mappers = [
        UnicycleMapper(
            k_yaw=config.k_yaw, max_lin_vel=config.v_max, max_ang_vel=config.omega_max,
            allow_reverse=config.allow_reverse, turn_full_force=config.turn_full_force,
            stop_force=config.stop_force, resume_force=config.resume_force,
        )
        for _ in range(n)
    ]
    nav_guides: List[Optional[NavigationGuide]] = [None] * n
    if config.use_navigation:
        inflation = config.nav_inflation
        if inflation is None:
            inflation = required_clearance(
                safety_dist_obs=config.cbf_safety_dist_obs,
                lookahead=config.cbf_lookahead,
                max_linear_vel=config.v_max,
                delay_bound_sec=cbf_delay_bound,
            )
        grid = _grid_for(round(inflation, 4))
        nav_guides = [
            NavigationGuide(grid, carrot_dist=config.nav_carrot_dist,
                            replan_period=config.nav_replan_period)
            for _ in range(n)
        ]
    target_offsets = {
        rid: compute_target_offsets(rid, n, config.formation_radius) for rid in range(1, n + 1)
    }

    # Channel state. Packet: (deliver_time, src_j, range, bearing, measurement_time)
    in_flight: List[List[Tuple[float, int, float, float, float]]] = [[] for _ in range(n)]
    delivered: List[Dict[int, Tuple[float, float, float, float]]] = [{} for _ in range(n)]
    # Estimated heading per control step (robot_node / safety_filter_node YawHistory)
    est_yaw_hist: List[List[float]] = [[] for _ in range(n)]
    est_pos_hist: List[List[Tuple[float, float]]] = [[] for _ in range(n)]
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
    min_obstacle_clearance = float('inf')  # surface-to-robot-edge distance
    total_collision_events = 0
    effort_lin = 0.0
    effort_ang = 0.0
    late_abs_w = 0.0  # mean |w|, |v| over the last quarter of the run (settled phase)
    late_abs_v = 0.0
    late_samples = 0
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
        if config.use_world_obstacles:
            for i in range(n):
                for _, ox, oy, orad in static_obstacles_in_range(cur_x[i], cur_y[i], 1.5):
                    min_obstacle_clearance = min(
                        min_obstacle_clearance,
                        math.hypot(ox - cur_x[i], oy - cur_y[i]) - orad - config.robot_radius,
                    )

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
                l_drop, l_delay, l_jitter = links[(j + 1, i + 1)]
                if l_drop > 0.0 and (l_drop >= 1.0 or rng.uniform() < l_drop):
                    packets_dropped += 1
                    continue
                latency = max(0.0, l_delay)
                if l_jitter > 0.0:
                    jitter = rng.uniform(-l_jitter, l_jitter)
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
                peer_p = ekfs[src - 1].get_covariance()
                ekfs[i].update_range_bearing(
                    px, py, rng_m, brg_m,
                    var_range=max(config.range_noise_std ** 2, 0.05),
                    var_bearing=max(config.bearing_noise_std ** 2, 0.03),
                    target_cov=[peer_p[0][:2], peer_p[1][:2]]
                    if config.use_peer_covariance else None,
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
            ex_h, ey_h, _ = ekfs[i].get_pose()
            est_pos_hist[i].append((ex_h, ey_h))
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
            if config.delay_compensation:
                phist = est_pos_hist[i]

                def own_pos_at(tq: float, phist=phist) -> Tuple[float, float]:
                    return phist[min(len(phist) - 1, max(0, int(round(tq / dt))))]

                comp = compensate_delay(
                    rel_store[i], t, own_pos_at, config.peer_velocity_gain
                )
                # Apply the same correction on top of any EMA filtering
                for pid, (cx, cy) in comp.items():
                    if pid in rel_disp:
                        raw = rel_store[i][pid]
                        rel_disp[pid] = (
                            rel_disp[pid][0] + cx - raw[0], rel_disp[pid][1] + cy - raw[1]
                        )
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

            # Distance control leaves the orientation free: anchor and navigation target
            # follow the formation's best-fit orientation (robot_node does the same).
            free_rotation = controller.name == 'distance'
            pattern_rotation = 0.0
            if free_rotation and config.anchor_gain > 0.0:
                pattern_rotation = fit_template_rotation(
                    rel_disp, target_offsets[robot_id], active_mask
                )

            guide = nav_guides[i]
            if guide is not None:
                ex_n, ey_n, _ = ekfs[i].get_pose()
                rel_now = {pid: rel_disp.get(pid, (e[0], e[1]))
                           for pid, e in rel_store[i].items()}
                anchor_slot = anchor_slot_position(
                    robot_id, n, config.formation_radius, config.formation_center,
                    rotation=pattern_rotation,
                )
                target = formation_target(
                    (ex_n, ey_n), rel_now, target_offsets[robot_id],
                    consensus_gain=config.k_p,
                    anchor_slot=anchor_slot, anchor_gain=config.anchor_gain,
                    fit_rotation=free_rotation,
                )
                steer_point = guide.steer(t, (ex_n, ey_n), target)
                if steer_point is not None:
                    fx, fy = navigation_force(
                        (fx, fy), (ex_n, ey_n), steer_point, config.nav_min_force
                    )

            # A robot that hears no neighbour (e.g. its receiver failed) has no formation
            # force; it falls back to driving to its own slot at formation strength.
            anchor_gain = config.anchor_gain
            if not any(active_mask.get(pid, True) for pid in rel_disp):
                anchor_gain = max(anchor_gain, config.isolated_anchor_gain)
            if anchor_gain > 0.0:
                ex_i, ey_i, _ = ekfs[i].get_pose()
                fax, fay = compute_anchor_force(
                    robot_id, n, config.formation_radius, config.formation_center,
                    (ex_i, ey_i), anchor_gain, rotation=pattern_rotation,
                )
                fx += fax
                fy += fay

            fx -= config.k_d * cmd_vel_xy[i][0]
            fy -= config.k_d * cmd_vel_xy[i][1]

            if config.twist_mapper == 'calm':
                raw_v, raw_w = mappers[i].to_twist(fx, fy, est_yaw)
            else:
                raw_v, raw_w = compute_unicycle_twist(
                    fx, fy, est_yaw, config.k_yaw, config.v_max, config.omega_max
                )
            cmd_vel_xy[i] = [raw_v * math.cos(est_yaw), raw_v * math.sin(est_yaw)]

            sensed_obstacles: List[Tuple[float, float, float]] = []
            if config.use_world_obstacles:
                # Onboard sensing (not networked), rotated with the current estimated yaw
                for _, ox, oy, orad in static_obstacles_in_range(
                    cur_x[i], cur_y[i], config.sensing_range
                ):
                    dxw, dyw = ox - cur_x[i], oy - cur_y[i]
                    rng_o = math.hypot(dxw, dyw)
                    brg_o = wrap_angle(math.atan2(dyw, dxw) - cur_yaw[i])
                    sensed_obstacles.append((
                        rng_o * math.cos(brg_o + est_yaw), rng_o * math.sin(brg_o + est_yaw),
                        orad,
                    ))

            if config.use_cbf:
                safe_v, safe_w, interv = compute_cbf_safe_twist(
                    v_raw=raw_v,
                    w_raw=raw_w,
                    robot_yaw=est_yaw,
                    relative_peers=current_list,
                    relative_obstacles=sensed_obstacles,
                    safety_dist_peer=config.cbf_safety_dist,
                    safety_dist_obs=config.cbf_safety_dist_obs,
                    gamma_peer=config.cbf_gamma,
                    delay_bound_sec=cbf_delay_bound,
                    max_linear_vel=config.v_max,
                    max_angular_vel=config.omega_max,
                    lookahead_dist=config.cbf_lookahead,
                    peer_responsibility=config.cbf_peer_responsibility,
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
            if t >= 0.75 * config.duration_sec:
                late_abs_w += abs(w)
                late_abs_v += abs(v)
                late_samples += 1
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
        'final_centroid': (float(sum(cur_x) / n), float(sum(cur_y) / n)),
        'final_positions': [(float(cur_x[i]), float(cur_y[i])) for i in range(n)],
        'min_shape_error': float(np.min(shape_errors)),
        'global_min_distance': float(running_min_dist),
        'min_obstacle_clearance': float(min_obstacle_clearance),
        'obstacle_collision_free': bool(min_obstacle_clearance >= 0.0),
        'total_collisions': int(total_collision_events),
        'empirically_collision_free': bool(running_min_dist >= config.collision_threshold),
        'is_converged': bool(is_converged),
        'convergence_time': convergence_time,
        'mean_est_error': float(np.mean(est_errors)),
        'mean_yaw_error': float(np.mean(yaw_errors)),
        'control_effort_lin': float(effort_lin),
        'control_effort_ang': float(effort_ang),
        'late_mean_abs_w': float(late_abs_w / late_samples) if late_samples else 0.0,
        'late_mean_abs_v': float(late_abs_v / late_samples) if late_samples else 0.0,
        'path_length': float(path_length),
        'total_safety_interventions': float(total_interventions),
        'packet_drop_fraction': float(packets_dropped / packets_sent) if packets_sent else 0.0,
        'mean_applied_latency_sec': float(latency_sum / latency_count) if latency_count else 0.0,
        'cbf_delay_bound_sec': float(cbf_delay_bound if config.use_cbf else 0.0),
    }
