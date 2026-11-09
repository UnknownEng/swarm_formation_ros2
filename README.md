# Resilient Swarm Formation

Decentralized multi-robot formation control for ROS 2 Jazzy and Gazebo Harmonic, with a
control barrier function (CBF) safety filter, relative sensing, cooperative localization
and per-direction network impairments.

[![CI](https://github.com/UnknownEng/resilient-swarm-formation/actions/workflows/ci.yml/badge.svg)](https://github.com/UnknownEng/resilient-swarm-formation/actions/workflows/ci.yml)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-F58113.svg)](https://gazebosim.org/docs/harmonic)
[![Python 3.12](https://img.shields.io/badge/Python-3.12-3776AB.svg)](https://www.python.org/)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

Five differential-drive robots in a collapsed-building arena converge to a regular
pentagon (radius 2 m). There is no leader robot and no central coordinator: every robot
runs the same nodes. It measures neighbours in its own body frame, estimates its own pose
with an EKF, computes a formation command and passes it through a CBF-QP filter before it
reaches the motors.

The formation law uses relative measurements only. Each robot also has a map of the
arena (obstacles and landmarks) and its start pose; its EKF pose estimate in that map is
used for path planning around obstacles and for a weak anchor that holds the formation
at a fixed point. No node reads simulator ground truth except the sensor model and the
metrics logger.

## Demo Video

https://github.com/user-attachments/assets/9dd9311d-a70c-474c-8fd0-7ad09454dfed

## Contents

- [Results](#results)
- [Quick start](#quick-start)
- [Usage](#usage)
- [Architecture](#architecture)
- [Algorithms](#algorithms)
- [Configuration](#configuration)
- [Testing and reproduction](#testing-and-reproduction)
- [Project structure](#project-structure)
- [Limitations](#limitations)

## Results

Gazebo runs: 40 s of simulated time, start poses perturbed by up to ±0.15 m and ±0.2 rad
(`spawn_jitter:=0.15 spawn_yaw_jitter:=0.2`, seeds 1-5).

| Scenario | Converged | Time to converge | Final shape error | Min. robot distance |
| :--- | :---: | :---: | :---: | :---: |
| Clean network | 5/5 | 7.8 ± 1.8 s | 0.056 ± 0.017 m | 1.14 m |
| 15% loss, 80 ± 40 ms delay, robot 4 stopped 5-10 s | 5/5 | 10.8 ± 0.7 s | 0.18 ± 0.12 m | 0.84 m |
| One-way links (`robot1_mute`, `robot1_deaf`, `one_way_ring`), 1 run each | 3/3 | 6.7-7.5 s | 0.06-0.12 m | 0.81 m |

Spot checks of the released code (seed 1): clean network, converged in 7.7 s to 0.045 m;
with faults, in 13.9 s to 0.13 m; no collisions. Single runs with the bearing controller
under faults and with large random starts (±0.5 m, any heading) also converged without
collisions (12.3 s / 0.15 m and 23.8 s / 0.08 m).

Headless campaign: a kinematic simulator that reuses the controller, CBF, EKF, navigation
and network-model modules inside its own simulation loop; 22 conditions × 25 seeds = 550
runs from the default start poses with random jitter:

| Condition | Converged | Collision-free | Final shape error (mean) |
| :--- | :---: | :---: | :---: |
| Default stack, clean | 100% | 100% | 0.046 m |
| Default stack, 15% loss, 80 ± 40 ms delay, sensor noise | 100% | 100% | 0.041 m |
| Asymmetric links (5 patterns, 125 runs) | 96-100% | 100% | 0.04-0.15 m |
| Robot 4 stopped permanently | 72% | 100% | 0.27 m |
| No collision avoidance | — | 0% | — |

All 500 runs with the safety filter enabled were collision-free (minimum robot distance
0.65 m). Shape error is the Procrustes RMS distance to the target pentagon (optimal
translation and rotation); a run counts as converged once the error stays below 0.25 m
for 2 s.

Known failure cases, measured with the kinematic simulator (6 seeds each): robots that
start in each other's slots deadlock on a clean network (1/6 converged); with a one-way
link in a head-on encounter the robots came within 0.18 m; with no communication at all,
within 0.06 m. See [Limitations](#limitations).

## Quick start

Requirements: Ubuntu 24.04 with
[ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
(desktop). WSL2 on Windows 11 works, with Gazebo running below real time.

```bash
git clone https://github.com/UnknownEng/resilient-swarm-formation.git swarm_ws
cd swarm_ws
./setup.sh          # installs missing apt packages (Gazebo bridge, rosbag2, test tools) and builds
./setup.sh --run    # builds, then launches the simulation with the Gazebo GUI
```

`setup.sh` options: `--run`, `--test` (run the test suite), `--skip-deps`; any
`name:=value` argument is passed to the launch file.

Manual launch:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch swarm_formation gazebo_swarm_launch.py headless:=false
```

## Usage

```bash
# Packet loss, delay and a temporary motor failure on robot 4
ros2 launch swarm_formation gazebo_swarm_launch.py headless:=false enable_faults:=true

# One-directional link failures from a YAML profile
ros2 launch swarm_formation gazebo_swarm_launch.py headless:=false network_profile:=one_way_ring

# Bearing-only controller
ros2 launch swarm_formation gazebo_swarm_launch.py headless:=false controller_type:=bearing

# Randomized start poses (same seed = same poses)
ros2 launch swarm_formation gazebo_swarm_launch.py headless:=false \
  spawn_seed:=3 spawn_jitter:=0.15 spawn_yaw_jitter:=0.2

# Headless 40 s run with a rosbag, then offline metrics and plots
ros2 launch swarm_formation gazebo_swarm_launch.py sim_duration:=40.0 \
  record_bag:=true bag_path:=results/my_bag
ros2 run swarm_formation process_bag results/my_bag \
  --output-csv results/metrics.csv --output-json results/summary.json
ros2 run swarm_formation plot_bag results/my_bag --output-dir results/plots

# Change a link at runtime (robot 1 -> robot 2: 50% loss, 100 ms delay)
ros2 service call /network/set_link swarm_interfaces/srv/SetLinkState \
  "{from_robot_id: 1, to_robot_id: 2, enabled: true, drop_rate: 0.5, delay_sec: 0.1, jitter_sec: 0.0}"
```

### Start poses

By default every run starts from the same five poses, all with heading 0:
(-3, 0), (3, 0), (0, -3), (0, 3) and (2.2, -2.2). The robots must cross the arena
centre, which forces them to negotiate around each other. `spawn_jitter` and
`spawn_yaw_jitter` add a uniform random offset to each pose; `spawn_seed` selects the
random draw, so a run is reproducible from its seed.

### Launch arguments

| Argument | Default | Description |
| :--- | :---: | :--- |
| `headless` | `true` | Run Gazebo without the GUI |
| `controller_type` | `displacement` | `displacement`, `distance` or `bearing` |
| `use_safety_filter` | `true` | Enable the CBF-QP safety filter |
| `enable_faults` | `false` | 15% packet loss, 80 ± 40 ms delay, robot 4 motors off from 5 to 10 s |
| `fault_profile` | `""` | Fault scenario from `config/fault_profiles/` |
| `network_profile` | `""` | Per-direction link profile from `config/network_profiles/`, or a YAML path |
| `formation_radius` | `2.0` | Pentagon radius (m) |
| `sensing_range` | `6.0` | Neighbour sensing range (m) |
| `use_landmarks` | `true` | Fuse the landmark map in the EKF |
| `use_peer_covariance` | `true` | Account for peer pose covariance in the EKF |
| `spawn_seed` / `spawn_jitter` / `spawn_yaw_jitter` | `0` / `0.0` / `0.0` | Start pose randomization (m, rad) |
| `sim_duration` | `0.0` | Shut down after this many simulated seconds (`0` = run until Ctrl-C) |
| `record_bag` / `bag_path` | `false` / `""` | Record all swarm topics with rosbag2 |
| `params_file` | `config/swarm_params.yaml` | Node parameter file |
| `world` | `worlds/collapsed_building.sdf` | Gazebo world |
| `total_robots` | `5` | Number of robots |
| `use_resilient_method` | `false` | Replace the formation law with the robust Huber/EMA + potential-field variant |
| `csv_path` | `swarm_metrics.csv` | Output of the online metrics logger |
| `use_sim_time` | `true` | Run all nodes on the Gazebo clock |

## Architecture

Per robot `i`, four nodes run in a pipeline; `network_impairment`, `fault_injector` and
`metrics_logger` are shared.

```
 Gazebo ──odom/pose──▶ neighbor_sensor_i ──neighbors_raw──▶ network_impairment ──neighbors──┐
   ▲                                                                                          │
   │                  localization_node_i ◀── odom, landmarks, peer estimates ◀────────────────┤
   │                          │ estimated_pose, estimated_pose_cov                            │
   │                          ▼                                                               │
   └──cmd_vel── safety_filter_i ◀──cmd_vel_raw── robot_node_i (controller + navigation) ◀────┘
                                                        ▲
                               fault_injector ──motor_kill, sensor_fault
```

### Topics

| Topic | Type | Publisher → Subscriber |
| :--- | :--- | :--- |
| `/robot_i/odom` | `nav_msgs/Odometry` | Gazebo bridge → localization |
| `/robot_i/neighbors_raw` | `swarm_interfaces/NeighborList` | neighbor_sensor → network_impairment |
| `/robot_i/neighbors` | `swarm_interfaces/NeighborList` | network_impairment → localization, controller, safety filter |
| `/robot_i/estimated_pose` | `geometry_msgs/PoseStamped` | localization → controller, peers |
| `/robot_i/estimated_pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | localization → peers |
| `/robot_i/cmd_vel_raw` | `geometry_msgs/Twist` | controller → safety filter |
| `/robot_i/cmd_vel` | `geometry_msgs/Twist` | safety filter → Gazebo |
| `/robot_i/safety_intervention` | `std_msgs/Float32` | safety filter (size of the command correction) |
| `/robot_i/motor_kill`, `/robot_i/sensor_fault` | `std_msgs/Bool`, `swarm_interfaces/FaultInjection` | fault_injector |
| `/network/status` | `swarm_interfaces/NetworkStatus` | network_impairment |
| `/network/link_command` | `swarm_interfaces/LinkState` | user → network_impairment |
| `/swarm/visualization` | `visualization_msgs/MarkerArray` | metrics_logger |

Services: `/network/set_link` (`SetLinkState`) and `/fault_injector/trigger_fault`
(`TriggerFault`). For `set_link`, robot id 0 means every robot. Both services reject invalid ids and values.

### Interfaces (`swarm_interfaces`)

| Interface | Fields |
| :--- | :--- |
| `NeighborMeasurement` | `stamp`, `neighbor_id`, `distance`, `bearing`, `relative_x/y`, `range_variance`, `bearing_variance` |
| `ObstacleMeasurement` | `obstacle_id`, `distance`, `bearing`, `relative_x/y`, `radius` |
| `NeighborList` | `header`, `robot_id`, `neighbors[]`, `obstacles[]`, `landmarks[]` |
| `LinkState` | `from_robot_id`, `to_robot_id`, `enabled`, `drop_rate`, `delay_sec`, `jitter_sec` |
| `NetworkStatus` | `header`, `links[]`, `total_packets_forwarded`, `total_packets_dropped` |
| `FaultInjection` | `robot_id`, `fault_type`, `active`, `range_bias`, `bearing_bias`, `intermittent_duty_cycle` |
| `SetLinkState.srv` | link fields → `success`, `message` |
| `TriggerFault.srv` | `robot_id`, `fault_type`, `enable`, biases, `period`, `duty_cycle` → `success`, `message` |

## Algorithms

### Formation control

Three controllers implement `SwarmControllerBase.compute_control_force` and are selected
with `controller_type`:

| Controller | Measurements used | Converges to |
| :--- | :--- | :--- |
| Displacement consensus | relative positions | target shape up to translation |
| Distance + signed area | inter-robot distances | target shape up to rotation and translation; the area term rules out the reflected shape |
| Bearing-only + scale term | relative bearings | target shape up to translation; the scale term fixes the size |

The resulting force is mapped to unicycle commands `(v, ω)` with a turn-then-drive
mapper with hysteresis (`stop_force`, `resume_force`). A navigation layer plans around
known obstacles toward the robot's slot in the best-fit template, and a weak anchor term
holds the formation centre. A robot that receives no neighbour data falls back to driving
to its own slot (`isolated_anchor_gain`).

### Safety filter

Each robot filters its command independently. The QP acts on the look-ahead point
$q_i = p_i + \ell\,[\cos\theta_i, \sin\theta_i]^\top$ ($\ell = 0.15$ m), whose velocity
$u_i$ is linear in $(v, \omega)$:

$$
\min_{u_i} \lVert u_i - u_i^{\mathrm{raw}} \rVert^2
\quad \text{s.t.} \quad
p_{ij}^\top u_i \le \tfrac{r\gamma}{2}\, h_{ij}, \qquad
h_{ij} = \lVert p_{ij} \rVert^2 - \left(D_s + \ell + 2 v_{\max} \tau_{\max}\right)^2
$$

for each sensed peer $j$ (relative position $p_{ij}$), with analogous constraints for
obstacles. $r = 0.5$ splits each pairwise barrier between the two robots, and the margin
$2 v_{\max} \tau_{\max}$ covers a bounded communication delay. The 2D QP is solved
exactly by an active-set method. A second barrier on the robot centre then caps $v$ so
that alternating rotations and forward steps cannot walk the robot into a wall. When not
all constraints can be met, the solver minimizes obstacle violation first, then peer
violation. Rotation in place is passed through because it cannot reduce any distance.
Non-finite inputs never produce motion, and if no neighbour update has arrived for
`perception_timeout_sec` the filter only allows turning in place.

The barrier split assumes both robots see each other. If robot $j$ cannot sense robot
$i$ (a one-way link), $j$ does not enforce its half and the pair is protected only by
$i$'s half and the margins; this case is not guaranteed safe.

### Localization

Each robot runs an EKF over $(x, y, \theta)$: wheel odometry for prediction, landmark
range and bearing, and relative measurements of neighbours whose estimated poses arrive
on their `estimated_pose` topics. Neighbour uncertainty enters as a consider update,
$R_{\mathrm{eff}} = R + J \Sigma_j J^\top$, so an uncertain peer cannot pull a confident
robot off its estimate. Updates whose squared Mahalanobis distance exceeds the 99.99%
chi-square bound (2 DOF) are rejected as outliers.

### Network model

`network_impairment` keeps one link per ordered pair `src → dst` and sits on the
neighbour-measurement stream (`neighbors_raw` → `neighbors`): a link decides whether `dst`
receives its measurement of `src`, as with radio beacons or UWB ranging, where a robot
can only measure a neighbour whose packets it receives. Each link has a Bernoulli drop
rate, a base delay and uniform jitter; late packets never overwrite newer ones. The
`estimated_pose` topics shared for cooperative localization are not impaired.
Links are configured from YAML profiles, randomized per direction
(`random_link_max_drop`, `random_link_max_delay`, `random_link_seed`) or changed at
runtime through the `/network/set_link` service. Example profile:

```yaml
network_profile:
  links:
    - {from: 1, to: 0, drop_rate: 1.0}   # nobody hears robot 1 (0 = any robot)
```

## Configuration

Node parameters live in
[`config/swarm_params.yaml`](src/swarm_formation/config/swarm_params.yaml). The main ones:

| Parameter | Default | Meaning |
| :--- | :---: | :--- |
| `control_rate`, `filter_rate`, `sensor_rate` | 50, 50, 20 Hz | Loop rates |
| `max_linear_vel`, `max_angular_vel` | 1.4 m/s, 3.0 rad/s | Command limits |
| `linear_gain_kp`, `damping_gain_kd` | 1.8, 0.5 | Formation gains |
| `safety_dist_peer`, `safety_dist_obs` | 0.55 m, 0.30 m | CBF safety distances |
| `cbf_gamma_peer`, `cbf_gamma_obs` | 1.5, 2.0 | CBF class-K gains |
| `delay_bound_sec` | 0.05 s | Delay bound in the CBF margin |
| `peer_responsibility` | 0.5 | Share of each pairwise barrier |
| `range_noise_std`, `bearing_noise_std` | 0.0 | Sensor noise |
| `perception_timeout_sec` | 0.5 s | Safety filter stops forward motion without fresh neighbour data |
| `anchor_gain`, `formation_center_x/y` | 0.4, (0.44, -0.44) | Formation anchor |

## Testing and reproduction

```bash
cd src/swarm_formation
pytest-3 test -q
```

The suite covers the controllers, CBF solver, EKF, network and fault models, navigation,
bag tools and launch configuration, plus flake8, pep257 and license-header checks. GitHub Actions
builds the workspace in a `ros:jazzy` container and runs it on every push and pull
request.

`./reproduce_all.sh` runs the full pipeline: build, tests, a headless Gazebo run with a
rosbag, offline bag analysis, the 550-run campaign and its figures, and a check of every
result against fixed thresholds. It takes 30-60 minutes and writes to
`results/reproduction/`. The campaign alone:

```bash
ros2 run swarm_formation run_campaign --seeds 25 --duration 30.0 --workers "$(nproc)" \
  --output-dir results/campaign
ros2 run swarm_formation plot_campaign --input-json results/campaign/campaign_results.json \
  --output-dir results/campaign/plots
```

## Project structure

```
.
├── .github/workflows/ci.yml       # Build and test on every push
├── setup.sh                       # Dependency install, build, run
├── reproduce_all.sh               # Full reproduction pipeline
├── Dockerfile
└── src/
    ├── swarm_interfaces/          # Messages and services
    └── swarm_formation/
        ├── config/                # swarm_params.yaml, fault_profiles/, network_profiles/
        ├── launch/                # gazebo_swarm_launch.py
        ├── swarm_formation/
        │   ├── robot_node.py              # Formation controller node
        │   ├── controllers.py             # Displacement, distance, bearing
        │   ├── navigation.py              # Template fit, slot assignment, path following
        │   ├── safety_filter_node.py
        │   ├── cbf_qp.py                  # CBF constraints and 2D active-set QP
        │   ├── localization_node.py
        │   ├── localization.py            # EKF
        │   ├── neighbor_sensor_node.py
        │   ├── network_impairment_node.py
        │   ├── network_model.py           # Per-direction links, YAML profiles
        │   ├── fault_injector_node.py
        │   ├── campaign_simulator.py      # Headless kinematic simulator
        │   ├── campaign_runner.py         # Parallel campaign with 95% CIs
        │   └── rosbag_processor.py        # Offline metrics
        ├── test/
        ├── urdf/robot.urdf
        └── worlds/collapsed_building.sdf
```

## Limitations

- Simulation only; not yet run on hardware. Gazebo evidence is small (5 runs per network
  setting, 1 per one-way profile); the statistics come from the kinematic simulator.
- Not map-free: robots use a known obstacle and landmark map, their start pose, and a
  fixed anchor point.
- Sensing is idealized: no occlusion by walls, and zero measurement noise by default.
- The network model impairs neighbour measurements but not the shared pose estimates, and
  link quality does not depend on distance or obstacles.
- Safety is not guaranteed when sensing is one-way or lost (see Safety filter); there is
  no acknowledgement mechanism.
- Symmetric starts (robots in each other's slots) can deadlock; start poses in the
  evaluation are a fixed crossing pattern plus bounded jitter.
- The distance-based controller converges less reliably than the other two (23/25 clean,
  21/25 impaired in the campaign).
- A* replanning runs inside the 50 Hz control callback and can take about 0.2 s.

## License

Apache License 2.0. See [LICENSE](LICENSE).

## Author

Mansoor Ahmed Rind, School of Electrical Engineering and Computer Sciences, National
University of Sciences and Technology (NUST), Islamabad, Pakistan.
