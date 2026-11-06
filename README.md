# Physics-Verified Decentralized Multi-Robot Formation Control Under Relative Sensing, Asymmetric Network Impairments, and Control Barrier Function Safety Layer

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic_8.x-orange.svg)](https://gazebosim.org/docs/harmonic)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04_LTS-E95420.svg)](https://ubuntu.com/)
[![License](https://img.shields.io/badge/License-Apache_2.0-green.svg)](LICENSE)
[![Tests](https://img.shields.io/badge/Tests-107%20(pytest)-blue.svg)](src/swarm_formation/test)
[![Docker](https://img.shields.io/badge/Docker-Ready-2496ED.svg)](Dockerfile)

> **Author**: Mansoor Ahmed Rind  
> **Affiliation**: Department of Robotics & Artificial Intelligence, National University of Sciences and Technology (NUST), Islamabad, Pakistan  
> **Workspace**: `~/swarm_ws` | **Package**: `swarm_formation`

---

## 1. Project Overview & Scientific Highlights

This repository delivers a ROS 2 / Gazebo robotics software framework for **decentralized multi-robot geometric formation control** in GPS-denied, obstacle-strewn disaster environments (modeled after an urban structural collapse). 

A swarm of **5 nonholonomic differential-drive mobile robots** autonomously self-organizes into a regular pentagon ($R = 2.0\,\text{m}$) without any central coordinator, global GPS/mocap positioning, or designated leader.

> **Validation status (2026-11-07):** the statistical results in Section 6 come from a headless kinematic simulator that mirrors the ROS node pipeline. The look-ahead CBF and deadlock fix have **not yet been re-run in Gazebo**. Every committed Gazebo run that used the old safety layer ended without reaching the formation (`results/M6`, `results/M7`, `results/re_eval`).

```
                Robot 2 (0.62, 1.90)
                   *
                 /    \
Robot 3        /        \
(-1.62, 1.18) *          \
              |           * Robot 1 (2.00, 0.00)
(-1.62,-1.18) *          /
Robot 4        \        /
                 \    /
                   *
                Robot 5 (0.62, -1.90)

Target slot of robot k: angle 72 deg * (k - 1), counter-clockwise from +x
(see consensus.compute_target_offsets). Only relative offsets are controlled,
so the whole pentagon may be translated.
```

### Key Engineering & Scientific Capabilities:
1. **Decentralized Relative Polar Sensing**: No robot reads global $(x, y)$. Sensing is restricted to onboard relative polar range and bearing within $R_{\text{sense}} = 6.0\,\text{m}$.
2. **Three Interchangeable Controller Families**:
   - **Displacement-based consensus** with Huber robust penalty (Ren & Beard 2008).
   - **Distance-based rigidity control** with stress matrix and orientation alignment (Oh & Ahn 2014).
   - **Bearing-based consensus** with orthogonal projection nullspace operators (Zhao & Zelazo 2016).
3. **Cooperative EKF Localization Law**: An onboard Extended Kalman Filter fuses wheel odometry and relative neighbor measurements to estimate local ego-pose. Ground truth is strictly isolated for offline metric evaluation.
4. **Active Safety Layer (CBF-QP)**: A standalone look-ahead Control Barrier Function QP filter (exact 2D solver, exact unicycle mapping, right-hand deadlock breaking). In the kinematic campaign it kept every run collision-free ($d_{\min} \ge 0.50\,\text{m}$) **while still reaching the formation in 100% of clean-channel runs** (see Section 6).
5. **Delay-Compensated Safety**: Barrier expansion $D_s^{\text{eff}} = D_s + \ell + 2 v_{\max} \tau_{\max}$. With latency actually applied to every peer measurement, all runs stayed collision-free up to 200 ms, but formation accuracy only holds to about 100 ms (at 200 ms, 36% of runs reach the formation).
6. **Decoupled Network Impairments & Fault Injection**: Standalone nodes simulate Bernoulli packet loss, stochastic delay queues, sensor bias, and agent dropouts, with runtime ROS 2 service toggles.
7. **Offline Rosbag-Driven Analytics**: Rosbag recording (`ros2 bag`), offline Procrustes metric extraction (`process_bag`), and publication plot generation (`plot_bag`).
8. **Statistical Rigor & One-Command Replication**: 400-run kinematic campaign (25 seeds $\times$ 16 conditions, leave-one-out ablation, applied packet-drop and latency sweeps) with Student's $t$ 95% Confidence Intervals, reproducible via `./reproduce_all.sh`. The campaign uses a headless unicycle simulator that mirrors the ROS node pipeline; it is not a physics simulation.

---

## 2. System Architecture

The software architecture is modularized into decoupled ROS 2 nodes and pure Python algorithmic libraries:

### 2.1 Component Flow Diagram

```mermaid
flowchart TD
    subgraph Physics_Simulation ["Simulation Layer (Gazebo Harmonic)"]
        GZ["Gazebo Harmonic + DiffDrive plugin"]
    end

    subgraph Perception_Layer ["Perception & Estimation Layer"]
        SENS["neighbor_sensor_node\n(simulated range/bearing from Gazebo poses,\nR_sense = 6.0m, timestamped)"]
        EKF["localization_node\n(Cooperative EKF: odometry, landmarks, peers)"]
    end

    subgraph Network_Fault_Layer ["Network & Fault Simulation Layer"]
        NET["network_impairment_node\n(drop, delay, jitter on peer measurements)"]
        FAULT["fault_injector_node\n(sensor bias/blackout, motor kill)"]
    end

    subgraph Formation_Control_Layer ["Decentralized Formation Layer"]
        CTRL["robot_node\n(formation controller, optional Huber/EMA/heartbeat/APF)"]
    end

    subgraph Safety_Filter_Layer ["Active Safety Filter Layer"]
        CBF["safety_filter_node\n(look-ahead CBF-QP)"]
    end

    subgraph Evaluation_Layer ["Offline Analytics"]
        BAG["ros2 bag record"]
        PROC["rosbag_processor (process_bag)"]
        PLOT["plot_generator (plot_bag)"]
    end

    GZ -->|/robot_i/pose: sensor simulation + evaluation only| SENS
    GZ -->|/robot_i/odom| EKF
    SENS -->|/robot_i/neighbors_raw| NET
    NET -->|/robot_i/neighbors| EKF
    NET -->|/robot_i/neighbors| CTRL
    NET -->|/robot_i/neighbors| CBF
    EKF -->|/robot_i/estimated_pose: heading| CTRL
    EKF -->|/robot_i/estimated_pose: heading| CBF
    FAULT -.->|/robot_i/sensor_fault| SENS
    FAULT -.->|/robot_i/motor_kill| CTRL
    CTRL -->|/robot_i/cmd_vel_raw| CBF
    CBF -->|/robot_i/cmd_vel| GZ
    GZ -.->|/robot_i/pose| BAG
    CBF -.->|/robot_i/cmd_vel| BAG
    BAG --> PROC
    PROC --> PLOT
```

---

## 3. Node, Topic, Service & Parameter Reference

### 3.1 Node Summary

| Node Name | Executable | Description |
| :--- | :--- | :--- |
| `neighbor_sensor_i` | `neighbor_sensor` | Measures relative range and bearing to peers within $R_{\text{sense}}$. Strictly relative polar. |
| `localization_node_i` | `localization_node` | Cooperative EKF state estimator fusing odometry and neighbor bearings. |
| `gazebo_node_i` | `robot_node` | Formation controller executing displacement, distance, or bearing control law. |
| `safety_filter_i` | `safety_filter` | Standalone Control Barrier Function QP filter outputting safe unicycle velocity commands. |
| `network_impairment` | `network_impairment` | Central wireless degradation node simulating packet drop, delay, jitter, and link toggles. |
| `fault_injector` | `fault_injector` | Injects deterministic or stochastic faults (agent kill, sensor bias, intermittent dropouts). |
| `swarm_metrics_logger` | `metrics_logger` | Continuous online ground-truth logger writing `swarm_metrics.csv` for run monitoring. |

### 3.2 Topic Interfaces

| Topic Name | Message Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `/robot_{i}/odom` | `nav_msgs/msg/Odometry` | Subscribed | Raw wheel odometry from differential drive base. |
| `/robot_{i}/relative_neighbors` | `swarm_interfaces/msg/RelativeNeighborList` | Published | Ego-centric relative polar measurements $(r, \phi)$ of visible peers. |
| `/robot_{i}/estimated_pose` | `geometry_msgs/msg/PoseStamped` | Published | Cooperative EKF estimated pose $(\hat{x}, \hat{y}, \hat{\theta})$ in local frame. |
| `/robot_{i}/neighbors` | `swarm_interfaces/msg/NeighborList` | Subscribed | Degraded neighbor state packets received after network impairment simulation. |
| `/robot_{i}/cmd_vel_raw` | `geometry_msgs/msg/Twist` | Published | Unconstrained formation control twist before safety filtering. |
| `/robot_{i}/cmd_vel` | `geometry_msgs/msg/Twist` | Published | Certified safe twist output by CBF-QP filter sent to robot actuator. |
| `/robot_{i}/safety_intervention` | `std_msgs/msg/Bool` | Published | Flag indicating active CBF barrier deflection ($u_{\text{safe}} \neq u_{\text{raw}}$). |
| `/swarm/network_status` | `std_msgs/msg/String` | Published | Real-time diagnostic telemetry of dropped packets, latency, and active links. |
| `/swarm/fault_status` | `std_msgs/msg/String` | Published | Active fault injection scenario and affected agent IDs. |

### 3.3 Service Interfaces

| Service Name | Service Type | Description |
| :--- | :--- | :--- |
| `/network/toggle_link` | `swarm_interfaces/srv/ToggleLink` | Dynamically enables or disables a directed wireless link $(i \to j)$ at runtime. |
| `/faults/trigger` | `swarm_interfaces/srv/TriggerFault` | Triggers immediate agent death, sensor bias, or dropout via programmatic ROS 2 call. |

### 3.4 Launch Arguments (`gazebo_swarm_launch.py`)

| Argument | Default | Type | Description |
| :--- | :---: | :---: | :--- |
| `controller_type` | `displacement` | `string` | Formation controller family: `displacement`, `distance`, `bearing`. |
| `use_safety_filter` | `true` | `bool` | Enables standalone CBF-QP minimal-correction safety filter. |
| `use_resilient_method`| `false` | `bool` | Enables Huber gradient thresholding and EMA smoothing. |
| `enable_faults` | `false` | `bool` | Enables 15% packet drop, 80 ms latency, and agent-4 dropout. |
| `formation_radius` | `2.0` | `float` | Target pentagon radius in meters ($R$). |
| `sensing_range` | `6.0` | `float` | Onboard sensor range limit in meters ($R_{\text{sense}}$). |
| `record_bag` | `false` | `bool` | Enables automated recording of swarm topics with `ros2 bag`. |
| `bag_path` | `""` | `string` | Destination directory path for recorded rosbag. |
| `headless` | `true` | `bool` | Runs Gazebo Harmonic in headless server mode. |
| `sim_duration` | `0.0` | `float` | Auto-shutdown duration in seconds (`0.0` = manual Ctrl-C). |
| `world` | `collapsed_building.sdf` | `string` | World environment path. |

---

## 4. Theory of the Three Controller Families

All three controllers inherit from `FormationController` and require **zero global Cartesian positions**.

### 4.1 Displacement-Based Consensus (Ren & Beard 2008)
Target formation is defined by relative inter-agent offsets $\Delta p_{ij}^* = d_i^* - d_j^*$ in a shared coordinate frame:
$$\dot{p}_i = -k_p \sum_{j \in \mathcal{N}_i} a_{ij} \psi_{\delta}\left( (p_i - p_j) - \Delta p_{ij}^* \right)$$
where $\psi_\delta(e)$ is the Huber loss gradient bounding transient velocity spikes caused by packet burst recovery:
$$\psi_\delta(e) = \begin{cases} e, & \|e\| \le \delta \\ \delta \frac{e}{\|e\|}, & \|e\| > \delta \end{cases}$$

### 4.2 Distance-Based Formation Rigidity (Oh & Ahn 2014)
Requires **no shared orientation frame**. The target geometry is defined strictly by desired inter-agent Euclidean distances $d_{ij}^*$:
$$\dot{p}_i = -k_{\text{dist}} \sum_{j \in \mathcal{N}_i} \left( \|p_i - p_j\|^2 - (d_{ij}^*)^2 \right) (p_i - p_j) + u_i^{\text{align}}$$
A stress matrix formulation guarantees infinitesimal rigidity, while an alignment term $u_i^{\text{align}}$ prevents manifold foldovers and reflection ambiguity.

### 4.3 Bearing-Based Formation Control (Zhao & Zelazo 2016)
Specified by inter-agent unit bearing vectors $g_{ij}^* = \frac{d_j^* - d_i^*}{\|d_j^* - d_i^*\|} \in \mathbb{S}^1$. The orthogonal projection operator onto the nullspace of $g_{ij}^*$ is:
$$P_{g_{ij}^*} = I_2 - g_{ij}^* (g_{ij}^*)^T$$
The decentralized control law minimizes bearing error:
$$\dot{p}_i = -k_{\text{bear}} \sum_{j \in \mathcal{N}_i} P_{g_{ij}^*} (p_j - p_i)$$
Because $P_{g_{ij}^*} g_{ij}^* = 0$, the controller achieves scale-invariant geometric rigidity.

---

## 5. Control Barrier Function (CBF-QP) Safety Layer

### 5.1 Look-Ahead Formulation for Unicycles
A unicycle cannot move sideways, so a barrier written for a single integrator ($\dot p_i = u_i$) stops holding once its output is converted back to $(v, \omega)$. The filter therefore works on the look-ahead point $q_i = p_i + \ell\,[\cos\psi_i,\ \sin\psi_i]^T$ ($\ell = 0.15\,\text{m}$), whose velocity is fully actuated:
$$\dot q_i = R(\psi_i)\,\mathrm{diag}(1, \ell)\,[v_i,\ \omega_i]^T$$
For each neighbor $j$, with $D^{\text{eff}} = D_s + \ell + 2 v_{\max} \tau_{\max}$ and $D_s = 0.55\,\text{m}$:
$$h_{ij} = \|p_j - q_i\|^2 - (D^{\text{eff}})^2, \qquad (p_j - q_i)^T \dot q_i \le \tfrac{\gamma}{4}\, h_{ij}$$
Keeping $\|p_j - q_i\| \ge D_s + \ell$ implies $\|p_j - p_i\| \ge D_s$ for the robot centers. At each 50 Hz step the robot solves
$$\min_{u} \tfrac12 \|u - \dot q_i^{\text{nom}}\|^2 \quad \text{s.t.}\quad A_k u \le b_k\ \ \forall k,\quad \|u\| \le v_{\max}$$
exactly (2D active-set enumeration, including the speed-circle intersections), and maps $u$ back to $(v, \omega)$ with the exact inverse. The inequality the QP enforces is therefore the one the robot actually executes.

### 5.2 Deadlock Breaking
Symmetric multi-robot CBF filters are known to deadlock: in the original implementation the clean-channel CBF stack reached the formation in only 36% of runs (12% for the full stack). When the nominal command pushes into a barrier, the filter now adds a tangential component (every robot veers to its right, like a roundabout). This changes only the *nominal* input, so the barrier constraint is unaffected.

### 5.3 What Is and Is Not Guaranteed
- The barrier is enforced on the *estimated* relative positions. Estimation error, measurement noise and latency beyond $\tau_{\max}$ are not covered, so collision avoidance is reported as **empirical**, not certified.
- The reciprocal split ($\gamma/4$) assumes neighbors run the same filter.
- Safety under delay relies on the margin $2 v_{\max}\tau_{\max}$; the campaign checks this with the delay actually applied to every peer measurement.

---

## 6. Statistical Campaign (400 Runs)

Headless kinematic simulator (`campaign_simulator.py`) that mirrors the ROS node pipeline: real `RobotEKF`, the same controller and CBF code as the nodes, and drop, delay and jitter actually applied to every peer measurement. Crossing initial conditions, 30 s per run. "Impaired" = the launch file's fault defaults (15% drop, 80 ± 40 ms delay) plus range/bearing noise (0.05 m / 0.02 rad). "Reached formation" = shape error below 0.25 m for 2 s.

| Condition | Reached formation | Final shape error (m), mean [95% CI] | Min distance (m), mean / worst | Collision-free | Path length (m) | Est. error (m) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| Unshielded consensus (clean) | 100% | 0.009 [0.007, 0.011] | 0.140 / 0.071 | 0% | 22.0 | 0.004 |
| APF only (clean) | 100% | 0.009 [0.007, 0.011] | 0.867 / 0.829 | 100% | 23.4 | 0.004 |
| CBF-QP only (clean) | 100% | 0.009 [0.007, 0.011] | 0.955 / 0.797 | 100% | 23.6 | 0.005 |
| Full stack (clean) | 100% | 0.009 [0.007, 0.011] | 0.766 / 0.664 | 100% | 26.8 | 0.004 |
| Full stack, impaired | 100% | 0.202 [0.185, 0.218] | 1.086 / 1.023 | 100% | 74.8 | 0.029 |
| Full stack without Huber, impaired | 100% | 0.193 [0.178, 0.209] | 1.065 / 0.934 | 100% | 74.2 | 0.046 |
| Full stack without EMA, impaired | 100% | 0.104 [0.094, 0.114] | 1.081 / 0.937 | 100% | 49.0 | 0.025 |
| Full stack without heartbeat, impaired | 100% | 0.202 [0.185, 0.218] | 1.086 / 1.023 | 100% | 74.8 | 0.029 |
| CBF only (no resilience), impaired | 100% | 0.108 [0.099, 0.116] | 1.064 / 0.966 | 100% | 48.5 | 0.026 |
| Full stack, 15% drop | 100% | 0.010 [0.008, 0.012] | 0.767 / 0.691 | 100% | 27.0 | 0.004 |
| Full stack, 30% drop | 100% | 0.010 [0.009, 0.012] | 0.759 / 0.648 | 100% | 27.7 | 0.005 |
| Full stack, 50% drop | 100% | 0.011 [0.009, 0.013] | 0.731 / 0.576 | 100% | 30.5 | 0.005 |
| Full stack, 50 ms delay (matched margin) | 100% | 0.067 [0.056, 0.078] | 0.883 / 0.810 | 100% | 43.5 | 0.014 |
| Full stack, 100 ms delay (matched margin) | 100% | 0.152 [0.133, 0.171] | 1.013 / 0.865 | 100% | 65.8 | 0.029 |
| Full stack, 200 ms delay (matched margin) | 36% | 0.309 [0.289, 0.329] | 1.272 / 1.172 | 100% | 89.9 | 0.054 |
| Full stack, 200 ms delay (fixed 50 ms margin) | 4% | 0.352 [0.329, 0.374] | 0.825 / 0.711 | 100% | 99.0 | 0.070 |

*Results regenerated on 2026-11-07 after the Phase A fixes. Measurements now carry their measurement time, are rotated with the heading at that time, take relative velocity only from distinct measurements, and are dropped once no longer delivered.*

**What the data supports:**
- **Safety:** every run in all 15 conditions with avoidance stayed above the 0.50 m collision limit. The worst case was 0.576 m (full stack, 50% drop). Without avoidance, every run collided.
- **Safety no longer costs convergence:** on a clean channel, CBF-only and the full stack reach the formation in 100% of runs (original implementation: 36% and 12%).
- **No energy advantage:** CBF path length (23.6 m) is about the same as APF (23.4 m). The earlier 47.2% figure is withdrawn.
- **Packet loss:** the formation forms precisely even at 50% drop (final error 0.011 m).
- **Delay:** safety holds up to 200 ms, even with the fixed 50 ms margin (worst 0.711 m). Formation accuracy degrades with delay: 0.067 m at 50 ms and 0.152 m at 100 ms. At 200 ms only 36% of runs reach the formation (4% with the fixed margin). The stack works to about 100 ms of delay.
- **Resilience components, under nominal impairment:**
  - **EMA hurts.** Removing it halves the final error (0.202 m → 0.104 m) and the path length (74.8 m → 49.0 m), because its lag adds to the network delay.
  - **Huber is neutral** (0.193 m without it, overlapping CIs).
  - **The heartbeat now works** (it uses measurement age, and is unit-tested), but it changes nothing here, because no peer is silent for more than 1.2 s in these conditions. A robot-loss scenario is needed to evaluate it.
  - **CBF with no resilience component** (0.108 m) does as well as any combination.

### Campaign Figures
Generated from `campaign_results.json` into `results/M8/plots/`:
1. `ablation_study_barchart.png`: leave-one-out ablation under nominal impairment.
2. `safety_comparison_apf_vs_cbf.png`: minimum inter-robot distance distributions.
3. `difficulty_sweep_packet_drop.png`: packet drop sweep (0–50%).
4. `difficulty_sweep_latency.png`: applied latency sweep (0–200 ms).

> The M8 numbers published before 2026-11-07 (325 runs, "47.2% energy reduction", "safe under 200 ms") came from a simulator that did not apply delay and from CBF runs that deadlocked. They are superseded; see the erratum in `reports/TECHNICAL_REPORT.md`.

---

## 7. Quick Start: Build, Run & Reproduce

### 7.1 Native Installation (Ubuntu 24.04 LTS + ROS 2 Jazzy)
```bash
# Clone and enter workspace
cd ~/swarm_ws

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Build packages
colcon build --symlink-install

# Source overlay
source install/setup.bash

# Run full test suite (107 tests; 7 files need a sourced ROS 2 environment)
colcon test --packages-select swarm_formation && colcon test-result --verbose
```

### 7.2 Docker Installation
```bash
# Build Docker image
docker build -t swarm_formation:latest .

# Run reproduction inside container
docker run --rm -it swarm_formation:latest ./reproduce_all.sh
```

### 7.3 Interactive 3D Gazebo Simulation
To watch the 5-robot swarm form the pentagon in the collapsed-building arena with live 3D visualization:
```bash
ros2 launch swarm_formation gazebo_swarm_launch.py \
  headless:=false \
  controller_type:=displacement \
  use_safety_filter:=true \
  enable_faults:=true \
  use_resilient_method:=true
```

### 7.4 Headless Simulation with Rosbag Recording
```bash
ros2 launch swarm_formation gazebo_swarm_launch.py \
  headless:=true \
  sim_duration:=40.0 \
  record_bag:=true \
  bag_path:=results/my_bag \
  controller_type:=displacement \
  use_safety_filter:=true
```

### 7.5 Offline Bag Processing & Plotting
```bash
# Extract offline Procrustes metrics from recorded bag
ros2 run swarm_formation process_bag \
  --input-bag results/my_bag \
  --output-csv results/offline_metrics.csv

# Generate publication trajectory plots from bag
ros2 run swarm_formation plot_bag \
  --input-bag results/my_bag \
  --output-dir results/plots
```

### 7.6 One-Command Master Reproduction
To run the end-to-end pipeline (build, test, simulation, bag processing, 325-run campaign, and figure generation):
```bash
./reproduce_all.sh
```

---

## 8. Repository Structure

```
~/swarm_ws/
├── Dockerfile                         # Container definition (ROS 2 Jazzy + Gazebo Harmonic)
├── README.md                          # Master system documentation
├── reproduce_all.sh                   # One-command automated reproduction pipeline
├── reports/
│   ├── TECHNICAL_REPORT.md            # Comprehensive research report
│   ├── M8_report.md                   # Milestone M8 campaign report
│   ├── M8_audit_by_M9.md              # Milestone M8 independent audit
│   └── ...                            # Full audit & milestone history (M0 to M9)
├── results/
│   ├── M7/gazebo_swarm_bag/           # Physics-verified 68k-message rosbag
│   ├── M8/campaign_summary.csv        # Aggregated statistical summary (325 runs)
│   ├── M8/campaign_results.json       # Full raw JSON results across 13 conditions
│   └── M8/plots/                      # 4 publication figures (Ablation, Safety, Sweeps)
└── src/
    ├── swarm_interfaces/              # Custom ROS 2 msg & srv definitions
    │   ├── msg/
    │   │   ├── RelativeNeighbor.msg
    │   │   ├── RelativeNeighborList.msg
    │   │   └── ...
    │   └── srv/
    │       ├── ToggleLink.srv
    │       └── TriggerFault.srv
    └── swarm_formation/               # Core swarm robotics stack
        ├── config/                    # YAML parameter files & fault profiles
        ├── launch/
        │   └── gazebo_swarm_launch.py # Master unified launch file
        ├── swarm_formation/           # Pure library algorithms & ROS 2 nodes
        │   ├── cbf_qp.py              # 2D Active-Set CBF-QP safety filter solver
        │   ├── consensus.py           # Huber resilient consensus & graph Laplacians
        │   ├── controllers.py         # Displacement, Distance, Bearing controllers
        │   ├── localization.py        # Cooperative EKF state estimator
        │   ├── potential_field.py     # Dual-layer APF obstacle avoidance
        │   ├── campaign_runner.py     # Batch campaign executor (Student's t CI)
        │   ├── campaign_visualizer.py # Publication figure generator
        │   ├── rosbag_processor.py    # Offline bag metrics extractor
        │   └── plot_generator.py      # Bag trajectory plot generator
        ├── test/                      # 107 unit and acceptance tests
        └── worlds/
            └── collapsed_building.sdf # Gazebo Harmonic disaster arena
```

---

## 9. License & Citations

Distributed under the Apache 2.0 License. If you use this software stack or benchmark results in academic work, please cite:

```bibtex
@article{rind2026decentralized,
  title={Decentralized Multi-Robot Pentagon Formation Control Under Relative Sensing, Asymmetric Network Impairments, and Control Barrier Function Safety Layer},
  author={Rind, Mansoor Ahmed},
  journal={arXiv preprint},
  year={2026}
}
```
