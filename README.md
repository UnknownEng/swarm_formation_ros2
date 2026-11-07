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

> **Validation status (2026-11-07):** in Gazebo (Section 6.1), with the default configuration the formation converges in 7.9 s to a 0.060 m final shape error (threshold 0.25 m) without collisions on a clean channel, and the robots then stand still. Under the launch file's fault settings (Section 6.2: 15% drop, 80 +/- 40 ms delay, robot death, sensor bias) it stays collision-free and converges, but the error wobbles around 0.1 to 0.3 m. Statistics (Section 6) come from a headless kinematic simulator that mirrors the ROS node pipeline. All Gazebo results before 2026-11-07 ran without wheel odometry (a bridge type mismatch) and are superseded.

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

### 6.1 Gazebo Validation (Phase C)

All runs: 5 robots, crossing start positions, CBF on, clean channel, 40 s of simulated
time, ROS 2 Jazzy + Gazebo Harmonic in WSL2 (Ubuntu 24.04, 4 cores). Raw CSVs and launch
logs are in `results/phaseC/`.

| Run | Change | Final shape error | Closest robots | What happened |
| :--- | :--- | :---: | :---: | :--- |
| `cbf_clean` | nodes on wall clock (old default) | 2.49 m | 1.13 m | Gazebo ran at ~5% of real time; controllers saw huge lag and robots crawled |
| `cbf_simtime` | `use_sim_time:=true` | 0.59 m | 0.83 m | formation formed (0.32 m at ~25 s), then drifted and pinned robots on the east wall |
| `cbf_walls` | walls sensed, true obstacle sizes | 2.43 m | 0.72 m | no wall contact, but robots 2 and 5 trapped between rubble pieces |
| `cbf_boxes` | exact box shapes, 0.10 m obstacle margin | 1.37 m | 0.83 m | robot 2 freed; robot 5 still trapped at a 0.66 m gap the CBF cannot pass |
| `cbf_nav` | navigation layer | 0.46 m | 0.96 m | every robot moves and reaches the formation area; error hovers 0.30 to 0.46 m against the east wall |
| `cbf_nav_anchor` | + anchor at the spawn centroid | 1.61 m | 0.75 m | not converged; formation centre held within 0.28 m of target, but robots 2 and 5 froze near their old trap spots from ~5 s; mean localization error 0.18 m (vs 0.10 m in `cbf_nav`) |
| `diag_anchor` | same as `cbf_nav_anchor`, with rosbag | 0.66 m | 0.96 m | converged at 18.8 s; max error over last 10 s 0.66 m; mean localization error 0.075 m. The bag showed `/robot_i/odom` had **0 messages** and heading errors of 0.1 to 0.4 rad |
| `diag_anchor_odom` | wheel odometry fixed (bridge type + EKF fallback) | 0.10 m | 0.91 m | converged at 13.0 s; max error over last 10 s 0.18 m; mean localization error 0.015 m |
| `cbf_final_default` | anchor + navigation (no bag) | 0.06 m | 0.88 m | converged at 13.7 s; max error over last 10 s 0.49 m; mean localization error 0.014 m. The formation first assembled ~1.6 m from the anchor point; from ~22 to 35 s the anchor dragged it past the two slabs, which held robots 1 and 2 back and stretched the shape to 0.49 m, before it settled at its target |
| `diag_calm` | + calm twist mapper (force-scaled turning, stop band) | 0.52 m | 0.92 m | converged at 12.0 s; settled robots stopped turning (0.00 rad/s), but from ~30 s robots 1 and 2 were frozen: their force pointed behind them, the controller asked to turn in place, and the look-ahead CBF cancelled the rotation |
| `diag_calm2` | + pure rotation passes the CBF | 0.08 m | 0.87 m | converged at 22.8 s; no freezing; robot 1 was blocked head-on by the CBF for ~15 s while following a navigation route; the anchor stayed saturated (formation ~1 m off its anchor point), so residual forces kept some robots turning |
| `diag_clearance` | planner clearance derived from the CBF margins | 0.069 m | 1.16 m | converged at 6.5 s (fastest so far); max error over the last 10 s 0.10 m; anchor satisfied (force 0.05 to 0.17 instead of saturated at 0.40); CBF interventions near zero once settled; navigation idle after ~10 s |
| `tune_A_band` | stop band 0.15/0.30 -> 0.5/0.9 (**current defaults**) | **0.060 m** | 1.04 m | converged at 7.9 s; after 20 s the robots stand completely still (turn rate 1.71 -> 0.00 rad/s, forward speed 0.087 -> 0.000 m/s) and the settled error improves (mean 0.068 -> 0.060 m, max 0.113 -> 0.060 m) |

What changed to get there:
- **Wheel odometry never reached the EKF** (fixed). The bridge declared the odometry
  topic as `gz.msgs.OdometryWithCovariance`, but the DiffDrive plugin publishes
  `gz.msgs.Odometry`, so `/robot_i/odom` carried no messages. The EKF's fallback then
  accepted a commanded velocity only while its stored velocity was exactly zero, so it
  dead-reckoned with the robot's *first* command for the whole run, kept on track only by
  landmark sightings. Heading errors of 0.1 to 0.4 rad rotated every neighbour
  measurement, so robots chased phantom formation errors and spun in place. **This
  affected every Gazebo result before 2026-11-07, including M5 to M7.** The localization
  node now warns if no odometry arrives within 5 s.
- **Simulated time.** All nodes and the rosbag recorder use the Gazebo `/clock`
  (`use_sim_time`, launch default `true`). Gazebo in WSL runs at a real-time factor of
  about 0.05, so on the wall clock the control loop saw ~20x artificial delay.
- **Static world model** (`world_model.py`): pillars as circles, slabs and rubble as exact
  oriented boxes, boundary walls as boxes. Every box is sensed as its closest surface point;
  landmark centres travel separately in `NeighborList.landmarks` for the EKF.
- **Navigation layer** (`navigation.py`, `use_navigation: true`): each robot plans an A*
  route on the known map, with obstacles inflated by the clearance the CBF needs (0.5 m),
  toward the point where its formation forces balance. It overrides formation control
  only while the straight line to that point is blocked. It uses the map and the robot's
  EKF position estimate.
- **Formation anchor** (`anchor_gain: 0.4`, on by default): a virtual-leader pull toward the
  spawn centroid (0.44, -0.44); the navigation target includes it, so robots are routed
  around obstacles toward it. It removes the formation drift that pinned robots on the
  wall. It only works with good localization: every anchored failure in Gazebo happened
  while odometry was missing.

Still open:
- **Turning at rest** (fixed). The `calm` twist mapper (`twist_mapper: calm`) scales the
  turn rate with the force size and stops below `stop_force` until the force exceeds
  `resume_force`; the CBF passes pure rotations, which cannot reduce any distance. With a
  0.15/0.30 band, Gazebo robots still turned at 1.71 rad/s once settled, because the ~7 cm
  settled formation error leaves residual forces of 0.4 to 0.9. The default band 0.5/0.9
  makes them stand still (0.00 rad/s, 0.000 m/s) and slightly improves accuracy. Trade-off:
  robots stop at about 6 cm formation error (kinematic sim clean: 0.014-0.021 -> 0.057-0.065
  m; impaired: 0.12-0.16 -> 0.09-0.12 m, turning 1.84 -> 0.77 rad/s). Reverse driving was
  tried and rejected: it fought the CBF look-ahead point (0/4 runs converged).
- **Planner clearance vs CBF** (fixed). The navigation grid used a fixed 0.50 m
  inflation, below the 0.52 m the look-ahead CBF needs at default settings and far below
  it under delay, so routes led into gaps the CBF refused to pass. The inflation is now
  derived from the CBF settings (`nav_inflation: -1`): `safety_dist_obs + cbf_lookahead +
  max_linear_vel * delay_bound_sec + 0.05` (0.57 m at defaults, 0.67 m with a 120 ms
  bound). The line-of-sight check tolerates a start or target inside an obstacle band only
  up to that point's own depth in it. Kinematic sim with 80 +/- 40 ms delay in the arena:
  0 to 1 of 4 runs converged before, 4 of 4 after.
- **Anchor transients.** The formation can assemble away from the anchor point and is
  then dragged into place; on the way, obstacles can hold some robots back and distort
  the shape for tens of seconds (`cbf_final_default`, 22 to 35 s).
- **Single samples.** Each configuration was run once or twice in Gazebo (3 to 16 min per
  run in WSL), so these are examples, not statistics; earlier runs showed large
  run-to-run variation.
- **Only the clean channel was tested in Gazebo.** Packet loss, delay, noise and robot
  faults have so far only been evaluated in the kinematic campaign (Section 6).

### 6.2 Gazebo Fault Tests

Default configuration with `enable_faults:=true` (the launch file's network impairment), one
run each, 40 s of simulated time, rosbags recorded. The CBF delay bound was left at its
default 50 ms although the applied delay reaches 120 ms, as it would be deployed.

| Run | Faults | Converged at | Settled shape error, mean / max | Final | Closest robots | Localization error | Settled turn / speed (rad/s, m/s) |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| `tune_A_band` | none (clean channel, reference) | 8.0 s | 0.060 / 0.060 m | 0.060 m | 1.04 m | 0.004 m | 0.00 / 0.000 |
| `fault_net_only` | 15% drop, 80 +/- 40 ms delay | 7.8 s | 0.180 / 0.288 m | 0.181 m | 1.15 m | 0.034 m | 2.05 / 0.236 |
| `fault_robot_death` | network + robot 4 motors off 5-10 s | 9.8 s | 0.141 / 0.271 m | 0.271 m | 1.05 m | 0.021 m | 1.73 / 0.177 |
| `fault_composite` | network + robot 4 death + robot 2 sensor bias (0.25 m, 0.08 rad, 2-40 s) + robot 3 sensor cut-outs (6-25 s) | 20.6 s | 0.230 / 0.312 m | 0.289 m | 0.90 m | 0.101 m | 1.48 / 0.130 |
| `fault_net_only_comp` | 15% drop, 80 +/- 40 ms delay; + delay compensation | 10.7 s | 0.187 / 0.306 m | 0.285 m | 1.13 m | 0.027 m | 1.86 / 0.165 |
| `fault_net_sensorfix` | 15% drop, 80 +/- 40 ms delay; + compensation + sensor timing fix (current defaults) | 7.0 s | 0.161 / 0.290 m | 0.282 m | 1.12 m | 0.018 m | 1.55 / 0.124 |

Findings:
- **Safety held in every fault run**: no collisions (closest approach 0.90 m), no node
  crashes, and robot 4's motor kill and restore fired as configured.
- **Delay costs accuracy, not stability.** With 80 +/- 40 ms delay the error does not settle
  flat (0.06 m on a clean channel) but wobbles between about 0.08 and 0.25 m without
  growing, and the robots keep making corrections: they react to neighbour positions that
  are 80 to 120 ms old. This matches the kinematic campaign (accuracy degrades with delay,
  Section 6).
- **A temporary robot death leaves no lasting damage**: after robot 4 recovers, the error
  stays in the same band as with delay alone.
- **Persistent sensor bias is the hardest case.** In the composite run the formation recovers
  from robot 4's death (0.95 m at 10 s to 0.14 m at 22 s), then drifts back to about 0.3 m
  while robot 2's sensors stay biased. The bias also corrupts robot 2's landmark sightings,
  so localization error rises to 0.10 m (0.02 to 0.03 m in the other fault runs). Nothing
  in the stack detects or rejects a biased sensor.
- **Delay compensation** (`delay_compensation: true`): each delayed neighbour measurement
  is corrected by the robot's own motion since it was taken (exact, from the EKF pose
  history). In the kinematic sim this cut the final error under the launch fault settings
  from 0.09-0.12 to 0.07-0.10 m and rescued 200 ms runs (1/4 -> 3/4 converge); also
  extrapolating the neighbour's motion made things worse (the velocity estimate is too
  noisy), so `peer_velocity_gain` stays 0. In Gazebo the effect was within run-to-run
  noise: settled robots move only ~2 cm in 100 ms, too little to explain the wobble.
- **The rosbag showed the real Gazebo error source: sensor timing.** The simulated sensor
  combined poses of different ages (poses at 50 Hz, sensor at 20 Hz) and stamped the result
  with its publish time. Once robots moved and turned (~2 rad/s), the measurement was off
  by 9 cm on average even though no sensor noise is configured (0 cm when robots stand
  still). The sensor now stamps with the time of the own pose it uses and evaluates
  neighbours at that time, as a real sensor would: measurement error 0.091 -> 0.027 m,
  staleness error 0.104 -> 0.052 m, settled formation error 0.180 -> 0.161 m.
- **Remaining wobble under delay** (~0.16 m mean, up to 0.29 m) now comes mostly from the
  control loop acting on 80-200 ms old data, plus a tail of large errors when jittered
  packets arrive out of order (~730 per run). Lowering the formation gain under delay,
  or dropping out-of-order packets, are the next things to try.
- These are single runs; the delay wobble makes the final value of any one run noisy (it
  crossed 0.25 m at the last sample in two of three runs).

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
