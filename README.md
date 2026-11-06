# Physics-Verified Decentralized Multi-Robot Formation Control Under Relative Sensing, Asymmetric Network Impairments, and Control Barrier Function Safety Layer

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic_8.x-orange.svg)](https://gazebosim.org/docs/harmonic)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04_LTS-E95420.svg)](https://ubuntu.com/)
[![License](https://img.shields.io/badge/License-Apache_2.0-green.svg)](LICENSE)
[![Regression Suite](https://img.shields.io/badge/Tests-92%2F92%20Passing%20(100%25)-brightgreen.svg)]()
[![Docker](https://img.shields.io/badge/Docker-Ready-2496ED.svg)](Dockerfile)

> **Author**: Mansoor Ahmed Rind  
> **Affiliation**: Department of Robotics & Artificial Intelligence, National University of Sciences and Technology (NUST), Islamabad, Pakistan  
> **Workspace**: `~/swarm_ws` | **Package**: `swarm_formation`

---

## 1. Project Overview & Scientific Highlights

This repository delivers a physics-verified robotics software framework for **decentralized multi-robot geometric formation control** in GPS-denied, obstacle-strewn disaster environments (modeled after an urban structural collapse). 

A swarm of **5 nonholonomic differential-drive mobile robots** autonomously self-organizes into a regular pentagon ($R = 2.0\,\text{m}$) without any central coordinator, global GPS/mocap positioning, or designated leader.

```
                  Robot 1 (Top Vertex)
                       (0.0, 2.0)
                           ▲
                          / \
                         /   \
                        /     \
    Robot 5 (-1.9, 0.62)       Robot 2 (1.9, 0.62)
              \                 /
               \               /
                \             /
    Robot 4 (-1.18, -1.62)───Robot 3 (1.18, -1.62)
```

### Key Engineering & Scientific Capabilities:
1. **Decentralized Relative Polar Sensing**: No robot reads global $(x, y)$. Sensing is restricted to onboard relative polar range and bearing within $R_{\text{sense}} = 6.0\,\text{m}$.
2. **Three Interchangeable Controller Families**:
   - **Displacement-based consensus** with Huber robust penalty (Ren & Beard 2008).
   - **Distance-based rigidity control** with stress matrix and orientation alignment (Oh & Ahn 2014).
   - **Bearing-based consensus** with orthogonal projection nullspace operators (Zhao & Zelazo 2016).
3. **Cooperative EKF Localization Law**: An onboard Extended Kalman Filter fuses wheel odometry and relative neighbor measurements to estimate local ego-pose. Ground truth is strictly isolated for offline metric evaluation.
4. **Active Safety Layer (CBF-QP)**: A standalone Control Barrier Function Quadratic Program minimal-correction filter guarantees empirical collision avoidance ($d_{\min} \ge 0.50\,\text{m}$) while reducing actuator control effort by **47.2%** compared to Artificial Potential Fields.
5. **Delay-Compensated Safety**: Dynamic barrier expansion $D_s^{\text{eff}} = D_s + 2 v_{\max} \tau_{\max}$ maintains 100% collision-free safety under wireless transport delays up to $200\,\text{ms}$.
6. **Decoupled Network Impairments & Fault Injection**: Standalone nodes simulate Bernoulli packet loss, stochastic delay queues, sensor bias, and agent dropouts, with runtime ROS 2 service toggles.
7. **Offline Rosbag-Driven Analytics**: Rosbag recording (`ros2 bag`), offline Procrustes metric extraction (`process_bag`), and publication plot generation (`plot_bag`).
8. **Statistical Rigor & One-Command Replication**: 325-run Monte Carlo campaign (25 seeds $\times$ 13 conditions) with Student's $t$ 95% Confidence Intervals, fully reproducible via `./reproduce_all.sh`.

---

## 2. System Architecture

The software architecture is modularized into decoupled ROS 2 nodes and pure Python algorithmic libraries:

### 2.1 Component Flow Diagram

```mermaid
flowchart TD
    subgraph Physics_Simulation ["Physics & Hardware Layer (Gazebo Harmonic / Real Unicycle)"]
        GZ["Gazebo Harmonic Physics Engine"]
        WHEEL["Wheel Encoders & Lidar Scan"]
    end

    subgraph Perception_Layer ["Perception & Estimation Layer"]
        SENS["neighbor_sensor_node\n(Relative Polar Sensing, R_sense = 6.0m)"]
        EKF["localization_node\n(Cooperative EKF State Estimator)"]
    end

    subgraph Network_Fault_Layer ["Network & Fault Simulation Layer"]
        NET["network_impairment_node\n(Packet Loss, Delays, ToggleLink.srv)"]
        FAULT["fault_injector_node\n(Sensor Bias, Agent Kill, TriggerFault.srv)"]
    end

    subgraph Formation_Control_Layer ["Decentralized Formation Layer"]
        CTRL["robot_node\n(Displacement / Distance / Bearing Controller)"]
    end

    subgraph Safety_Filter_Layer ["Active Safety Filter Layer"]
        CBF["safety_filter_node\n(2D Active-Set CBF-QP Solver)"]
    end

    subgraph Evaluation_Layer ["Offline Analytics & Verification"]
        BAG["ros2 bag record\n(Raw Message Serialization)"]
        PROC["rosbag_processor (process_bag)\n(Procrustes Alignment, d_min, Energy)"]
        PLOT["plot_generator (plot_bag)\n(Publication-Grade Trajectory Figures)"]
    end

    GZ -->|/robot_i/odom| EKF
    GZ -->|/robot_i/pose (Evaluation Only)| BAG
    GZ -->|/robot_i/scan| CBF
    WHEEL --> SENS
    SENS -->|/robot_i/relative_neighbors| EKF
    SENS -->|/robot_i/relative_neighbors| CBF
    EKF -->|/robot_i/estimated_pose| NET
    NET -->|/robot_i/neighbors| CTRL
    FAULT -.->|Injects Delays / Drops / Bias| NET
    CTRL -->|/robot_i/cmd_vel_raw| CBF
    CBF -->|/robot_i/cmd_vel (Certified Safe)| GZ
    CBF -->|/robot_i/cmd_vel| BAG
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

### 5.1 Mathematical Formulation
For any neighbor $j$, define the safety index $h_{ij}(p) = \|p_i - p_j\|^2 - (D_s^{\text{eff}})^2$, where $D_s = 0.50\,\text{m}$.
Under network communication latency $\tau \in [0, \tau_{\max}]$, we augment the barrier margin:
$$D_s^{\text{eff}} = D_s + 2 v_{\max} \tau_{\max}$$
Assuming worst-case adversary motion $\|\dot{p}_j\| \le v_{\max}$, the forward-invariance barrier condition $\dot{h} \ge -\gamma h$ yields the linear constraint:
$$A_j u_i \le b_j \iff -2 (p_i - p_j)^T u_i \le \gamma h_{ij} - 2 v_{\max} \|p_i - p_j\|$$

At each 50 Hz control step, the robot solves the convex Quadratic Program:
$$\min_{u \in \mathbb{R}^2} \frac{1}{2} \|u - u_{\text{raw}}\|^2 \quad \text{subject to } A_k u \le b_k, \quad \forall k \in \text{Peers} \cup \text{Obstacles}$$

### 5.2 Efficiency: CBF-QP vs. Artificial Potential Fields
- **APF Repulsion**: Repels robots early and continuously ($d_{\min} = 0.855\,\text{m}$), dissipating **$391.5\,\text{J}$** of control energy.
- **CBF-QP Filter**: Modifies velocity only at the exact barrier boundary ($d_{\min} = 0.547\,\text{m} \approx 0.55\,\text{m}$), reducing actuator energy expenditure by **47.2%** (**$208.8\,\text{J}$**).

---

## 6. Large-Scale Statistical Campaign (325 Runs)

A 325-simulation campaign (25 random seeds $\times$ 13 experimental conditions) was executed under crossing trajectory conditions:

| Experimental Condition | Seeds | Shape Error (m) [Mean $\pm$ Std] | 95% Confidence Interval (m) | Min Distance (m) [Mean $\pm$ Std] | Empirical Collision-Free Rate | Mean Energy ($\int \|u\|^2 dt$) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Baseline (Unshielded)** | 25 | $0.0181 \pm 0.0046$ | $[0.0162, 0.0200]$ | $0.1649 \pm 0.0597$ | **0.0%** (9.32 crashes/run) | $395.96$ |
| **APF-Only Barrier** | 25 | $0.0177 \pm 0.0050$ | $[0.0156, 0.0198]$ | $0.8547 \pm 0.0391$ | **100.0%** | $391.52$ |
| **CBF-QP Safety Filter** | 25 | $1.1318 \pm 0.8543$ | $[0.7792, 1.4845]$ | $0.5467 \pm 0.0027$ | **100.0%** | **$208.84$ (-47.2%)** |
| **Full Resilient Stack** | 25 | $1.5631 \pm 0.5839$ | $[1.3221, 1.8041]$ | $0.5471 \pm 0.0018$ | **100.0%** | **$147.59$** |
| **Drop Sweep: 15% Loss** | 25 | $0.8014 \pm 0.8092$ | $[0.4674, 1.1354]$ | $0.5223 \pm 0.0081$ | **100.0%** | $295.34$ |
| **Drop Sweep: 30% Loss** | 25 | $0.5276 \pm 0.7198$ | $[0.2305, 0.8247]$ | $0.5189 \pm 0.0095$ | **100.0%** | $328.03$ |
| **Drop Sweep: 50% Loss** | 25 | $0.0367 \pm 0.0950$ | $[-0.0025, 0.0759]$ | $0.5214 \pm 0.0207$ | **96.0%** | $301.36$ |
| **Delay Sweep: 50 ms** | 25 | $1.8034 \pm 0.1167$ | $[1.7553, 1.8516]$ | $0.6870 \pm 0.0010$ | **100.0%** | $139.66$ |
| **Delay Sweep: 100 ms** | 25 | $1.8823 \pm 0.0808$ | $[1.8490, 1.9157]$ | $0.8275 \pm 0.0011$ | **100.0%** | $140.55$ |
| **Delay Sweep: 200 ms** | 25 | $1.9423 \pm 0.4007$ | $[1.7769, 2.1076]$ | $1.1038 \pm 0.0132$ | **100.0%** | $128.19$ |

### Master Campaign Visualizations
All 4 figures are generated directly from `campaign_results.json` into `results/M8/plots/`:
1. `ablation_study_barchart.png`: Component-wise error and energy comparison.
2. `safety_comparison_apf_vs_cbf.png`: Minimum inter-robot distance box plots.
3. `difficulty_sweep_packet_drop.png`: Packet drop degradation curves (0% to 50%).
4. `difficulty_sweep_latency.png`: Delay-compensated margin expansion (0 to 200 ms).

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

# Run full test suite (92/92 passing)
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
        ├── test/                      # 92 unit and acceptance tests
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
