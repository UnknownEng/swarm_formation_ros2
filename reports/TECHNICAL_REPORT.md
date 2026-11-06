# Technical Report: Decentralized Multi-Robot Pentagon Formation Control Under Relative Sensing, Asymmetric Network Impairments, and Control Barrier Function Safety Layer

**Author**: Mansoor Ahmed Rind  
**Affiliation**: Department of Robotics & Artificial Intelligence, National University of Sciences and Technology (NUST), Islamabad, Pakistan  
**Target Venue**: IEEE Robotics and Automation Letters (RA-L) / IEEE Transactions on Robotics (T-RO) Draft  
**Workspace**: `~/swarm_ws` | **Package**: `swarm_formation` | **Date**: November 2026  

---

> ## ⚠️ Erratum (2026-11-07): M8 campaign results replaced
>
> An independent code review found that the original 325-run M8 campaign did not support its headline claims:
>
> 1. **Latency was never applied.** `delay_sec` only widened the CBF safety margin; no peer measurement was delayed, and `jitter_sec` was unused. The "safe under 200 ms delay" result therefore tested a bigger margin, not delay.
> 2. **The "47.2% energy reduction" came from deadlock.** In that data the CBF-QP stack reached the formation in only 36% of runs (full stack: 12%; delay sweep: 0–4%). Robots that stop use little energy. The "energy" metric also added linear and angular speeds and was not in joules.
> 3. **The campaign EKF was not an EKF.** Each step it pulled the estimate 5% toward ground truth.
> 4. **The ablations did not isolate components.** They used APF while the full stack used CBF. "Heartbeat only" produced byte-identical results to "APF only".
> 5. **The CBF was unsound for unicycles, and its solver could pick infeasible points.** It used a single-integrator barrier whose output was converted to (v, ω), and a soft-penalty candidate selection returned constraint-violating velocities in about 0.8% of feasible problems.
>
> **Fixes:** a look-ahead CBF with an exact solver and deadlock breaking (`cbf_qp.py`), and a campaign simulator that mirrors the ROS pipeline (`campaign_simulator.py`): applied drop, delay and jitter, the real `RobotEKF`, and the same controller and filter code as the nodes. The new campaign uses leave-one-out ablations and 16 conditions × 25 seeds. The tests now require the formation to actually form, not only collision avoidance. The APF closing-velocity damping sign was also fixed.
>
> **The tables and claims further down this document are from the superseded campaign and are kept only for traceability.** The current numbers are in `results/M8/campaign_summary.csv` and in the table below.
>
> **Still not validated:** none of the fixes has been run in Gazebo yet. Every Gazebo run committed before this erratum that used the safety layer ended without reaching the formation.

### Current M8 results (400 runs, 25 seeds × 16 conditions)

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

---

## Abstract

We present a decentralized, relative-sensing multi-robot system for geometric shape formation in GPS-denied, obstacle-strewn environments, modeled after an urban collapsed-building scenario. A swarm of $N = 5$ nonholonomic differential-drive mobile robots coordinates to achieve and maintain a regular pentagon formation ($R = 2.0\,\text{m}$) without central leadership, global Cartesian positioning, or shared world coordinates. The architecture decouples into: (i) an onboard relative range-and-bearing sensing layer ($R_{\text{sense}} = 6.0\,\text{m}$), (ii) a cooperative Extended Kalman Filter (EKF) state estimator fusing unicycle odometry and neighbor bearings, (iii) three interchangeable controller families (displacement-based consensus, distance-based rigidity with stress matrices, and bearing-based orthogonal projections), (iv) an independent network impairment and fault injection engine simulating packet drop, stochastic latency, and agent blackout, and (v) an active safety layer formulated as a Control Barrier Function Quadratic Program (CBF-QP) with delay-compensated barrier expansion. 

We evaluate the system in a 400-run kinematic campaign (25 seeds $\times$ 16 conditions) that mirrors the ROS node pipeline, with packet drop, delay and jitter applied to every peer measurement and Student's $t$ 95% confidence intervals. Unshielded consensus collides in every crossing run. A look-ahead CBF-QP with deadlock breaking keeps every run collision-free (worst $d_{\min} = 0.576\,\text{m}$) while still reaching the formation in 100% of clean-channel runs. It gives no energy advantage over APF. Formation accuracy holds at 50% packet loss and to about 100 ms of delay; at 200 ms only 36% of runs reach the formation. Among the resilience components, EMA filtering hurts accuracy under delay, Huber is neutral, and the heartbeat is not exercised by the tested conditions. The fixed stack has not yet been re-validated in Gazebo.

---

## 1. Introduction & Motivation

Search-and-rescue operations in collapsed structures require autonomous robotic swarms to deploy into confined, GPS-denied subterranean rubble, establish communication meshes, and maintain spatial formations for distributed sensing, localization, and victim search. In such harsh environments, multi-robot systems encounter four fundamental operational challenges:
1. **Absence of Global Coordinate Infrastructure**: Neither GNSS nor external motion-capture cameras are available. Each agent possesses only local ego-centric polar sensors (range $r_{ij}$ and bearing $\phi_{ij}$).
2. **Asymmetric & Degraded Communications**: Ad-hoc wireless transmissions experience severe multipath reflection, RF attenuation, packet dropouts, and transport delays.
3. **Internal Agent & Sensor Faults**: Agents risk hardware failure, wheel slippage, sensor bias, or permanent communication blackout.
4. **Safety Guarantees Under Nonholonomic Constraints**: While navigating toward formation setpoints, robots must avoid inter-agent collisions without overly conservative repulsive forces that destabilize the formation.

Existing literature frequently assumes holonomic double-integrator dynamics, perfect global state awareness, or omniscient collision avoidance via heuristic potential fields. In this work, we address these limitations by developing an open-source, mathematically sound ROS 2 Jazzy and Gazebo Harmonic robotics software architecture.

---

## 2. System Formulation & Modeling

### 2.1 Nonholonomic Mobile Robot Kinematics
Consider a swarm of $N$ differential-drive mobile robots indexed by $\mathcal{V} = \{1, \dots, N\}$. The kinematic state of robot $i$ in its local inertial reference frame is given by:
$$\dot{x}_i = v_i \cos \theta_i, \quad \dot{y}_i = v_i \sin \theta_i, \quad \dot{\theta}_i = \omega_i$$
where $p_i = [x_i, y_i]^T \in \mathbb{R}^2$ represents 2D planar position, $\theta_i \in (-\pi, \pi]$ is heading orientation, $v_i \in [-v_{\max}, v_{\max}]$ is forward linear velocity, and $\omega_i \in [-\omega_{\max}, \omega_{\max}]$ is angular yaw rate ($v_{\max} = 0.5\,\text{m/s}$, $\omega_{\max} = 1.0\,\text{rad/s}$).

To resolve nonholonomic unicycle tracking under planar velocity commands $u_i = [u_{i,x}, u_{i,y}]^T$, we define a look-ahead offset point $p_{i,L} = p_i + \epsilon [\cos \theta_i, \sin \theta_i]^T$ ($\epsilon = 0.15\,\text{m}$). The unicycle drive commands are mapped via:
$$\begin{bmatrix} v_i \\ \omega_i \end{bmatrix} = \begin{bmatrix} \cos \theta_i & \sin \theta_i \\ -\frac{1}{\epsilon} \sin \theta_i & \frac{1}{\epsilon} \cos \theta_i \end{bmatrix} \begin{bmatrix} u_{i,x} \\ u_{i,y} \end{bmatrix}$$
To eliminate wheel slip and chatter during sharp turns, forward velocity is continuously modulated by heading alignment:
$$v_i \leftarrow v_i \cdot \max(0, \cos e_{\theta, i})$$

### 2.2 Relative Polar Sensing Graph
Communication and sensing topologies are modeled as a dynamic directed graph $\mathcal{G}(t) = (\mathcal{V}, \mathcal{E}(t))$. An edge $(j, i) \in \mathcal{E}(t)$ exists if and only if robot $j$ is within the physical sensing range $R_{\text{sense}} = 6.0\,\text{m}$ of robot $i$:
$$\mathcal{N}_i(t) = \{j \in \mathcal{V} \setminus \{i\} \mid \|p_j - p_i\| \le R_{\text{sense}}\}$$
Crucially, robot $i$ does **not** have access to the absolute coordinate $p_j$. Instead, onboard relative sensors provide noisy polar measurements:
$$r_{ij} = \|p_j - p_i\| + \nu_r, \quad \phi_{ij} = \text{atan2}(y_j - y_i, x_j - x_i) - \theta_i + \nu_\phi$$
with zero-mean Gaussian measurement noise $\nu_r \sim \mathcal{N}(0, \sigma_r^2)$ and $\nu_\phi \sim \mathcal{N}(0, \sigma_\phi^2)$.

---

## 3. Cooperative State Estimation (EKF)

To decouple controller execution from global ground truth, each robot executes a standalone Cooperative Extended Kalman Filter (`localization_node`). 

### 3.1 State and Dynamics
The state vector $\hat{x}_{k} = [\hat{x}_k, \hat{y}_k, \hat{\theta}_k]^T$ is propagated at 50 Hz using wheel odometry $(\Delta s_k, \Delta \theta_k)$:
$$\hat{x}_{k|k-1} = \hat{x}_{k-1} + \begin{bmatrix} \Delta s_k \cos(\hat{\theta}_{k-1} + \frac{\Delta \theta_k}{2}) \\ \Delta s_k \sin(\hat{\theta}_{k-1} + \frac{\Delta \theta_k}{2}) \\ \Delta \theta_k \end{bmatrix}$$
The process covariance $P_k$ is propagated using linearized Jacobian $F_k = \frac{\partial f}{\partial x}$ and wheel encoder covariance $Q_k$.

### 3.2 Measurement Update via Relative Bearings
When relative neighbor measurements $\tilde{z}_{ij} = [r_{ij}, \phi_{ij}]^T$ arrive from `neighbor_sensor_node`, the filter computes innovation:
$$\tilde{y}_{ij} = z_{ij} - h(\hat{x}_i, \hat{x}_j)$$
where $h(\hat{x}_i, \hat{x}_j)$ is the non-linear polar measurement model. Using innovation covariance $S = H P H^T + R$, the Kalman gain $K = P H^T S^{-1}$ updates the local ego-pose.

> **Ground Truth Isolation**: In strict conformance with benchmark ethics, Gazebo Harmonic ground truth (`/model/robot_i/pose`) is utilized solely by `metrics_logger` and `rosbag_processor` for offline validation error calculation $e_{\text{est}} = \|\hat{p}_i - p_i^{\text{gt}}\|$, and is never exposed to the control loop.

---

## 4. Three Interchangeable Controller Families

All three controllers implement a common abstract class `FormationController` and are dynamically selectable at launch via `controller_type:=displacement|distance|bearing`.

```
                  ┌────────────────────────────────────────┐
                  │ FormationController (Common Interface) │
                  └───────────────────┬────────────────────┘
                                      │
            ┌─────────────────────────┼─────────────────────────┐
            ▼                         ▼                         ▼
  ┌───────────────────┐     ┌───────────────────┐     ┌───────────────────┐
  │   Displacement    │     │     Distance      │     │      Bearing      │
  │ Consensus (M4.1)  │     │ Consensus (M4.2)  │     │ Consensus (M4.3)  │
  └───────────────────┘     └───────────────────┘     └───────────────────┘
```

### 4.1 Displacement-Based Formation Consensus
*Theory*: Based on Ren & Beard (2008), the target formation is defined by inter-agent translation offsets $\Delta p_{ij}^* = d_i^* - d_j^* \in \mathbb{R}^2$, where $d_i^*$ is the desired coordinate of robot $i$ in a shared orientation frame.
$$\dot{p}_i = -k_p \sum_{j \in \mathcal{N}_i} a_{ij} \psi_{\delta}\left( (p_i - p_j) - (d_i^* - d_j^*) \right)$$
where $\psi_\delta(\cdot)$ is the Huber penalty gradient with threshold $\delta = 0.5\,\text{m}$:
$$\psi_\delta(e) = \begin{cases} e, & \|e\| \le \delta \\ \delta \frac{e}{\|e\|}, & \|e\| > \delta \end{cases}$$
Huber thresholding clips extreme gradient surges caused by bursty network packet delivery, preventing actuator saturation.

### 4.2 Distance-Based Formation Rigidity
*Theory*: Based on Oh & Ahn (2014), the formation is defined purely by desired pairwise scalar distances $d_{ij}^* = \|d_i^* - d_j^*\|$, requiring no shared coordinate orientation. The energy potential is:
$$V(p) = \frac{1}{4} \sum_{(i,j) \in \mathcal{E}} \left( \|p_i - p_j\|^2 - (d_{ij}^*)^2 \right)^2$$
The resulting gradient descent control law is:
$$\dot{p}_i = -k_{\text{dist}} \sum_{j \in \mathcal{N}_i} \left( \|p_i - p_j\|^2 - (d_{ij}^*)^2 \right) (p_i - p_j) + u_i^{\text{align}}$$
where $u_i^{\text{align}}$ is a rotational alignment torque preventing mirror-image reflections and manifold topological folds.

### 4.3 Bearing-Based Formation Control
*Theory*: Based on Zhao & Zelazo (2016), the target geometry is specified by inter-agent unit bearing vectors $g_{ij}^* = \frac{d_j^* - d_i^*}{\|d_j^* - d_i^*\|} \in \mathbb{S}^1$. The orthogonal projection matrix onto the nullspace of $g_{ij}^*$ is:
$$P_{g_{ij}^*} = I_2 - g_{ij}^* (g_{ij}^*)^T$$
The decentralized control law minimizes bearing misalignment:
$$\dot{p}_i = -k_{\text{bear}} \sum_{j \in \mathcal{N}_i} P_{g_{ij}^*} (p_j - p_i)$$
Since $P_{g_{ij}^*} g_{ij}^* = 0$, velocity commands vanish if and only if all agents lie on their designated bearing rays, enforcing scale-invariant geometric rigidity.

---

## 5. Control Barrier Function (CBF-QP) Active Safety Layer

To enforce collision avoidance while minimizing disruption to formation consensus, each robot executes a standalone `safety_filter_node`.

```
  ┌──────────────┐   u_raw   ┌───────────────────────────┐   u_safe   ┌────────────────┐
  │ Robot Node   ├──────────►│    Safety Filter Node     ├───────────►│ Differential   │
  │ (Controller) │           │ (Active-Set 2D QP Solver) │            │ Drive Actuator │
  └──────────────┘           └─────────────▲─────────────┘            └────────────────┘
                                           │
                         Relative Neighbors & Obstacles
```

### 5.1 Barrier Formulation
For any peer robot $j$, define the safety index $h_{ij}(p) = \|p_i - p_j\|^2 - (D_s^{\text{eff}})^2$, where $D_s = 0.50\,\text{m}$ is the physical clearance threshold. 
Under delay bound $\tau \in [0, \tau_{\max}]$, we augment the barrier with a forward-prediction latency margin:
$$D_s^{\text{eff}} = D_s + 2 v_{\max} \tau_{\max}$$
For $\tau_{\max} = 0.10\,\text{s}$ and $v_{\max} = 0.5\,\text{m/s}$, $D_s^{\text{eff}} = 0.50 + 2(0.5)(0.1) = 0.60\,\text{m}$.

The time-derivative of the barrier satisfies:
$$\dot{h}_{ij} = 2 (p_i - p_j)^T (\dot{p}_i - \dot{p}_j) \ge -\gamma h_{ij}(p_i, p_j)$$
Assuming worst-case uncooperative peer velocity $\|\dot{p}_j\| \le v_{\max}$, the decentralized safety constraint on robot $i$'s command $u_i = \dot{p}_i$ is:
$$2 (p_i - p_j)^T u_i \ge -\gamma h_{ij} + 2 v_{\max} \|p_i - p_j\|$$

### 5.2 Quadratic Program Minimal-Correction Filter
At each control cycle, robot $i$ solves the convex Quadratic Program:
$$\min_{u \in \mathbb{R}^2} \frac{1}{2} \|u - u_{\text{raw}}\|^2$$
$$\text{subject to } A_k u \le b_k, \quad \forall k \in \mathcal{N}_i^{\text{peers}} \cup \mathcal{N}_i^{\text{obstacles}}$$
where $A_k = -2 (p_i - p_k)^T$ and $b_k = \gamma h_k - 2 v_{\max} \|p_i - p_k\|$.
We solve this 2D QP deterministically using an active-set solver in $< 0.1\,\text{ms}$, guaranteeing real-time execution at 50 Hz.

---

## 6. Experimental Evaluation & Statistical Rigor

### 6.1 Monte Carlo Experimental Campaign Design
To ensure scientific validity, we executed **325 full simulations** comprising 25 independent random seeds across 13 distinct experimental conditions. All conditions were executed under identical initial crossing configurations to force inter-agent collision threats.

### 6.2 Master Statistical Benchmark Table

| Experimental Condition | Seeds ($N$) | Final RMS Shape Error (m) | 95% Confidence Interval (m) | Min Distance $d_{\min}$ (m) | Mean Collisions | Empirical Collision-Free Rate | Mean Control Energy ($\int \|u\|^2 dt$) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **1. Baseline (Unshielded)** | 25 | $0.0181 \pm 0.0046$ | $[0.0162, 0.0200]$ | $0.1649 \pm 0.0597$ | 9.32 | **0.0%** | $395.96 \pm 41.2$ |
| **2. APF-Only Barrier** | 25 | $0.0177 \pm 0.0050$ | $[0.0156, 0.0198]$ | $0.8547 \pm 0.0391$ | 0.00 | **100.0%** | $391.52 \pm 38.7$ |
| **3. CBF-QP Safety Filter** | 25 | $1.1318 \pm 0.8543$ | $[0.7792, 1.4845]$ | $0.5467 \pm 0.0027$ | 0.00 | **100.0%** | **$208.84 \pm 22.4$** |
| **4. Ablation: Huber-Only** | 25 | $0.0167 \pm 0.0054$ | $[0.0145, 0.0189]$ | $0.9093 \pm 0.0489$ | 0.00 | **100.0%** | $354.27 \pm 33.1$ |
| **5. Ablation: EMA-Only** | 25 | $0.0181 \pm 0.0044$ | $[0.0162, 0.0199]$ | $0.8338 \pm 0.0447$ | 0.00 | **100.0%** | $385.94 \pm 36.8$ |
| **6. Ablation: Heartbeat-Only**| 25 | $0.0177 \pm 0.0050$ | $[0.0156, 0.0198]$ | $0.8547 \pm 0.0391$ | 0.00 | **100.0%** | $391.52 \pm 38.7$ |
| **7. Full Resilient Stack** | 25 | $1.5631 \pm 0.5839$ | $[1.3221, 1.8041]$ | $0.5471 \pm 0.0018$ | 0.00 | **100.0%** | **$147.59 \pm 18.2$** |
| **8. Drop Sweep: 15% Loss** | 25 | $0.8014 \pm 0.8092$ | $[0.4674, 1.1354]$ | $0.5223 \pm 0.0081$ | 0.00 | **100.0%** | $295.34 \pm 31.0$ |
| **9. Drop Sweep: 30% Loss** | 25 | $0.5276 \pm 0.7198$ | $[0.2305, 0.8247]$ | $0.5189 \pm 0.0095$ | 0.00 | **100.0%** | $328.03 \pm 34.5$ |
| **10. Drop Sweep: 50% Loss** | 25 | $0.0367 \pm 0.0950$ | $[-0.0025, 0.0759]$ | $0.5214 \pm 0.0207$ | 0.64 | **96.0%** | $301.36 \pm 32.8$ |
| **11. Delay Sweep: 50 ms** | 25 | $1.8034 \pm 0.1167$ | $[1.7553, 1.8516]$ | $0.6870 \pm 0.0010$ | 0.00 | **100.0%** | $139.66 \pm 15.4$ |
| **12. Delay Sweep: 100 ms** | 25 | $1.8823 \pm 0.0808$ | $[1.8490, 1.9157]$ | $0.8275 \pm 0.0011$ | 0.00 | **100.0%** | $140.55 \pm 14.8$ |
| **13. Delay Sweep: 200 ms** | 25 | $1.9423 \pm 0.4007$ | $[1.7769, 2.1076]$ | $1.1038 \pm 0.0132$ | 0.00 | **100.0%** | $128.19 \pm 13.9$ |

### 6.3 Quantitative Findings & Discussion

1. **Unshielded Catastrophic Collision**: Without an active safety layer, standard consensus controllers guide robots along straight-line geodesics directly through the formation centroid, resulting in severe physical collisions in 100% of trials ($d_{\min} = 0.165\,\text{m}$, 9.32 collision impacts per run).
2. **Minimal-Correction Efficiency (CBF-QP vs. APF)**:
   - While Artificial Potential Fields repel oncoming robots early and aggressively ($d_{\min} = 0.855\,\text{m}$), they generate high non-conservative forces that drive the swarm wide, dissipating $391.5\,\text{J}$ of energy.
   - CBF-QP acts as a minimal-intervention barrier filter: it applies zero control deflection when $h(x) \gg 0$ and smoothly brakes or deflects agents only at the boundary ($d_{\min} = 0.547\,\text{m}$ vs $D_s = 0.55\,\text{m}$), cutting energy expenditure by **47.2%** ($208.8\,\text{J}$).
3. **Delay-Compensated Barrier Expansion**: In presence of communication delays, static barriers fail due to state staleness. Our dynamic expansion $D_s^{\text{eff}} = D_s + 2 v_{\max} \tau$ automatically widens the safety perimeter from $0.547\,\text{m}$ ($\tau = 0$) to $1.104\,\text{m}$ ($\tau = 200\,\text{ms}$), ensuring that not a single collision occurs across 100 delayed simulation runs.

---

## 7. Limitations & Honest Engineering Caveats

1. **Empirical Collision Avoidance vs. Formal Guarantees**: While 100% collision-free in our 325-run evaluation, we designate this as **empirically collision-free**. Formal forward invariance guarantees hold under continuous control and unbounded acceleration; real physical unicycles possess nonholonomic non-integrable Pfaffian constraints and finite torque limits.
2. **Topological Disconnection Under Extreme Dropout**: At 50% packet dropouts, 1 out of 25 seeds experienced a glancing inter-agent touch ($d_{\min} = 0.491\,\text{m} < 0.50\,\text{m}$), yielding a 96% collision-free rate. High packet drop combined with unicycle rotation limits can temporarily breach margin thresholds.
3. **Hardware-in-the-Loop Transfer**: All benchmarks were executed in Gazebo Harmonic physics and 50 Hz deterministic unicycle simulations. Ground contact friction, tire deformation, and real WiFi packet fragmentation must be validated on physical TurtleBot3 hardware.

---

## 8. Conclusion

We built a modular ROS 2 formation-control stack with relative sensing, cooperative EKF localization, interchangeable controllers, network and fault injection, and a look-ahead CBF-QP safety filter. In a kinematic campaign that mirrors the node pipeline, the stack is collision-free in every run and reaches the formation reliably up to about 100 ms of delay. Physics validation of the fixed safety layer in Gazebo, and a real heading reference for hardware, remain open.

---

## References

1. W. Ren and R. W. Beard, *Distributed Consensus in Multi-vehicle Cooperative Control*, Springer, 2008.
2. K.-K. Oh and H.-S. Ahn, "Distance-based formation control using Euclidean distance dynamics," *Automatica*, vol. 50, no. 10, pp. 2526–2533, 2014.
3. S. Zhao and D. Zelazo, "Bearing rigidity and almost global bearing-only formation stabilization," *IEEE Transactions on Automatic Control*, vol. 61, no. 5, pp. 1255–1268, 2016.
4. A. D. Ames, S. Coogan, M. Egerstedt, G. Notomista, K. Sreenath, and P. Tabuada, "Control barrier functions: Theory and applications," in *Proc. IEEE Conf. Decision and Control (CDC)*, 2019, pp. 3420–3431.
5. J. Cortez, D. Oetomo, C. Manzie, and P. Choong, "Control barrier functions for mechanical systems: Complete characterization and safe control," *IEEE Transactions on Automatic Control*, vol. 67, no. 2, pp. 611–626, 2022.
