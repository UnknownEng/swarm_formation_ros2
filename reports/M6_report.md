# Milestone M6 Report: Standalone CBF-QP Safety Layer

- **Milestone**: M6 (Safety Layer)
- **Engineer Role**: Senior Robotics Safety Systems Engineer (Control Barrier Functions & QP Filtering)
- **Date**: 2026-11-06
- **Git Commit Hash**: `d209c18` (Branch: `milestone/M6`)

---

## 1. Scope: Required vs Delivered

| Requirement | Status | Details |
| :--- | :--- | :--- |
| Standalone Safety Filter Node | **DELIVERED** | `safety_filter_node.py` sits strictly between high-level controllers (`/cmd_vel_raw`) and actuators (`/cmd_vel`) |
| Control Barrier Function Formulation | **DELIVERED** | Reciprocal pairwise CBF for inter-robot collisions and static obstacle barrier certificates in `cbf_qp.py` |
| Quadratic Program Minimal-Correction Filter | **DELIVERED** | Convex 2D QP $\min \frac{1}{2}\|u - u_{\text{des}}\|^2 + \frac{W}{2}\delta^2$ with active-set boundary resolution |
| Explicit Safety Assumptions | **DELIVERED** | Documented finite sensing range $R_{\text{sense}} \le 6.0\text{m}$, unicycle speed limits $\|u\| \le v_{\max}$, and latency bound $\tau \le \tau_{\max}$ |
| Communication Delay Bounds | **DELIVERED** | Delay margin expansion: $D_s^{\text{eff}} = D_s + 2 v_{\max} \tau_{\max}$ augmenting safety threshold under network latency |
| Honest Safety Claims | **DELIVERED** | Documentation strictly states "empirically collision-free", avoiding unfounded "guaranteed" claims |
| Zero Collisions in Crossing Benchmarks | **DELIVERED** | Tested in Gazebo Harmonic: baseline controller which previously crashed ($d_{\min}=0.38\text{m}$) maintained $d_{\min}=1.1021\text{m}$ with 0 collisions |

---

## 2. Safety Layer Theory & Mathematical Formulation

### 1. Control Barrier Function (CBF) Formulation
For planar mobile robot $i$ with state $p_i = [x_i, y_i]^T$, unicycle forward speed $v_i$, and orientation $\psi_i$:
 translational velocity is $u_i = [v_i \cos \psi_i, v_i \sin \psi_i]^T$.

For any peer robot $j$ separated by relative displacement $p_{ij} = p_j - p_i$, the collision avoidance certificate is:
$$h_{ij}(p_{ij}) = \|p_{ij}\|^2 - (D_s^{\text{eff}})^2 \ge 0$$
where $D_s^{\text{eff}} = D_s + 2 v_{\max} \tau_{\max}$ compensates for maximum communication and sensing latency $\tau_{\max}$.

Taking the time derivative along trajectories:
$$\dot{h}_{ij} = 2 p_{ij}^T (u_j - u_i) \ge -\gamma h_{ij}$$
Under reciprocal avoidance symmetry (both agents sharing mutual responsibility):
$$-2 p_{ij}^T u_i \ge -\frac{1}{2} \gamma h_{ij} \implies p_{ij}^T u_i \le \frac{\gamma}{4} \left( \|p_{ij}\|^2 - (D_s^{\text{eff}})^2 \right)$$

For static environmental obstacles $k$ at relative position $p_{ik} = p_{\text{obs}, k} - p_i$ with radius $R_k$:
$$\dot{h}_{\text{obs}, k} = -2 p_{ik}^T u_i \ge -\gamma_{\text{obs}} h_{\text{obs}, k} \implies p_{ik}^T u_i \le \frac{\gamma_{\text{obs}}}{2} \left( \|p_{ik}\|^2 - (D_{\text{obs}}^{\text{eff}})^2 \right)$$

### 2. Quadratic Programming (QP) Minimal-Correction Optimization
Given nominal desired velocity $u_i^{\text{des}} \in \mathbb{R}^2$ from the formation controller:
$$\min_{u_i \in \mathbb{R}^2, \delta \ge 0} \frac{1}{2} \|u_i - u_i^{\text{des}}\|^2 + \frac{W_{\text{slack}}}{2} \delta^2$$
subject to:
$$\begin{aligned}
A_k u_i &\le b_k + \delta, \quad \forall k \in \mathcal{C}_{\text{peer}} \cup \mathcal{C}_{\text{obs}} \\
\|u_i\| &\le v_{\max}
\end{aligned}$$
where $\delta \ge 0$ is a high-penalty slack variable ($W_{\text{slack}} = 1000$) preventing solver infeasibility if initialized within the boundary.
The optimal velocity $u_i^*$ is mapped back to physical differential-drive twist $(v_i^*, \omega_i^*)$ via unicycle kinematic tracking.

---

## 3. Changes Made

### Files Added
1. **`src/swarm_formation/swarm_formation/cbf_qp.py`**:
   - `CBFConstraint`: Dataclass storing linear inequality barrier parameters ($a_x, a_y, b, h$, entity metadata).
   - `build_peer_cbf_constraints`: Reciprocal peer barrier formulation with latency margin augmentation.
   - `build_obstacle_cbf_constraints`: Static obstacle barrier formulation with radius and latency compensation.
   - `solve_2d_cbf_qp`: Active-set 2D convex QP solver with slack relaxation and microsecond execution.
   - `compute_cbf_safe_twist`: End-to-end unicycle twist safety filter.
2. **`src/swarm_formation/swarm_formation/safety_filter_node.py`**:
   - Standalone ROS 2 node subscribing to `/robot_{i}/cmd_vel_raw`, `/robot_{i}/neighbors`, and `/robot_{i}/estimated_pose`.
   - Solves CBF-QP at 50Hz, publishing safe `/robot_{i}/cmd_vel` and monitoring signal `/robot_{i}/safety_intervention`.
   - Incorporates stale command timeout (stops if controller fails to publish within 0.6s).
3. **`src/swarm_formation/test/test_cbf_qp.py`**:
   - 6 unit tests covering delay expansion, obstacle constraints, unconstrained pass-through, head-on deflection, slack relaxation, and twist computation.
4. **`src/swarm_formation/test/test_m6_acceptance.py`**:
   - 3 acceptance tests verifying `cmd_vel_raw` topic routing, ROS 2 node execution, and a head-on collision simulation maintaining $d_{\min} \ge 0.50\text{m}$.
5. **`results/M6/`**:
   - `gazebo_cbf_safety.csv`: Headless Gazebo run showing zero collisions on unshielded baseline crossing trajectories.
   - `gazebo_cbf_resilient.csv`: Headless Gazebo run showing safety layer filtering during full swarm formation.

### Files Modified
1. **`src/swarm_formation/swarm_formation/robot_node.py`**:
   - Added parameter `'use_safety_filter'`, default `True`.
   - Reroutes publisher to `/robot_{i}/cmd_vel_raw` when safety filter is active.
2. **`src/swarm_formation/config/swarm_params.yaml`**:
   - Added CBF-QP safety parameters (`safety_dist_peer: 0.55`, `safety_dist_obs: 0.35`, `cbf_gamma_peer: 1.5`, `cbf_gamma_obs: 2.0`, `delay_bound_sec: 0.05`).
3. **`src/swarm_formation/launch/gazebo_swarm_launch.py`**:
   - Added launch argument `use_safety_filter` (default: `'true'`).
   - Spawns `safety_filter_node` for each robot.
4. **`src/swarm_formation/setup.py`**:
   - Registered entry point `'safety_filter = swarm_formation.safety_filter_node:main'`.

---

## 4. Commands Run and Execution Logs

| Command | Timeout | Exit Code | Duration | Outcome |
| :--- | :--- | :--- | :--- | :--- |
| `colcon build --symlink-install` | 120s | 0 | 2.91s | Compiled cleanly |
| `colcon test && colcon test-result --verbose` | 180s | 0 | 4.86s | **81 tests passed, 0 failures, 0 errors, 0 skipped** |
| Gazebo Simulation: `use_safety_filter:=true` (Baseline) | 75s | 0 | 44.5s | $d_{\min} = 1.1021\text{m}$, **0 collisions** (baseline previously crashed at $0.38\text{m}$) |
| Gazebo Simulation: `use_resilient:=true use_safety_filter:=true` | 75s | 0 | 49.3s | $d_{\min} = 0.9362\text{m}$, **0 collisions**, mean est error $0.0831\text{m}$ |

---

## 5. Test Evidence & Acceptance Criteria

### Acceptance Criteria Table

| Criterion | Result | Evidence |
| :--- | :--- | :--- |
| **AC-1**: Standalone safety_filter node between controller and cmd_vel | **PASS** | `safety_filter_node.py` verified; intercepts `/cmd_vel_raw` and outputs `/cmd_vel` |
| **AC-2**: Control Barrier Function with QP minimal-correction filter | **PASS** | `cbf_qp.py` active-set 2D QP solver verified in `test_cbf_qp.py` |
| **AC-3**: Explicit assumptions stated (sensing range, speed, delay) | **PASS** | Fully documented in module docstrings and report Section 2 |
| **AC-4**: Delay bounds accounted for in safety formulation | **PASS** | $D_s^{\text{eff}} = D_s + 2 v_{\max} \tau_{\max}$ verified in `test_cbf_peer_constraints_delay_expansion` |
| **AC-5**: Honest safety claims ("empirically collision-free") | **PASS** | Documented throughout docs and reports |
| **AC-6**: Full regression suite for M0 to M5 passes | **PASS** | All 81 unit/acceptance tests pass cleanly |

---

## 6. Issues Found and Root-Cause Remediation

1. **Unicycle Twist Parameter Signature Mismatch**:
   - *Issue*: `cbf_qp.py` called `compute_unicycle_twist` using `target_vel_x` and `target_vel_y`, causing a `TypeError`.
   - *Root Cause*: `kinematics.py` defines parameter names as `fx` and `fy`.
   - *Remediation*: Corrected argument names to `fx=safe_ux, fy=safe_uy`.
2. **Flake8 Docstring Line Length in Acceptance Test**:
   - *Issue*: Line lengths in `test_m6_acceptance.py` reached 101 characters.
   - *Remediation*: Re-wrapped docstrings to adhere strictly to PEP 8 / Flake8 99-character limit.

---

## 7. Known Limitations and Honest Caveats

- **Empirically Collision-Free, Not Unconditionally Guaranteed**: Although the CBF-QP filter eliminates collisions across all evaluated scenarios ($d_{\min} \ge 0.93\text{m}$), unmodeled higher-order actuator dynamics (wheel slip on debris, motor torque saturation, unmodeled packet loss exceeding $\tau_{\max}$) mean safety is **empirically validated**, not guaranteed under arbitrary physical disturbances.
- **Deadlock at Symmetrical Obstacle Oppositions**: In highly congested static environments where barrier constraints from multiple obstacles conflict with zero-slack feasible sets, the QP safely stops the robot ($\|u\| \to 0$) rather than risking collision. A higher-level path planner would be required to break geometric symmetry.

---

## 8. Handoff Notes for Milestone M7 (Rosbag Pipeline)

- The entire swarm system now features:
  - Relative sensing (`neighbor_sensor`)
  - Impairments (`network_impairment`)
  - Fault injection (`fault_injector`)
  - 3 controller families (`robot_node`)
  - Decentralized localization (`localization_node`)
  - Safety filter layer (`safety_filter`)
- Milestone M7 objective:
  - Implement a **rosbag recording and offline metrics pipeline**.
  - Record data via `ros2 bag record`.
  - Offline extraction script to calculate all performance metrics (RMS shape error, min distance, collisions, estimation error) and recreate plots from rosbag files alone, removing reliance on live CSV logging.
