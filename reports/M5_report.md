# Milestone M5 Report: Decentralized Cooperative EKF Localization Law

- **Milestone**: M5 (Localization Law)
- **Engineer Role**: Senior Robotics State Estimation & Localization Engineer
- **Date**: 2026-11-06
- **Git Commit Hash**: `ebd72d2` (Branch: `milestone/M5`)

---

## 1. Scope: Required vs Delivered

| Requirement | Status | Details |
| :--- | :--- | :--- |
| Onboard State Estimation Law | **DELIVERED** | Implemented 2D planar unicycle Cooperative Extended Kalman Filter (`RobotEKF` in `localization.py`) |
| Strict Isolation of Ground Truth | **DELIVERED** | `robot_node.py` AST verified: NO subscriptions to `/robot_{i}/pose` or global coordinates for control; subscribes strictly to `/robot_{i}/estimated_pose` |
| Evaluation-Only Ground Truth | **DELIVERED** | `localization_node.py` subscribes to `/robot_{i}/pose` exclusively to compute estimation error $\|\hat{p}_i - p_i^{\text{gt}}\|$ and $\Delta\psi$; never used in EKF predict/update |
| Fused Sensor Sources | **DELIVERED** | Wheel odometry kinematics ($\Delta s, \Delta \psi$), known disaster landmarks range-bearing updates, and peer relative range-bearing measurements |
| Continuous Error Reporting | **DELIVERED** | Published `/robot_{i}/estimation_error` (`geometry_msgs/msg/Vector3`), logged dynamically into CSV by `metrics_logger` |
| Headless Gazebo Harmonic Validation | **DELIVERED** | Verified multi-robot formation control driven entirely by estimated poses; reached $0.2396\text{m}$ shape error with $0.0515\text{m}$ mean estimation error and 0 collisions |

---

## 2. Localization Law Mathematical Formulation

### State Representation
The state vector of robot $i$ in the local reference coordinate frame is:
$$x_i = \begin{bmatrix} p_{i,x} \\ p_{i,y} \\ \psi_i \end{bmatrix} \in \mathbb{R}^3, \quad P_i \in \mathbb{R}^{3 \times 3}$$

### Motion Prediction Model
Using differential-drive unicycle kinematics driven by wheel odometry $(\nu_i, \omega_i)$:
$$\begin{aligned}
\Delta s &= \nu_i \Delta t, \quad \Delta \psi = \omega_i \Delta t, \quad \psi_{\text{mid}} = \psi_i + \frac{\Delta \psi}{2} \\
x_i(t + \Delta t) &= x_i(t) + \Delta s \cos(\psi_{\text{mid}}) \\
y_i(t + \Delta t) &= y_i(t) + \Delta s \sin(\psi_{\text{mid}}) \\
\psi_i(t + \Delta t) &= \text{wrap}(\psi_i(t) + \Delta \psi)
\end{aligned}$$
State transition Jacobian $F = \frac{\partial f}{\partial x}$ and process covariance propagation $P \leftarrow F P F^T + Q \Delta t$.

### Observation Updates
1. **Direct Heading Correction** from wheel odometry integration:
   $$H_{\text{yaw}} = \begin{bmatrix} 0 & 0 & 1 \end{bmatrix}, \quad \tilde{y} = \text{wrap}(\psi_{\text{meas}} - \hat{\psi}_i)$$
2. **Relative Range-Bearing to Known Landmarks** $(l_x, l_y)$:
   $$\begin{aligned}
   \hat{d} &= \sqrt{(l_x - \hat{x}_i)^2 + (l_y - \hat{y}_i)^2} \\
   \hat{\beta} &= \text{wrap}(\text{atan2}(l_y - \hat{y}_i, l_x - \hat{x}_i) - \hat{\psi}_i) \\
   H_{\text{obs}} &= \begin{bmatrix} -\frac{l_x - \hat{x}_i}{\hat{d}} & -\frac{l_y - \hat{y}_i}{\hat{d}} & 0 \\ \frac{l_y - \hat{y}_i}{\hat{d}^2} & -\frac{l_x - \hat{x}_i}{\hat{d}^2} & -1 \end{bmatrix}
   \end{aligned}$$
   Kalman Gain $K = P H^T (H P H^T + R)^{-1}$, state update $\hat{x} \leftarrow \hat{x} + K \tilde{y}$, and covariance update $P \leftarrow (I - K H) P$.

---

## 3. Changes Made

### Files Added
1. **`src/swarm_formation/swarm_formation/localization.py`**:
   - `RobotEKF`: Pure Python 2D planar EKF estimator with predict, range-bearing correction, heading update, and covariance tracking.
   - `compute_estimation_error`: Computes Euclidean distance error and wrapped absolute heading error against evaluation ground truth.
2. **`src/swarm_formation/swarm_formation/localization_node.py`**:
   - Standalone ROS 2 node subscribing to `/robot_{i}/odom`, `/robot_{i}/cmd_vel`, `/robot_{i}/neighbors`, and peer `/robot_{j}/estimated_pose`.
   - Fuses disaster landmark observations and cooperative peer estimates.
   - Publishes `/robot_{i}/estimated_pose` (`PoseStamped`) and `/robot_{i}/estimation_error` (`Vector3`).
   - Subscribes to Gazebo `/robot_{i}/pose` strictly for offline/online evaluation metric calculations, completely isolated from state estimation.
3. **`src/swarm_formation/test/test_localization.py`**:
   - 8 unit tests covering EKF initialization, straight-line/turn prediction, heading updates, landmark corrections, cooperative peer corrections, error computation, and drift-bounding verification.
4. **`src/swarm_formation/test/test_m5_acceptance.py`**:
   - 4 acceptance tests verifying AST isolation of ground truth in `robot_node.py` and `localization_node.py`, node ROS interfaces, and control execution with estimated poses.
5. **`results/M5/gazebo_localization.csv`**:
   - Real Gazebo Harmonic simulation telemetry recording shape error, minimum distance, zero collisions, and continuous mean estimation error.

### Files Modified
1. **`src/swarm_formation/swarm_formation/robot_node.py`**:
   - Replaced `self.sub_pose` with `self.sub_estimated_pose` subscribing to `/robot_{self.robot_id}/estimated_pose` (`PoseStamped`).
   - Ground truth `/robot_{i}/pose` completely removed from `robot_node.py`.
2. **`src/swarm_formation/swarm_formation/metrics_logger.py`**:
   - Subscribes to `/robot_{i}/estimation_error` and logs `mean_est_error` in console outputs, summary reports, and CSV exports.
3. **`src/swarm_formation/launch/gazebo_swarm_launch.py`**:
   - Added `parameter_bridge` bridge for `/model/robot_{i}/odometry` mapped to `/robot_{i}/odom`.
   - Spawns `localization_node` for each robot with initial coordinates.
4. **`src/swarm_formation/setup.py`**:
   - Registered entry point `'localization_node = swarm_formation.localization_node:main'`.

---

## 4. Commands Run and Execution Logs

| Command | Timeout | Exit Code | Duration | Outcome |
| :--- | :--- | :--- | :--- | :--- |
| `colcon build --symlink-install` | 120s | 0 | 3.11s | Compiled `swarm_interfaces` and `swarm_formation` cleanly |
| `colcon test && colcon test-result --verbose` | 180s | 0 | 4.56s | **72 tests passed, 0 failures, 0 errors, 0 skipped** |
| `ros2 launch swarm_formation gazebo_swarm_launch.py sim_duration:=50.0 use_resilient_method:=true` | 80s | 0 | 54.2s | Converged to $0.2396\text{m}$ RMS error, $0.0515\text{m}$ mean est error, 0 collisions |

---

## 5. Test Evidence & Acceptance Criteria

### Acceptance Criteria Table

| Criterion | Result | Evidence |
| :--- | :--- | :--- |
| **AC-1**: Estimate position from relative measurements/odometry | **PASS** | `RobotEKF` implemented and verified in `test_localization.py` and `localization_node.py` |
| **AC-2**: Ground truth is for evaluation ONLY, never control | **PASS** | Verified via AST check in `test_m5_acceptance.py`; `robot_node.py` has no `/robot_{i}/pose` subscriber |
| **AC-3**: Report estimation error | **PASS** | `/robot_{i}/estimation_error` published at 50Hz and logged into CSV (`mean_est_error`) |
| **AC-4**: Swarm forms pentagon driven by estimated poses | **PASS** | Headless Gazebo run achieved $0.2396\text{m}$ RMS shape error (<0.25m threshold) with 0 collisions |
| **AC-5**: Full regression suite for M0 to M4 passes | **PASS** | All 72 unit/acceptance tests passed cleanly |

### Gazebo Simulation Results (`results/M5/gazebo_localization.csv`)
- **Initial Shape Error**: $3.0945\text{ m}$
- **Final Shape Error**: $0.2396\text{ m}$ (below $0.25\text{m}$ threshold)
- **Global Minimum Inter-Agent Distance**: $0.5801\text{ m}$ (Safety limit: $0.50\text{ m}$)
- **Total Collisions**: **0 crash events**
- **Mean State Estimation Error $\|\hat{p} - p^{\text{gt}}\|$**: **$0.0515\text{ m}$** ($5.1\text{ cm}$)

---

## 6. Issues Found and Root-Cause Remediation

1. **Heading Integration Delay**:
   - *Issue*: EKF state prediction based solely on finite-differenced velocity inputs exhibited yaw drift under high-rate collision avoidance maneuvers.
   - *Root Cause*: Differential drive wheel encoders provide direct integrated angular orientation in `msg.pose.pose.orientation`.
   - *Remediation*: Added `update_heading` in `RobotEKF` to fuse direct odometric orientation at 50Hz, reducing heading estimation error to near zero ($<0.02\text{ rad}$).
2. **Flake8 Import Ordering & Line Length**:
   - *Issue*: Flake8 flagged import order in `localization_node.py` and a 100-character docstring in `test_localization.py`.
   - *Remediation*: Sorted imports alphabetically and re-wrapped docstring. Confirmed 100% flake8 and pep257 compliance.

---

## 7. Known Limitations and Honest Caveats

- **Initial Heading Assumption**: `localization_node` initializes state using the declared launch parameter `(initial_x, initial_y, initial_yaw)`. If the initial spawn heading is completely unknown, global localization requires an initial landmark triangulation or kidnap-recovery phase.
- **Landmark Visibility in Large Arenas**: Localization accuracy depends on observing environmental landmarks. In areas far from known obstacles (>6m), the EKF relies on dead reckoning and cooperative peer measurements, leading to bounded but slowly increasing drift until a landmark is re-acquired.

---

## 8. Handoff Notes for Milestone M6 (Safety Layer: CBF-QP Filter)

- `robot_node.py` now outputs `cmd_vel` based purely on estimated pose and relative sensing.
- Milestone M6 requires implementing a standalone **`safety_filter` node** inserted between the controller and the physical actuators:
  - Input topic: `/robot_{i}/cmd_vel_raw` (from controller).
  - Output topic: `/robot_{i}/cmd_vel` (to Gazebo DiffDrive actuator).
  - Algorithm: Control Barrier Function (CBF) solved via Quadratic Programming (QP minimal-correction filter) with explicit sensing range, speed limits, and communication delay bounds.
  - Until formally guaranteed across all operational envelopes, documentation must honestly state "empirically collision-free".
