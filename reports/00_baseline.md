# Baseline Project Assessment & Empirical Benchmark Report

**Date**: 2026-11-06  
**Environment**: Ubuntu 24.04 LTS (Noble Numbat), ROS 2 Jazzy Jalisco, Gazebo Harmonic 8.x, WSL2  
**Workspace**: `/home/kali/swarm_ws`  
**Package**: `swarm_formation`  
**Git Baseline Commit**: `26b9572` (`origin/main`)

---

## 1. Executive Summary & Workspace Inspection

An exhaustive audit and verification of the existing decentralized multi-robot formation control stack was conducted. The project targets decentralized formation of $N=5$ nonholonomic differential-drive robots into a regular pentagon ($R = 2.0\,\text{m}$) within a collapsed building disaster arena (`worlds/collapsed_building.sdf`).

### Existing Workspace Assets
- `package.xml` & `setup.py`: Standard `ament_python` ROS 2 package `swarm_formation`.
- `launch/gazebo_swarm_launch.py`: Spawns 5 robots, launches `ros_gz_bridge`, 5 instances of `gazebo_node`, and `metrics_logger`.
- `swarm_formation/gazebo_node.py`: 505-line monolithic controller node.
- `swarm_formation/metrics_logger.py`: 354-line logger recording real-time telemetry to CSV and publishing RViz markers.
- `urdf/robot.urdf`: 5.0 kg differential-drive mobile robot with rigid chassis, two drive wheels, passive spherical casters, diff-drive plugin, and pose publisher.
- `worlds/collapsed_building.sdf`: Disaster zone with steel columns, fallen concrete slabs, and rubble.
- `decentralized_swarm_quickstart_guide.pdf`: Complete quickstart and installation manual.

---

## 2. Empirical Baseline Reproduction (Real Numbers)

All three existing benchmark configurations were executed headlessly using strict timeouts. Process cleanup was verified post-execution with zero orphaned background tasks.

### Quantitative Benchmark Results Matrix

| Metric | Part 1: Baseline Consensus | Part 2: Real-World Faults | Part 3: Resilient + APF Barrier | Safety Specification |
| :--- | :---: | :---: | :---: | :---: |
| **Command Execution** | `sim_duration:=25.0` | `sim_duration:=25.0` | `sim_duration:=45.0` | Headless execution |
| **Exit Code** | `0` (Clean exit) | `0` (Clean exit) | `0` (Clean exit) | Clean shutdown |
| **Telemetry Rows** | 501 rows (25.0 s) | 501 rows (25.03 s) | 864 rows (45.0 s) | High rate (20 Hz) |
| **Global Min Distance ($d_{\min}$)** | **0.3963 m** (CRASH) | **0.4708 m** (CRASH) | **0.7366 m** (SAFE) | $\ge 0.5000\,\text{m}$ |
| **Physical Collision Timesteps** | **17** | **9** | **0 (Zero collisions)** | $0$ collisions |
| **Final Shape Error RMS** | 0.0350 m | 0.0409 m | 0.0405 m | $\le 0.1500\,\text{m}$ |
| **Convergence Status** | CONVERGED | CONVERGED | CONVERGED | Settle $\le 45.0\,\text{s}$ |
| **Settle Time** | 9.753 s | 16.581 s | 14.901 s | Stable convergence |
| **Safety Barrier Status** | ❌ **CRASH VIOLATION** | ❌ **CRASH VIOLATION** | ✅ **CERTIFIED SAFE** | $d_{\min} \ge 0.50\,\text{m}$ |

### Raw Excerpt Logs
- **Part 1 Summary**:
  ```text
  Configuration:       Part 1: Baseline Consensus
  Total Swarm Agents:  5
  Final Shape Error:   0.0350 m (RMS)
  Convergence Status:  CONVERGED at t = 9.75 s
  Global Min Distance: 0.3963 m (Safety Limit: 0.50 m)
  Total Collisions:    17 crash events detected
  Metrics Saved:       results/baseline/gazebo_part1_baseline.csv
  ```
- **Part 2 Summary**:
  ```text
  Configuration:       Part 2: Real-World Faults Injection (Degraded Baseline)
  Total Swarm Agents:  5
  Final Shape Error:   0.0409 m (RMS)
  Convergence Status:  CONVERGED at t = 16.58 s
  Global Min Distance: 0.4708 m (Safety Limit: 0.50 m)
  Total Collisions:    9 crash events detected
  Metrics Saved:       results/baseline/gazebo_part2_faults.csv
  ```
- **Part 3 Summary**:
  ```text
  Configuration:       Part 3: Resilient Method + APF Safety Barrier
  Total Swarm Agents:  5
  Final Shape Error:   0.0405 m (RMS)
  Convergence Status:  CONVERGED at t = 14.90 s
  Global Min Distance: 0.7366 m (Safety Limit: 0.50 m)
  Total Collisions:    0 crash events detected
  Metrics Saved:       results/baseline/gazebo_part3_resilient.csv
  ```

---

## 3. Detailed Architectural Gap Analysis

Comparison between the current code base and the milestone requirements (M0 through M9):

1. **Monolithic Architecture**: `gazebo_node.py` implements sensing, communication, packet queuing, dropout logic, Huber consensus, APF repulsion, obstacle deflection, and nonholonomic unicycle twist conversion in a single class.
2. **Global Pose Broadcast**: Robots receive ground truth from `/model/robot_{i}/pose` and broadcast absolute coordinates $(x, y, v_x, v_y)$ to `/robot_all_poses`. There is no relative range-and-bearing neighbor sensing node.
3. **Controller Variety**: Only displacement-based consensus is implemented; distance-based and bearing-based controllers do not exist.
4. **Network Impairment In-Code**: Bernoulli drop ($p=0.15$), stochastic latency ($50-150\,\text{ms}$), and Gaussian noise ($\sigma=0.18\,\text{m}$) are embedded directly inside `beacon_callback()` of `gazebo_node.py`. There is no standalone network node, and link states cannot be dynamically toggled.
5. **Hard-Coded Parameters**: Robot 4 dropout ($t \in [5, 10]\,\text{s}$), gains ($k_p, k_d, k_{yaw}$), and APF parameters ($d_{infl}, d_{safe}, k_{rep}, k_{barrier}$) are hard-coded in class initialization rather than declared via YAML parameter files.
6. **Ground Truth Dependency**: No state estimation or relative localization filter exists; control feedback relies entirely on ground truth pose.
7. **Ad-Hoc Safety vs. Formal Filter**: Safety is implemented via an artificial potential field blended with consensus forces in `_compute_resilient_control()`, rather than an independent Control Barrier Function (CBF) quadratic programming safety filter.
8. **Logging & Experimentation Pipeline**: Telemetry logs directly to CSV via `metrics_logger.py`. No MCAP/rosbag recording or offline rosbag evaluation pipeline is present. Experiments are single-seed runs without statistical sweeps or ablation studies.
