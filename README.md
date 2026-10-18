# Physics-Verified Decentralized Swarm Shape Formation Under Adversarial Faults in Gazebo Harmonic

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic_8.x-orange.svg)](https://gazebosim.org/docs/harmonic)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04_LTS-E95420.svg)](https://ubuntu.com/)
[![License](https://img.shields.io/badge/License-Apache_2.0-green.svg)](LICENSE)
[![Build & Tests](https://img.shields.io/badge/Tests-100%25_Passing-brightgreen.svg)]()

> **Author**: Mansoor Ahmed Rind  
> **Affiliation**: Department of Robotics & Artificial Intelligence, National University of Sciences and Technology (NUST), Islamabad, Pakistan  
> **Workspace**: `~/swarm_ws`  
> **Package**: `swarm_formation`

---

## 1. Project Scope & Overview

This repository contains a complete, physics-verified robotics software stack for **decentralized multi-robot geometric shape formation** operating in GPS-denied, obstacle-dense disaster environments. 

A swarm of **5 nonholonomic differential-drive mobile robots** self-organizes into a regular pentagon ($R = 2.0\,\text{m}$) without any central coordinator, global localization, or designated leader. Inter-agent coordination is achieved strictly through peer-to-peer state broadcasts and onboard potential field safety certificates.

### Key Capabilities & Scientific Scope:
1. **High-Fidelity Physics Simulation**: Simulates rigid-body dynamics, chassis inertia ($5.0\,\text{kg}$), contact friction, passive spherical casters, and nonholonomic drive constraints inside Gazebo Harmonic (Gz Sim 8.x).
2. **Search-and-Rescue Disaster Arena (`collapsed_building.sdf`)**: An urban structural collapse site populated with structural steel columns, tilted fallen concrete slabs, and broken rubble blocks.
3. **Adversarial Real-World Fault Injection Engine**:
   - **15% Bernoulli Packet Loss** on inter-agent communication links.
   - **50–150 ms Stochastic Transport Delay** simulating wireless buffer queues.
   - **Additive Gaussian Sensor Noise** ($\sigma = 0.18\,\text{m}$) on relative localization.
   - **Mid-Mission Agent Blackout Window**: Robot 4 experiences complete radio and drive failure during $t \in [5, 10]\,\text{s}$.
4. **Resilient Control & Dual-Layer APF Safety Barrier**:
   - **Robust Huber Consensus**: Bounds transient outlier tracking errors during packet burst recovery.
   - **Exponential Moving Average (EMA) Filtering** ($\alpha = 0.35$): Smooths delayed and noisy state packets.
   - **Dynamic Heartbeat Timeout**: Gracefully excludes silent agents from the graph Laplacian without destabilizing the centroid.
   - **Anti-Symmetric Lateral Deflection ("Rule of the Road")**: Diverts oncoming peer robots to their respective right ($F_{\text{lat}} = 0.6 \, F_{\text{rep}} \hat{t}$), resolving head-on collinear deadlocks and guaranteeing $d_{\min} \ge 0.50\,\text{m}$.
   - **Obstacle Tangential Circulation**: Prevents local minima traps by driving robots along the tangent of structural obstacles ($F_{\text{tang}} = 1.0 \, F_{\text{norm}}$).
   - **Unicycle Continuous Velocity Profiling**: Smoothly modulates forward speed via $\max(0, \cos(e_\theta))$, enabling zero-radius in-place rotation and eliminating wheel-slip chatter.

---

## 2. Quantitative Performance & Comparative Benchmarks

The system was benchmarked across three standardized experimental conditions inside Gazebo Harmonic:

| Metric | Part 1: Baseline | Part 2: Real-World Faults | Part 3: Resilient + APF Barrier | Safety Specification |
| :--- | :---: | :---: | :---: | :---: |
| **Minimum Inter-Robot Distance ($d_{\min}$)** | **$0.3969\,\text{m}$ (CRASH)** | $0.5683\,\text{m}$ (Vulnerable) | **$0.6757\,\text{m}$ (Safe)** | $\ge 0.5000\,\text{m}$ |
| **Physical Collision Events ($d < 0.50\,\text{m}$)**| **$1$ collision** | $0$ collisions | **$0$ crashes (100% collision-free)** | $0$ crashes |
| **Final Shape Error ($\text{RMS}$)** | **$0.9580\,\text{m}$ (Interlocked)**| $0.0684\,\text{m}$ | **$0.0368\,\text{m}$ ($3.6\,\text{cm}$)** | $\le 0.1500\,\text{m}$ |
| **Convergence / Settle Time** | Failed (No convergence) | $15.14\,\text{s}$ | **$19.00\,\text{s}$** | $< 45.0\,\text{s}$ |
| **Safety Barrier Status** | ❌ **CRASH VIOLATION** | ⚠️ **UNCERTIFIED** |  **CERTIFIED SAFE** | $d_{\min} \ge 0.50\,\text{m}$ |
| **Obstacle Transit Guarantee** | Blocked / Collision | Unstable transit | **Smooth Tangential Navigation**| Zero collisions |

---

## 3. Repository Architecture

```
~/swarm_ws/
├── README.md                            # Main project overview and instructions
├── step_by_step_installation_guide.md   # Zero-to-Hero installation manual for Ubuntu 24.04
├── ieee_research_report.md              # Academic IEEE research paper authored by Mansoor Ahmed Rind (NUST)
├── run_gazebo_experiments.sh            # Automated bash runner for all 3 comparative experiments
├── generate_gazebo_comparison.py        # Generates comparison plots & individual run diagnostics
├── gazebo_swarm_comparison.png          # 4-panel master benchmark comparison figure
├── gazebo_part3_resilient.png           # 4-panel diagnostic plot for Resilient APF
├── gazebo_part2_faults.png              # 4-panel diagnostic plot for Fault injection
├── gazebo_part1_baseline.png            # 4-panel diagnostic plot for Baseline crash
├── gazebo_part3_resilient.csv           # Raw time-series telemetry for Part 3
├── swarm_metrics.csv                    # Continuous quantitative metrics log
└── src/
    └── swarm_formation/
        ├── package.xml                  # ROS 2 Jazzy package manifest & dependencies
        ├── setup.py                     # Python package configuration and console entry points
        ├── setup.cfg                    # Setuptools install configuration
        ├── launch/
        │   └── gazebo_swarm_launch.py   # Unified master launch file
        ├── urdf/
        │   └── robot.urdf               # Differential-drive physical model & Gazebo plugins
        ├── worlds/
        │   └── collapsed_building.sdf   # Disaster arena with concrete slabs, columns, and rubble
        ├── swarm_formation/
        │   ├── __init__.py
        │   ├── gazebo_node.py           # Decentralized consensus, fault engine, and APF barrier
        │   └── metrics_logger.py        # Real-time quantitative CSV metrics logger
        └── test/
            ├── test_copyright.py
            ├── test_flake8.py
            └── test_pep257.py
```

---

## 4. Quick Start: How to Run

### Step 1: Source the Environment & Build
```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Verify that all package linters and tests pass with 0 warnings:
```bash
colcon test --packages-select swarm_formation && colcon test-result --verbose
```

---

### Step 2: Choose Your Execution Mode

#### Mode A: Interactive 3D Gazebo Simulation (Live GUI Visualization)
Watch the physical 5-robot swarm navigate through debris and avoid obstacles in real time:

```bash
ros2 launch swarm_formation gazebo_swarm_launch.py \
  world:=src/swarm_formation/worlds/collapsed_building.sdf \
  headless:=false \
  use_resilient_method:=true \
  enable_faults:=true
```

> **Controls**:
> - **Right-Click + Drag**: Zoom in / out.
> - **Left-Click + Drag**: Rotate camera.
> - **Middle-Click (Scroll Wheel)**: Pan view.

---

#### Mode B: Automated Headless Benchmark Suite
Executes all three test configurations sequentially, records telemetry to CSV, and regenerates all publication-quality comparison plots:

```bash
cd ~/swarm_ws
./run_gazebo_experiments.sh
```

---

#### Mode C: Launch Part 1 Baseline (To Observe the Physics Collision)
Demonstrates the failure of un-barrier-certified linear consensus:

```bash
ros2 launch swarm_formation gazebo_swarm_launch.py \
  world:=src/swarm_formation/worlds/collapsed_building.sdf \
  headless:=false \
  use_resilient_method:=false \
  enable_faults:=false
```

---

## 5. What is Expected vs. What is Not Expected

### What is EXPECTED:
- **Zero Collisions in Part 3**: The minimum inter-robot distance strictly stays above $0.50\,\text{m}$ (measured: $0.6757\,\text{m}$).
- **Anti-Symmetric Lateral Avoidance**: On head-on encounters, robots steer right and pass without mutual deadlocks.
- **Robot 4 Dropout Handling**: Between $t = 5\,\text{s}$ and $t = 10\,\text{s}$, Robot 4 stops responding. The remaining swarm maintains stability; upon recovery at $t = 10\,\text{s}$, Robot 4 rejoins smoothly.
- **Accurate Convergence**: Formation settles into a regular pentagon within $19.0\,\text{s}$ with sub-$4\,\text{cm}$ RMS accuracy.

### What is NOT Expected:
- **No Omnidirectional Sideways Sliding**: Robots are nonholonomic unicycles; they pivot on their center axes to align headings before driving forward.
- **No Fixed $(0, 0)$ Center**: The swarm has **no GPS or central anchor**. It converges relative to its own barycenter (initial centroid $\approx (+0.44, -0.44)\,\text{m}$).
- **Part 1 Baseline Crashes**: Part 1 is intentionally uncertified to prove why the APF safety barrier is mandatory.
- **Normal Process Teardown**: `SIGTERM` messages at simulation shutdown are normal socket teardowns from the ROS 2 launch manager.

---

## 7. License

This project is licensed under the Apache License 2.0. See [package.xml](src/swarm_formation/package.xml) for package details.
