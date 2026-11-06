# Milestone M10: Hardware-in-the-Loop (HIL) & Physical Deployment Preparation Plan

- **Milestone**: M10 (Physical Robots & HIL Preparation Plan - OPTIONAL)
- **Engineer Role**: Senior Robotics Engineer (M10 Specialist: Hardware-in-the-Loop, Embedded Systems & Physical Deployment)
- **Date**: 2026-11-06
- **Branch**: `milestone/M10`
- **Head Commit Before Handoff**: `4e909e6` (audit commit)
- **Audit Verdict of M9**: PASS (see `reports/M9_audit_by_M10.md`, commit `4e909e6`)

---

> [!CAUTION]
> **MANDATORY ETHICAL DISCLAIMER: NO PHYSICAL EXPERIMENTS WERE CONDUCTED IN THIS ENVIRONMENT.**
> No physical robot hardware was present, interfaced, or claimed in this virtual workspace. All performance figures, collision logs, and statistical metrics presented in prior reports derive exclusively from physics-verified Gazebo Harmonic (Gz Sim 8.x) simulations and 50 Hz deterministic unicycle simulations. This document provides a complete, production-ready Hardware-in-the-Loop (HIL) engineering plan and deployment roadmap for physical hardware transition.

---

## 1. Hardware Architecture & Bill of Materials

The proposed deployment targets a swarm of $N = 5$ physical differential-drive mobile robots using commercial-off-the-shelf (COTS) educational robotics platforms.

```
       ┌────────────────────────────────────────────────────────┐
       │             TurtleBot3 Burger / Waffle Pi              │
       │                                                        │
       │  ┌─────────────────────────┐   ┌────────────────────┐  │
       │  │    Raspberry Pi 4 / 5   │   │  LDS-01/02 LiDAR   │  │
       │  │ (Ubuntu 24.04 + ROS 2)  │◄──┤ (360 deg, 10 Hz)   │  │
       │  └────────────┬────────────┘   └────────────────────┘  │
       │               │ UART / USB                             │
       │  ┌────────────▼────────────┐   ┌────────────────────┐  │
       │  │   OpenCR 1.0 Controller │───► 2x Dynamixel       │  │
       │  │  (Cortex-M7 Motor Core) │   │   XL430 Servos     │  │
       │  └─────────────────────────┘   └────────────────────┘  │
       └────────────────────────────────────────────────────────┘
```

### 1.1 Physical Components (Per Robot, 5x Total)
1. **Chassis & Drive**: TurtleBot3 Burger (or Waffle Pi) differential-drive chassis with two DYNAMIXEL XL430-W250-T actuators and one rear/front passive ball caster. Wheel radius $r = 0.033\,\text{m}$, axle track baseline $L = 0.160\,\text{m}$, total mass $m \approx 1.0\,\text{kg}$.
2. **Embedded Single Board Computer (SBC)**: Raspberry Pi 4 Model B (4 GB or 8 GB RAM) or Raspberry Pi 5 running 64-bit Ubuntu 24.04 LTS Server with ROS 2 Jazzy Jalisco.
3. **Low-Level Motor Controller**: ROBOTIS OpenCR 1.0 board (ARM Cortex-M7 with 32-bit floating point unit) executing real-time closed-loop PID velocity control (200 Hz) via internal magnetic encoders (4096 pulses/rev).
4. **Primary Relative Sensors**:
   - **Planar 360° LiDAR**: LDS-01 / LDS-02 (or RPLiDAR A1M8) sampling at 10 Hz with 360 angular bins and detection range $0.12\,\text{m} \le r \le 3.50\,\text{m}$.
   - **Relative Bearing Identification**: Low-cost wide-angle USB camera or Raspberry Pi Camera Module v2 scanning for unique ArUco / AprilTag markers (IDs 1 through 5) affixed to each robot's top plate.
5. **Power Supply**: 3-cell 11.1V 1800mAh LiPo battery supplying OpenCR and stepped-down 5V/3A DC-DC regulator for the Raspberry Pi.

### 1.2 External Ground-Truth Benchmark (Evaluation Only)
- **Infrastructure**: An overhead optical motion-capture system (e.g. 6x OptiTrack Prime 13 or VICON cameras) covering a $6.0\,\text{m} \times 6.0\,\text{m}$ flight/ground arena.
- **Data Flow**: MoCap server streams rigid-body pose at 100 Hz over Ethernet to the base station, which rebroadcasts `/robot_i/gt_pose` onto the ROS 2 network.
- **Strict Isolation**: As in Gazebo simulation, physical robot controllers are strictly forbidden from subscribing to `/robot_i/gt_pose`. Ground truth is recorded via `ros2 bag` exclusively for post-experiment offline error computation (`process_bag`).

---

## 2. Wireless Network & Middleware Architecture

In multi-robot WiFi environments, unmanaged UDP multicast flooding frequently degrades network performance. To maintain reliable inter-agent communication, the following network configuration is specified:

```
                  ┌─────────────────────────────────────────┐
                  │    Dedicated 5 GHz WiFi 6 AP (Router)   │
                  │        SSID: SwarmMesh_5G_Isolated      │
                  └────────────────────┬────────────────────┘
                                       │
            ┌──────────────────────────┼──────────────────────────┐
            ▼                          ▼                          ▼
   ┌─────────────────┐        ┌─────────────────┐        ┌─────────────────┐
   │ Robot 1 (SBC)   │        │ Robot 2 (SBC)   │        │ Base Station PC │
   │ 192.168.10.101  │        │ 192.168.10.102  │        │ 192.168.10.10   │
   │ Fast-DDS Client │        │ Fast-DDS Client │        │ Discovery Server│
   └─────────────────┘        └─────────────────┘        └─────────────────┘
```

1. **Hardware**: Dedicated dual-band WiFi 6 (802.11ax) router operating exclusively on an uncrowded 5 GHz DFS channel with 80 MHz channel width.
2. **Static Addressing**: Deterministic IP allocation (`192.168.10.101` through `192.168.10.105` for robots 1–5, `192.168.10.10` for ground station).
3. **DDS Discovery Server**: Avoid standard peer-to-peer multicast discovery. A centralized Fast-DDS discovery server runs on the base station:
   ```bash
   fastdds discovery --server-id 0 -p 11811
   ```
   Each robot sets:
   ```bash
   export ROS_DISCOVERY_SERVER=192.168.10.10:11811
   ```
4. **Time Synchronization**: Base station runs Chrony NTP server. Each robot SBC synchronizes every 10 seconds, bounding clock skew within $\pm 1.5\,\text{ms}$.

---

## 3. Onboard Node Deployment & Topic Mapping

The identical modular ROS 2 nodes developed and verified in simulation run onboard the physical robot SBC:

```
  ┌────────────────────────────────────────────────────────────────────────┐
  │                            Robot SBC (Ubuntu 24.04)                    │
  │                                                                        │
  │  ┌────────────────────┐   /robot_i/odom    ┌────────────────────────┐  │
  │  │ turtlebot3_node    ├───────────────────►│ localization_node      │  │
  │  │ (Hardware Driver)  │                    │ (Cooperative EKF)      │  │
  │  └─────────▲──────────┘                    └───────────┬────────────┘  │
  │            │ /robot_i/cmd_vel                          │ /estimated_pose│
  │  ┌─────────┴──────────┐                    ┌───────────▼────────────┐  │
  │  │ safety_filter_node ◄─── /cmd_vel_raw ───┤ robot_node (Controller)│  │
  │  │ (Active-Set CBF-QP)│                    │ (Displacement/Dist/Brg)│  │
  │  └─────────▲──────────┘                    └────────────────────────┘  │
  │            │ /robot_i/relative_neighbors                               │
  │  ┌─────────┴──────────┐                                                │
  │  │ neighbor_sensor    │◄─── LiDAR scan & AprilTag camera               │
  │  └────────────────────┘                                                │
  └────────────────────────────────────────────────────────────────────────┘
```

1. **`turtlebot3_node`**: OpenCR hardware interface publishing `/robot_i/odom` (50 Hz) and executing `/robot_i/cmd_vel`.
2. **`neighbor_sensor_node`**: Subscribes to local `/robot_i/scan` and camera detections, extracts relative polar clusters, and publishes `/robot_i/relative_neighbors`.
3. **`localization_node`**: Executes Cooperative EKF, publishing `/robot_i/estimated_pose`.
4. **`robot_node`**: Runs selected formation control law (`controller_type:=displacement|distance|bearing`), publishing `/robot_i/cmd_vel_raw`.
5. **`safety_filter_node`**: Solves CBF-QP with latency-expanded barrier $D_s^{\text{eff}} = D_s + 2 v_{\max} \tau_{\max}$, outputting safe `/robot_i/cmd_vel`.

---

## 4. Sim-to-Real Modeling Gaps & Mitigations

| Physical Reality Gap | Effect on Simulation Model | Engineering Mitigation & Solution |
| :--- | :--- | :--- |
| **Actuator Deadband & Stiction** | Low velocity commands ($\|u\| < 0.03\,\text{m/s}$) fail to overcome gearbox stiction. | Implement smooth feedforward deadband compensation in `kinematics.py`: apply small dither or snap to zero below threshold. |
| **Battery Voltage Degradation** | Motor terminal voltage drops from $12.6\,\text{V}$ to $10.5\,\text{V}$, altering velocity gain. | Use closed-loop encoder PID on the OpenCR board to regulate wheel surface speed independently of battery level. |
| **Wheel Slip & Odometry Drift** | Floor dust and sharp turns cause unmeasured rotational slip. | EKF incorporates relative neighbor range/bearing updates to continuously bound unbounded dead-reckoning drift. |
| **LiDAR Multipath & Glass Reflections** | Highly reflective concrete or dark surfaces cause false/dropped returns. | Statistical range filtering, outlier rejection, and cluster centroid thresholding in `neighbor_sensor_node`. |
| **Real WiFi Latency & Jitter** | Discrete RF buffer queuing introduces variable latency $\tau \in [10, 50\,\text{ms}]$. | The CBF-QP safety filter dynamically scales $D_s^{\text{eff}} = D_s + 2 v_{\max} \tau_{\max}$, maintaining buffer margins. |

---

## 5. Step-by-Step Hardware Validation Protocol

To safely transition from simulation to physical hardware without risk of physical equipment damage, the following phased staging protocol is prescribed:

### Phase 1: Static Benchtop Verification (Day 1)
- Mount single TurtleBot3 on test stand (wheels elevated).
- Verify bidirectional ROS 2 communication, motor directionality, encoder ticks, and emergency stop (`killall -9`).
- Record 60 seconds of static LiDAR data; verify zero-mean sensor noise matches simulation parameters ($\sigma_r \approx 0.03\,\text{m}$).

### Phase 2: Single-Robot Kinematic Calibration (Day 2)
- Drive robot along a $2.0\,\text{m}$ straight line tape on floor; calibrate wheel diameter parameter.
- Rotate robot $3600^\circ$ (10 full turns); calibrate wheel separation baseline $L$ to eliminate heading drift.

### Phase 3: Wireless Discovery Server & Network Stress (Day 3)
- Power on all 5 physical robots.
- Verify Fast-DDS discovery server connections.
- Run continuous 50 Hz ping tests for 15 minutes; confirm packet drop rate is $< 1.0\%$ under baseline conditions.

### Phase 4: Two-Robot Pairwise CBF-QP Safety Validation (Day 4)
- Place Robot 1 at $(-1.5, 0)$ and Robot 2 at $(1.5, 0)$ facing head-on.
- Command them to cross paths using raw consensus velocity ($u_{\text{raw}}$).
- **Pass Criterion**: CBF-QP filter must intervene before physical contact occurs, actively braking and deflecting robots to maintain empirical clearance $d_{\min} \ge 0.50\,\text{m}$.

### Phase 5: Five-Robot Open-Field Pentagon Formation (Day 5)
- Place all 5 robots in arbitrary dispersed positions in open $5\,\text{m} \times 5\,\text{m}$ arena.
- Launch decentralized formation stack (`controller_type:=displacement`).
- Record experiment via `ros2 bag record`.
- Compute offline Procrustes RMS shape error; verify convergence within $30\,\text{s}$.

### Phase 6: Five-Robot Cluttered Disaster Arena Deployment (Day 6)
- Add physical obstacles (cardboard boxes, concrete blocks, foam slabs) mimicking `collapsed_building.sdf`.
- Evaluate full resilient stack (Huber + EMA + CBF-QP).
- Confirm zero physical collisions and successful obstacle circumnavigation.

---

## 6. Summary & Readiness Assessment

The modular, pure-Python library architecture established in milestones M0 through M9 directly decouples algorithmic logic from simulation middleware. Because ROS 2 node interfaces, message types, and coordinate conventions are 100% standard (`geometry_msgs/Twist`, `nav_msgs/Odometry`, `sensor_msgs/LaserScan`), the identical binaries tested in Gazebo Harmonic will execute natively on physical TurtleBot3 SBCs with zero algorithmic modifications.
