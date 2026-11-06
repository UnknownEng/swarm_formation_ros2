# Milestone M2 Report: Standalone Network Impairment Node

- **Milestone**: M2 (Network Impairment Node)
- **Engineer Role**: Senior Robotics Engineer (Network & Communications Specialist)
- **Date**: 2026-11-06
- **Git Commit Hash**: `a5bae71a0481f0e0950694221f9e16cbea07f91c`

---

## 1. Scope: Required vs Delivered

### Required
1. Implement standalone `network_impairment` node for simulating packet drop, delay, and jitter.
2. Provide runtime link on/off toggling via ROS 2 service and/or topic without modifying or touching robot controller code.
3. Decouple network communications completely from `robot_node.py`.
4. Add custom interfaces for link state inspection and runtime link modification.
5. Acceptance criterion: DONE WHEN impairments change without touching robot code.

### Delivered
- Created `swarm_interfaces/srv/SetLinkState.srv`:
  - Request: `from_robot_id`, `to_robot_id`, `enabled`, `drop_rate`, `delay_sec`, `jitter_sec`.
  - Response: `success`, `message`.
- Created `swarm_interfaces/msg/LinkState.msg` and `NetworkStatus.msg`:
  - Real-time link telemetry and packet accounting (`total_packets_forwarded`, `total_packets_dropped`).
- Created pure channel link mathematics library: `src/swarm_formation/swarm_formation/network_model.py`:
  - `ChannelLink`: Data model for directed peer communication links.
  - `should_drop_packet`: Evaluates deterministic link disablement and stochastic loss rate.
  - `compute_packet_latency`: Calculates total transmission delay with uniform bounded jitter.
- Created `NetworkImpairmentNode` (`network_impairment_node.py`):
  - Bridges raw sensor reports from `/robot_{i}/neighbors_raw` to impaired `/robot_{i}/neighbors`.
  - Hosts `/network/set_link` service and `/network/link_command` topic for dynamic runtime link control.
  - Maintains queue of delayed packets, automatically dispatching them when delivery timestamps mature.
- Updated `gazebo_swarm_launch.py` to instantiate `network_impairment` alongside swarm nodes.
- Preserved `robot_node.py` completely untouched—controllers subscribe to `/robot_{i}/neighbors` and require zero knowledge of underlying network mechanics.
- Added comprehensive unit and acceptance tests (`test_network_model.py`, `test_m2_acceptance.py`). Full suite: **42 tests passing, 0 errors, 0 failures, 0 skipped**.
- Ran empirical simulation in Gazebo Harmonic:
  - Part 3 under realistic network impairment (15% loss, 80ms delay, 40ms jitter, agent 4 dropout):
    $d_{\min} = 0.4539\text{ m}$, 12 transient collision events, convergence achieved at $t = 26.13\text{ s}$ with RMS shape error $0.2020\text{ m}$.

---

## 2. Changes Made

| File | Status | Reason |
| :--- | :--- | :--- |
| `src/swarm_interfaces/srv/SetLinkState.srv` | Added | Service interface for runtime link control (on/off, drop, delay, jitter). |
| `src/swarm_interfaces/msg/LinkState.msg` | Added | Message definition for individual link telemetry and topic commands. |
| `src/swarm_interfaces/msg/NetworkStatus.msg` | Added | Telemetry message aggregating active link states and packet counters. |
| `src/swarm_interfaces/CMakeLists.txt` | Modified | Registered new service and messages in rosidl generation. |
| `src/swarm_formation/swarm_formation/network_model.py` | Added | Pure dataclass and math functions for link loss and jitter latency. |
| `src/swarm_formation/swarm_formation/network_impairment_node.py` | Added | Standalone ROS 2 node bridging raw neighbor observations to impaired topics. |
| `src/swarm_formation/swarm_formation/neighbor_sensor_node.py` | Modified | Parameterized output topic, routing default raw output to `/robot_{i}/neighbors_raw`. |
| `src/swarm_formation/config/swarm_params.yaml` | Modified | Added default network impairment parameters (`default_drop_rate`, etc.). |
| `src/swarm_formation/launch/gazebo_swarm_launch.py` | Modified | Launched `network_impairment` node with swarm parameters. |
| `src/swarm_formation/setup.py` | Modified | Registered `network_impairment` entry point. |
| `src/swarm_formation/test/test_network_model.py` | Added | Unit tests covering link defaults, drop boundaries, stochastic loss, and jitter. |
| `src/swarm_formation/test/test_m2_acceptance.py` | Added | Acceptance tests for service link toggling, topic commands, and controller code isolation. |
| `results/M2/gazebo_part3_network.csv` | Added | Simulation logs under active network impairments. |

---

## 3. Commands Run

| Command | Timeout | Exit Code | Duration | Purpose |
| :--- | :--- | :--- | :--- | :--- |
| `colcon build --packages-select swarm_interfaces` | 60s | 0 | 20.2s | Compile new ROS 2 service and message types |
| `colcon build --symlink-install` | 60s | 0 | 5.16s | Build workspace with updated nodes |
| `colcon test --event-handlers console_direct+` | 60s | 0 | 4.17s | Run 42 unit and lint tests |
| `colcon test-result --all --verbose` | 30s | 0 | 0.5s | Validate test report |
| `ros2 launch swarm_formation ... Part 3 Network (45s)` | 80s | 0 | ~55s | Headless simulation under active network impairments |

---

## 4. Test Evidence

### Colcon Test Output
```text
build/swarm_formation/pytest.xml: 42 tests, 0 errors, 0 failures, 0 skipped

Summary: 42 tests, 0 errors, 0 failures, 0 skipped
======================== 42 passed, 4 warnings in 2.99s ========================
```

Test breakdown:
- `test_consensus.py`: 4 passed
- `test_copyright.py`: 1 passed
- `test_flake8.py`: 1 passed
- `test_kinematics.py`: 5 passed
- `test_m1_acceptance.py`: 3 passed
- `test_m2_acceptance.py`: 3 passed (runtime service toggle, topic command, controller code untouched)
- `test_network_model.py`: 6 passed
- `test_pep257.py`: 1 passed
- `test_potential_field.py`: 4 passed
- `test_relative_consensus.py`: 4 passed
- `test_relative_potential_field.py`: 4 passed
- `test_relative_sensing.py`: 6 passed

### Simulation Empirical Results (Active Network Impairment)

| Metric | Ideal Relative Sensing (M1) | Impaired Network Channel (M2) |
| :--- | :--- | :--- |
| Packet Loss Rate | 0.0% | 15.0% |
| Transmission Latency | 0.0 ms | 80 ms $\pm$ 40 ms jitter |
| Agent Dropout | None | Robot 4 dropped $t \in [5, 10]\text{ s}$ |
| Minimum Inter-Robot Dist ($d_{\min}$) | 0.6945 m | 0.4539 m |
| Safety Threshold ($<0.50$ m) | SAFE (0 events) | **TRANSIENT BREACH** (12 events) |
| Convergence Settle Time | 10.55 s | 26.13 s |
| Final Shape RMS Error | 0.0282 m | 0.2020 m |
| Link Control Mechanism | None | Runtime ROS 2 Service & Topic |

---

## 5. Acceptance Criteria Table

| Criterion | Result | Evidence |
| :--- | :--- | :--- |
| Standalone network impairment node | **PASS** | `NetworkImpairmentNode` operates as an independent ROS 2 node. |
| Configurable drop, delay, jitter | **PASS** | `network_model.py` and `swarm_params.yaml` support loss, delay, and jitter bounds. |
| Runtime link on/off toggle via service | **PASS** | `test_network_impairment_runtime_link_toggle` passed. Service `/network/set_link` dynamically disconnects and reconnects links. |
| Runtime link update via topic | **PASS** | `test_network_impairment_link_command_topic` passed. |
| Impairments change without touching robot code | **PASS** | AST inspection in `test_robot_code_untouched_by_impairments` confirmed `robot_node.py` contains zero impairment code and subscribes strictly to `/robot_{i}/neighbors`. |

---

## 6. Regression Results for All Earlier Milestones
- **M0 Regression**: Modularization, kinematics, consensus, and APF remain fully intact.
- **M1 Regression**: Relative neighbor sensing and out-of-range invisibility remain verified. All 33 previous tests continue passing cleanly.

---

## 7. Issues Found & Root Cause Fixes

1. **Issue**: `neighbor_sensor_node` crashed on startup with `AttributeError: 'NeighborSensorNode' object has no attribute 'robot_id'`.
   - *Root cause*: `self.declare_parameter('output_topic', f'/robot_{self.robot_id}/neighbors_raw')` evaluated `self.robot_id` before it was assigned.
   - *Fix*: Initialized `output_topic` parameter with default empty string `''`, then formatted with `self.robot_id` after reading parameter.
2. **Issue**: Flake8 E501 line length errors in `network_impairment_node.py`.
   - *Root cause*: Long list comprehensions for wildcard link updates exceeded 99 characters.
   - *Fix*: Formatted list comprehensions across multiple lines.

---

## 8. Known Limitations & Honest Caveats
- **Safety Degradation Under Latency**: With 80 ms latency and 40 ms jitter, the heuristic APF is delayed in generating repulsive forces, leading to a minimum inter-robot distance of $0.4539\text{ m}$ ($< 0.50\text{ m}$ contact threshold). Heuristic APF does not guarantee forward invariance under network delays. This empirical finding reinforces the necessity of the Control Barrier Function (CBF-QP) safety layer designed for Milestone M6.
- Fault profiles (robot death, sensor bias, intermittent hardware failure) are currently simulated in ad-hoc fashion; Milestone M3 will extract them into a dedicated configurable fault injector driven by YAML profiles.

---

## 9. Handoff Notes for M3 Engineer
- Milestone M2 is 100% complete and verified on branch `milestone/M2` (commit `a5bae71`).
- All 42 tests pass with zero skips.
- The network impairment node is fully operational and provides runtime link manipulation via `/network/set_link` and `/network/link_command`.
- Ready for M3 audit and implementation of the configurable fault injector node (`fault_injector`).
