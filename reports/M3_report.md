# Milestone M3 Report: Configurable Faults and Noise Injection

- **Milestone**: M3 — Configurable Faults and Noise
- **Engineer Role**: Senior Robotics Systems & Fault Tolerance Engineer (M3 Lead)
- **Date**: 2026-11-06
- **Git Commit Hash**: `d6741ba` (Implementation commit)
- **Git Branch**: `milestone/M3`

---

## 1. Scope: Required vs. Delivered

| Capability Required | Delivered Status | Verification Evidence |
| :--- | :--- | :--- |
| Standalone `fault_injector` ROS 2 node | **DELIVERED** | `src/swarm_formation/swarm_formation/fault_injector_node.py` |
| Complete decoupling of robot nodes from hardcoded faults | **DELIVERED** | Removed `_check_agent_dropout()` and internal dropout timers from `robot_node.py`; robot actuates zero velocity only when commanded via `/robot_{i}/motor_kill` |
| Configurable fault types: robot death, sensor bias, intermittent dropout | **DELIVERED** | Modeled in `fault_model.py` and supported via YAML profiles |
| Profile-driven scenarios via external YAML configurations | **DELIVERED** | `robot_death.yaml`, `sensor_bias.yaml`, `intermittent_failure.yaml`, `composite_faults.yaml` in `config/fault_profiles/` |
| Runtime interactive service to trigger and release faults | **DELIVERED** | `/fault_injector/trigger_fault` (`swarm_interfaces/srv/TriggerFault.srv`) |
| Headless Gazebo simulation validation with real metrics logging | **DELIVERED** | Successfully reproduced Part 2 baseline dropout and composite fault benchmarks logged to `results/M3/` |

---

## 2. Changes Made

### Files Added
1. **`src/swarm_interfaces/msg/FaultInjection.msg`**: Interface for propagating sensor bias and intermittent status to robot sensor pipelines.
2. **`src/swarm_interfaces/srv/TriggerFault.srv`**: Interactive ROS 2 service for triggering/releasing faults at runtime.
3. **`src/swarm_formation/swarm_formation/fault_model.py`**: Pure math and data-model library containing `FaultConfig`, continuous and duty-cycle active checks (`is_fault_active`), sensor bias application (`apply_sensor_bias`), and YAML parser (`parse_fault_yaml`).
4. **`src/swarm_formation/swarm_formation/fault_injector_node.py`**: Standalone node managing scheduled scenarios and interactive service requests, publishing actuation kill signals to `/robot_{i}/motor_kill` and sensor impairments to `/robot_{i}/sensor_fault`.
5. **`src/swarm_formation/config/fault_profiles/`**:
   - `robot_death.yaml`: Incapacitates Robot 4 between $t=5.0\text{s}$ and $t=10.0\text{s}$.
   - `sensor_bias.yaml`: Injects range and bearing offsets for Robot 2.
   - `intermittent_failure.yaml`: 50% duty-cycle periodic blackout for Robot 3.
   - `composite_faults.yaml`: Multi-agent stress benchmark combining death, bias, and intermittent dropouts.
6. **`src/swarm_formation/test/test_fault_model.py`**: 5 unit tests verifying fault activations, bias math, angle wrapping, and YAML ingestion.
7. **`src/swarm_formation/test/test_m3_acceptance.py`**: 3 acceptance tests checking AST decoupling of `robot_node.py`, runtime trigger service, and `neighbor_sensor_node` fault response.
8. **`results/M3/gazebo_part2_fault_injector.csv`**: Headless Gazebo run verifying external fault injection of Robot 4 death.
9. **`results/M3/gazebo_composite_faults.csv`**: Headless Gazebo run verifying swarm behavior under composite multi-fault profile.

### Files Modified
1. **`src/swarm_formation/swarm_formation/robot_node.py`**: Completely eradicated internal `_check_agent_dropout()`, `fault_dropout_robot_id`, and hardcoded timing parameters. Added subscription to `/robot_{self.robot_id}/motor_kill`.
2. **`src/swarm_formation/swarm_formation/neighbor_sensor_node.py`**: Added subscription to `/robot_{self.robot_id}/sensor_fault`. When active, applies range/bearing bias or drops packets during intermittent blackouts.
3. **`src/swarm_formation/setup.py`**: Registered console script `'fault_injector = swarm_formation.fault_injector_node:main'` and packaged fault profiles.
4. **`src/swarm_formation/launch/gazebo_swarm_launch.py`**: Added `fault_profile` launch argument and automatic launching of `fault_injector`.
5. **`src/swarm_interfaces/CMakeLists.txt`**: Registered `FaultInjection.msg` and `TriggerFault.srv`.

---

## 3. Commands Run and Execution Logs

| Command | Timeout | Exit Code | Duration | Outcome |
| :--- | :--- | :--- | :--- | :--- |
| `colcon build --symlink-install` | 300s | 0 | 4.89s | Built both packages cleanly |
| `pytest-3 src/swarm_formation/test -v` | 180s | 0 | 0.71s | 50 tests passed (100%) |
| `colcon test && colcon test-result --verbose` | 180s | 0 | 3.99s | 50 tests passed, 0 failures, 0 errors, 0 skipped |
| Headless Gazebo Part 2 (Robot 4 death via fault injector) | 120s | 0 | 35.8s | Reached 30.0s, converged at 25.90s, $d_{\min}=0.7198\text{m}$ |
| Headless Gazebo Composite Faults Profile | 140s | 0 | 41.2s | Reached 35.0s, converged at 26.85s, transient $d_{\min}=0.3975\text{m}$ |

---

## 4. Test Evidence and Acceptance Criteria

### Acceptance Criteria Table

| Criterion | Result | Evidence |
| :--- | :--- | :--- |
| **AC-1**: Robot nodes do not contain hardcoded dropout or noise | **PASS** | `test_robot_node_decoupled_from_faults` verifies AST contains no `_check_agent_dropout` and no `fault_dropout_*` parameters |
| **AC-2**: Standalone `fault_injector` node handles death, bias, intermittent | **PASS** | `test_fault_injector_runtime_trigger` and unit tests in `test_fault_model.py` pass |
| **AC-3**: External YAML config files drive fault scenarios | **PASS** | `robot_death.yaml`, `sensor_bias.yaml`, `intermittent_failure.yaml`, `composite_faults.yaml` |
| **AC-4**: Interactive runtime service enables/disables faults dynamically | **PASS** | Service `/fault_injector/trigger_fault` validated in automated test |
| **AC-5**: Regression suite for M0, M1, M2 continues to pass | **PASS** | 50 tests passing across all suites with 0 skips |

### Empirical Simulation Comparison

| Experiment Condition | Sim Duration | Settle Time | RMS Shape Error | Min Peer Distance | Collisions Detected |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Part 2 with Fault Injector** (`robot_death.yaml`, non-resilient) | 30.0s | 25.90s | 0.3320 m | 0.7198 m | 0 |
| **Composite Faults** (death + bias + intermittent, resilient) | 35.0s | 26.85s | 0.1292 m | **0.3975 m** | **41** |

---

## 5. Issues Found and Root-Cause Remediation

1. **Flake8 Docstring & Import Violations**:
   - *Issue*: `test_flake8` reported line-length and import ordering issues in `fault_model.py` and `setup.py`.
   - *Root Cause*: PEP8 import groupings separated stdlib and third-party modules; one multiline docstring needed single-line representation.
   - *Remediation*: Reordered imports and standardized docstrings; verified with `colcon test` (0 warnings).
2. **Launch Profile Path Resolution**:
   - *Issue*: Passing relative paths to `fault_profile:=composite_faults.yaml` failed to resolve when current working directory differed.
   - *Root Cause*: `os.path.exists()` tested against relative working directory instead of installed share directory.
   - *Remediation*: Added fallback lookup using `os.path.basename()` within `install/swarm_formation/share/swarm_formation/config/fault_profiles/`.

---

## 6. Known Limitations and Honest Findings

1. **Vulnerability to Unmodeled Sensor Bias**: Under the composite fault scenario (Robot 2 experiencing +0.25m distance bias and +0.08 rad bearing bias), the heuristic APF permitted minimum distance to drop to $0.3975\text{m}$, triggering 41 collision breach events. The APF computes repulsion based on perceived coordinates; when perceived distances are biased, the repulsive force is incorrectly scaled. This directly motivates Milestone M5 (State Estimation / Localization) to filter and estimate states, and Milestone M6 (Control Barrier Functions) for provable safety margins.
2. **Motor Kill Emulation**: In simulation, robot death is enforced by publishing zero velocity (`Twist()`) to `/robot_{i}/cmd_vel`. On physical hardware, a true hardware watchdog or power disconnect would be used.

---

## 7. Handoff Notes for Milestone M4 (Three Controller Families)

- The robot control node `robot_node.py` is now decoupled from sensing, network impairments, and faults.
- In M4, the objective is to implement three controllers behind a common interface, selectable by launch argument `controller_type`:
  1. `displacement`: relative displacement consensus (existing baseline).
  2. `distance`: inter-agent distance / range-only consensus (rigidity matrix / edge lengths).
  3. `bearing`: inter-agent bearing-only consensus (parallel projection / orientation consensus).
- A base controller class or interface (`SwarmControllerBase`) should define `compute_control(self, ...)` with modular pluggability.
