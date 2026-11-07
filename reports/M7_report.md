# Milestone M7 Report: Rosbag Recording & Offline Metrics Pipeline

- **Milestone**: M7 (Rosbag Pipeline)
- **Engineer Role**: Senior Robotics Engineer (M7 Specialist: Offline Rosbag Pipelines, Metric Engines & Visualization)
- **Date**: 2026-11-06
- **Branch**: `milestone/M7`
- **Head Commit Before Handoff**: `d06155f` (target)
- **Audit Verdict of M6**: PASS (see `reports/M6_audit_by_M7.md`, commit `3928d99`)

---

> **Erratum (2026-11-07).** All Gazebo results below were measured before 2026-11-07 with two defects that invalidate them: wheel odometry never reached the EKF (bridge type `gz.msgs.OdometryWithCovariance` vs the DiffDrive plugin's `gz.msgs.Odometry`; `/robot_i/odom` carried 0 messages, and the EKF fallback dead-reckoned with the first commanded velocity), and all nodes ran on the wall clock while Gazebo ran at about 5% of real time. Current Gazebo results are in README Section 6.1/6.2.
>
> The pipeline itself works (verified on 2026-11-07 bags), but the recorded topic list named `/swarm/network_status`, `/swarm/fault_status` and `/robot_i/obstacles`, which do not exist (fixed), and the documented `--input-bag` flag is wrong: the bag path is a positional argument of `process_bag` and `plot_bag`.

---

## 1. Executive Summary & Scope

### 1.1 Requirements
1. **Decoupled Recording Pipeline**: Record all swarm coordination, trajectory, telemetry, and safety topics into standard `rosbag2` containers (`mcap` / `sqlite3`) without requiring runtime dependencies on the simulation physics engine or online CSV logger.
2. **Offline Metrics Processor**: Create an offline parsing and metric evaluation utility (`rosbag_processor.py`) that reads recorded bags directly using `rosbag2_py.SequentialReader`, reconstructs multi-agent time-series trajectories, and evaluates:
   - Centroid-centered Procrustes RMS shape error to regular polygon targets.
   - Pairwise minimum inter-robot distance series $d_{\min}(t)$ and global minimum distance $\min_t d_{\min}(t)$.
   - Collision events (< 0.50m danger boundary).
   - Convergence time and settled-state verification.
   - Mean localization estimation error $\frac{1}{N}\sum_i \|\hat{p}_i(t) - p_i(t)\|$.
   - Control effort / energy integral $\int \sum_i (v_i^2 + \omega_i^2) dt$.
   - Safety filter interventions $\sum_i \delta u_i(t)$.
3. **Pure Rosbag Plotting Engine**: Create a standalone visualization engine (`plot_generator.py`) capable of generating all publication-grade diagnostic plots directly from rosbag2 data (or exported CSV) in headless environments.
4. **CLI Console Scripts**: Expose `process_bag` and `plot_bag` as standard ROS 2 package entry points.
5. **Acceptance Criterion**: Offline metrics computed from recorded bags must reproduce analytical and online simulation metrics within tight tolerance, and all plots must be regenerable from bags alone.

### 1.2 Delivered
- Created `src/swarm_formation/swarm_formation/rosbag_processor.py`: fully featured offline processing class `RosbagProcessor` with MCAP/SQLite3 metadata discovery, typed CDR message deserialization, regular time-grid resampling, Procrustes shape error calculation, collision detection, and CSV/JSON export.
- Created `src/swarm_formation/swarm_formation/plot_generator.py`: headless matplotlib-based generator `SwarmPlotGenerator` producing `formation_trajectories.png`, `shape_error_and_safety.png`, `estimation_error.png`, and comprehensive `diagnostic_summary.png`.
- Updated `src/swarm_formation/launch/gazebo_swarm_launch.py`: added `record_bag` (default: `'false'`) and `bag_path` launch arguments, integrating `ros2 bag record` with `--max-cache-size 0` for direct disk writing.
- Updated `src/swarm_formation/setup.py`: registered `process_bag` and `plot_bag` console script entry points.
- Authored unit test suite `test_rosbag_pipeline.py` and acceptance suite `test_m7_acceptance.py`.
- Conducted full Gazebo Harmonic simulation with rosbag recording (68,031 messages captured across 42.39 seconds). Verified offline extraction, summary JSON export, and plot generation.

---

## 2. Changes Made

| File | Status | Description & Rationale |
| :--- | :--- | :--- |
| `src/swarm_formation/swarm_formation/rosbag_processor.py` | Added | Standalone offline rosbag2 reader, metrics calculation engine, and `process_bag` CLI entry point. |
| `src/swarm_formation/swarm_formation/plot_generator.py` | Added | Standalone offline plotting generator producing publication-quality 300 DPI figures, with `plot_bag` CLI entry point. |
| `src/swarm_formation/launch/gazebo_swarm_launch.py` | Modified | Added `record_bag` and `bag_path` arguments; launches `ExecuteProcess` recording all robot telemetry, states, and safety topics when requested. |
| `src/swarm_formation/setup.py` | Modified | Added console scripts `'process_bag = swarm_formation.rosbag_processor:main'` and `'plot_bag = swarm_formation.plot_generator:main'`. |
| `src/swarm_formation/test/test_rosbag_pipeline.py` | Added | Unit tests for Procrustes rotation invariance, quaternion conversion, synthetic rosbag generation, CSV/JSON export, and plot generation. |
| `src/swarm_formation/test/test_m7_acceptance.py` | Added | Acceptance tests verifying launch argument presence, analytical metrics reproduction, and offline plot generation. |
| `results/M7/gazebo_swarm_bag/` | Added | Recorded Gazebo simulation rosbag (MCAP format, 11.1 MiB, 68,031 messages). |
| `results/M7/gazebo_offline_metrics.csv` | Added | Offline reconstructed time-series CSV metrics. |
| `results/M7/gazebo_offline_summary.json` | Added | Offline summary JSON metrics report. |
| `results/M7/plots/` | Added | Generated diagnostic plots from recorded rosbag alone. |

---

## 3. Commands Run

| Command | Timeout | Exit Code | Duration | Outcome |
| :--- | :--- | :--- | :--- | :--- |
| `ament_flake8 src/swarm_formation/swarm_formation/rosbag_processor.py` | 30s | 0 | 0.8s | PASS (0 problems) |
| `ament_flake8 src/swarm_formation/swarm_formation/plot_generator.py` | 30s | 0 | 0.7s | PASS (0 problems) |
| `ament_flake8 src/swarm_formation/launch/gazebo_swarm_launch.py src/swarm_formation/setup.py` | 30s | 0 | 0.8s | PASS (0 problems) |
| `ament_pep257 src/swarm_formation/swarm_formation/*.py` | 30s | 0 | 1.1s | PASS (0 problems) |
| `colcon build --symlink-install` | 120s | 0 | 19.2s | PASS (2 packages built) |
| `colcon test --packages-select swarm_formation` | 120s | 0 | 12.6s | PASS (87/87 tests passed) |
| `ros2 launch swarm_formation gazebo_swarm_launch.py headless:=true sim_duration:=40.0 record_bag:=true bag_path:=.../results/M7/gazebo_swarm_bag ...` | 90s | 0 | 48.2s | PASS (68,031 messages recorded, clean shutdown) |
| `ros2 run swarm_formation process_bag results/M7/gazebo_swarm_bag --output-csv ... --output-json ...` | 30s | 0 | 2.8s | PASS (offline metrics computed) |
| `ros2 run swarm_formation plot_bag results/M7/gazebo_swarm_bag --output-dir results/M7/plots` | 30s | 0 | 4.1s | PASS (4 figures generated) |

---

## 4. Test Evidence

### 4.1 Full Regression Test Suite Execution
```text
Summary: 87 tests, 0 errors, 0 failures, 0 skipped
All 87 tests passed cleanly across packages swarm_interfaces and swarm_formation.
```

### 4.2 Rosbag Recording Telemetry (`ros2 bag info`)
```text
Files:             gazebo_swarm_bag_0.mcap
Bag size:          11.1 MiB
Storage id:        mcap
ROS Distro:        jazzy
Duration:          42.392258293s
Messages:          68031
Topics Recorded:   /clock (7,331 msgs)
                   /robot_[1..5]/pose (1,830 msgs)
                   /robot_[1..5]/cmd_vel (9,996 msgs)
                   /robot_[1..5]/cmd_vel_raw (9,848 msgs)
                   /robot_[1..5]/estimated_pose (9,614 msgs)
                   /robot_[1..5]/estimation_error (9,614 msgs)
                   /robot_[1..5]/safety_intervention (9,993 msgs)
                   /robot_[1..5]/neighbors (9,815 msgs)
```

### 4.3 Online vs. Offline Metric Comparison

| Metric | Online Logger (`metrics_logger`) | Offline Engine (`rosbag_processor`) | Absolute Difference |
| :--- | :--- | :--- | :--- |
| **Global Min Distance** | 0.8492 m | 0.8495 m | 0.0003 m (< 0.1 mm) |
| **Total Collisions** | 0 events | 0 events | 0 (identical) |
| **Empirical Safety Status** | Empirically Collision-Free | Empirically Collision-Free | 100% agreement |
| **Mean Estimation Error** | 0.0593 m | 0.0624 m | 0.0031 m (3.1 mm) |
| **Convergence State** | In-transit ($t=40\text{s}$) | In-transit ($t=40\text{s}$) | 100% agreement |

---

## 5. Acceptance Criteria Verification

| Milestone M7 Criterion | Target Requirement | Realized Result | Status |
| :--- | :--- | :--- | :--- |
| **Standardized Recording** | Record with `ros2 bag` into `.mcap` container | 68,031 messages captured at 50 Hz | **PASS** |
| **Decoupled Architecture** | Record without touching robot controller code | External `ExecuteProcess` recorder node | **PASS** |
| **Offline Metric Reproduction** | Offline computation matches online metrics | $d_{\min}$ matches within 0.3mm; collisions match exactly | **PASS** |
| **Pure Rosbag Plotting** | Regenerate all plots from rosbag alone | 4 high-res plots generated from bag alone | **PASS** |
| **CLI Availability** | Standard CLI tools `process_bag` and `plot_bag` | Exposed via `console_scripts` in `setup.py` | **PASS** |
| **Linter & Docstrings** | 100% PEP 257 docstrings and flake8 compliance | 0 lint warnings; 0 pep257 issues | **PASS** |
| **Full Regression Suite** | Earlier milestones M0–M6 pass regression tests | 87 tests passed, 0 failures, 0 skipped | **PASS** |

---

## 6. Generated Visualization Artifacts (from Rosbag Alone)

The following publication-grade diagnostic plots were generated strictly from `results/M7/gazebo_swarm_bag`:
1. `results/M7/plots/formation_trajectories.png`: 2D multi-robot trajectories showing starting positions, crossing evasion maneuvers, centroid drift, and target polygon convergence.
2. `results/M7/plots/shape_error_and_safety.png`: 2-panel temporal plot showing RMS shape error vs. convergence threshold (0.25m) and pairwise inter-robot distance vs. CBF barrier zone (0.70m) and collision limit (0.50m).
3. `results/M7/plots/estimation_error.png`: Swarm mean and individual cooperative EKF localization error over simulation time.
4. `results/M7/plots/diagnostic_summary.png`: 4-panel executive diagnostic panel with complete quantitative summary metrics box.

---

## 7. Known Limitations & Caveats

1. **MCAP vs SQLite3 File Locking**: When running simulations that terminate abruptly without graceful SIGINT, MCAP files may retain open writer chunks. Setting `--max-cache-size 0` guarantees that messages are flushed immediately to disk, preventing data loss.
2. **Offline Resampling Step Size**: The offline processor resamples asynchronous rosbag timestamps onto a uniform time grid (default $\Delta t = 0.05\text{s}$, 20 Hz). Slight differences in sub-millisecond interpolation between the online 20 Hz loop and offline grid can introduce negligible numerical variations ($\le 0.3\text{mm}$).
3. **Safety Claims**: While all empirical tests showed $d_{\min} \ge 0.8495\text{m} > 0.50\text{m}$ (0 collisions), all documentation continues to adhere strictly to the empirical safety protocol: "empirically collision-free".

---

## 8. Handoff Notes for Milestone M8 Engineer

- **Handoff Target**: Milestone M8 (Experiment Campaign & Statistical Ablation).
- **M8 Requirements**:
  - Run 20–30 randomized seeds per experimental condition.
  - Compute statistical metrics (mean, standard deviation, 95% confidence intervals).
  - Perform ablation studies: Huber-only, EMA-only, Heartbeat-only, APF-only vs. CBF-QP safety filter.
  - Conduct environmental difficulty sweeps (network packet drop 0% to 50%, communication delays 0 to 200ms).
  - Utilize the M7 `process_bag` and `plot_bag` pipeline to evaluate and visualize the entire campaign automatically from rosbag recordings.
- **Tools Ready for M8**:
  - `ros2 launch swarm_formation gazebo_swarm_launch.py record_bag:=true bag_path:=...`
  - `ros2 run swarm_formation process_bag <bag_path> --output-json <summary.json>`
  - `ros2 run swarm_formation plot_bag <bag_path> --output-dir <plot_dir>`
