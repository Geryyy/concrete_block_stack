# Concrete Block Assembly Context

## Resume Snapshot (2026-03-10)

This is the latest working state before Docker rebuild.

### What was changed in this session

1. Concrete block visibility in Gazebo
- Kept block visual as simple neon/orange box for reliability (mesh visualization deferred).
- Model exists/spawns as `concrete_block_1`.

2. Move-empty RViz interaction (new stack)
- Added `rviz_move_empty_interface.py` in `concrete_block_motion_planning`.
- Flow on goal click: `plan_geometric_path -> compute_trajectory -> execute_trajectory`.
- Added arming topic `/cb_move_empty/enable` to mimic old UX:
  - press Move button first (arm), then click RViz goal.
- Added fallback for missing geometric backend (`fcl`):
  - if geometric planning unavailable, compute trajectory using direct start/goal path.

3. Execution path to controller
- `execute_trajectory` now supports optional dispatch when enabled:
  - `execution.enabled`
  - `execution.trajectory_topic` (default `/trajectory_controllers/joint_trajectory`)
- Exposed in sim launch:
  - `cbmp_execution_enabled`
  - `cbmp_execution_topic`

4. RViz panel compatibility bridge
- Patched existing `behavior_tree_panel` (outside concrete_block_stack) so Move/Cancel publish:
  - `/cb_move_empty/enable` true/false
- Fallback behavior preserved:
  - if no bridge subscriber exists, panel keeps legacy `move_empty.xml` behavior.
- Added `BehaviorTreePanel` to `concrete_bt.rviz`.

5. BT startup stability
- Added small delay before `lifecycle_manager` in `concrete_block_behavior_tree/launch/bt.launch.py`
  to reduce configure race on startup.

6. Dependency robustness (`fcl`)
- Added `python-fcl` to:
  - `concrete_block_motion_planning/requirements-core.txt`
  - `concrete_block_motion_planning/pyproject.toml`
- Added dependency note in `concrete_block_motion_planning/VENDOR_README.md`.
- Installed `python-fcl` in current container for immediate test.

7. Docker build robustness
- Updated `.devcontainer/Dockerfile.vscode` to install CBMP runtime deps at image build:
  - `libfcl-dev`, `python-fcl`, and core Python stack.
- This is the main fix so fresh containers have geometric runtime ready.

### Recommended post-rebuild validation

1. Rebuild/reopen container, then:
- `source /opt/ros/humble/setup.bash`
- `source /workspaces/ros2_baustelle_ws/install/setup.bash`

2. Launch:
- `ros2 launch concrete_block_behavior_tree sim_wall_build.launch.py cbmp_execution_enabled:=True`

3. In RViz:
- Open `BehaviorTreePanel`.
- Press `Move` once (arms move-empty interface).
- Use `SetGoal` tool on `/goal_pose`.

4. Expected logs:
- `rviz_move_empty_interface`: `Move-empty interface ENABLED.`
- then `Move-empty request ...`
- then either normal geometric planning path or fallback direct-path path (if geometric backend unavailable).

## Simulation Roadmap
- Detailed roadmap: `docs/simulation_roadmap.md`
- Active phase: Phase 1 (in progress)
- Base strategy: Timber sim as bootstrap
- Execution strategy: dry-run first, MPC integration after stable BT+planning loop
- Parallelization: Agent A (Sim/BT/Execution), Agent B (Perception/Validation/Docs)
- Canonical launch (phase-1 baseline):
  - `ros2 launch concrete_block_behavior_tree sim_wall_build.launch.py`
  - with perception enabled: `ros2 launch concrete_block_behavior_tree sim_wall_build.launch.py use_perception:=True`

## Repository Setup
- Multi-repo manifest: `concrete_block_stack.repos`
- Import/update command:
  - `vcs import . < concrete_block_stack.repos`
  - `vcs pull .`

## Goal
Build an integrated ROS 2 stack for autonomous concrete block assembly with three coordinated pillars:
- Perception
- Motion Planning
- Sequencing (Behavior Tree)

## System Pillars

### 1. Perception
Package: `concrete_block_perception`

Responsibilities:
- Detect and segment blocks from camera + point cloud input.
- Run registration on demand for precise block poses.
- Publish and maintain a persistent world model.

Current status:
- Implemented and actively refactored for BT-driven on-demand operation.
- Core APIs already available: `RunPoseEstimation`, `GetCoarseBlocks`, `SetPerceptionMode`.
- Not finalized yet (interfaces and runtime behavior still being hardened).

### 2. Motion Planning
Package: `concrete_block_motion_planning`

Responsibilities:
- Convert pose-level assembly intents into executable motion plans.
- Provide deterministic service interfaces for planning and execution.
- Bridge ROS requests to vendored planning logic.

Current status:
- Added as a new package in this stack.
- Based on vendored snapshot of `git@github.com:Geryyy/motion_planning.git`.
- Wrapped behind concrete-block-specific ROS service contracts.
- Stage-split service contracts are now in place:
  - geometric path planning
  - trajectory computation
  - trajectory execution
- Trajectory stage currently requires acados runtime dependencies.

### 3. Sequencing
Package: `concrete_block_behavior_tree`

Responsibilities:
- Orchestrate task phases and recovery logic.
- Trigger perception one-shots and motion planning actions in the right order.
- Maintain blackboard context for target/reference blocks and phase outcomes.

Current status:
- Added as a dedicated BT package using existing `lsrl_behavior_tree` / `nav2_behavior_tree` conventions.
- Includes custom BT plugins and initial assembly tree skeleton.

## Integration Architecture

Data/control flow:
1. BT requests scene discovery and pose refinement via `concrete_block_perception`.
2. BT selects target/reference context and calls geometric planning service.
3. BT calls trajectory computation service on top of geometric result.
4. BT triggers trajectory execution service and checks completion status.
4. BT repeats refine-plan-execute cycles for pickup and assembly.

Expected runtime orchestration:
- BT is the top-level coordinator.
- Perception remains mostly idle and is triggered on demand.
- Motion planning is called per task phase, not continuously.
- Recovery branch handles perception/planning failures deterministically.

## Near-Term Milestones
1. Stabilize perception contract for all one-shot modes and target ID handling.
2. Validate stage-split motion-planning services (`PlanGeometricPath`, `ComputeTrajectory`, `ExecuteTrajectory`).
3. Validate BT plugin/service interactions and blackboard wiring for split stages.
4. Run full-stack launch with perception + planning + BT action server.
5. Add rosbag/sim regression scenario for pickup-to-placement sequence.

## Acceptance Criteria
- All three packages build in one workspace invocation.
- BT can call all three `RunPoseEstimation` modes successfully.
- Stage-split motion planning services return deterministic success/failure and never crash.
- End-to-end tree reaches terminal status in both success and induced-failure runs.
- Public perception APIs remain backward compatible for existing callers.

## Open Risks / Unresolved Items
- Vendored planner dependencies may require optional runtime packages not installed in all environments.
- Trajectory solver integration is acados-gated; missing runtime dependencies should fail clearly.
- Motion execution is currently represented by service-level abstraction; controller-level execution integration still requires follow-up.
- Perception-to-planning frame conventions must be validated in integrated tests.
- Final BT tree logic and thresholds still require task-level tuning on real scenarios.
