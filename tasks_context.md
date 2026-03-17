# Concrete Block Stack Task Context

## Purpose
This workspace extends the timber-crane stack toward a new use case:

Build a wall of concrete blocks with the timber crane using the `PZS100` parallel gripper.

The concrete-block work lives under `src/concrete_block_stack/`.

## Main Goal
The target experiment is an end-to-end wall-building workflow:

1. Move the crane to scan viewpoints.
2. Detect and register concrete blocks.
3. Store block poses and task state in a persistent world model.
4. Select the next wall-assembly task from a wall plan.
5. Plan transport and placement motion for the selected block.
6. Execute the motion in simulation first, then on hardware later.
7. Repeat until the wall is complete.

The intended orchestration layer is a behavior tree so that experiments such as wall building can later be configured and modified without changing core motion-planning or perception code.

## System Pillars

### 1. Perception
`concrete_block_perception`

Current perception architecture already supports BT-friendly one-shot calls through `world_model_node`:
- `SCENE_DISCOVERY`
- `REFINE_BLOCK`
- `REFINE_GRASPED`

Perception responsibilities:
- block detection and segmentation
- coarse and refined block pose estimation
- world-model maintenance
- block task-state handling such as `TASK_MOVE` / `TASK_PLACED`

Validation strategy:
- mainly rosbag-based and commissioning-launch based
- visualization through RViz markers and debug topics

Useful current references:
- `concrete_block_perception/README_PERCEPTION_MODES.md`
- `concrete_block_perception/docs/vision_pipeline_modes.md`
- `concrete_block_perception/docs/marker_color_coding.md`

### 2. Motion Planning
`concrete_block_motion_planning`

The planner is already structured as a two-stage pipeline:
- geometric Cartesian path planning
- joint-space trajectory generation / optimization

Current exposed services:
- `plan_geometric_path`
- `compute_trajectory`
- `plan_and_compute_trajectory`
- `execute_trajectory`
- `execute_named_configuration`
- `get_next_assembly_task`

Important current characteristics:
- wall-building progression is loaded from `motion_planning/data/wall_plans.yaml`
- named crane poses are loaded from `config/named_configurations.yaml`
- world-model-assisted planning is supported through `get_coarse_blocks`
- execution can dispatch via topic or action, but still needs end-to-end validation in the active simulation setup

Validation strategy:
- unit/integration tests for planner internals and services
- Gazebo smoke tests for runtime bringup

Useful current references:
- `concrete_block_motion_planning/doc/ARCHITECTURE.md`
- `concrete_block_motion_planning/doc/SERVICES.md`
- `concrete_block_motion_planning/launch/motion_planning.launch.py`

### 3. Sequencing / Behavior Trees
`concrete_block_behavior_tree`

Behavior trees are the intended orchestration mechanism for:
- scene scanning
- task acquisition from the wall plan
- perception refinement
- transport planning
- trajectory execution
- recovery behavior

Current repository state already contains:
- BT plugins for the main planner/perception services
- a default wall-build tree in `behavior_trees/concrete_block_assembly.xml`
- scan and recovery subtrees
- simulation launch entrypoints for smoke and fuller runs

Useful current references:
- `concrete_block_behavior_tree/doc/BEHAVIOR_TREES.md`
- `concrete_block_behavior_tree/doc/GROOT.md`
- `concrete_block_behavior_tree/launch/sim_wall_build.launch.py`

## Current Workspace Reality Check
The goal is understandable and the current codebase already contains the major building blocks for the full framework:
- perception services and world model
- motion-planning services and wall-plan progression
- behavior-tree plugins and XML trees
- Gazebo/RViz simulation launch files

At the same time, the stack is not yet proven as a fully commissioned wall-building pipeline. The main remaining work is integration, staged validation, and reliable execution of the whole chain.

In other words:
- the architecture is present
- important service interfaces are present
- test coverage exists mainly on the motion-planning side
- end-to-end commissioning is still the key gap

## Current Simulation Entry Points
Main launch flows:
- `concrete_block_behavior_tree/launch/sim_wall_build.launch.py`
- `concrete_block_behavior_tree/launch/sim_wall_build_smoke.launch.py`
- `concrete_block_behavior_tree/launch/sim_wall_build_full.launch.py`

Current bringup understanding:
- crane description comes from `epsilon_crane_description`
- the default tool is `pzs100_description`
- Gazebo world comes from `testsite_description`
- concrete blocks can be spawned directly from the simulation launch
- motion planning is included from `concrete_block_motion_planning`
- perception is optional in the simulation launch
- RViz can be started from the same launch flow

## What This Document Is For
This file should remain a short orientation document:
- what the project is trying to achieve
- which packages own which responsibilities
- what is already present in the workspace
- what still needs integration and commissioning

Detailed implementation planning is tracked separately in:
- `src/concrete_block_stack/cbs_plan.md`

## Future Outlook
A future step is to make experiment setup more user-configurable through behavior trees, including a practical guide for editing and running BT experiments with GUI tooling.

That future deliverable should likely become:
- `guide.md` for experiment setup and BT authoring

Current precursor material already exists in:
- `concrete_block_behavior_tree/doc/BEHAVIOR_TREES.md`
- `concrete_block_behavior_tree/doc/GROOT.md`
