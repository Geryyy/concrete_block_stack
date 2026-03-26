# Concrete Block Stack Task Context

## Purpose

This workspace extends the timber-crane stack toward concrete-block experiments with the `PZS100` gripper.

The concrete-block work lives under:

- `src/concrete_block_stack/`

## Current commissioning model

The near-term operating model is manual, task-oriented commissioning rather than one large automatic wall-building BT.

Canonical operator tasks:

- `Move empty`
- `Single block plan`
- `Single block execute`

This keeps commissioning fast and observable:

- plan only when we want to inspect world-model and planner behavior
- approval-gated execute when we want to test the live motion path

## Package roles

### `concrete_block_perception`

Owns the persistent world model and planning scene.

Current important interfaces:

- `get_coarse_blocks`
- `get_planning_scene`
- `set_block_task_status`
- `run_pose_estimation`

Current commissioning reality:

- seeded world-model startup is available
- static `B0` seeding is in use
- static planning-scene obstacles are now owned here as shared config
- perception-driven scan/refine is not the active integration priority right now

### `concrete_block_motion_planning`

Owns the shared planner services.

Current shared service surface:

- `plan_and_compute_trajectory`
- `execute_trajectory`
- `execute_named_configuration`
- `get_next_assembly_task`

Backend selection is launch/config driven:

- `planner.backend:=timber`
- `planner.backend:=concrete`

Current reality:

- timber path is the validated planning/execution reference
- concrete/CBS path is being brought online behind the same service contract
- the centralized planning scene is now the intended source of truth for CBS/FCL collision queries
- the concrete online trajectory stage is now being simplified to a fully actuated IK + TOPP-RA path-following pipeline

### `concrete_block_behavior_tree`

Owns the operator-facing commissioning tasks and the longer-term assembly orchestration.

Current BT panel surface should expose only:

- `Move empty`
- `Single block plan`
- `Single block execute`

Legacy scan-smoke naming remains only for compatibility and should not be treated as the primary commissioning surface.

## Current priorities

1. Keep the validated timber execution path green.
2. Use the standalone `motion_planning` lab as the primary development loop for CBS planner work.
3. Fix solver/model consistency first, then improve the simplified joint-space planner.
4. Keep `acados` and heavier trajectory optimizers in offline comparison mode until the lightweight stack is trustworthy.
5. Return to perception-driven scan/refine integration only after the planner/world-model seam is stable again.

## Short planner comparison

### Timber / current reference

- direct A-to-B / iLQR-like runtime
- fewer moving parts in the online path
- currently best for live commissioning

### CBS / staged concrete path

- explicit geometric scene + FCL
- cleaner architecture for centralized world-model obstacles
- better long-term interchangeability
- current online planner direction is:
  - static/steady-state goal solve
  - actuated joint-space path generation
  - simple timing / TOPP-RA later if needed

## Current planner development reality

The active planner development surface is now the standalone lab under:

- `src/concrete_block_stack/concrete_block_motion_planning/motion_planning/standalone/`
- `src/concrete_block_stack/concrete_block_motion_planning/motion_planning_tools/standalone/run_planner_experiment.py`

What is working:

- standalone path-planning and solver comparison runs without ROS/Gazebo
- matplotlib plotting of TCP path and joint path
- overlay of real block scenes from the scenario library
- a reachable scene-backed demo:
  - `scene_demo_step_01_reachable`

What is not solved yet:

- raw scene tasks like `scene_step_01_first_on_ground` still expose solver limitations
- `single_block_transfer` remains a real failing/planning-debug case
- the CBS runtime is no longer the right place to invent planner logic; it should consume standalone-validated logic

## Verified standalone commands

Path + timing:

```bash
python3 src/concrete_block_stack/concrete_block_motion_planning/motion_planning_tools/standalone/run_planner_experiment.py \
  --mode planner \
  --stack joint_goal_interpolation \
  --scenario scene_demo_step_01_reachable \
  --timing simple \
  --plot
```

Anchor planner:

```bash
python3 src/concrete_block_stack/concrete_block_motion_planning/motion_planning_tools/standalone/run_planner_experiment.py \
  --mode planner \
  --stack cartesian_anchor_joint_spline \
  --scenario short_reachable_move
```

Solver comparison:

```bash
python3 src/concrete_block_stack/concrete_block_motion_planning/motion_planning_tools/standalone/run_planner_experiment.py \
  --mode solver_compare \
  --scenario short_reachable_move
```

## Handoff reference

For the current planner-specific handoff, see:

- `src/concrete_block_stack/concrete_block_motion_planning/doc/STANDALONE_PLANNER_HANDOFF.md`

### Future free-end-time OCP

- best long-term dynamics fidelity
- highest tuning burden
- should stay in standalone R&D mode first

## Useful references

- `src/concrete_block_stack/cbs_plan.md`
- `src/concrete_block_stack/concrete_block_motion_planning/doc/ARCHITECTURE.md`
- `src/concrete_block_stack/concrete_block_motion_planning/doc/SERVICES.md`
- `src/concrete_block_stack/concrete_block_behavior_tree/doc/BEHAVIOR_TREES.md`
- `src/concrete_block_stack/concrete_block_behavior_tree/doc/GROOT.md`
