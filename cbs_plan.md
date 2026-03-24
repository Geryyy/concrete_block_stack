# Concrete Block Stack Plan

## Planning Goal
Commission the concrete-block stack into a reliable staged workflow that can progress from isolated component tests to a repeatable wall-building simulation experiment.

## Goal Clarification
The intended outcome is clear:

Build concrete-block wall experiments with the timber crane and `PZS100` gripper, using:
- perception for block/world-state estimation
- motion planning for transport and placement motion
- behavior trees for orchestration

The current repository already contains the main software pillars. The open work is not "start from zero", but "align, verify, integrate, and commission".

## What Is Required

### Functional requirements
- detect visible blocks in the scene
- keep a persistent world model of block poses and task states
- select the next assembly task from a wall plan
- compute transport and placement motion for the selected block
- execute trajectories in simulation
- recover from failed perception or planning steps
- repeat until the wall plan is completed

### System requirements
- Gazebo simulation with crane + `PZS100`
- RViz visualization
- world-model service availability
- planner service availability
- BT action server and plugins
- consistent frame names, controller names, and execution topics
- usable commissioning entrypoints for small tests

### Validation requirements
- unit tests for config, data loading, and service contracts
- smoke tests for launch bringup
- subsystem commissioning for perception, planner, and execution separately
- end-to-end simulation run for at least one simplified wall plan

## What Is Implemented

### Perception
- `world_model_node` exists and exposes:
  - `run_pose_estimation`
  - `get_coarse_blocks`
  - `set_block_task_status`
- one-shot modes are documented and present:
  - `SCENE_DISCOVERY`
  - `REFINE_BLOCK`
  - `REFINE_GRASPED`
- commissioning and rosbag-oriented launch files already exist
- RViz marker state/color behavior is documented

### Motion planning
- `concrete_block_motion_planning_node` exists
- planner services exist for:
  - geometric planning
  - trajectory computation
  - named configurations
  - execution dispatch
  - wall-plan progression
- wall plans already exist in `motion_planning/data/wall_plans.yaml`
- named scan configurations already exist
- planner tests exist and currently pass in this workspace when run with `python3 -m pytest`

Validated in this session:
- `test_cbmp_config.py`: passed
- `test_cbmp_yaml_validation.py`: passed
- `test_cbmp_integration_services.py`: passed

### Behavior trees
- BT plugins for planner/perception services are implemented
- default wall-build tree exists
- scan, transport, and recovery subtrees exist
- launch files exist for:
  - generic BT bringup
  - smoke wall-build simulation
  - fuller wall-build simulation
- Groot-oriented documentation already exists as a starting point

### Simulation integration
- `sim_wall_build.launch.py` already wires:
  - crane model
  - `PZS100` tool
  - Gazebo
  - RViz
  - planner bringup
  - optional perception
  - delayed BT startup

## What Is Missing Or Not Yet Proven

### End-to-end commissioning gaps
- reliable proof that the full wall-build sequence runs from scan to placement in simulation
- clear acceptance criteria for each subsystem handoff
- documented commissioning order across perception, planner, execution, and BT

### Execution-path gaps
- actual non-dry-run execution path needs verification with the active controller setup
- controller/topic/action assumptions should be tested explicitly in Gazebo
- grasp / release semantics are not yet documented as a commissioned sequence

### Planning gaps
- pickup, transport, and placement scenarios need explicit scenario-level validation
- collision-sensitive "last mile" placement behavior needs practical acceptance tests
- wall-plan progression should be verified against world-model state during integrated runs

### Perception gaps
- scene-discovery reliability across intended scan viewpoints needs a commissioning checklist
- `REFINE_BLOCK` and `REFINE_GRASPED` need explicit pass/fail criteria for integrated BT use
- world-model update timing and persistence behavior need to be checked during repeated runs

### BT / orchestration gaps
- current tree exists, but integrated success/recovery behavior is not yet commissioned as a full workflow
- experiment customization by users through BT authoring is not yet packaged as a practical guide

### Documentation gaps
- old task notes had stale references and mixed historical session notes
- there is no single current commissioning document before this file
- the future user-facing BT experiment guide is still missing

## Roadmap

### Phase 1. Baseline workspace validation
- confirm package build and launch prerequisites
- keep planner regression tests green
- verify that key launch files start without broken references
- document exact environment requirements such as `python3 -m pytest` instead of assuming `pytest`

### Phase 2. Planner-only commissioning
- use `sim_wall_build_smoke.launch.py` as the fast planner/simulation smoke baseline
- validate named scan configurations and planner service availability
- verify geometric planning, trajectory computation, and dry-run execution first
- then verify non-dry-run execution to the active trajectory consumer

### Phase 3. Perception commissioning
- use `scan_sequence_smoke.launch.py` only as a legacy alias while perception commissioning still needs a dedicated entrypoint
- commission `SCENE_DISCOVERY` with fixed scan viewpoints
- define expected world-model results after each scan
- commission `REFINE_BLOCK`
- commission `REFINE_GRASPED`
- document failure artifacts and debug workflow

### Phase 4. Planner + world-model integration
- verify planner requests with `use_world_model=true`
- confirm `get_coarse_blocks` data is sufficient for obstacle-aware planning
- validate at least one pickup-to-placement planning flow against current block state
- use `single_block_plan.xml` as the canonical planner/world-model commissioning BT
- keep `scan_sequence_smoke.launch.py` only as a legacy alias during the transition

### Phase 5. BT subsystem integration
- run scan subtree with live planner + perception services
- run transport subtree with planner execution enabled
- validate recovery subtree behavior on forced failures
- confirm blackboard data flow between scan, task selection, planning, and execution stages

### Phase 6. End-to-end wall-build simulation
- start with a reduced scenario:
  - one block pickup and placement
- expand to a partial wall:
  - first row only
- expand to the full available sample wall plan:
  - `basic_interlocking_3_2`
- record issues, timing, and operator steps needed for repeatability

### Phase 7. Future experiment authoring workflow
- add a dedicated `guide.md`
- explain how to create/edit behavior trees safely
- explain how to use Groot with this stack
- explain how to bind new experiment XML/YAML files into launch files

## Testing Strategy

### Automated tests
- keep motion-planning unit/integration tests as the fast regression baseline
- add tests for wall-plan execution assumptions where feasible
- add smoke checks for configuration files, BT config references, and smoke/default launch separation

### Manual subsystem tests
- perception:
  - rosbag test modes
  - commissioning launch stages
  - expected world-model and marker outputs
- motion planning:
  - service-call tests for geometric path, trajectory, named configurations, wall tasks
  - dry-run and non-dry-run execution checks
- behavior tree:
  - smoke trees
  - subtree-by-subtree integration runs

### End-to-end tests
- one-block pick/place in Gazebo
- partial wall assembly
- full sample wall assembly
- repeatability run after restart to check initialization robustness

## Commissioning In Small Manageable Parts

### Step A. Service availability
- launch planner only
- verify all planner services appear
- launch perception only
- verify world-model services appear

### Step B. Named configurations
- move to `scan_left`, `scan_center`, `scan_right`
- confirm controller execution and resulting TF/joint-state behavior

### Step C. Perception scan loop
- at each scan pose, trigger `SCENE_DISCOVERY`
- confirm blocks appear in world model with expected marker state/color

### Step D. Wall-task loop without execution
- request `get_next_assembly_task`
- plan geometric path
- compute trajectory
- keep execution in dry-run mode

### Step E. Single execution trial
- execute a single planned trajectory in Gazebo
- confirm controller/topic compatibility

### Step F. Single block assembly trial
- refine target/reference data
- plan and execute one pickup/place cycle
- confirm world-model update after placement

### Step G. Recovery trial
- force one perception or planning failure
- verify recovery subtree and operator observability

### Step H. Multi-block wall trial
- run the sample wall plan progressively
- collect failure cases before expanding complexity

## Suggested Near-Term Priorities
1. Keep `sim_wall_build_smoke.launch.py` as the main fast planner/simulation smoke entrypoint.
2. Prove the non-dry-run planner execution path in Gazebo.
3. Commission scene scanning and world-model updates from fixed viewpoints through `scan_sequence_smoke.launch.py`.
4. Validate one full single-block assembly cycle before attempting a full wall.
5. Only after that, refine the BT authoring workflow and user guide.

## Future Documentation
Future work should add a dedicated `guide.md` for self-configured experiments with behavior trees and GUI tools.

That guide should build on the current existing references:
- `concrete_block_behavior_tree/doc/BEHAVIOR_TREES.md`
- `concrete_block_behavior_tree/doc/GROOT.md`
