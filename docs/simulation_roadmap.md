# Gazebo Wall-Building Roadmap

## Summary
This roadmap defines how to build a simulation-first validation pipeline for wall construction with `concrete_block_stack`, starting from a minimal Gazebo setup and incrementally integrating spawning, planning, execution, behavior trees, and perception.

The strategy is intentionally staged:
- Start fast with existing timber simulation bringup as technical bootstrap.
- Add concrete-specific simulation assets and scenarios.
- Stabilize motion planning and execution interfaces in simulation.
- Integrate behavior-tree orchestration and perception for end-to-end wall-building loops.

## Current State (Exists vs Missing)

### Exists
- Behavior Tree package with launch files and initial assembly trees.
- Motion planning package with stage-split services:
  - geometric path planning
  - trajectory computation
  - trajectory execution abstraction
- Perception package with one-shot modes and world model integration.
- Rosbag-driven validation flows for perception modes.
- Timber simulation launch path that can be reused for early Gazebo smoke tests.

### Missing
- Concrete-specific Gazebo scenario package with deterministic spawn control.
- Canonical wall-building simulation launch for full-stack runs.
- Fully integrated execution backend path (A2B/MPC/jerk) for non-dry-run testing.
- Formal KPI harness for repeated simulation trials.

## Phased Roadmap (Phase 0..8)

### Phase 0: Documentation baseline
- Add this roadmap and reference it from `context.md`.
- Define canonical launch/config naming.
- Define ownership split for parallel work.

### Phase 1: Minimal simulation loop
- Use timber Gazebo bringup as bootstrap.
- Launch BT + planning + sim with dry-run trajectory execution.
- Validate discovery -> plan -> trajectory compute loop.

### Phase 2: Concrete block spawning
- Add concrete block spawn presets (`single_block`, `wall_3_2_seed`).
- Ensure deterministic seeded scenarios and stable block IDs.
- Add scenario reset/reseed controls.

### Phase 3: Planning integration hardening
- Validate planning service contracts under simulation timing.
- Improve error propagation and retry-safe responses.
- Add scenario-level planning regression checks.

### Phase 4: Trajectory execution integration
- Integrate backend switch (`dry_run`, `mpc`, optional jerk/feedforward).
- Wire trajectory execution abstraction to selected backend.
- Validate timeout and failure handling in BT-facing interfaces.

### Phase 5: Behavior-tree assembly flow
- Move from smoke trees to wall-task tree (`basic_interlocking_3_2`).
- Add recoveries for perception/planning/execution failures.
- Validate complete task progression across all placements.

### Phase 6: Perception in simulation
- Use simulated sensor data for scene discovery + refine loops.
- Run with perception-on/perception-off toggles for A/B behavior.
- Validate world model consistency during wall-building cycles.

### Phase 7: KPI validation harness
- Run multi-seed simulation campaigns.
- Track success rate, placement error, cycle time, retries.
- Export summary artifacts for regression comparison.

### Phase 8: Hardening and handover
- Consolidate launch profiles and remove config duplication.
- Finalize documentation and troubleshooting notes.
- Freeze known-good simulation profiles for future agents.

## Two-Agent Parallel Task Plan

### Agent A: Sim/BT/Execution
- Owns Gazebo orchestration, spawn integration, BT wall flow, execution backend wiring.
- Primary deliverables: Phases 1, 2, 4, 5.

### Agent B: Perception/Validation/Docs
- Owns perception-in-sim integration, test harness, KPI reporting, doc quality.
- Primary deliverables: Phases 0, 6, 7, 8 (with A support in final integration).

## Dependencies and Sequential Gates

### Parallelizable tracks
- After Phase 0:
  - Agent A can start Phase 1.
  - Agent B can expand test/docs scaffolding in parallel.

### Sequential gates
1. Phase 2 depends on Phase 1.
2. Phase 4 depends on Phase 1.
3. Phase 5 depends on Phase 2 + Phase 4.
4. Phase 6 depends on Phase 2.
5. Phase 7 depends on Phase 5 + Phase 6.
6. Phase 8 depends on Phase 7.

## Launch/Config Target Structure

### Canonical launch
- `launch/sim_wall_build.launch.py` as the main entrypoint.

### Sub-launch composition
- Simulation bringup
- Motion planning
- Behavior tree
- Perception (optional by launch flag)

### Profile configs
- `config/profiles/sim_smoke.yaml`
- `config/profiles/sim_wall_build_dryrun.yaml`
- `config/profiles/sim_wall_build_mpc.yaml`

### Config design rules
- Keep launch args explicit and mapped to YAML defaults.
- Avoid hidden hardcoded mode switches.
- Keep simulation profiles reproducible via seed.

## Test Matrix and Acceptance Criteria

### Test matrix
1. Smoke: sim + BT + planner, dry-run execution.
2. Execution: same flow with MPC backend.
3. Perception A/B: perception on vs off in same seeded scenario.
4. Stress: multiple seeds and occlusion variants.
5. Fault injection: planner timeout, execution backend unavailable, perception failure.

### Acceptance criteria
- End-to-end wall sequence can run in simulation without node crashes.
- Deterministic scenario replay works with identical seeds.
- Execution backend selection is configurable and observable.
- KPI report can be generated from repeated runs.

## Risks and Mitigations

1. Bootstrap drift from timber to concrete assets.
- Mitigation: isolate timber dependency behind a single launch adapter layer.

2. Execution backend instability.
- Mitigation: keep dry-run fallback, enforce clear timeout/failure contracts.

3. Perception sensitivity to simulated sensing quality.
- Mitigation: maintain perception-off baseline and compare against perception-on results.

4. Config sprawl and hidden coupling.
- Mitigation: profile-based configs, explicit launch args, documented ownership.

## Assumptions and Defaults
- No ROS message/service schema changes are required for this roadmap step.
- `basic_interlocking_3_2` remains the baseline wall assembly plan.
- Early phases prioritize speed and integration visibility over final realism.
- Execution strategy: dry-run first, MPC integration after stable BT + planning loop.
- Parallelization default:
  - Agent A: Sim/BT/Execution
  - Agent B: Perception/Validation/Docs
