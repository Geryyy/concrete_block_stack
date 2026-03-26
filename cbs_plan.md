# CBS Integration Plan

## Current state

Commissioning is now organized around three canonical operator tasks:

- `Move empty`
- `Single block plan`
- `Single block execute`

Validated today:

- seeded world-model startup with static `B0`
- timber-backed `Move empty`
- timber-backed `Single block plan`
- timber-backed `Single block execute`

Current CBS target:

- bring the concrete / CBS backend online behind the same BT and service contracts
- make the concrete backend plan and execute online with a fully actuated IK + TOPP-RA path-following stage

## Current priorities

### 1. Centralized planning scene

Move obstacle knowledge into the world model and expose it through:

- `/world_model_node/get_planning_scene`

That planning scene should include:

- dynamic blocks
- static crane/environment obstacles used for collision checking

CBS/FCL should build its scene from this one service instead of hardcoded obstacle tables.

### 2. Plan-only CBS milestone

Near-term success means:

- `Single block plan` succeeds with `planner.backend:=concrete`
- the concrete backend consumes the centralized planning scene
- planned path is visible in RViz
- the path starts at the live tool pose

Execution is intentionally deferred until this plan-only step is stable.

### 3. Standalone acados development

The acados trajectory stage is still sensitive enough that it should be tuned outside the online BT loop.

The current standalone bench is:

- `concrete_block_motion_planning/scripts/acados_benchmark.py`
- `concrete_block_motion_planning/motion_planning/data/acados_bench_cases.yaml`

Purpose:

- run curated start/goal cases
- measure solve success, timing, terminal error, and diagnostics
- tune solver settings without RViz / BT / Gazebo noise

## Planner comparison

### Timber staged path

Current timber path:

- direct A-to-B target request
- iLQR / iLQR-jerk style trajectory generation
- currently the most commissioned path

Best for:

- fast operator turnaround
- execution validation
- reference behavior while CBS is still being integrated

Weakness:

- historically owned obstacle knowledge outside the centralized world model

### CBS geometric + acados path-following

Current CBS direction:

- geometric planning in an explicit collision scene
- then trajectory generation / optimization

Best for:

- clean architecture
- centralized obstacle handling
- backend interchangeability

Weakness:

- more seams to commission
- current acados online usage is still brittle

### Future free-end-time full-dynamics OCP

The proposed future direction is:

- free end time
- full under-actuated dynamics
- single direct OCP

Best long-term fit when:

- model fidelity is trusted
- initialization is strong
- standalone benchmark results are consistently good

Near-term recommendation:

- keep staged CBS as the production integration path
- use TOPP-RA for the online concrete trajectory stage
- mature the full-dynamics OCP in the standalone bench first

## Notes on legacy aliases

The old scan-oriented commissioning names are now legacy aliases:

- `scan_sequence_smoke.launch.py`
- `scan_smoke.yaml`

They should not be treated as the primary commissioning surface anymore. The canonical intent-based tasks are the BT panel entries above.
