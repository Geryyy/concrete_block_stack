# CBS Plan Review

Code review of the concrete-block stack against `cbs_plan.md` and `tasks_context.md`.

## What Checks Out

- All three packages exist with the claimed structure: `concrete_block_behavior_tree`, `concrete_block_motion_planning`, `concrete_block_perception`.
- Planner services (`plan_geometric_path`, `compute_trajectory`, `execute_trajectory`, `execute_named_configuration`, `get_next_assembly_task`, `plan_and_compute_trajectory`) all have corresponding BT plugins and are referenced correctly in the XML trees.
- Perception modes (`SCENE_DISCOVERY`, `REFINE_BLOCK`, `REFINE_GRASPED`) are used correctly in the BT XML and perception C++ code.
- Named configurations `scan_left`, `scan_center`, `scan_right` exist in `config/named_configurations.yaml`.
- Wall plan `basic_interlocking_3_2` exists in `motion_planning/data/wall_plans.yaml`.
- Simulation launch `sim_wall_build.launch.py` correctly wires crane, PZS100, Gazebo, planner, optional perception, and delayed BT startup.

---

## Issues and Gaps Not Covered in the Plan

### 1. The main BT has no loop — it handles only one block

`behavior_trees/concrete_block_assembly.xml` runs
`GetNextAssemblyTask → RefineBlock → TransportBlock → RefineGrasped`
once and then stops. There is no loop construct around this sequence.

The plan repeatedly says "repeat until the wall is complete" but the tree cannot do this as written. The `plan_has_task` blackboard output is written by `GetNextAssemblyTask` but is never read by any conditional node that could drive a loop or early exit.

**Impact:** Blocks Phase 6. A loop node (e.g. `WhileDoElse` or a `Repeat` variant) wrapping the per-block sequence, with a `plan_has_task` check as the exit condition, is missing.

---

### 2. No `set_block_task_status` BT plugin — world model is never updated after placement

The full BT plugin set is:
- `run_pose_estimation`
- `plan_geometric_path`
- `compute_trajectory`
- `plan_and_compute_trajectory`
- `execute_trajectory`
- `move_to_named_configuration`
- `get_next_assembly_task`

There is no plugin to call `set_block_task_status`. After a placement trajectory executes, the world model retains the block in its pre-placement state. `GetNextAssemblyTask` will therefore keep returning the same task on the next iteration, regardless of what the crane actually did.

**Impact:** Wall plan progression is broken at the world-model boundary. The plugin and a call site in the assembly tree are both missing.

---

### 3. No gripper open/close BT plugin

`behavior_trees/subtree_transport_block.xml` plans and executes a trajectory but contains no gripper actions. The plan flags "grasp / release semantics not yet documented as a commissioned sequence", but this understates the situation: there is no BT plugin for gripper control at all. This is not a configuration or commissioning gap — it is a missing plugin.

**Impact:** Physical pick-and-place is impossible from the BT layer without adding a gripper action plugin and inserting open/close calls at the correct points in the transport subtree.

---

### 4. Smoke launch and scan commissioning are mismatched

`sim_wall_build_smoke.launch.py` passes `use_perception=False` to `sim_wall_build.launch.py`. When `use_perception=False`, the BT server starts with `config/dummy_start.yaml` (pointing to `dummy_start.xml`), not `config/scan_smoke.yaml`.

The plan suggests using the smoke launch as the main fast commissioning entrypoint (Phase 2, Steps A–C). However, scan commissioning (Step C: trigger `SCENE_DISCOVERY` at each scan pose) cannot be done through the smoke launch as configured. It requires either:
- `concrete_block_behavior_tree/launch/scan_sequence_smoke.launch.py` (exists but not mentioned as the scan entrypoint), or
- passing `use_perception=True` and a custom `bt_params_file` pointing to `scan_smoke.yaml`.

---

### 5. BT config files use relative `src/...` paths — not install-space safe

`config/default.yaml` and `config/dummy_start.yaml` both specify:
```yaml
behaviortree: "src/concrete_block_stack/concrete_block_behavior_tree/behavior_trees/..."
```
These paths only resolve correctly if the BT action server process is started from the workspace root.

At the same time, `behavior_trees/concrete_block_assembly.xml` uses `install/...` paths for subtree includes:
```xml
<include path="install/concrete_block_behavior_tree/share/.../subtree_scene_scan.xml"/>
```
These only resolve after a build, again from the workspace root.

The inconsistency means the working-directory requirement is load-bearing but undocumented. A path that works in one config will silently fail if used from a different working directory.

---

### 6. Transport subtree covers only the carry segment, not the full pick-place motion

`subtree_transport_block.xml` calls `PlanGeometricPath` with:
- `start_pose = target_block_pose_coarse` (approximate block pickup location)
- `goal_pose = target_block_pose_precise` (target placement location)

This represents motion from pickup to placement, but the subtree does not cover:
- moving to a pre-grasp approach position above the block
- the post-place retract motion after release

The plan lists "pickup, transport, and placement scenarios need explicit scenario-level validation" as a gap, but does not note that the current BT subtree structure only handles the carry segment. The pre-grasp and post-place segments are unrepresented in the tree.

---

## Summary Table

| Issue | Where | Severity |
|---|---|---|
| No loop in main BT — one block only | `concrete_block_assembly.xml` | High — blocks Phase 6 |
| No `set_block_task_status` BT plugin | BT plugin set + assembly tree | High — wall plan progression stalls |
| No gripper open/close BT plugin | `subtree_transport_block.xml` | High — physical pick/place impossible |
| Smoke launch does not reach scan commissioning path | `sim_wall_build_smoke.launch.py` | Medium — Step C needs different entrypoint |
| Relative `src/...` paths in BT configs | `default.yaml`, `dummy_start.yaml` | Low-medium — fragile, undocumented requirement |
| Transport subtree missing pre-grasp and post-place motion | `subtree_transport_block.xml` | Medium — full pick-place motion unplanned |
