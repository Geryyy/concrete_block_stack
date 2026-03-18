# CBS Plan Review

Code review of the concrete-block stack against `cbs_plan.md` and `tasks_context.md`.

## What Checks Out

- All three packages exist with the claimed structure: `concrete_block_behavior_tree`, `concrete_block_motion_planning`, `concrete_block_perception`.
- Planner services (`plan_geometric_path`, `compute_trajectory`, `execute_trajectory`, `execute_named_configuration`, `get_next_assembly_task`, `plan_and_compute_trajectory`) all have corresponding BT plugins and are referenced correctly in the XML trees.
- Perception modes (`SCENE_DISCOVERY`, `REFINE_BLOCK`, `REFINE_GRASPED`) are used correctly in the BT XML and perception C++ code.
- Named configurations `scan_left`, `scan_center`, `scan_right` exist in `config/named_configurations.yaml`.
- Wall plan `basic_interlocking_3_2` exists in `motion_planning/data/wall_plans.yaml`.
- Simulation launch `sim_wall_build.launch.py` correctly wires crane, PZS100, Gazebo, planner, optional perception, and delayed BT startup.
- The main assembly tree now loops until `GetNextAssemblyTask` reports no remaining task.
- The BT plugin set now includes `SetBlockTaskStatus`, and the default assembly tree uses it after transport/refinement.
- BT configs now use install-space `behaviortree` paths consistently.

---

## Issues and Gaps Not Covered in the Plan

### 1. Gripper control is still only a smoke stub

`behavior_trees/subtree_transport_block.xml` now includes `GripperAction` calls, but the plugin implementation is still a log-and-succeed stub.

**Impact:** The wall-build tree shape is aligned with the intended workflow, but real pick/place semantics are still not commissioned from the BT layer until a real PZS100 control interface is wired.

---

### 2. Smoke launch and scan commissioning are intentionally different paths

`sim_wall_build_smoke.launch.py` passes `use_perception=False` to `sim_wall_build.launch.py`. When `use_perception=False`, the BT server starts with `config/dummy_start.yaml` (pointing to `dummy_start.xml`), not `config/scan_smoke.yaml`.

This is acceptable, but the commissioning docs must make the split explicit:
- `sim_wall_build_smoke.launch.py` is the fast planner/simulation smoke path
- `scan_sequence_smoke.launch.py` is the perception-backed scan commissioning path

Scan commissioning requires either:
- `concrete_block_behavior_tree/launch/scan_sequence_smoke.launch.py` (exists but not mentioned as the scan entrypoint), or
- passing `use_perception=True` and a custom `bt_params_file` pointing to `scan_smoke.yaml`.

---

### 3. Transport subtree still covers only the carry segment

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
| Gripper control is still a stub | `subtree_transport_block.xml`, `gripper_action.cpp` | High — full pick/place still uncommissioned |
| Smoke launch and scan commissioning use different entrypoints | `sim_wall_build_smoke.launch.py`, `scan_sequence_smoke.launch.py` | Medium — docs must keep the split explicit |
| Transport subtree missing pre-grasp and post-place motion | `subtree_transport_block.xml` | Medium — full pick-place motion unplanned |
