# CBS Plan Review

Current review of the concrete-block stack after the timber-compat refactor.

## What Is Tightened Up

- `subtree_transport_block.xml` now follows the timber manipulation contract more closely:
  - empty A2B move to pickup approach
  - timber grip phase for approach plus close
  - timber lift phase
  - loaded A2B move to place
  - timber laydown phase
- `GripperAction` is no longer just a no-op smoke placeholder in the effective runtime path; it now drives the `gripper_state` bridge used by the timber-compatible stack.
- CBS still keeps the motion-planning boundary intact:
  - BT talks only to `concrete_block_motion_planning`
  - timber services remain hidden behind the CBS planner shell
- The PZS100 timber-compat launch now includes the minimum bridge/runtime pieces needed for headless commissioning:
  - dummy joint states
  - joint-state compatibility shim
  - gripper-state bridge
  - FollowJointTrajectory proxy

## Remaining Gaps Worth Keeping Visible

### 1. Full end-to-end single-block execution is not yet commission-proven

The timber-compatible pieces now initialize and both `a2b_movement` and `grip_traj_movement` can return real trajectories under the right conditions, but the complete CBS-driven single-block execution path still needs runtime commissioning as one integrated sequence.

**Impact:** Architecture is in much better shape, but the final "one block placed in sim through CBS" milestone is not yet signed off.

### 2. Helper-node shutdown is improved but still not elegant everywhere

The Python bridge/proxy nodes now suppress noisy `KeyboardInterrupt` stack traces during shutdown, but launch teardown is still not entirely clean because the runtime is a temporary compat harness rather than a fully productionized bringup path.

**Impact:** Low. Mostly a commissioning ergonomics issue.

### 3. Collision and payload semantics are still conservative compatibility shims

The current timber path uses compatibility geometry and payload proxies so we can reuse tested timber code with PZS100 and CBS. That is the right short-term tradeoff, but it is still a shim layer, not a final CBS-native planning model.

**Impact:** Medium. Safe enough for the current fast path as long as the limitation remains explicit.

## Dead Ends Removed

- The older CBS transport idea that split pickup into an extra descend step before `TIMBER_GRIP` has been removed. Native timber grip planning behaves better when it owns that approach segment itself.
- Nominal placement no longer misuses the timber release/failure phase. CBS now uses the proper timber laydown phase for unload.

## Recommended Next Focus

1. Commission the full CBS single-block execution flow in simulation.
2. Tighten any remaining runtime mismatches found during that end-to-end test.
3. Only then remove more legacy/dead-end CBS planning paths that are no longer part of the chosen timber-reuse route.
