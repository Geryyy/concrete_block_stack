# Timber Crane Framework Outline

## Purpose

This note captures the timber crane framework as it exists in this workspace so
the CBS stack can mirror the tested structure instead of inventing a parallel
application architecture.

The key pattern to preserve is:

- BTs orchestrate task phases
- motion-planning services generate trajectories
- the timber controller path executes those trajectories
- payload state and obstacle inputs stay explicit runtime interfaces

## Main Modules

### 1. Behavior-tree execution layer

Packages:

- `lsrl_behavior_tree`
- `epsilon_crane_behavior_tree`

Role:

- operator-facing task orchestration
- controller switching and approval gating
- planner service calls
- trajectory execution dispatch

Main interfaces:

- action `/execute_bt`
- service `a2b_movement`
- service `grip_traj_movement`
- action `/trajectory_controller_a2b/follow_joint_trajectory`
- action `/trajectory_controller_grasping/follow_joint_trajectory`
- service `/controller_manager/switch_controller`

Reference files:

- `src/epsilon_crane_behavior_tree/behavior_trees/subtree_a2b_movement.xml`
- `src/epsilon_crane_behavior_tree/behavior_trees/subtree_grip.xml`

### 2. Motion-planning service layer

Package:

- `timber_crane_motion_planning`

Role:

- planner-facing ROS service boundary
- conversion from task requests to joint trajectories and TCP paths
- use of timber collision inputs and robot state

Main services:

- `a2b_movement`
- `grip_traj_movement`

Service contracts:

- `src/timber_crane_planning_interfaces/srv/CalcMovement.srv`
- `src/timber_crane_planning_interfaces/srv/CalcGripMovement.srv`

Reference files:

- `src/timber_crane_motion_planning/src/a2b_ilqr_server.cpp`
- `src/timber_crane_motion_planning/src/grip_traj_server.cpp`
- `src/timber_crane_motion_planning/src/mp_node.cpp`

### 3. Local planning and controller layer

Packages:

- `src/timber_crane_cpp/control/timber_crane_mpc`
- `src/timber_crane_cpp/ilqr/*`
- `src/timber_crane_cpp/planning/*`

Role:

- sway-damped trajectory tracking
- mapping from full-state trajectories to commanded joints
- optional payload and ESDF-aware local control

Runtime interfaces:

- action goals on `/trajectory_controller_a2b/follow_joint_trajectory`
- topic `gripper_state`
- topic `esdf_map`

Reference files:

- `src/timber_crane_cpp/control/timber_crane_mpc/src/jtc_mpc_plugin.cpp`
- `src/timber_crane_cpp/planning/timber_crane_motion_planning_cpp/src/a2b_esdf_server.cpp`

### 4. Payload-state estimation layer

Packages:

- `src/timber_crane_control/epsilon_crane_estimators`
- related ros2_control estimator packages

Role:

- explicit payload-state publication for planning and control
- update of carried-object geometry, mass, and grip-point estimate

Main interface:

- topic `gripper_state`

Reference files:

- `src/timber_crane_control/epsilon_crane_estimators/src/grip_estimator.cpp`
- `src/timber_crane_control/epsilon_crane_control_interfaces/msg/GripperState.msg`

### 5. Perception, mapping, and obstacle layer

Packages:

- `timber_crane_perception`
- `src/timber_crane_cpp/mapping/*`
- `collision_body_handler`

Role:

- publish collision objects
- optional ESDF generation
- keep obstacle representations available to planning and control

Main interfaces:

- topic `/collision_objects`
- topic `esdf_map`

Reference files:

- `src/epsilon_crane_bringup_mp/config/motion_planning/collision_objects.yaml`
- `src/timber_crane_cpp/mapping/timber_crane_mapping_cpp/src/esdf_mapping_node.cpp`

### 6. Bringup and configuration layer

Packages:

- `epsilon_crane_bringup_common`
- `epsilon_crane_bringup_mp`
- `epsilon_crane_bringup_sim`
- HMI and RViz plugin packages

Role:

- assemble robot description, BT executor, planners, estimators, controllers,
  and simulation into one runtime

Reference files:

- `src/epsilon_crane_bringup_sim/launch/gazebo_model_bt.launch.py`
- `src/epsilon_crane_bringup_mp/launch/mp_esdf.launch.py`

## Core Runtime Interfaces To Mirror In CBS

- `/execute_bt`
- `a2b_movement`
- `grip_traj_movement`
- `/trajectory_controller_a2b/follow_joint_trajectory`
- `/controller_manager/switch_controller`
- `gripper_state`
- `/collision_objects`
- `esdf_map` (optional in the commissioning path)

## CBS Reuse Rule

CBS should keep its own operator surface and world model, but it should mirror
the timber structural split:

- CBS BT owns phase sequencing and operator interaction
- CBS motion-planning shell owns the stable planner API
- the timber backend owns trajectory generation and tested execution behavior
- payload state stays explicit on `gripper_state`
- obstacle limitations stay documented when timber is active

That gives CBS the fastest path to a working single-block pipeline while
preserving a future swap to a CBS-native planner behind the same shell.
