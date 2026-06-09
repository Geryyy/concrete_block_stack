# concrete_block_stack

ROS 2 stack for autonomous concrete-block wall assembly with the Epsilon timber crane.
Five packages glue together perception, a world model, motion planning, and a behavior
tree on top of the upstream `timber_crane_*` and `epsilon_crane_*` packages.

## Architecture

```
sensors (ZED2i camera, Seyond lidar)
   │
   ▼
PERCEPTION  (concrete_block_perception, ros2_yolos_cpp)
   • segmentor service          (yolos_cpp)
   • block_detection_tracking_node
   • block_registration_node
   │   internal pipeline
   ▼
WORLD MODEL  (block_world_model_node — concrete_block_world_model)
   • read services: ~/get_coarse_blocks, ~/get_planning_scene, ~/run_pose_estimation
   • write services: ~/upsert_block, ~/set_block_task_status, ~/set_mode
   • viz topics:    block_world_model, block_world_model_markers
   │   pull queries
   ▼                                              ▲ direct write of task_status
MOTION PLANNING  (concrete_block_motion_planning) │
   • wall_plan_server.py    →  ~/get_next_assembly_task
   • grip_traj_server_simple.py → grip_traj_movement (IK + cosine traj)
   │   pull queries                               │
   ▼                                              │
BEHAVIOR TREE  (concrete_block_behavior_tree)─────┘
   Trees:    basic_pick_and_place.xml, wall_assembly.xml
   Subtree:  subtree_pick_and_place_block.xml  (the primitive skill)
   Plugins:  GetNextAssemblyTask, SetBlockTaskStatus, SetPlaceApproachPose
   Calls:    motion planning, world model, joint trajectory controller
```

**Contract:** state queries use **services** (pull). The two `block_world_model*` topics
are visualization-only — never subscribe to them for state.

## Packages

| Package | Role |
|---|---|
| `concrete_block_world_model_interfaces` | Pure msg/srv definitions for the world model API. No nodes. |
| `concrete_block_perception` | Detection, tracking, registration providers. |
| `concrete_block_perception_interfaces` | Pure msg/srv/action definitions for perception providers. |
| `concrete_block_world_model` | Hosts `world_model_node` and owns persistent block world state. |
| `concrete_block_motion_planning` | Wall plan progress, IK, gripper trajectory generation. |
| `concrete_block_behavior_tree` | BT XMLs and action plugins. |
| `ros2_yolos_cpp` | Vendored YOLO inference wrapper (segmentor service used by world model). |

## Build

```bash
# From workspace root
colcon build --packages-up-to concrete_block_behavior_tree
source install/setup.bash
```

The interfaces package builds first; cbP / cbMP / cbBT depend on it.

## Run

Simulation, basic pick-and-place test (single block):

```bash
ros2 launch concrete_block_behavior_tree gazebo_basic_pick_and_place_pzs100.launch.py
```

Simulation, full wall assembly loop:

```bash
ros2 launch concrete_block_behavior_tree gazebo_wall_assembly_pzs100.launch.py
```

Perception bringup only (real hardware):

```bash
ros2 launch concrete_block_perception perception.launch.py
```

## Test

```bash
# C++ unit tests for world model utils
colcon test --packages-select concrete_block_perception
colcon test-result --verbose

# Dependency smoke (pinocchio + casadi + acados + open3d)
python3 -m pytest tests/test_dependency_smoke.py -v
```

## Behavior tree

The BT primitive lives in `concrete_block_behavior_tree/behavior_trees/subtree_pick_and_place_block.xml`
(9-step sequence: approach → open → descend → close → lift → approach_place → descend → open → lift).
Both `basic_pick_and_place.xml` (hardcoded coords) and `wall_assembly.xml` (loop with
`GetNextAssemblyTask`) re-use that subtree.

Edit XMLs directly for parameter tweaks; for structural changes (add/remove steps,
fallbacks, recovery branches) use Groot v1:

```bash
source /opt/groot_ws/install/setup.bash
ros2 run groot Groot
```

## Interface map (services)

| Service | Owner | Type |
|---|---|---|
| `~/get_coarse_blocks`, `~/get_planning_scene` | block_world_model_node | `concrete_block_world_model_interfaces/srv/...` |
| `~/set_block_task_status`, `~/upsert_block`, `~/set_mode`, `~/run_pose_estimation` | block_world_model_node | `concrete_block_world_model_interfaces/srv/...` |
| `~/get_next_assembly_task` | concrete_block_motion_planning_node | `concrete_block_motion_planning/srv/GetNextAssemblyTask` |
| `grip_traj_movement` | grip_traj_server | `timber_crane_planning_interfaces/srv/CalcGripMovement` |
| `register_block_pose` (+ `register_block` action) | block_registration_node | `concrete_block_perception_interfaces/srv/RegisterBlock` |
| `~/track` | block_detection_tracking_node | `concrete_block_perception_interfaces/srv/TrackDetections` |
| `/yolos_segmentor_service/segment` | yolos segmentor | `ros2_yolos_cpp/srv/SegmentImage` |

## Design notes

- **Pull > push for state.** The world model is the source of truth for blocks. Consumers
  (motion planning, BT) call services on demand instead of subscribing — avoids the
  "last message wins" race when multiple consumers want a consistent snapshot.
- **PerceptionOrchestratorNode owns both pipeline and state.** Single node by intent —
  perception has the freshest data, so co-locating the model avoids extra IPC. Split
  only if the perception pipeline ever needs to be a swappable component.
- **BT writes task status directly to the world model.** The wall plan server observes
  progress via `~/get_coarse_blocks`; it doesn't intercept BT writebacks.
- **Trajectory generation is timber-compatible.** `grip_traj_server_simple.py` uses
  the timber `CalcGripMovement` srv so the same BT works against the timber stack and
  this concrete-block stack.
