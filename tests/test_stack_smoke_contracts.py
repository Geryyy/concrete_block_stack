from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


STACK_ROOT = Path(__file__).resolve().parents[1]
BT_ROOT = STACK_ROOT / "concrete_block_behavior_tree"

PLUGIN_BY_NODE = {
    "RunPoseEstimation": "BT_cb_run_pose_estimation_action",
    "PlanGeometricPath": "BT_cb_plan_geometric_path_action",
    "PlanAndComputeTrajectory": "BT_cb_plan_and_compute_trajectory_action",
    "ComputeTrajectory": "BT_cb_compute_trajectory_action",
    "ExecuteTrajectory": "BT_cb_execute_trajectory_action",
    "MoveToNamedConfiguration": "BT_cb_move_to_named_configuration_action",
    "GetNextAssemblyTask": "BT_cb_get_next_assembly_task_action",
    "SetBlockTaskStatus": "BT_cb_set_block_task_status_action",
    "GripperAction": "BT_cb_gripper_action",
}

BUILTIN_TAGS = {
    "root",
    "include",
    "BehaviorTree",
    "Sequence",
    "Fallback",
    "KeepRunningUntilFailure",
    "AlwaysSuccess",
    "SubTree",
}


def _load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text())


def _load_bt_plugins(config_relpath: str) -> list[str]:
    config_path = BT_ROOT / "config" / config_relpath
    payload = _load_yaml(config_path)
    return list(payload["/**"]["ros__parameters"]["plugin_lib_names"])


def _custom_nodes_for_tree(tree_name: str) -> set[str]:
    tree_path = BT_ROOT / "behavior_trees" / tree_name
    root = ET.fromstring(tree_path.read_text())
    return {
        elem.tag
        for elem in root.iter()
        if elem.tag not in BUILTIN_TAGS and elem.tag in PLUGIN_BY_NODE
    }


def test_bt_mode_configs_match_canonical_workflows() -> None:
    assembly_plugins = set(_load_bt_plugins("bt_assembly.yaml"))
    operator_plugins = set(_load_bt_plugins("bt_operator.yaml"))

    assembly_nodes = (
        _custom_nodes_for_tree("concrete_block_assembly.xml")
        | _custom_nodes_for_tree("subtree_scene_scan.xml")
        | _custom_nodes_for_tree("subtree_transport_block.xml")
        | _custom_nodes_for_tree("subtree_recovery_scan.xml")
    )
    dummy_nodes = _custom_nodes_for_tree("dummy_start.xml")
    expected_operator_plugins = {
        "BT_wait_goal_pose_decorator",
        "BT_get_user_approval_decorator",
        "BT_check_user_approval_condition",
        "BT_optimize_target_angle_bt_node",
        "BT_cb_prepare_move_empty_request_action",
        "BT_cb_plan_and_compute_trajectory_action",
        "BT_cb_execute_trajectory_action",
        "BT_cb_get_next_assembly_task_action",
    }

    assert assembly_plugins == {PLUGIN_BY_NODE[node] for node in assembly_nodes}
    assert operator_plugins == expected_operator_plugins
    assert dummy_nodes == set()


def test_bt_profile_configs_use_install_space_tree_paths() -> None:
    config_names = (
        "profiles/move_empty.yaml",
        "profiles/single_block_plan.yaml",
        "profiles/single_block_execute.yaml",
        "profiles/assembly.yaml",
        "dummy_start.yaml",
    )
    for config_name in config_names:
        config_path = BT_ROOT / "config" / config_name
        payload = _load_yaml(config_path)
        behaviortree = payload["/**"]["ros__parameters"]["behaviortree"]
        assert behaviortree.startswith(
            "install/concrete_block_behavior_tree/share/concrete_block_behavior_tree/behavior_trees/"
        )


def test_legacy_bt_alias_configs_point_to_canonical_entrypoints() -> None:
    expected = {
        "move_empty_shared.yaml": "move_empty.xml",
        "phase1_timber_backend.yaml": "move_empty.xml",
        "scan_smoke.yaml": "single_block_plan.xml",
        "single_block_planning_probe.yaml": "single_block_plan.xml",
    }

    for config_name, tree_name in expected.items():
        payload = _load_yaml(BT_ROOT / "config" / config_name)
        behaviortree = payload["/**"]["ros__parameters"]["behaviortree"]
        assert behaviortree.endswith(f"/{tree_name}")


def test_stack_entrypoints_and_referenced_files_exist() -> None:
    expected_paths = [
        STACK_ROOT / "concrete_block_behavior_tree" / "launch" / "bt.launch.py",
        STACK_ROOT / "concrete_block_behavior_tree" / "launch" / "scan_sequence_smoke.launch.py",
        STACK_ROOT / "concrete_block_behavior_tree" / "launch" / "sim_wall_build.launch.py",
        STACK_ROOT / "concrete_block_behavior_tree" / "launch" / "sim_wall_build_smoke.launch.py",
        STACK_ROOT / "concrete_block_behavior_tree" / "config" / "bt_common.yaml",
        STACK_ROOT / "concrete_block_behavior_tree" / "config" / "bt_operator.yaml",
        STACK_ROOT / "concrete_block_behavior_tree" / "config" / "bt_assembly.yaml",
        STACK_ROOT / "concrete_block_behavior_tree" / "config" / "profiles" / "move_empty.yaml",
        STACK_ROOT / "concrete_block_behavior_tree" / "config" / "profiles" / "single_block_plan.yaml",
        STACK_ROOT / "concrete_block_behavior_tree" / "config" / "profiles" / "single_block_execute.yaml",
        STACK_ROOT / "concrete_block_perception" / "launch" / "perception.launch.py",
        STACK_ROOT / "concrete_block_motion_planning" / "launch" / "motion_planning.launch.py",
    ]

    for path in expected_paths:
        assert path.exists(), f"Missing expected stack entrypoint: {path}"


def test_plan_docs_distinguish_single_block_probe_from_legacy_scan_alias() -> None:
    cbs_plan = (STACK_ROOT / "cbs_plan.md").read_text()
    tasks_context = (STACK_ROOT / "tasks_context.md").read_text()

    assert "single_block_plan.xml" in cbs_plan
    assert "scan_sequence_smoke.launch.py" in cbs_plan
    assert "legacy alias" in cbs_plan
    assert "Single block plan" in tasks_context
    assert "scan_sequence_smoke.launch.py" in tasks_context


def test_perception_world_model_defaults_match_staged_workflow() -> None:
    config_path = STACK_ROOT / "concrete_block_perception" / "config" / "world_model.yaml"
    payload = yaml.safe_load(config_path.read_text())
    params = payload["world_model_node"]["ros__parameters"]

    assert params["pipeline_mode"] == "full"
    assert params["perception_mode"] == "IDLE"


def test_seeded_world_model_overlay_defines_static_b0_anchor() -> None:
    config_path = (
        STACK_ROOT
        / "concrete_block_perception"
        / "config"
        / "world_model_seed_b0.yaml"
    )
    payload = yaml.safe_load(config_path.read_text())
    initial_blocks_yaml = payload["world_model_node"]["ros__parameters"]["world_model"][
        "initial_blocks"
    ]
    seeded = yaml.safe_load(initial_blocks_yaml)

    assert len(seeded) == 1
    block = seeded[0]
    assert block["id"] == "B0"
    assert block["frame_id"] == "world"
    assert block["pose_status"] == "POSE_COARSE"
    assert block["task_status"] == "TASK_PLACED"


def test_gazebo_bt_launch_exposes_seed_world_model_commissioning_toggle() -> None:
    launch_path = BT_ROOT / "launch" / "gazebo_model_bt.launch.py"
    text = launch_path.read_text()

    assert "seed_world_model" in text
    assert "world_model_seed_b0.yaml" in text


def test_gazebo_bt_launch_uses_composable_operator_bt_profiles() -> None:
    launch_path = BT_ROOT / "launch" / "gazebo_model_bt.launch.py"
    text = launch_path.read_text()

    assert "bt_common_params_file" in text
    assert "bt_mode_params_file" in text
    assert "bt_profile_params_file" in text
    assert "bt_operator.yaml" in text
    assert '"profiles"' in text
    assert '"move_empty.yaml"' in text
