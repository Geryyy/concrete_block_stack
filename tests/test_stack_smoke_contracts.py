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


def _load_bt_plugins(config_name: str) -> list[str]:
    config_path = BT_ROOT / "config" / config_name
    payload = yaml.safe_load(config_path.read_text())
    return list(payload["/**"]["ros__parameters"]["plugin_lib_names"])


def _custom_nodes_for_tree(tree_name: str) -> set[str]:
    tree_path = BT_ROOT / "behavior_trees" / tree_name
    root = ET.fromstring(tree_path.read_text())
    return {
        elem.tag
        for elem in root.iter()
        if elem.tag not in BUILTIN_TAGS and elem.tag in PLUGIN_BY_NODE
    }


def test_bt_configs_load_only_needed_plugin_libraries() -> None:
    default_plugins = set(_load_bt_plugins("default.yaml"))
    scan_plugins = set(_load_bt_plugins("scan_smoke.yaml"))
    dummy_plugins = set(_load_bt_plugins("dummy_start.yaml"))

    default_nodes = (
        _custom_nodes_for_tree("concrete_block_assembly.xml")
        | _custom_nodes_for_tree("subtree_scene_scan.xml")
        | _custom_nodes_for_tree("subtree_transport_block.xml")
        | _custom_nodes_for_tree("subtree_recovery_scan.xml")
    )
    scan_nodes = _custom_nodes_for_tree("scan_sequence_smoke.xml")
    dummy_nodes = _custom_nodes_for_tree("dummy_start.xml")

    assert default_plugins == {PLUGIN_BY_NODE[node] for node in default_nodes}
    assert scan_plugins == {PLUGIN_BY_NODE[node] for node in scan_nodes}
    assert dummy_nodes == set()
    assert dummy_plugins == set()


def test_bt_configs_use_install_space_tree_paths() -> None:
    for config_name in ("default.yaml", "scan_smoke.yaml", "dummy_start.yaml"):
        config_path = BT_ROOT / "config" / config_name
        payload = yaml.safe_load(config_path.read_text())
        behaviortree = payload["/**"]["ros__parameters"]["behaviortree"]
        assert behaviortree.startswith(
            "install/concrete_block_behavior_tree/share/concrete_block_behavior_tree/behavior_trees/"
        )


def test_stack_entrypoints_and_referenced_files_exist() -> None:
    expected_paths = [
        STACK_ROOT / "concrete_block_behavior_tree" / "launch" / "bt.launch.py",
        STACK_ROOT / "concrete_block_behavior_tree" / "launch" / "scan_sequence_smoke.launch.py",
        STACK_ROOT / "concrete_block_behavior_tree" / "launch" / "sim_wall_build.launch.py",
        STACK_ROOT / "concrete_block_behavior_tree" / "launch" / "sim_wall_build_smoke.launch.py",
        STACK_ROOT / "concrete_block_behavior_tree" / "config" / "default.yaml",
        STACK_ROOT / "concrete_block_behavior_tree" / "config" / "scan_smoke.yaml",
        STACK_ROOT / "concrete_block_perception" / "launch" / "perception.launch.py",
        STACK_ROOT / "concrete_block_motion_planning" / "launch" / "motion_planning.launch.py",
    ]

    for path in expected_paths:
        assert path.exists(), f"Missing expected stack entrypoint: {path}"


def test_plan_docs_distinguish_planner_smoke_from_perception_scan_smoke() -> None:
    cbs_plan = (STACK_ROOT / "cbs_plan.md").read_text()
    tasks_context = (STACK_ROOT / "tasks_context.md").read_text()

    assert "scan_sequence_smoke.launch.py" in cbs_plan
    assert "planner/simulation smoke" in cbs_plan
    assert "scan_sequence_smoke.launch.py" in tasks_context


def test_perception_world_model_defaults_match_staged_workflow() -> None:
    config_path = STACK_ROOT / "concrete_block_perception" / "config" / "world_model.yaml"
    payload = yaml.safe_load(config_path.read_text())
    params = payload["world_model_node"]["ros__parameters"]

    assert params["pipeline_mode"] == "full"
    assert params["perception_mode"] == "IDLE"
