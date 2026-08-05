"""Standalone launch file for the DINO proposer debug overlay node.

Diagnostic/visualization only -- intentionally NOT wired into any
production launch file (e.g. `real_wall_assembly_pzs100.launch.py` or
anything under `concrete_block_behavior_tree`). Start it opt-in:

    ros2 launch dino_proposer_debug dino_proposer_overlay.launch.py
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Mirror dino_proposer_overlay_node's own sibling-repo defaults so this
# launch file's --show-args output is truthful. Relies on this workspace's
# default symlink-install so __file__ resolves back into the source tree
# (same assumption the node itself documents/relies on).
_HERE = Path(__file__).resolve()
_CONCRETE_BLOCK_STACK_DIR = _HERE.parents[2]
_DEFAULT_CHECKPOINT_DIR = str(
    _CONCRETE_BLOCK_STACK_DIR / 'blockpose' / 'models' / 'production' / 'dino_proposer'
)
_DEFAULT_BLOCKPOSE_PYTHON_PATH = str(_CONCRETE_BLOCK_STACK_DIR / 'blockpose' / 'python')


def generate_launch_description() -> LaunchDescription:
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/blackfly_rotated/image_rect',
        description='Input sensor_msgs/Image RGB topic to run the proposer on.',
    )
    overlay_topic_arg = DeclareLaunchArgument(
        'overlay_topic',
        default_value='/cbp/debug/dino_proposer_overlay',
        description='Output sensor_msgs/Image topic with proposal boxes drawn on it.',
    )
    checkpoint_dir_arg = DeclareLaunchArgument(
        'checkpoint_dir',
        default_value=_DEFAULT_CHECKPOINT_DIR,
        description='Directory containing dense_head.pt + config.json for dino_proposer.',
    )
    blockpose_python_path_arg = DeclareLaunchArgument(
        'blockpose_python_path',
        default_value=_DEFAULT_BLOCKPOSE_PYTHON_PATH,
        description="Directory added to sys.path so 'import blockpose' resolves.",
    )

    node = Node(
        package='dino_proposer_debug',
        executable='dino_proposer_overlay_node',
        name='dino_proposer_overlay_node',
        output='screen',
        parameters=[
            {
                'image_topic': LaunchConfiguration('image_topic'),
                'overlay_topic': LaunchConfiguration('overlay_topic'),
                'checkpoint_dir': LaunchConfiguration('checkpoint_dir'),
                'blockpose_python_path': LaunchConfiguration('blockpose_python_path'),
            }
        ],
    )

    return LaunchDescription(
        [
            image_topic_arg,
            overlay_topic_arg,
            checkpoint_dir_arg,
            blockpose_python_path_arg,
            node,
        ]
    )
