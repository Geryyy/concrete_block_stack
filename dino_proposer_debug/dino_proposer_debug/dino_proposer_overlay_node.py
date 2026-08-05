#!/usr/bin/env python3
"""Diagnostic overlay node for blockpose's DINOv2-based region proposer.

Subscribes to a live RGB image topic, runs the already-trained
`dino_proposer` checkpoint (`blockpose.rgb_region_proposer`) on each frame,
draws each proposal's `bbox_xywh` as a box, and republishes the annotated
frame on a debug topic -- purely so a human can eyeball whether the proposer
looks like it is finding real blocks. This is a rough go/no-go signal for a
research direction, NOT a production perception component:

  * Nothing in `concrete_block_perception`/`concrete_block_behavior_tree`
    subscribes to this node's output.
  * This node is never launched from the production stack -- start it
    standalone with `ros2 launch dino_proposer_debug
    dino_proposer_overlay.launch.py`.
  * It must not (and does not) import anything from the production
    detection/planning/BT packages.

Runtime deps: torch/torchvision/transformers are NOT declared via rosdep --
they come from the sibling `blockpose` package's own environment
(`uv sync --extra sam` in `src/concrete_block_stack/blockpose`; see that
package's pyproject.toml `[project.optional-dependencies].sam`). If that
environment isn't importable (missing torch, missing checkpoint, blockpose
not on `sys.path`), this node logs one clear error on the first frame and
then stays alive without publishing -- it does not crash and does not retry
every frame.
"""

from __future__ import annotations

import sys
import traceback
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def _default_blockpose_python_path() -> str:
    """Sibling blockpose repo's `python/` dir (contains the `blockpose` package).

    Assumes this package and `blockpose` are checked out side by side under
    `concrete_block_stack/`, and (for the default to survive a colcon
    install) that the workspace uses symlink-install so `__file__` resolves
    back to the source tree -- both true per this workspace's defaults.
    Override with the `blockpose_python_path` parameter if either changes.
    """
    here = Path(__file__).resolve()
    return str(here.parents[2] / 'blockpose' / 'python')


def _default_checkpoint_dir() -> str:
    """Sibling blockpose repo's production dino_proposer checkpoint dir."""
    here = Path(__file__).resolve()
    return str(here.parents[2] / 'blockpose' / 'models' / 'production' / 'dino_proposer')


class DinoProposerOverlayNode(Node):
    """Overlay blockpose `dino_proposer` region-proposal boxes on a live RGB topic."""

    def __init__(self) -> None:
        super().__init__('dino_proposer_overlay_node')

        self.declare_parameter('image_topic', '/blackfly_rotated/image_rect')
        self.declare_parameter('overlay_topic', '/cbp/debug/dino_proposer_overlay')
        self.declare_parameter('checkpoint_dir', _default_checkpoint_dir())
        self.declare_parameter('blockpose_python_path', _default_blockpose_python_path())
        self.declare_parameter('box_color', [0, 255, 0])
        self.declare_parameter('box_width', 3)

        self._image_topic: str = self.get_parameter('image_topic').value
        self._overlay_topic: str = self.get_parameter('overlay_topic').value
        self._checkpoint_dir: str = self.get_parameter('checkpoint_dir').value
        self._blockpose_python_path: str = self.get_parameter('blockpose_python_path').value
        self._box_color = tuple(self.get_parameter('box_color').value)
        self._box_width: int = int(self.get_parameter('box_width').value)

        self._bridge = CvBridge()
        # Lazy-loaded on the first image callback; see `_ensure_loaded`.
        self._head = None
        self._config = None
        self._extract_region_proposals = None
        self._load_attempted = False
        self._load_ok = False

        self._sub = self.create_subscription(
            Image, self._image_topic, self._on_image, qos_profile_sensor_data
        )
        self._pub = self.create_publisher(Image, self._overlay_topic, 10)

        self.get_logger().info(
            f"dino_proposer_overlay_node up: subscribing '{self._image_topic}', "
            f"publishing overlays on '{self._overlay_topic}'. Diagnostic tool only "
            "-- not consumed by the production stack. Checkpoint model will load "
            "lazily on the first frame."
        )

    # ------------------------------------------------------------------
    def _ensure_loaded(self) -> bool:
        """Import blockpose and load the checkpoint once. Returns success.

        Only ever attempted once: a missing torch/checkpoint/blockpose is a
        stable condition for the lifetime of a diagnostic run, so retrying
        every frame would just spam the log for no benefit.
        """
        if self._load_attempted:
            return self._load_ok
        self._load_attempted = True
        try:
            if self._blockpose_python_path not in sys.path:
                sys.path.insert(0, self._blockpose_python_path)
            from blockpose.rgb_region_proposer import (
                extract_region_proposals,
                load_dino_region_proposer,
            )

            head, config = load_dino_region_proposer(Path(self._checkpoint_dir))
            self._head = head
            self._config = config
            self._extract_region_proposals = extract_region_proposals
            self._load_ok = True
            self.get_logger().info(
                f"loaded dino_proposer checkpoint from '{self._checkpoint_dir}' "
                f"(device={head.device})"
            )
        except Exception:
            self.get_logger().error(
                "failed to load the DINO region proposer -- this node will stay "
                "up but will not publish overlays until restarted. Likely causes: "
                "torch/transformers not installed (run `uv sync --extra sam` in "
                f"blockpose), blockpose not importable from "
                f"'{self._blockpose_python_path}' (see the 'blockpose_python_path' "
                f"parameter), or no checkpoint at '{self._checkpoint_dir}' (see "
                f"the 'checkpoint_dir' parameter).\n{traceback.format_exc()}"
            )
            self._load_ok = False
        return self._load_ok

    def _on_image(self, msg: Image) -> None:
        if not self._ensure_loaded():
            return

        try:
            rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as exc:  # noqa: BLE001 - diagnostic tool: log and skip frame
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')
            return

        try:
            probability_map = self._head.predict_proposal_map(rgb)
            proposals = self._extract_region_proposals(
                probability_map, self._config, provider='dino_proposer_debug_node'
            )
        except Exception:  # noqa: BLE001 - diagnostic tool: log and skip frame
            self.get_logger().error(
                f'dino_proposer inference failed on this frame\n{traceback.format_exc()}'
            )
            return

        if not proposals:
            self.get_logger().debug('dino_proposer: 0 proposals in this frame')

        annotated = self._draw_proposals(rgb, proposals)

        try:
            out_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='rgb8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'cv_bridge re-encode failed: {exc}')
            return
        out_msg.header = msg.header
        self._pub.publish(out_msg)

    def _draw_proposals(self, rgb: np.ndarray, proposals) -> np.ndarray:
        """Draw each proposal's axis-aligned `bbox_xywh` plus its score.

        Pattern matches `experiments/rgb_candidate_prior/render.py::render_overlay`
        (PIL.ImageDraw.rectangle on a copy of the frame) -- adapted from a
        posed-cuboid wireframe to a plain axis-aligned box since a region
        proposal has no pose, only a 2-D bbox + mask.
        """
        from PIL import Image as PILImage, ImageDraw

        image = PILImage.fromarray(np.asarray(rgb, dtype=np.uint8).copy())
        draw = ImageDraw.Draw(image)
        for proposal in proposals:
            x, y, w, h = proposal.bbox_xywh
            draw.rectangle([x, y, x + w - 1, y + h - 1], outline=self._box_color, width=self._box_width)
            draw.text((x, max(0, y - 12)), f'{proposal.score:.2f}', fill=self._box_color)
        return np.asarray(image)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DinoProposerOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
