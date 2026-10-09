"""
The one carry move Figs. 7 and 8 show, planned by the deployed crane_planning Planner offline.

First `example_wall` carry: block 1 from its pickup approach to slot c0_b0's place approach
(wall_plan_server.yaml: K0 z 3.0 -> 2.7 m; T_world_K0 as in fig_wall_sequence), tool yaw =
block yaw + 90 deg, 800 kg (wall_assembly.xml). Scene: the four other staged blocks as boxes
(world_model_seed_pick_place.yaml), wall still empty. The start pose is the end of a plan to
the pickup approach, pinned (START_Q): derived from plan_example's default start, it
moved with that constant (231: 14 -> 39 SQP it.). Needs ROS sourced (pinocchio, acados).
"""

import sys
import time
from pathlib import Path

import numpy as np

PLANNING = Path(__file__).resolve().parents[2] / "crane_planning"
sys.path[:0] = [
    str(PLANNING / "scripts"),
    str(PLANNING),
    str(PLANNING.parent / "crane_model"),
]
import pinocchio as pin  # noqa: E402
import plan_example as pe  # noqa: E402
from crane_model import CollisionPrimitive, Payload  # noqa: E402
from crane_planning import Planner, PlannerConfig, Start  # noqa: E402

PICK, PLACE = np.array([6.06, 0.35, 3.0]), np.array([8.06, 3.35, 2.7])  # TCP in K0, m
YAW, MASS, TOOL = -np.pi / 2, 800.0, 0.3
BLOCK = np.array([0.9, 0.6, 0.6])
STAGED = [
    (7.06, 0.35),
    (8.06, 0.35),
    (4.06, 1.85),
    (6.06, 1.85),
]  # blocks 2..5, K0 x, y
#: carry start: end of the plan to PICK from plan_example.DEFAULT_START as of 231 (q1..q8)
START_Q = (0.057692, 0.937413, 0.433722, 0.594973, 0.202839, 1.563845, 1.628488, TOOL)
BLOCK_Z = -0.8407  # K0 z of a ground block's centre (world 0.3 m)


def scene() -> list:
    return [
        CollisionPrimitive(
            id=f"block_{i + 1}",
            shape="box",
            pose_in_mounting_base=pin.SE3(np.eye(3), np.array([x, y, BLOCK_Z])),
            dimensions_m=BLOCK.copy(),
        )
        for i, (x, y) in enumerate(STAGED)
    ]


def plan_carry():
    """(planner, start Start, plan, seconds of the carry call)."""
    planner = Planner(pe.description(), PlannerConfig())
    start = Start(q=np.array(START_Q), dq_a=np.zeros(5))
    began = time.monotonic()
    plan = planner.plan(
        start,
        PLACE,
        YAW,
        payload=Payload(mass_kg=MASS, valid=True),
        scene=scene(),
        avoid_collisions=True,
    )
    return planner, start, plan, time.monotonic() - began
