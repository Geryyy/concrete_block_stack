"""
Fig. 7 (paper draft): the Fig. 8 carry in task space, candidates and sway bound.

Candidates are crane_planning's own deterministic set for this request (geometry
`cartesian_candidates` + the joint line), tried in order; the first that lifts, fits and
certifies is taken. Clearance (geometry `_decide`): hanging pose >= margins + envelope
passes at once, else the swinging half (below the tip hinge) is swept over the sway box
+-q_sway_max. Drawn: that swinging half ghosted at one hinge's limits, at the goal pose.
Move and scene: carry_move.py.

    python3 examples/fig_planner_move.py out [dpi] [look] [tip|tilt]
"""

import sys
from pathlib import Path

import numpy as np
from cbs_figures import Scene
from cbs_figures import palette as pal

sys.path.insert(0, str(Path(__file__).parent))
import carry_move as cm  # noqa: E402  (puts crane_planning on sys.path)
from crane_model import Frame, Payload  # noqa: E402
from crane_planning.geometry import Geometry, cartesian_candidates  # noqa: E402
from crane_planning.planner import (  # noqa: E402
    PLANNED_INDICES,
    payload_parameters,
    settled_offset,
)

out = sys.argv[1] if len(sys.argv) > 1 else "out"
dpi = int(sys.argv[2]) if len(sys.argv) > 2 else 600
look = sys.argv[3] if len(sys.argv) > 3 else "lineart"
P = list(PLANNED_INDICES)

planner, start, plan, _ = cm.plan_carry()
print(plan.message)
model, config = planner.model, planner.config
payload = Payload(mass_kg=cm.MASS, valid=True)
offset = settled_offset(model, payload, cm.TOOL)
geometry = Geometry(
    model,
    planner.limits.relaxed_to(start.q_a),
    planner.prepare_scene(cm.scene(), True),
    config,
    cm.TOOL,
    payload_parameters(payload),
    None,
    offset(start.q_a),
)


def tcp(q):
    return model.forward_kinematics(q, Frame.MOUNTING_BASE, Frame.TCP).position_m


start_tcp = tcp(start.q)
candidates = cartesian_candidates(geometry, config, start_tcp, cm.PLACE)
# joint line: straight in planned coordinates, passive pair at rest
joint_line = []
for a in np.linspace(0, 1, 40):
    q = plan.q[-1].copy()
    q[P] = (1 - a) * start.q[P] + a * plan.q[-1][P]
    q[[4, 5]] = model.passive_equilibrium(q[[0, 1, 2, 3, 6, 7]])
    joint_line.append(tcp(q))
executed = np.array([tcp(q) for q in plan.q])
print(
    f"{len(candidates) + 1} candidates; envelope {geometry.envelope:.3f} m; "
    f"required clearance {geometry.required:.3f} m"
)

s = Scene(look=look)
k8 = s.robot.fk(plan.q[-1])["K8_tool_center_point"][:3, 3]
print(f"TCP planner vs figure robot: {np.linalg.norm(k8 - executed[-1]) * 1e3:.1f} mm")
s.ground()
s.crane(start.q, style="ghost")
s.crane(plan.q[-1])
s.block_in_gripper(plan.q[-1], color="block_active")
for x, y in cm.STAGED:
    s.block((x, y, cm.BLOCK_Z, np.pi))
s.block((*cm.PLACE[:2], cm.BLOCK_Z, np.pi), style="ghost")  # slot c0_b0, still empty

HINGE = {"tip": (4, "theta6_tip_joint"), "tilt": (5, "theta7_tilt_joint")}
hinge = sys.argv[4] if len(sys.argv) > 4 else "tip"
k, joint = HINGE[hinge]
bound = float(config.q_sway_max[k - 4])
goal_q = plan.q[-1]

for c in candidates[1:]:  # corridors tried after the direct line
    s.path(np.array(c.positions_m), color=pal.MUTED, lw=0.5)
s.path(np.array(joint_line), color=pal.MUTED, lw=0.5, ls=(0, (2, 1.5)))
s.path(executed, color=pal.BLUE, lw=1.3)
s.label(start_tcp, "start", offset=(-26, 10), leader=True)
s.label(cm.PLACE, "goal", offset=(26, 18), leader=True)
corner = np.array(candidates[-1].positions_m)[2]
s.label(corner, "candidate corridors", offset=(-30, 16), leader=True)
s.label(
    executed[len(executed) // 3],
    "selected: direct tool line",
    offset=(-34, -26),
    leader=True,
)
s.label(executed[-1] + [0, 0, -1.2], "(b)", offset=(26, -20), leader=True)
s.camera(azimuth=-34, elevation=28)  # side-on to the move
for ext in ("pdf", "svg", "png"):
    s.save(f"{out}/fig_planner_move.{ext}", width="onehalf", aspect=0.62, dpi=dpi)

# (b) detail: the swinging half (below the tip hinge) at goal, seen along the hinge axis,
# at rest and at the hinge's sway limits -- what the sweep certifies against the scene
d = Scene(look=look)
d.crane(goal_q, below="theta6_tip_joint")
d.block_in_gripper(goal_q, color="block_active")
for sign in (-1, 1):
    q = goal_q.copy()
    q[k] += sign * bound
    d.crane(q, style="ghost", below="theta6_tip_joint")
    d.block_in_gripper(q, style="ghost")
pivot, axis = d.robot.joint_frame(joint, goal_q)
low = d.robot.fk(goal_q)["K8_tool_center_point"][:3, 3] - [0, 0, 1.15]  # block bottom
lever = low - pivot
th = np.linspace(-bound, bound, 40)[:, None]  # Rodrigues about the hinge axis
arc = (
    pivot
    + lever * np.cos(th)
    + np.cross(axis, lever) * np.sin(th)
    + axis * (axis @ lever) * (1 - np.cos(th))
)
d.path(arc, color=pal.INK, lw=0.6)
for end in (arc[0], low, arc[-1]):
    d.path(np.array([pivot, end]), color=pal.INK, lw=0.5, ls=(0, (3, 2)))
d.label(pivot, f"{hinge} hinge", offset=(18, 8), leader=True)
d.label(arc[-1], rf"$\pm${np.degrees(bound):.1f}°", offset=(4, -8))
view = (
    axis
    if axis @ np.array([np.cos(np.radians(-34)), np.sin(np.radians(-34)), 0]) > 0
    else -axis
)
d.camera(
    azimuth=np.degrees(np.arctan2(view[1], view[0])),
    elevation=np.degrees(np.arcsin(np.clip(view[2], -1, 1))),
    ortho=True,
    margin=0.12,
)
for ext in ("pdf", "svg", "png"):
    d.save(f"{out}/fig_planner_move_b.{ext}", width=55 / 25.4, aspect=1.25, dpi=dpi)
