"""
Fig. 7 (paper draft): the Fig. 8 carry in task space -- candidate corridors, the selected
curve and its sway envelope, over the crane line art.

Candidates are crane_planning's own deterministic set for this request (geometry
`cartesian_candidates` + the joint line), tried in order; the first that lifts, fits and
certifies is taken. The tube is `Geometry.envelope` (how far the carried point can swing
inside q_sway_max) around the executed TCP path. Move and scene: carry_move.py.

    python3 examples/fig_planner_move.py out [dpi] [look]
"""

import sys
from pathlib import Path

import numpy as np
import vtk
from cbs_figures import GROUND_Z, Scene
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
geometry = Geometry(model, planner.limits.relaxed_to(start.q_a),
                    planner.prepare_scene(cm.scene(), True), config, cm.TOOL,
                    payload_parameters(payload), None, offset(start.q_a))


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
print(f"{len(candidates) + 1} candidates; envelope {geometry.envelope:.3f} m; "
      f"required clearance {geometry.required:.3f} m")

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

# sway envelope: a tube of radius envelope around the executed TCP path
line = vtk.vtkPolyLineSource()
line.SetNumberOfPoints(len(executed))
for i, p in enumerate(executed):
    line.SetPoint(i, *p)
tube = vtk.vtkTubeFilter()
tube.SetInputConnection(line.GetOutputPort())
tube.SetRadius(geometry.envelope)
tube.SetNumberOfSides(32)
tube.CappingOn()
tri = vtk.vtkTriangleFilter()
tri.SetInputConnection(tube.GetOutputPort())
tri.Update()
s.mesh(tri.GetOutput(), color=pal.WASH, style="ghost")

for c in candidates[1:]:  # corridors tried after the direct line
    s.path(np.array(c.positions_m), color=pal.MUTED, lw=0.5)
s.path(np.array(joint_line), color=pal.MUTED, lw=0.5, ls=(0, (2, 1.5)))
s.path(executed, color=pal.BLUE, lw=1.3)
s.label(start_tcp, "start", offset=(-24, 6))
s.label(cm.PLACE, "goal", offset=(8, 8))
top = max(np.array(c.positions_m)[:, 2].max() for c in candidates)
corner = np.array(candidates[-1].positions_m)[2]
s.label(corner, "candidate corridors", offset=(-30, 16), leader=True)
s.label(executed[len(executed) // 3], "selected: direct tool line", offset=(-34, -26),
        leader=True)
s.label(executed[2 * len(executed) // 3] + [0, 0, -geometry.envelope],
        "sway envelope", offset=(30, -22), leader=True)
AZ = float(sys.argv[4]) if len(sys.argv) > 4 else -34.0
s.camera(azimuth=AZ, elevation=24)
for ext in ("pdf", "svg", "png"):
    s.save(f"{out}/fig_planner_move.{ext}", width="onehalf", aspect=0.62, dpi=dpi)
