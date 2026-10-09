"""
Fig. 6 (paper draft): what the world model holds, mid-assembly. Illustrative, not a snapshot.

State of Fig. 11 (e)-(f): `example_wall` course 1 placed, block 4 carried, block 5 staged,
slots 4-5 the plan's goals. Same frames and poses as fig_wall_sequence.py (world -> K0,
tool yaw = block yaw). Dashed: lidar sight lines (the column-mounted lidar re-measures
placed and free blocks; the carried one is FK-tracked between measurements).

    python3 examples/fig_world_model.py out [dpi] [look]
"""

import sys

import numpy as np

from cbs_figures import Scene
from cbs_figures import palette as pal

out = sys.argv[1] if len(sys.argv) > 1 else "out"
dpi = int(sys.argv[2]) if len(sys.argv) > 2 else 600
look = sys.argv[3] if len(sys.argv) > 3 else "lineart"
GRIP, LIFT, YAW = (
    0.85,
    1.6,
    np.pi,
)  # TCP over block centre; carry lift; plan yaw 0 in K0


def k0(x, y, z):
    """Map world -> K0: yaw pi, K0 at (-6.93852, 0.35, 1.1407) in world."""
    return np.array([-(x + 6.93852), -(y - 0.35), z - 1.1407])


STAGED = [
    k0(x, y, 0.3) for x, y in [(-13, 0), (-14, 0), (-15, 0), (-11, -1.5), (-13, -1.5)]
]
SLOTS = [
    k0(x, -3.0, z)
    for x, z in [(-15, 0.3), (-14.08, 0.3), (-13.16, 0.3), (-14.7, 0.9), (-13.78, 0.9)]
]
CARRIED = 3


def tint(hex_, a=0.3):
    """Hue mixed into white: a pale fill that line art still reads."""
    rgb = np.array([int(hex_[k : k + 2], 16) for k in (1, 3, 5)])
    return "#%02X%02X%02X" % tuple(np.round(255 - a * (255 - rgb)).astype(int))


# task status -> fill: categorical hues, pale (the labels name the state too)
STATE = {"FREE": tint(pal.VIOLET), "MOVE": tint(pal.OCHRE), "PLACED": tint(pal.TEAL)}

s = Scene(look=look)
tcp = (STAGED[CARRIED] + SLOTS[CARRIED]) / 2 + [0, 0, GRIP + LIFT]
q = s.robot.reach(
    tcp, yaw=YAW, q0=[np.arctan2(tcp[1], tcp[0]), 0.8, 0.5, 1.0, 0, np.pi / 2, 0, 0]
)
s.ground(margin=10)
s.crane(q)
s.block_in_gripper(q, color=STATE["MOVE"])
for i in range(3):
    s.block((*SLOTS[i], YAW), color=STATE["PLACED"])
for i in (3, 4):
    s.block((*SLOTS[i], YAW), style="ghost")
s.block((*STAGED[4], YAW), color=STATE["FREE"])

lidar = s.link_center("seyond", q)
for target in (SLOTS[1] + [0, 0, 0.3], STAGED[4] + [0, 0, 0.3]):
    s.path(np.array([lidar, target]), color=pal.GRAY, lw=0.5, ls=(0, (3, 2)))

B = r"\mathbf"
held = s.robot.fk(q)["K8_tool_center_point"][:3, 3] - [0, 0, 0.85]
s.label(
    held,
    "grasped: MOVE, FK-tracked\n"
    + rf"${B}{{H}}_0^\mathrm{{B}}={B}{{H}}_0^\mathrm{{T}}({B}{{q}})\,{B}{{H}}_\mathrm{{T}}^\mathrm{{B}}$",
    offset=(30, 18),
    leader=True,
)
s.label(
    SLOTS[4] + [0, 0, 0.3], "wall plan goal (empty slot)", offset=(34, -8), leader=True
)
s.label(
    SLOTS[0] + [0, 0, 0.3], "placed: PLACED, lidar pose", offset=(24, -26), leader=True
)
s.label(STAGED[4] + [0, 0, 0.3], "detected: FREE", offset=(-30, -20), leader=True)
s.label(lidar, r"lidar $\mathcal{K}_\mathrm{L}$", offset=(-30, 14), leader=True)
s.camera(azimuth=-60, elevation=26)  # crane left, wall in front
for ext in ("pdf", "svg", "png"):
    s.save(f"{out}/fig_world_model.{ext}", width="onehalf", aspect=0.62, dpi=dpi)
