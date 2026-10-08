"""Pick a block from the ground, carry it, stack it: ghosts for earlier poses, TCP path."""

import sys

import numpy as np
from cbs_figures import BLOCK, GROUND_Z, Scene
from cbs_figures import palette as pal

out = sys.argv[1] if len(sys.argv) > 1 else "out"
s = Scene(look=sys.argv[2] if len(sys.argv) > 2 else "cad")
r = s.robot
h = BLOCK["size"][2]
grip = 0.85  # TCP above block centre when gripped, = TCP_TO_BLOCK
pick = np.array([5.5, -2.5, GROUND_Z + h / 2 + grip])
place = np.array([6.0, 2.5, GROUND_Z + 1.5 * h + grip])  # second course
lift = 1.2


def tcp_at(t):  # smooth lift-carry-lower, illustration only
    return pick + (place - pick) * t + [0, 0, lift * np.sin(np.pi * t)]


q_pick = r.reach(pick, yaw=0.0)
q_mid = r.reach(tcp_at(0.5), yaw=0.3, q0=q_pick)
q_place = r.reach(place, yaw=0.6, q0=q_mid)

s.ground()
s.block_on_ground(pick[0], pick[1], 0.0, style="ghost")
s.block_on_ground(place[0], place[1], 0.6)
for q in (q_pick, q_mid):
    s.crane(q, style="ghost")
s.crane(q_place)
s.block_in_gripper(q_place, color="block_active")
s.path([tcp_at(t) for t in np.linspace(0, 1, 40)], color=pal.BLUE, label="TCP")
s.camera(azimuth=-30, elevation=25)
for ext in ("pdf", "png"):
    s.save(f"{out}/pickup_sequence_{s.look}.{ext}", width="onehalf", aspect=0.7)
