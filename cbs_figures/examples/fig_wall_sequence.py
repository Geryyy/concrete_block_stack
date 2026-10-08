"""
Fig. 11 (paper draft): wall assembly sequence, six frames, Blender line art.

Plan `example_wall` (concrete_block_assembly_planning/config/wall_plans.yaml) on the
staging seed world_model_seed_pick_place.yaml, both in `world`; mapped to K0 with
T_world_K0 (blockpose/config/transforms_pzs100_initial_pose_1.yaml). Empty plan slots are
ghosts, the carried block is `block_active`. Tool yaw = block yaw: the jaws close across
the 0.6 m sides (TCP_TO_BLOCK).

    python3 examples/fig_wall_sequence.py out [dpi] [look]   -> fig_wall_sequence_{a..f}
"""

import sys

import matplotlib.pyplot as plt
import numpy as np
from cbs_figures import BLOCK, Scene
from cbs_figures import palette as pal

out = sys.argv[1] if len(sys.argv) > 1 else "out"
dpi = int(sys.argv[2]) if len(sys.argv) > 2 else 600
look = sys.argv[3] if len(sys.argv) > 3 else "lineart"
H, GRIP, LIFT = BLOCK["size"][2], 0.85, 1.6  # m; GRIP = TCP above block centre


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
YAW = np.pi  # plan yaw 0 in world = pi in K0

# frame: (blocks placed, carried block index or None, TCP at "pick" | "carry" | "place" | "park")
FRAMES = {
    "a": (0, None, "park", "plan and staged blocks"),
    "b": (0, 0, "pick", "grip block 1"),
    "c": (0, 0, "carry", "carry"),
    "d": (0, 0, "place", "place course 1"),
    "e": (3, 3, "place", "place course 2"),
    "f": (5, None, "park", "wall complete"),
}
PARK = k0(-11.5, -1.0, 2.6)
crane_cache = {}


def pose(i, where):
    a, b = STAGED[i] + [0, 0, GRIP], SLOTS[i] + [0, 0, GRIP]
    return {"pick": a, "place": b, "carry": (a + b) / 2 + [0, 0, LIFT], "park": PARK}[
        where
    ]


def solve(p):
    key = tuple(np.round(p, 3))
    if key not in crane_cache:
        q0 = [
            np.arctan2(p[1], p[0]),
            0.8,
            0.5,
            1.0,
            0,
            np.pi / 2,
            0,
            0,
        ]  # boom up, arm out
        crane_cache[key] = s.robot.reach(p, yaw=YAW, q0=q0)
    return crane_cache[key]


panels = []
for name, (placed, carried, where, caption) in FRAMES.items():
    s = Scene(look=look)
    q = solve(pose(carried or 0, where))
    s.ground(margin=10)  # cad look only; line art has no ground
    s.crane(q)
    for i, (staged, slot) in enumerate(zip(STAGED, SLOTS)):
        if i < placed:
            s.block((*slot, YAW))
        elif i == carried and where != "pick":
            s.block_in_gripper(q, color="block_active")
            s.block((*slot, YAW), style="ghost")
        else:
            color = "block_active" if i == carried else "block"
            s.block((*staged, YAW), color=color)
            s.block((*slot, YAW), style="ghost")
    s.camera(azimuth=-125, elevation=26, target=np.array([4.0, 1.7, 0.4]), distance=26)
    path = f"{out}/fig_wall_sequence_{name}"
    for ext in ("svg", "png"):
        s.save(f"{path}.{ext}", width=62 / 25.4, aspect=0.72, dpi=dpi, fit=False)
    panels.append((name, caption, f"{path}.png"))

# composite preview for the wiki; the paper sets the six panels as subfigures
pal.use()
fig, axs = plt.subplots(2, 3, figsize=(pal.WIDTH["double"], 2 * 62 * 0.72 / 25.4 + 0.4))
for ax, (name, caption, png) in zip(axs.flat, panels):
    ax.imshow(plt.imread(png))
    ax.set_title(f"({name}) {caption}", fontsize=8)
    ax.axis("off")
fig.savefig(f"{out}/fig_wall_sequence.png", dpi=min(dpi, 300))
