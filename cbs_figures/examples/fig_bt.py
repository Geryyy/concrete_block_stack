"""
Fig. 10 (paper draft): pick-and-place behaviour tree, simplified from the deployed XML.

concrete_block_behavior_tree/behavior_trees: wall_assembly.xml (loop) ->
subtree_pick_and_place_block.xml -> subtree_pick_block.xml / subtree_place_block.xml.
Blackboard plumbing (SetBlackboard, Script) and ForceSuccess wrappers are dropped; each
drawn node names the XML IDs it stands for, and every one must still be in the XML.

    python3 examples/fig_bt.py out
"""

import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle

from cbs_figures import palette as pal

out = Path(sys.argv[1] if len(sys.argv) > 1 else "out")
out.mkdir(exist_ok=True)
XML = (
    Path(__file__).resolve().parents[2] / "concrete_block_behavior_tree/behavior_trees"
)
TEXT = "".join(p.read_text() for p in XML.glob("*.xml"))

# fills by what a leaf drives; white = gripper / world-model bookkeeping
A2B = "#C9DDEA"  # crane_planning move, MPC-driven
GRIP_MOVE = (
    "#EBF3F8"  # grip-trajectory move: MPC-driven carrying, controller alone when empty
)
SENSE = "#F6EDD6"  # lidar / gripper measurement
FILL = {"m": A2B, "g": GRIP_MOVE, "s": SENSE, "": "w"}


def seq(label, *kids, refs=()):
    return ("seq", label, refs, kids)


def fb(label, *kids, refs=()):
    return ("fb", label, refs, kids)


def deco(sym, label, kid, refs=()):
    return ("deco", f"{sym} {label}", refs, (kid,))


def act(label, role="", refs=()):
    return ("act", label, refs, role)


def cond(label, refs=()):
    return ("cond", label, refs, "")


PICK = seq(
    "pick",
    act("move above pickup", "m", ["SubTreeMoveAbove", "CalcA2BMovement"]),
    act(
        "refine target, correct pickup", "s", ["REFINE_BLOCK", "EvaluatePickupResidual"]
    ),
    act("open gripper", "", ["SubTreePickFromHover", "SubTreeGripper"]),
    act("descend", "g", ["SubTreeDescendTo"]),
    act("close gripper", "", ["SubTreeGripper"]),
    cond("grasp detected", ["WaitForGraspSignal"]),
    act(
        "grasp offset, mark MOVE", "", ["CaptureBlockGraspOffset", "SetBlockTaskStatus"]
    ),
    act("set MPC payload", "", ["SetMpcPayload"]),
    act("lift", "g", ["SubTreeLift"]),
    act("refine grasped, tilt", "s", ["REFINE_GRASPED", "MeasureBlockFromGripper"]),
    refs=["SubTreePickBlock"],
)
GRIP = fb(
    "grip check",
    cond("tilt < 3°", ["grip_tilt &lt; 0.0524"]),
    seq(
        "regrip",
        act("set down", "m", ["SubTreeSetDown"]),
        act("pick again", "m", ["SubTreePickBlock"]),
        refs=["SubTreeRegrip"],
    ),
)
APPROACH = deco(
    "↺",
    "retry ×2",
    fb(
        "approach or back off",
        seq(
            "approach",
            act("carry above slot", "m", ["SubTreeMoveAbove"]),
            act("waypoint 0.3 m, open side", "g", ["place_z + 0.30"]),
            deco(
                "↻",
                "in air ×3",
                seq(
                    "",
                    act("refine grasped", "s", ["REFINE_GRASPED"]),
                    act("measure block, 2.7 s", "s", ['window_s="2.7"']),
                    act("descend, corrected", "g", ["off_x + miss_x"]),
                ),
                refs=['Repeat num_cycles="3"'],
            ),
            refs=["SubTreePlaceApproach"],
        ),
        act("lift, fail", "g", ["ForceFailure", "SubTreeLift"]),
    ),
    refs=['RetryUntilSuccessful" num_attempts="2"'],
)
FINISH = seq(
    "finish",
    act("angled entry, 5 cm", "g", ["place_z + 0.05"]),
    act("touchdown until unloaded", "g", ["SubTreeTouchdown", "WaitForLoadOff"]),
    act("write placed pose", "", ["WriteBlockPoseFromGripper"]),
    act("open gripper", "", ["SubTreeGripper"]),
    act("clear MPC payload", "", ['carries="false"']),
    act("lift away", "g", ["SubTreeLift"]),
    act("re-measure placed block", "s", ["REFINE_BLOCK"]),
    refs=["SubTreePlaceFinish"],
)
PLACE = seq("place", APPROACH, FINISH, refs=["SubTreePlaceBlock"])
NEXT = act("next task", "", ["GetNextAssemblyTask"])
ROOT = (
    "↻ assembly loop",
    ["KeepRunningUntilFailure", "SubTreePickAndPlaceBlock"],
    [NEXT, PICK, GRIP, PLACE],
)


def check(node):
    kind, _, refs, rest = node
    missing = [r for r in refs if r not in TEXT]
    assert not missing, f"{node[1]!r}: {missing} no longer in {XML}"
    if kind in ("seq", "fb", "deco"):
        for k in rest:
            check(k)


for r in ROOT[1]:
    assert r in TEXT, r
for k in ROOT[2]:
    check(k)

# --- drawing: root bus on top, each child of the per-block sequence as an outline ---
W, H = 190, 97  # mm
PITCH, NH, IND = 5.0, 3.8, 3.6  # row pitch, node height, indent
COLX = [4, 24, 70, 100]  # column left edges: next task, pick, grip check, place
SUBX = [104, 152]  # place: approach, finish
TOP = H - 22  # first outline row

pal.use()
fig = plt.figure(figsize=(W / 25.4, H / 25.4))
ax = fig.add_axes((0, 0, 1, 1))
ax.set_xlim(0, W)
ax.set_ylim(0, H)
ax.axis("off")
ren = fig.canvas.get_renderer()
LINE = dict(color=pal.INK, lw=0.5, zorder=1)


def text_w(s, size=6.5):
    t = ax.text(0, 0, s, fontsize=size)
    w = t.get_window_extent(ren).width / fig.dpi * 25.4
    t.remove()
    return w


def control(x, y, kind, label):
    """Square with the BT symbol; label to its right (gray for an unnamed node)."""
    sym = {"seq": "→", "fb": "?"}[kind] if kind != "deco" else label[0]
    ax.add_patch(
        Rectangle((x, y - NH / 2), NH, NH, fc="w", ec=pal.INK, lw=0.6, zorder=2)
    )
    ax.text(x + NH / 2, y, sym, ha="center", va="center", fontsize=6.5, zorder=3)
    name = label if kind != "deco" else label[2:]
    if name:
        ax.text(
            x + NH + 1.0,
            y,
            name,
            ha="left",
            va="center",
            fontsize=6.5,
            style="italic",
            color=pal.GRAY,
        )
    return x + NH + 1.0 + (text_w(name) if name else 0)


def leaf(x, y, kind, label, role):
    w = text_w(label) + 2.4
    style = (
        "round,pad=0,rounding_size=1.9"
        if kind == "cond"
        else "round,pad=0,rounding_size=0.8"
    )
    ax.add_patch(
        FancyBboxPatch(
            (x, y - NH / 2),
            w,
            NH,
            boxstyle=style,
            fc=FILL[role],
            ec=pal.INK,
            lw=0.6,
            zorder=2,
        )
    )
    ax.text(x + 1.2, y, label, ha="left", va="center", fontsize=6.5, zorder=3)
    return x + w


def outline(node, x, y):
    """Draw node at (x, y); children below, indented, on an elbow rail. Returns next y."""
    kind, label, _, rest = node
    if kind in ("act", "cond"):
        leaf(x, y, kind, label, rest)
        return y - PITCH
    control(x, y, kind, label)
    rail, ny, last = x + NH / 2, y - PITCH, y
    for k in rest:
        last = ny
        ax.plot([rail, x + IND], [ny, ny], **LINE)
        ny = outline(k, x + IND, ny)
    ax.plot([rail, rail], [y - NH / 2, last], **LINE)
    return ny


# root and the per-block sequence
rx = W / 2 - 18
control(rx, H - 4, "deco", ROOT[0])
ax.text(
    rx + NH + 1.0 + text_w("assembly loop") + 1.5,
    H - 4,
    "until the plan is complete",
    ha="left",
    va="center",
    fontsize=6,
    color=pal.GRAY,
)
control(rx, H - 11, "seq", "one block")
ax.plot([rx + NH / 2] * 2, [H - 4 - NH / 2, H - 11 + NH / 2], **LINE)
bus = H - 16
ax.plot([rx + NH / 2] * 2, [H - 11 - NH / 2, bus], **LINE)
ax.plot([COLX[0] + 2, COLX[-1] + 2], [bus, bus], **LINE)
for x, node in zip(COLX, ROOT[2]):
    ax.plot([x + 2, x + 2], [bus, TOP + NH / 2], **LINE)
    if node is not PLACE:
        outline(node, x, TOP)
# place: its two children side by side, on a bus of their own
control(COLX[3], TOP, "seq", "place")
sub = TOP - PITCH - 0.5
ax.plot([COLX[3] + NH / 2] * 2, [TOP - NH / 2, sub], **LINE)
ax.plot([COLX[3] + NH / 2, SUBX[-1] + 2], [sub, sub], **LINE)
for x, node in zip(SUBX, PLACE[3]):
    ax.plot([x + 2, x + 2], [sub, sub - PITCH + NH / 2 + 0.5], **LINE)
    outline(node, x, sub - PITCH + 0.5)

# legend
ly, lx = 9, 4
for role, name in (
    ("m", "A2B move (planner, MPC)"),
    ("g", "grip move (MPC when carrying)"),
    ("s", "lidar / gripper measurement"),
    ("", "gripper, world model"),
):
    lx = leaf(lx, ly, "act", name, role) + 3
lx = leaf(lx, ly, "cond", "condition", "") + 4
ly, lx = 3.5, 4
for kind, sym, name in (
    ("seq", "→", "sequence"),
    ("fb", "?", "fallback"),
    ("deco", "↻", "repeat"),
    ("deco", "↺", "retry"),
):
    ax.add_patch(Rectangle((lx, ly - NH / 2), NH, NH, fc="w", ec=pal.INK, lw=0.6))
    ax.text(lx + NH / 2, ly, sym, ha="center", va="center", fontsize=6.5)
    ax.text(lx + NH + 1.0, ly, name, ha="left", va="center", fontsize=6.5)
    lx += NH + 2.5 + text_w(name)

for ext in ("pdf", "svg", "png"):
    fig.savefig(out / f"fig_bt.{ext}", dpi=600)
