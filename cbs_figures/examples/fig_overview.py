"""
Fig. 3 (paper draft): system block diagram, after the ICRA26 overview.

Plain boxes, symbols on the wires. Paper §3-§8; rates from the configs, lidar 10 Hz.
Units: mm on the page.

    python3 examples/fig_overview.py out
"""

import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, Polygon, Rectangle

from cbs_figures import palette as pal

out = Path(sys.argv[1] if len(sys.argv) > 1 else "out")
out.mkdir(exist_ok=True)
W, H = 140, 70  # mm
BH = 9  # box height
C = [12, 42, 77, 112]  # column centres
BWS = [19, 19, 22, 19]  # box width per column (middle one holds "Joint velocity")
R = [52, 33, 12]  # row centres
B = r"\mathbf"
RATE = dict(fontsize=5.5, color=pal.GRAY)

pal.use()
fig = plt.figure(figsize=(W / 25.4, H / 25.4))
ax = fig.add_axes((0, 0, 1, 1))
ax.set_xlim(0, W)
ax.set_ylim(0, H)
ax.axis("off")


def box(c, y, text, h=BH):
    x, w = C[c], BWS[c]
    ax.add_patch(
        Rectangle((x - w / 2, y - h / 2), w, h, fc="w", ec=pal.INK, lw=0.6, zorder=2)
    )
    ax.text(
        x, y, text, ha="center", va="center", fontsize=7, linespacing=1.05, zorder=3
    )


def wire(
    *pts,
    label=None,
    at=0,
    off=(0, 1.0),
    ha="center",
    va="bottom",
    rate=None,
    roff=(0, -1.0),
):
    """Polyline with an arrowhead at its end; label/rate placed on segment `at`'s midpoint."""
    for a, b in zip(pts[:-2], pts[1:-1]):
        ax.plot(*zip(a, b), color=pal.INK, lw=0.6, solid_capstyle="butt", zorder=1)
    ax.add_patch(
        FancyArrowPatch(
            pts[-2],
            pts[-1],
            arrowstyle="-|>",
            mutation_scale=5,
            color=pal.INK,
            lw=0.6,
            shrinkA=0,
            shrinkB=0,
            zorder=1,
        )
    )
    mx = (pts[at][0] + pts[at + 1][0]) / 2
    my = (pts[at][1] + pts[at + 1][1]) / 2
    if label:
        ax.text(mx + off[0], my + off[1], label, ha=ha, va=va, fontsize=6.5)
    if rate:
        ax.text(mx + roff[0], my + roff[1], rate, ha=ha, va="top", **RATE)


def edge(c, r, side, d=0.0):
    """Point on a box side: l r t b, shifted d along it."""
    x, y, BW = C[c], R[r], BWS[c]
    return {
        "l": (x - BW / 2, y + d),
        "r": (x + BW / 2, y + d),
        "t": (x + d, y + BH / 2),
        "b": (x + d, y - BH / 2),
    }[side]


# boxes
box(0, R[0], "Lidar")
box(1, R[0], "Block\ndetector")
box(2, R[0], "World\nmodel")
box(3, R[0], "Task layer\n(BT)")
IMU = [(R[1] + 4.2, "IMU 1"), (R[1] - 4.2, "IMU 2")]
for y, name in IMU:
    box(0, y, name, h=6)
box(1, R[1], "Sway\nestimator", h=14.4)
box(2, R[1], "Path\nplanner")
box(0, R[2], "Crane\n(hydraulics)")
box(1, R[2], "Paltronic\n$\\Psi$, valves")
box(2, R[2], "Joint velocity\ncontroller")
box(3, R[2], "MPC")

# OEM side of the interface cut: Paltronic + machine
x0, x1 = edge(0, 2, "l")[0] - 2, edge(1, 2, "r")[0] + 2
ax.add_patch(
    Rectangle(
        (x0, R[2] - BH / 2 - 4.5),
        x1 - x0,
        BH + 6,
        fill=False,
        ec=pal.GRAY,
        lw=0.5,
        ls=(0, (3, 2)),
        zorder=0,
    )
)
ax.text(x0 + 1, R[2] - BH / 2 - 3.6, "OEM", ha="left", va="bottom", **RATE)

# wall plan: document icon above the task layer
dx, dy = C[3], R[0] + BH / 2 + 8
ax.add_patch(
    Polygon(
        [
            (dx - 3, dy - 3.5),
            (dx + 3, dy - 3.5),
            (dx + 3, dy + 2),
            (dx + 1.5, dy + 3.5),
            (dx - 3, dy + 3.5),
        ],
        fc="w",
        ec=pal.INK,
        lw=0.7,
        joinstyle="round",
    )
)
for k in range(3):
    ax.plot([dx - 1.8, dx + 1.8], [dy + 1.2 - 1.4 * k] * 2, color=pal.INK, lw=0.6)
ax.text(dx + 4.5, dy, "wall plan", ha="left", va="center", fontsize=6.5)
wire((dx, dy - 3.5), edge(3, 0, "t"))

# perception
wire(edge(0, 0, "r"), edge(1, 0, "l"), label=r"$\mathcal{P}$", rate="10 Hz")
wire(edge(1, 0, "r", 1.5), edge(2, 0, "l", 1.5), label=rf"${B}{{H}}_0^\mathrm{{B}}, c$")
wire(edge(2, 0, "r"), edge(3, 0, "l"), label=r"blocks")
wire(
    edge(2, 0, "b"),
    edge(2, 1, "t"),
    label=r"scene $\mathcal{O}$",
    off=(1.0, 0),
    ha="left",
    va="center",
)

# task -> planner -> MPC; MPC progress back to the task layer
xt = C[3] - 6
wire(
    edge(3, 0, "b", -6),
    (xt, R[1] + 2),
    edge(2, 1, "r", 2),
    label=r"plan$(\cdot)$",
    at=1,
)
wire(
    edge(2, 1, "r", -2),
    (C[3], R[1] - 2),
    edge(3, 2, "t"),
    label=rf"${B}{{c}}(s)$",
    at=0,
    off=(0, 0.8),
    rate="per move",
    roff=(0, -0.8),
)
xs = edge(3, 2, "r")[0] + 4
wire(
    edge(3, 2, "r"),
    (xs, R[2]),
    (xs, R[0] - 2),
    edge(3, 0, "r", -2),
    label="$s$",
    at=1,
    off=(1.0, 0),
    ha="left",
    va="center",
)

# sway estimation
xm = (edge(0, 1, "r")[0] + edge(1, 1, "l")[0]) / 2
for y, _ in IMU:
    wire((edge(0, 1, "r")[0], y), (edge(1, 1, "l")[0], y))
ax.text(
    xm,
    IMU[0][0] + 0.8,
    rf"$\{{{B}{{a}},\omega\}}_{{15}}$",
    ha="center",
    va="bottom",
    fontsize=6.5,
)
ax.text(
    xm,
    IMU[1][0] - 0.8,
    rf"$\{{{B}{{a}},\omega\}}_{{16}}$",
    ha="center",
    va="top",
    fontsize=6.5,
)
ax.text(xm, R[1], "66 Hz", ha="center", va="center", **RATE)

# joint states: encoders + sway estimate merge at J, then to MPC, controller, world model
J = (edge(1, 1, "r")[0] + 6, 25.0)
ax.add_patch(Circle(J, 1.0, fc="w", ec=pal.INK, lw=0.6, zorder=2))
wire(
    (C[0] + 5, R[2] + BH / 2),
    (C[0] + 5, 21.5),
    (J[0], 21.5),
    (J[0], J[1] - 1.0),
    label=rf"${B}{{q}}_\mathrm{{a}}, \dot{{{B}{{q}}}}_\mathrm{{a}}$",
    at=1,
)
ax.text((C[0] + 5 + J[0]) / 2 + 9, 22.4, "100 Hz", ha="left", va="bottom", **RATE)
wire(
    edge(1, 1, "r"),
    (J[0], R[1]),
    (J[0], J[1] + 1.0),
    label=rf"${B}{{q}}_\mathrm{{u}}, \dot{{{B}{{q}}}}_\mathrm{{u}}$",
    at=1,
    off=(0.8, 0),
    ha="left",
    va="center",
)
wire(
    (J[0] + 1.0, J[1]),
    (C[3] - 6, J[1]),
    edge(3, 2, "t", -6),
    label=rf"${B}{{q}}, \dot{{{B}{{q}}}}$",
    at=0,
    off=(4, 0.6),
)
wire(
    (C[2] - 6, J[1]),
    edge(2, 2, "t", -6),
    label=rf"$\dot{{{B}{{q}}}}_\mathrm{{a}}$",
    off=(-0.8, 0),
    ha="right",
    va="center",
)
xw = edge(2, 1, "l")[0] - 3
wire(
    (xw, J[1]),
    (xw, R[0] - 2.5),
    edge(2, 0, "l", -2.5),
    label=rf"${B}{{q}}$",
    at=0,
    off=(-0.8, 3),
    ha="right",
    va="center",
)
for x in (xw, C[2] - 6):  # branch dots
    ax.plot(x, J[1], "o", ms=1.8, color=pal.INK, zorder=2)

# actuation chain
wire(edge(3, 2, "l"), edge(2, 2, "r"), label=rf"${B}{{u}}$", rate="16.7 Hz")
wire(edge(2, 2, "l"), edge(1, 2, "r"), label=rf"${B}{{u}}(t_j)$", rate="100 Hz")
wire(edge(1, 2, "l"), edge(0, 2, "r"), label="valves", rate=None)

for ext in ("pdf", "svg", "png"):
    fig.savefig(out / f"fig_overview.{ext}", dpi=600)
