"""
Fig. 4a (paper draft): C3 actuator block diagram, per axis i, in the Fig. 3 style.

Paper §4.3: u_f' = w_v (u(t - T_d) - u_f), tau' = k (u_f - q'), clamp, rigid body
ABA(q, q', tau) with joint damping d. C3 lumps Psi, Paltronic, valves and cylinders as
seen from the joint; the rigid body couples all axes. Units: mm on the page.

    python3 examples/fig_c3_block.py out
"""

import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, Rectangle

from cbs_figures import palette as pal

out = Path(sys.argv[1] if len(sys.argv) > 1 else "out")
out.mkdir(exist_ok=True)
W, H = 190, 40  # mm
Y, BH = 22, 9  # signal row, box height
B = r"\mathbf"
SMALL = dict(fontsize=5.5, color=pal.GRAY)

pal.use()
fig = plt.figure(figsize=(W / 25.4, H / 25.4))
ax = fig.add_axes((0, 0, 1, 1))
ax.set_xlim(0, W)
ax.set_ylim(0, H)
ax.axis("off")


def box(x0, x1, text, size=7):
    ax.add_patch(
        Rectangle((x0, Y - BH / 2), x1 - x0, BH, fc="w", ec=pal.INK, lw=0.6, zorder=2)
    )
    ax.text(
        (x0 + x1) / 2,
        Y,
        text,
        ha="center",
        va="center",
        fontsize=size,
        linespacing=1.1,
        zorder=3,
    )


def wire(*pts, label=None, at=0, off=(0, 0.8), ha="center", va="bottom"):
    for a, b in zip(pts[:-2], pts[1:-1]):
        ax.plot(*zip(a, b), color=pal.INK, lw=0.6, zorder=1)
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
    if label:
        mx = (pts[at][0] + pts[at + 1][0]) / 2
        my = (pts[at][1] + pts[at + 1][1]) / 2
        ax.text(mx + off[0], my + off[1], label, ha=ha, va=va, fontsize=6.5)


def dot(x, y):
    ax.plot(x, y, "o", ms=1.8, color=pal.INK, zorder=2)


# blocks, left to right (x0, x1)
DEL, LAG, SUM, K, INT, CLP, RB, I1, I2 = (
    (11, 25),
    (39, 52),
    64,
    (70, 78),
    (84, 91),
    (99, 109),
    (119, 147),
    (156, 163),
    (172, 179),
)
box(*DEL, r"delay $T_\mathrm{d}$")
box(*LAG, r"$\dfrac{\omega_{\mathrm{v},i}}{s+\omega_{\mathrm{v},i}}$", size=8)
ax.add_patch(Circle((SUM, Y), 1.6, fc="w", ec=pal.INK, lw=0.6, zorder=2))
ax.text(SUM - 2.2, Y + 1.2, "+", ha="center", va="bottom", fontsize=6)
ax.text(SUM + 1.2, Y - 2.4, "−", ha="left", va="top", fontsize=6)
box(*K, r"$k_i$")
box(*INT, r"$\int$")
# clamp: saturation glyph
box(*CLP, "")
cx, s = (CLP[0] + CLP[1]) / 2, 2.4
ax.plot(
    [cx - 4, cx - s, cx + s, cx + 4],
    [Y - s, Y - s, Y + s, Y + s],
    color=pal.INK,
    lw=0.6,
    zorder=3,
)
box(
    *RB,
    "rigid body\n"
    + rf"ABA$({B}{{q}}, \dot{{{B}{{q}}}}, {B}{{\tau}}_\mathrm{{a}})$, $d_i$",
)
box(*I1, r"$\int$")
box(*I2, r"$\int$")

# forward path
wire((2, Y), (DEL[0], Y), label="$u_i$")
wire((DEL[1], Y), (LAG[0], Y), label=r"$u_i(t{-}T_\mathrm{d})$", off=(0, 0.8))
wire((LAG[1], Y), (SUM - 1.6, Y), label=r"$u_{\mathrm{f},i}$")
wire((SUM + 1.6, Y), (K[0], Y))
wire((K[1], Y), (INT[0], Y), label=r"$\dot\tau_{\mathrm{a},i}$")
wire((INT[1], Y), (CLP[0], Y), label=r"$\tau_{\mathrm{a},i}$")
wire((CLP[1], Y), (RB[0], Y))
wire((RB[1], Y), (I1[0], Y), label=r"$\ddot q_i$")
wire((I1[1], Y), (I2[0], Y), label=r"$\dot q_i$")
wire((I2[1], Y), (W - 3, Y), label=r"$q_i$")

# feedback: q_dot_i to the summing junction, (q, q_dot) of all axes to the rigid body
xq, xd = I2[1] + 4, (I1[1] + I2[0]) / 2
dot(xd, Y)
dot(xq, Y)
yd, yb, yq = 5.5, 10.0, 13.5
rc = (RB[0] + RB[1]) / 2
wire((xd, Y), (xd, yd), (SUM, yd), (SUM, Y - 1.6))
ax.text((SUM + xd) / 2, yd - 0.6, r"$\dot q_i$", ha="center", va="top", fontsize=6.5)
dot(xd, yb)
wire((xd, yb), (rc - 3, yb), (rc - 3, Y - BH / 2))
wire((xq, Y), (xq, yq), (rc + 3, yq), (rc + 3, Y - BH / 2))
ax.text(
    RB[1] + 1.0,
    yq + 0.5,
    rf"${B}{{q}}, \dot{{{B}{{q}}}}$ all axes",
    ha="left",
    va="bottom",
    fontsize=6,
)

# C3 boundary
x0, x1 = DEL[0] - 3, CLP[1] + 3
ax.add_patch(
    Rectangle(
        (x0, 2.5),
        x1 - x0,
        H - 5.5,
        fill=False,
        ec=pal.GRAY,
        lw=0.5,
        ls=(0, (3, 2)),
        zorder=0,
    )
)
ax.text(
    x0 + 1.2,
    H - 4,
    r"C3 actuator, axis $i$ — lumps $\Psi$, Paltronic, valves, cylinders",
    ha="left",
    va="top",
    **SMALL,
)
ax.text(
    (RB[0] + RB[1]) / 2,
    Y + BH / 2 + 1.2,
    "couples all axes",
    ha="center",
    va="bottom",
    **SMALL,
)

for ext in ("pdf", "svg", "png"):
    fig.savefig(out / f"fig_c3_block.{ext}", dpi=600)
