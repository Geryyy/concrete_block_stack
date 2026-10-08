"""
Fig. 9 (paper draft): MPC timing contract and data flow of one cycle, t - t_m axis.

wiki/mpc.md (timing) / paper §7.4: T_s = D = 60 ms, joint states and the joint velocity
controller (JTC) at 100 Hz. Cycle n reads x(t_m), rolls it forward D under the controller's
mean output u_bar over [t_m - T_s, t_m), then T_s under u_{n-1} -> x0_hat, knot 0 of
horizon n at t_m + D + T_s. SOLVE is illustrative (budget = T_s).
"""

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from cbs_figures import palette as pal
from matplotlib.patches import FancyArrowPatch, Rectangle

out = Path(sys.argv[1] if len(sys.argv) > 1 else "out")
out.mkdir(exist_ok=True)
TS = D = 60  # ms
TICK = 10  # joint states / controller period, ms
SOLVE = 34  # ms, drawn solve time (any value < T_s keeps the contract)
T0, T1 = -75, 215
Y = {"js": 4.0, "pred": 3.0, "mpc": 2.0, "ctrl": 1.0}
NAME = {
    "js": "joint states\n100 Hz",
    "pred": "state\nprediction",
    "mpc": "MPC",
    "ctrl": "joint velocity\ncontroller, 100 Hz",
}
B = r"\mathbf"  # bold vectors in mathtext

pal.use()
fig, ax = plt.subplots(figsize=(pal.WIDTH["onehalf"], 2.9))
for k, y in Y.items():
    ax.axhline(y, color=pal.GRID, lw=0.6, zorder=0)
    ax.text(T0 - 4, y, NAME[k], ha="right", va="center", linespacing=1.1)


def txt(t, y, s, **kw):
    kw = {"ha": "center", "va": "center", "fontsize": 7, **kw}
    ax.text(t, y, s, **kw)


def bar(t0, t1, y, color, text=None, h=0.3, **kw):
    ax.add_patch(Rectangle((t0, y - h / 2), t1 - t0, h, color=color, lw=0, **kw))
    if text:
        txt((t0 + t1) / 2, y, text, color="w")


def flow(a, b, color=pal.INK, rad=0.0, ls="-"):
    """Data-flow arrow a -> b, both (t, y)."""
    ax.add_patch(
        FancyArrowPatch(
            a,
            b,
            arrowstyle="-|>",
            mutation_scale=6,
            color=color,
            lw=0.6,
            ls=ls,
            shrinkA=1,
            shrinkB=1,
            connectionstyle=f"arc3,rad={rad}",
            zorder=4,
        )
    )


def dot(t, y, filled=True, ms=3.2):
    ax.plot(
        [t],
        [y],
        "o",
        ms=ms,
        mfc=pal.INK if filled else "w",
        mec=pal.INK,
        mew=0.6,
        zorder=5,
    )


def span(t0, t1, y, s):
    ax.annotate(
        "",
        (t0, y),
        (t1, y),
        arrowprops=dict(
            arrowstyle="<|-|>",
            lw=0.5,
            color=pal.GRAY,
            mutation_scale=5,
            shrinkA=0,
            shrinkB=0,
        ),
    )
    txt((t0 + t1) / 2, y + 0.1, s, color=pal.GRAY, va="bottom")


ticks = np.arange(-70, T1, TICK)
x_m = rf"${B}{{x}}(t_\mathrm{{m}})$"

# joint states: samples x(t_j); cycles n-1 and n use x(t_m - T_s) and x(t_m)
ax.vlines(ticks, Y["js"] - 0.1, Y["js"] + 0.1, color=pal.MUTED, lw=0.6)
for t, s in [(-TS, rf"${B}{{x}}(t_\mathrm{{m}}{{-}}T_\mathrm{{s}})$"), (0, x_m)]:
    dot(t, Y["js"])
    txt(t, Y["js"] + 0.22, s, va="bottom")

# controller: output u(t_j) every tick; its mean over the last cycle is u_bar
ax.vlines(ticks, Y["ctrl"] - 0.1, Y["ctrl"] + 0.1, color=pal.INK, lw=0.6)
ax.add_patch(
    Rectangle((-TS, Y["ctrl"] - 0.22), TS, 0.44, color=pal.TOLERANCE, lw=0, zorder=0)
)
txt(-TS / 2, Y["ctrl"] - 0.34, rf"$\bar{{{B}{{u}}}}$ = mean ${B}{{u}}(t_j)$", va="top")
txt(-68, Y["ctrl"] + 0.2, rf"${B}{{u}}(t_j)$", va="bottom", ha="left", color=pal.GRAY)
txt(
    SOLVE + 3,
    Y["ctrl"] - 0.34,
    "runs horizon $n$",
    va="top",
    ha="left",
    color=pal.ROLE["mpc"],
)

# MPC: solve n-1 and n start on their measurement, publish within T_s
bar(-TS, -TS + SOLVE, Y["mpc"], pal.MUTED, "solve $n{-}1$")
bar(0, SOLVE, Y["mpc"], pal.ROLE["mpc"], "solve $n$")
span(0, TS, Y["mpc"] - 0.42, "")
txt(TS + 3, Y["mpc"] - 0.42, r"budget $T_\mathrm{s}$", ha="left", color=pal.GRAY)
for t, c in [(-TS + SOLVE, pal.MUTED), (SOLVE, pal.ROLE["mpc"])]:
    flow((t, Y["mpc"] - 0.15), (t, Y["ctrl"] + 0.12), color=c)
txt(SOLVE + 2, 1.3, "publish", ha="left", color=pal.ROLE["mpc"])
# horizon n: knot k at t_m + D + T_s + k T_s, stamped t_m
for k in range(2):
    t = D + TS + k * TS
    dot(t, Y["mpc"], filled=False, ms=3.0)
    txt(t, Y["mpc"] - 0.22, rf"${B}{{u}}_{k}^{{(n)}}$", va="top", color=pal.ROLE["mpc"])
txt(D + 3 * TS - 22, Y["mpc"], r"$\cdots$", color=pal.ROLE["mpc"])
txt(D + 2.5 * TS, Y["mpc"] + 0.2, "horizon $n$", va="bottom", color=pal.ROLE["mpc"])

# prediction: x(t_m) --F(., u_bar) over D--> x_hat(t_m + D) --F(., u_{n-1}) over T_s--> x0_hat
bar(
    0,
    D,
    Y["pred"],
    pal.ROLE["model"],
    rf"$F(\cdot,\bar{{{B}{{u}}}})$ over $D$",
    alpha=0.55,
)
bar(
    D,
    D + TS,
    Y["pred"],
    pal.ROLE["model"],
    rf"$F(\cdot,{B}{{u}}_{{n-1}})$ over $T_\mathrm{{s}}$",
)
dot(0, Y["pred"])
dot(D, Y["pred"], filled=False)
dot(D + TS, Y["pred"])
txt(D, Y["pred"] + 0.2, rf"$\hat{{{B}{{x}}}}(t_\mathrm{{m}}{{+}}D)$", va="bottom")
txt(
    D + TS + 3,
    Y["pred"] + 0.2,
    rf"$\hat{{{B}{{x}}}}_0=\hat{{{B}{{x}}}}(t_\mathrm{{m}}{{+}}D{{+}}T_\mathrm{{s}})$",
    va="bottom",
    ha="left",
)

# data flow
flow((0, Y["js"] - 0.05), (0, Y["pred"] + 0.06))  # measurement -> prediction
flow((-TS / 2, Y["ctrl"] + 0.22), (D / 4, Y["pred"] - 0.16), rad=-0.12)  # u_bar
# u_{n-1}: knot 0 of horizon n-1, due at t_m - T_s + D + T_s = t_m + D
dot(D, Y["mpc"], filled=False, ms=3.0)
txt(
    D + 3,
    Y["mpc"] + 0.2,
    rf"${B}{{u}}_{{n-1}}={B}{{u}}_0^{{(n-1)}}$",
    va="bottom",
    ha="left",
    color=pal.GRAY,
)
flow((D, Y["mpc"] + 0.06), (D + 1, Y["pred"] - 0.16), color=pal.GRAY)
flow(
    (D + TS - 2, Y["pred"] - 0.12), (SOLVE + 1, Y["mpc"] + 0.12), rad=0.0
)  # x0_hat -> solve
txt(D + TS - 14, 2.64, "initial state", fontsize=6.5, ha="left", va="top")

ax.set_xlim(T0, T1)
ax.set_ylim(0.4, 4.7)
ax.set_yticks([])
ax.spines[["left"]].set_visible(False)
ax.set_xticks(np.arange(-60, T1, 60))
ax.set_xlabel(r"$t - t_\mathrm{m}$ in ms")
for ext in ("pdf", "svg", "png"):
    fig.savefig(out / f"fig_timing.{ext}")
