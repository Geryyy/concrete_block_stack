"""
Fig. 9 (paper draft): MPC timing contract and data flow of one cycle, t - t_m axis.

wiki/mpc.md (timing), paper §7.4, crane_mpc cycle.py / horizon.py: T_s = D = 60 ms,
joint states and the joint velocity controller (JTC) at 100 Hz. Horizon n is stamped t_m;
the controller plays u_{n-1} over [t_m, t_m+T_s), u_0^(n) from t_m+T_s; each command
reaches the valves D later. Cycle n reads x(t_m) and rolls it forward under what acts at
the valves -- D under u_bar (controller mean over [t_m-T_s, t_m)), T_s under u_{n-1} --
to x0_hat at t_m+D+T_s, where u_0^(n) arrives. Cycle n is highlighted, other cycles grey.
SOLVE is illustrative (budget = T_s).
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
Y = {"js": 5.0, "pred": 4.0, "mpc": 3.0, "ctrl": 2.0, "valve": 1.0}
NAME = {
    "js": "joint states\n100 Hz",
    "pred": "state\nprediction",
    "mpc": "MPC",
    "ctrl": "joint velocity\ncontroller, 100 Hz",
    "valve": "valves, 100 Hz\n(after delay $D$)",
}
B = r"\mathbf"  # bold vectors in mathtext
U_BAR = rf"$\bar{{{B}{{u}}}}$"
U_PREV = rf"${B}{{u}}_{{n-1}}$"
BLUE, H = pal.ROLE["mpc"], 0.26
OTHER = pal.MUTED  # elements of other cycles
FOCUS = "#EEF4F8"  # cycle n band: a whisper of TU blue

pal.use()
fig, ax = plt.subplots(figsize=(pal.WIDTH["onehalf"], 3.4))
ax.axvspan(0, D + TS, color=FOCUS, lw=0, zorder=-1)
ax.text(
    (D + TS) / 2, 5.62, "cycle $n$", ha="center", va="top", fontsize=6.5, color=BLUE
)
for k, y in Y.items():
    ax.axhline(y, color=pal.GRID, lw=0.6, zorder=0)
    ax.text(T0 - 4, y, NAME[k], ha="right", va="center", linespacing=1.1)


def txt(t, y, s, **kw):
    kw = {"ha": "center", "va": "center", "fontsize": 7, **kw}
    ax.text(t, y, s, **kw)


def bar(t0, t1, y, color, text=None, h=H, fg="w", **kw):
    ax.add_patch(Rectangle((t0, y - h / 2), t1 - t0, h, color=color, lw=0, **kw))
    if text:
        txt((t0 + min(t1, T1)) / 2, y, text, color=fg, fontsize=6.5)


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


def dot(t, y, filled=True, ms=3.2, color=pal.INK):
    ax.plot(
        [t], [y], "o", ms=ms, mfc=color if filled else "w", mec=color, mew=0.6, zorder=5
    )


ticks = np.arange(-70, T1, TICK)

# joint states: samples x(t_j); cycle n reads x(t_m), cycle n-1 read x(t_m - T_s)
ax.vlines(ticks, Y["js"] - 0.1, Y["js"] + 0.1, color=pal.MUTED, lw=0.6)
dot(-TS, Y["js"], color=OTHER)
txt(
    -TS,
    Y["js"] + 0.22,
    rf"${B}{{x}}(t_\mathrm{{m}}{{-}}T_\mathrm{{s}})$",
    va="bottom",
    color=OTHER,
)
dot(0, Y["js"])
txt(0, Y["js"] + 0.22, rf"${B}{{x}}(t_\mathrm{{m}})$", va="bottom")

# controller output u(t_j) and the same commands at the valves, D later
# Both lanes carry the controller's 100 Hz output u(t_j) (horizon feed-forward + PI),
# the valves D later; only the prediction replaces [t_m - T_s, t_m) by its mean u_bar.
segments = [  # (start, controller label, valve label, colour, text colour, alpha)
    (
        -TS,
        rf"${B}{{u}}(t_j)$, mean {U_BAR}",
        rf"${B}{{u}}(t_j)$",
        pal.TOLERANCE,
        pal.INK,
        1.0,
    ),
    (0, U_PREV, U_PREV, pal.GRAY, "w", 1.0),
    (TS, rf"${B}{{u}}_0^{{(n)}}$", rf"${B}{{u}}_0^{{(n)}}$", BLUE, "w", 1.0),
    (2 * TS, rf"${B}{{u}}_1^{{(n)}}$", rf"${B}{{u}}_1^{{(n)}}$", pal.WASH, "w", 1.0),
]
for t, label, valve, color, fg, alpha in segments:
    bar(t, t + TS, Y["ctrl"], color, label, fg=fg, alpha=alpha)
    bar(t + D, t + D + TS, Y["valve"], color, valve, fg=fg, alpha=alpha)
    for y, t0 in ((Y["ctrl"], t), (Y["valve"], t + D)):  # 10 ms cells: the 100 Hz grid
        cells = np.arange(t0 + TICK, min(t0 + TS, T1), TICK)
        ax.vlines(cells, y - H / 2, y + H / 2, color="w", lw=0.35, alpha=0.8, zorder=2)
    if t + D < T1 - 20:
        c = BLUE if t == TS else pal.GRAY
        flow((t + 1, Y["ctrl"] - H / 2), (t + D + 1, Y["valve"] + H / 2), color=c)
ax.vlines(
    ticks, Y["ctrl"] - 0.1, Y["ctrl"] + 0.1, color=pal.INK, lw=0.5, zorder=0.5
)  # under bars
txt(
    3 * TS + 6,
    (Y["ctrl"] + Y["valve"]) / 2,
    "transport\ndelay $D$",
    color=pal.GRAY,
    fontsize=6.5,
    ha="left",
    linespacing=1.2,
)

# MPC: solve n-1 (grey) and n, each within the budget T_s from its measurement
ax.add_patch(
    Rectangle(
        (0, Y["mpc"] - 0.2), TS, 0.4, fill=False, ec=pal.GRAY, lw=0.5, ls=(0, (2, 1.5))
    )
)
txt(
    TS + 3,
    Y["mpc"] - 0.12,
    r"budget $T_\mathrm{s}$",
    ha="left",
    color=pal.GRAY,
    fontsize=6.5,
)
bar(-TS, -TS + SOLVE, Y["mpc"], OTHER, "solve $n{-}1$", h=0.3)
bar(0, SOLVE, Y["mpc"], BLUE, "solve $n$", h=0.3)
# publish: horizon n-1 plays from t_m, horizon n's u_0 leaves the controller at t_m + T_s
flow((-TS + SOLVE, Y["mpc"] - 0.15), (6, Y["ctrl"] + H / 2), color=OTHER, rad=0.25)
flow((SOLVE, Y["mpc"] - 0.15), (TS + 8, Y["ctrl"] + H / 2), color=BLUE, rad=0.25)
txt(SOLVE + 18, 2.62, "publish", ha="left", color=BLUE, fontsize=6.5)

# prediction: x(t_m) --F(., u_bar) over D--> x_hat(t_m+D) --F(., u_{n-1}) over T_s--> x0_hat
bar(
    0,
    D,
    Y["pred"],
    pal.ROLE["model"],
    rf"$F(\cdot,${U_BAR}$)$ over $D$",
    h=0.3,
    alpha=0.55,
)
bar(
    D,
    D + TS,
    Y["pred"],
    pal.ROLE["model"],
    rf"$F(\cdot,${U_PREV}$)$ over $T_\mathrm{{s}}$",
    h=0.3,
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

# data flow into and out of the prediction
flow((0, Y["js"] - 0.05), (0, Y["pred"] + 0.06))  # measurement
flow((-3, Y["ctrl"] + H / 2), (D / 4, Y["pred"] - 0.16), rad=-0.1)  # u_bar
flow((12, Y["ctrl"] + H / 2), (D + 10, Y["pred"] - 0.16), rad=-0.1)  # u_{n-1}
flow((D + TS - 2, Y["pred"] - 0.12), (SOLVE + 1, Y["mpc"] + 0.15))  # x0_hat -> solve
txt(D + TS - 12, 3.62, "initial state", fontsize=6.5, ha="left", va="top")
# knot 0: x0_hat and u_0^(n) at the valves are the same instant
ax.vlines(
    D + TS, Y["valve"] + H / 2, Y["pred"] - 0.15, color=BLUE, lw=0.5, ls=":", zorder=1
)
txt(D + TS + 2, 2.62, "knot 0", ha="left", color=BLUE, fontsize=6.5)

ax.set_xlim(T0, T1)
ax.set_ylim(0.55, 5.65)
ax.set_yticks([])
ax.spines[["left"]].set_visible(False)
ax.set_xticks(np.arange(-60, T1, 60))
ax.set_xlabel(r"$t - t_\mathrm{m}$ in ms")
for ext in ("pdf", "svg", "png"):
    fig.savefig(out / f"fig_timing.{ext}")
