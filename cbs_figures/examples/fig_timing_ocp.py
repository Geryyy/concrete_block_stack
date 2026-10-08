"""
Fig. 8 (paper draft): timing-OCP result of one carry move, the deployed Planner offline.

Move and scene: carry_move.py (shared with Fig. 7). Bounds come from crane_planning's plan_example.bounds,
so every dashed line is one the solve used. Needs ROS sourced (pinocchio, acados).

    python3 examples/fig_timing_ocp.py out
"""

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from cbs_figures import palette as pal

sys.path.insert(0, str(Path(__file__).parent))
import plan_example as pe  # noqa: E402
from carry_move import plan_carry  # noqa: E402  (puts crane_planning on sys.path)

out = Path(sys.argv[1] if len(sys.argv) > 1 else "out")
out.mkdir(exist_ok=True)
AXES = ("slew $q_1$", "boom $q_2$", "arm $q_3$", "telescope $q_4$", "rotator $q_7$")

planner, _start, plan, call = plan_carry()
print(plan.message)
print(
    f"planner call {call:.2f} s = geometry {call - plan.timing.solve_time_s:.2f} s"
    f" + OCP {plan.timing.solve_time_s:.2f} s"
)
tr, cfg = plan.timing, planner.config
limit = pe.bounds(planner, 1.0)
t, T = tr.time, tr.duration

pal.use()
fig, axs = plt.subplots(
    5,
    1,
    figsize=(pal.WIDTH["onehalf"], 4.6),
    sharex=True,
    gridspec_kw={"hspace": 0.28, "height_ratios": [1, 1, 1.4, 1, 1.3]},
)
ax_s, ax_v, ax_q, ax_p, ax_u = axs

ax_s.plot(t, tr.sigma, color=pal.INK)
ax_s.set_ylabel("$s$")
ax_s.set_ylim(-0.05, 1.05)

ax_v.plot(t, tr.speed, color=pal.INK)
ax_v.set_ylabel(r"$\dot s$ in 1/s")

for k, name in enumerate(AXES):
    ax_q.plot(
        t,
        tr.dq_a[:, k] / limit["dq_a"][k],
        color=pal.CATEGORICAL[k],
        label=name,
        lw=0.9,
    )
for sign in (1, -1):
    ax_q.axhline(sign, color=pal.GRAY, ls="--", lw=0.6)
ax_q.set_ylabel(r"$\dot q_i / \dot q_i^{+}$")
ax_q.set_ylim(-1.15, 1.15)
ax_q.legend(
    ncol=5,
    loc="lower center",
    bbox_to_anchor=(0.5, 0.0),
    frameon=False,
    fontsize=6.5,
    handlelength=1.2,
    columnspacing=0.9,
)

ax_p.plot(t, tr.pump_flow, color=pal.INK)
ax_p.axhline(limit["flow"], color=pal.GRAY, ls="--", lw=0.6)
ax_p.text(
    T,
    limit["flow"] + 0.03,
    f"reservation {limit['flow']:.2f}",
    ha="right",
    va="bottom",
    fontsize=6.5,
    color=pal.GRAY,
)
ax_p.set_ylabel(r"$Q_\mathrm{P}/Q_\mathrm{P}^{\max}$")
ax_p.set_ylim(0, 1.0)

sway = np.degrees(tr.q_u - tr.q_u_eq)
box = np.degrees(cfg.terminal_q_sway_max[0])
# the terminal box binds the last node only (stage bound +-11.5 deg is off scale)
ax_u.add_patch(
    plt.Rectangle(
        (T - 0.35, -box),
        0.35,
        2 * box,
        color=pal.ROLE["tolerance"],
        lw=0,
        zorder=0,
        clip_on=False,
    )
)
ax_u.text(
    T + 0.12,
    0,
    f"terminal\nbox\n$\\pm${box:.2f}°",
    ha="left",
    va="center",
    fontsize=6,
    color=pal.GRAY,
    linespacing=1.1,
)
for k, (name, c) in enumerate((("tip $q_5$", pal.BLUE), ("tilt $q_6$", pal.RED))):
    ax_u.plot(t, sway[:, k], color=c, lw=0.9, label=name)
ax_u.plot([T, T], sway[-1], "o", ms=2.5, color=pal.INK, zorder=4, clip_on=False)
ax_u.set_ylabel(r"$q_\mathrm{u} - q_\mathrm{eq}$ in °")
ax_u.legend(ncol=2, loc="upper center", frameon=False, fontsize=6.5)
ax_u.set_xlabel(r"$t$ in s")
ax_u.set_xlim(0, T)

for ax in axs:
    ax.yaxis.set_label_coords(-0.075, 0.5)
print(
    f"duration {T:.2f} s, {tr.iterations} SQP it., peak pump {tr.pump_flow.max():.2f}, "
    f"terminal sway {np.degrees(tr.terminal_sway):.2f} deg"
)
for ext in ("pdf", "svg", "png"):
    fig.savefig(out / f"fig_timing_ocp.{ext}")
