"""Swatch of the palette, a styled time plot, and latex/cbs_colors.sty."""

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from cbs_figures import palette as pal

out = Path(sys.argv[1] if len(sys.argv) > 1 else "out")
out.mkdir(exist_ok=True)
(Path(__file__).parents[1] / "latex/cbs_colors.sty").write_text(pal.latex())
pal.use()

# swatch
groups = [
    ("categorical", pal.CATEGORICAL),
    ("neutral", (pal.INK, pal.GRAY, pal.MUTED, pal.GRID, pal.TOLERANCE)),
    ("axes x y z", pal.AXIS),
    ("materials", tuple(pal.MATERIAL.values())),
]
fig, axs = plt.subplots(
    len(groups) + 2, 1, figsize=(pal.WIDTH["onehalf"], 4.2), gridspec_kw={"hspace": 0.9}
)
for ax, (name, cols) in zip(axs, groups):
    for i, c in enumerate(cols):
        ax.add_patch(plt.Rectangle((i, 0), 0.92, 1, color=c))
        ax.text(i + 0.46, -0.15, c, ha="center", va="top", fontsize=5, color=pal.GRAY)
    ax.set_xlim(0, 9)
    ax.set_ylim(-0.6, 1)
    ax.set_title(name, loc="left", fontsize=7)
for ax, cmap in zip(axs[-2:], (pal.SEQUENTIAL, pal.DIVERGING)):
    ax.imshow(
        np.linspace(0, 1, 256)[None], aspect="auto", cmap=cmap, extent=(0, 9, 0, 1)
    )
    ax.set_title(cmap.name, loc="left", fontsize=7)
for ax in axs:
    ax.axis("off")
fig.savefig(out / "palette.png")

# styled plot: reference vs MPC vs PID with a tolerance band
t = np.linspace(0, 8, 400)
ref = np.where(t < 1, 0, 3 * np.deg2rad(1) * np.exp(-(t - 1)) * np.sin(4 * (t - 1)))
fig, ax = plt.subplots()
ax.axhspan(-0.4, 0.4, color=pal.ROLE["tolerance"], lw=0, label="settled")
ax.plot(t, 0 * t, color=pal.ROLE["reference"], ls="--", label="reference")
ax.plot(t, np.rad2deg(ref) * 0.4, color=pal.ROLE["mpc"], label="MPC")
ax.plot(t, np.rad2deg(ref), color=pal.ROLE["baseline"], label="PID")
ax.set_xlabel(r"$t$ in s")
ax.set_ylabel(r"$q_6 - q_{6,\mathrm{eq}}$ in deg")
ax.legend(ncol=4, loc="lower left", bbox_to_anchor=(0, 1.0))
fig.savefig(out / "plot_demo.pdf")
fig.savefig(out / "plot_demo.png")
