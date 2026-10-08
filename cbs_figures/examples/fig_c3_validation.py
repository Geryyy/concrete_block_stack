"""
Fig. 4 (b) (paper draft): C3 against held-out HydraulicCalib bags, one row per axis.

Replays `c3_full_model.json` (k, d, tau_v, n_d) through the fit's own `simulate`, free run
over the whole bag; shows the largest command step, -1..5 s. NRMSE, f_n, zeta are the JSON's
(`fit_full` owns them), so figure and Table 2 agree.

    PYTHONPATH=$PWD:<ws>/timber_crane_mujoco_py python3 examples/fig_c3_validation.py out
"""

import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from cbs_figures import palette as pal
from matplotlib.ticker import MaxNLocator
from timber_crane_mujoco_py.calibration.c3 import transmission as tr
from timber_crane_mujoco_py.calibration.c3.data import CACHE, DISPLAY, UNIT, campaign
from timber_crane_mujoco_py.calibration.c3.fit_full import DT, command, simulate
from timber_crane_mujoco_py.calibration.c3.rbd import Inertia

WS = Path(__file__).resolve().parents[4]
BAGS = Path.home() / "Documents/HydraulicCalib/bags"
MODEL = WS / "wiki/diagrams/hydraulic_calibration/c3_full_model.json"
AXES = ("sw", "ha", "ka", "sa", "ro")
SYMBOL = {"sw": "q_1", "ha": "q_2", "ka": "q_3", "sa": "q_4", "ro": "q_7"}
PRE, POST, RISE = 1.0, 5.0, 0.3  # s: window around the step, step = jump within RISE

out = Path(sys.argv[1] if len(sys.argv) > 1 else "out")
out.mkdir(exist_ok=True)
model = json.loads(MODEL.read_text())["axes"]


def held_out(axis):
    """Test bags exactly as compare_models / fit_full split them (every third)."""
    records = campaign(BAGS, axis, CACHE)
    inertia = Inertia(BAGS / records[0]["bag"], CACHE)
    for r in records:
        r["tau"] = tr.torque(axis, r["p_a"], r["p_b"], r["q2"], r["q3"])
        r["m_bag"] = float(np.median(inertia.diagonal(axis, r["pos"])))
    command(records, axis)
    return [r for i, r in enumerate(records) if i % 3 == 0]


def step(r):
    """(jump, index) of the largest command step with a single-axis window around it."""
    pre, post, rise = (int(x / DT) for x in (PRE, POST, RISE))
    jump = np.abs(r["uv"][rise:] - r["uv"][:-rise])
    ok = (
        np.convolve(~r["ok"], np.ones(pre + post), "valid") == 0
    )  # ok[i]: [i, i+pre+post)
    best = (0.0, None)
    for i in np.argsort(jump)[::-1][:2000]:
        if pre <= i < len(ok) and ok[i - pre]:
            return jump[i], i
    return best


pal.use()
fig, axs = plt.subplots(
    1, len(AXES), figsize=(pal.WIDTH["double"], 1.75), gridspec_kw={"wspace": 0.38}
)
for ax, axis in zip(axs, AXES):
    p = model[axis]
    best = (0.0, None, None, None)
    for r in held_out(axis):
        jump, i = step(r)
        y_c3 = simulate(r, p["k"], p["d"], p["tau_v"], p.get("n_d", 0))
        if i is not None and y_c3 is not None and jump > best[0]:
            best = (jump, r, y_c3, i)
    _, r, y_c3, i = best
    cut = slice(i - int(PRE / DT), i + int(POST / DT))
    t = np.arange(cut.stop - cut.start) * DT - PRE
    ax.plot(t, r["uv"][cut], color=pal.ROLE["reference"], ls="--", lw=0.7)
    ax.plot(t, r["y"][cut], color=pal.ROLE["measured"], lw=0.8)
    ax.plot(t, y_c3[cut], color=pal.ROLE["model"], lw=0.9)
    ax.set_title(
        rf"{DISPLAY[axis]} $\dot {SYMBOL[axis]}$ in {UNIT.get(axis, 'rad/s')}"
        f"\nNRMSE {p['nrmse_test']:.3f}",
        fontsize=7,
        linespacing=1.3,
    )
    ax.yaxis.set_major_locator(MaxNLocator(4))
    ax.set_xlabel(r"$t$ in s", labelpad=1)
    ax.set_xlim(-PRE, POST)
    print(f"{axis}: {r['bag']} t_step={i * DT:.1f}s jump={best[0]:.3f}")
axs[0].legend(
    axs[0].lines,
    [r"command $\Psi^{-1}(u)$", "measured", "C3 (free run)"],
    ncol=3,
    loc="lower left",
    bbox_to_anchor=(0, 1.22),
    frameon=False,
)
for ext in ("pdf", "svg", "png"):
    fig.savefig(out / f"fig_c3_validation.{ext}")
