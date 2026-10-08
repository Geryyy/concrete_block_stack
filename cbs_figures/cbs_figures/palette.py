"""
One palette for every plot, diagram and render of the AiC paper.

Anchored on TU blue and ACIN red. Categorical order validated (dataviz validator,
white surface): L band, chroma >= 0.1, adjacent CVD dE >= 11, normal dE >= 18,
contrast >= 3:1. Scatter / any-pair forms: first 3 slots only (all-pairs pass).
"""

from __future__ import annotations

import matplotlib as mpl
from matplotlib.colors import LinearSegmentedColormap, to_rgb

# --- categorical: identity of a series, assigned in this order, never cycled
BLUE = "#00669A"  # TU blue
RED = "#BA122B"  # ACIN red
OCHRE = "#B07D0A"
TEAL = "#00897B"
VIOLET = "#851F83"
CATEGORICAL = (BLUE, RED, OCHRE, TEAL, VIOLET)

# --- neutrals: text, reference/planned curves, grid, fills
INK = "#1F2329"  # text, axes, measured-vs-ref "truth"
GRAY = "#6B7280"  # reference / planned, secondary text
MUTED = "#B4B8BF"  # grid-adjacent, disabled, ghosts
GRID = "#E3E5E8"
PAPER = "#FFFFFF"
TOLERANCE = "#F3E6C4"  # tolerance band / settle box fill (light ochre)

# --- semantic roles; plots say what a curve *is*, not which hue it gets
ROLE = {
    "reference": GRAY,  # planned / desired, dashed
    "measured": INK,
    "mpc": BLUE,
    "baseline": RED,  # PID / FK-only / operator
    "model": OCHRE,  # C3 prediction, simulation
    "tolerance": TOLERANCE,
}

# --- coordinate frame axes x/y/z (RGB convention, muted to sit with the rest)
AXIS = ("#C0392B", "#2E8B57", "#1F5FAD")

# --- 3D render materials (light-surface render, outline-drawn)
MATERIAL = {
    "machine": "#D9D7D2",  # crane steel, warm light gray
    "tool": "#8FB3CF",  # PZS100 gripper, light TU blue
    "sensor": "#4A4F57",
    "block": "#C2BCB1",  # concrete
    "block_active": "#E2C27A",  # grasped / highlighted block, light ochre
    "ghost": "#C9CDD3",  # earlier/later configurations
    "ground": "#ECEAE5",
    "edge": "#3A3F47",
}

# --- "wire" look: pastel cage line + faint tint fill per role, watercolour ground
WIRE = {
    "machine": ("#8C84CF", "#F4F2FB"),  # lavender
    "tool": ("#4DAE9C", "#E9F6F2"),  # mint
    "sensor": ("#7D8AA8", "#EEF0F5"),
    "block": ("#D98B6F", "#FBEEE8"),  # peach
    "block_active": ("#D4A23F", "#FBF3DF"),  # butter
    "ghost": ("#C9C5E4", None),  # lines only
}
WASH = "#A9CBDD"  # ground wash, pale sky; "#B9D3B0" sage, "#E8C9A8" sand also work

# --- "lineart" look (Blender Freestyle): flat fill per role, ink outlines
LINEART = {
    "machine": "#FFFFFF",
    "tool": "#E4EDF4",  # faint TU blue: the gripper reads as the subject
    "sensor": "#D9DCE1",
    "block": "#F2EFEA",
    "block_active": "#F7EBCB",
}
LINE_PT = {"solid": (INK, 0.45), "ghost": (MUTED, 0.35)}  # colour, width pt

# --- continuous maps: sequential = one hue light->dark, diverging = blue|neutral|red
SEQUENTIAL = LinearSegmentedColormap.from_list(
    "cbs_seq", ["#EEF4F8", "#8FB3CF", BLUE, "#00304D"]
)
DIVERGING = LinearSegmentedColormap.from_list(
    "cbs_div", ["#00405F", BLUE, "#9DBFD6", "#F2F1EE", "#E3A1A9", RED, "#6E0A19"]
)

#: Elsevier column widths, inch: single, 1.5, double.
WIDTH = {"single": 90 / 25.4, "onehalf": 140 / 25.4, "double": 190 / 25.4}


def use(usetex: bool = False) -> None:
    """Apply the paper style to matplotlib, register `cbs_seq` / `cbs_div`."""
    for cmap in (SEQUENTIAL, DIVERGING):
        if cmap.name not in mpl.colormaps:
            mpl.colormaps.register(cmap)
    mpl.rcParams.update(
        {
            "axes.prop_cycle": mpl.cycler(color=CATEGORICAL),
            "figure.figsize": (WIDTH["single"], WIDTH["single"] * 0.62),
            "figure.dpi": 150,
            "savefig.dpi": 600,
            "savefig.bbox": "tight",
            "savefig.pad_inches": 0.02,
            "pdf.fonttype": 42,
            "font.family": "serif",
            "font.serif": ["CMU Serif", "Latin Modern Roman", "DejaVu Serif"],
            "mathtext.fontset": "cm",
            "text.usetex": usetex,
            "font.size": 8,
            "axes.labelsize": 8,
            "axes.titlesize": 8,
            "legend.fontsize": 7,
            "xtick.labelsize": 7,
            "ytick.labelsize": 7,
            "text.color": INK,
            "axes.edgecolor": INK,
            "axes.labelcolor": INK,
            "xtick.color": INK,
            "ytick.color": INK,
            "axes.linewidth": 0.6,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.grid": True,
            "grid.color": GRID,
            "grid.linewidth": 0.5,
            "lines.linewidth": 1.2,
            "lines.markersize": 4,
            "legend.frameon": False,
            "image.cmap": "cbs_seq",
            "xtick.major.width": 0.6,
            "ytick.major.width": 0.6,
        }
    )


def latex(prefix: str = "cbs") -> str:
    r"""Return xcolor definitions (`\definecolor{cbsBlue}{HTML}{00669A}`) for the .tex side."""
    named = {
        "Blue": BLUE,
        "Red": RED,
        "Ochre": OCHRE,
        "Teal": TEAL,
        "Violet": VIOLET,
        "Ink": INK,
        "Gray": GRAY,
        "Muted": MUTED,
        "Grid": GRID,
        "Tolerance": TOLERANCE,
        "AxisX": AXIS[0],
        "AxisY": AXIS[1],
        "AxisZ": AXIS[2],
    }
    named.update({k.title().replace("_", ""): v for k, v in MATERIAL.items()})
    lines = [
        "% generated by cbs_figures.palette.latex() -- do not edit",
        "\\RequirePackage{xcolor}",
    ]
    lines += [
        f"\\definecolor{{{prefix}{k}}}{{HTML}}{{{v[1:]}}}" for k, v in named.items()
    ]
    return "\n".join(lines) + "\n"


def rgb(color: str) -> tuple[float, float, float]:
    return to_rgb(color)
