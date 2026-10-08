"""Procedural watercolour wash: an RGBA image to lay on the ground plane."""

from __future__ import annotations

import numpy as np
from scipy.ndimage import gaussian_filter

from .palette import rgb


def _noise(rng, n: int, sigma: float) -> np.ndarray:
    f = gaussian_filter(rng.standard_normal((n, n)), sigma, mode="wrap")
    return (f - f.mean()) / (f.std() + 1e-9)


def watercolor(
    color: str, n: int = 1024, seed: int = 3, coverage: float = 0.72
) -> np.ndarray:
    """
    Return an (n, n, 4) float image: one wash of `color` on transparent paper.

    Ragged blot edge, pigment pooled darker along it, blotchy density inside, fine
    granulation. `coverage` is the blot radius as a fraction of the half-width.
    """
    rng = np.random.default_rng(seed)
    y, x = np.mgrid[-1 : 1 : n * 1j, -1 : 1 : n * 1j]
    r = np.hypot(x * 0.92, y) / coverage
    ragged = r + 0.10 * _noise(rng, n, n / 14) + 0.035 * _noise(rng, n, n / 60)
    mask = np.clip((1.0 - ragged) / 0.04, 0, 1)  # hard-ish wet edge
    mask = gaussian_filter(mask, n / 700)
    rim = np.clip(
        mask - gaussian_filter(mask, n / 90), 0, 1
    )  # pigment pools at the edge
    blotch = 0.5 + 0.18 * _noise(rng, n, n / 10) + 0.08 * _noise(rng, n, n / 35)
    grain = 0.035 * _noise(rng, n, 1.6)
    density = 0.8 * np.clip(mask * (blotch + grain) + 1.6 * rim, 0, 1)
    img = np.empty((n, n, 4))
    img[..., :3] = rgb(color)
    img[..., 3] = density
    return img
