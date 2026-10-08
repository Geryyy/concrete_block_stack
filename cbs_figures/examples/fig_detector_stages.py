"""
Fig. 5 (paper draft): lidar detector stages on one Seibersdorf scan.

Runs the built offline runner (concrete_block_detector_snapshot_runner --dump-stages) with
the package's detector.yaml, then draws raw -> ground removed -> clusters -> planes -> boxes
in one oblique orthographic view (world frame). Plane/box colours follow each candidate's
fate in the runner JSON (raw_lineage).

    python3 examples/fig_detector_stages.py out [snapshot-dir]
"""

import json
import subprocess
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml
from cbs_figures import palette as pal

WS = Path(__file__).resolve().parents[4]
RUNNER = WS / "install/concrete_block_detector/lib/concrete_block_detector"
RUNNER = RUNNER / "concrete_block_detector_snapshot_runner"
PARAMS = WS / "src/concrete_block_stack/concrete_block_detector/config/detector.yaml"
SNAPSHOT = Path.home() / (
    "Documents/seibersdorf_dataset/2026_07_07_static__1783427931_224065536"
)
EL, HALF = 40.0, 2.2  # view elevation (deg; camera's own is ~25), half window (m)
AZ_OFFSET = -30.0  # deg off the camera azimuth: row diagonal as in the photo, same side
CROP = (
    0.37,
    0.71,
    0.40,
    1.0,
)  # rgb.png rows / cols as fractions: blocks, pallets, gripper

out = Path(sys.argv[1] if len(sys.argv) > 1 else "out")
snap = Path(sys.argv[2]) if len(sys.argv) > 2 else SNAPSHOT
dump = out / "detector_stages"
run = subprocess.run(
    [RUNNER, "--params", PARAMS, "--snapshot", snap, "--dump-stages", dump],
    check=True,
    capture_output=True,
    text=True,
)
result = json.loads(run.stdout)["snapshots"][0]
stage = {
    n: np.loadtxt(dump / snap.name / f"{n}.xyz", ndmin=2)
    for n in ("raw", "input", "above", "clusters", "planes")
}

# view: orthographic, world z up, azimuth of the camera that took rgb.png
tf = yaml.safe_load((snap / "tf.yaml").read_text())
cam = next(t for t in tf["transforms"] if t["child"] == tf["image_frame"])
axis = np.array(cam["matrix"])[:3, 2]  # optical axis in world
az, el = np.arctan2(-axis[1], -axis[0]) + np.radians(AZ_OFFSET), np.radians(EL)
fwd = -np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
right = np.cross(fwd, [0, 0, 1])
right /= np.linalg.norm(right)
up = np.cross(right, fwd)
final = [l for l in result["raw_lineage"] if l["fate"] == "final"]
centre = np.mean([l["synthesized_pose"]["position"] for l in final], axis=0)


def view(p):
    """(N,3) world -> (u, v, depth)."""
    d = np.asarray(p)[:, :3] - centre
    return d @ right, d @ up, d @ fwd


def scatter(ax, p, color, s=0.15, **kw):
    if len(p) == 0:
        return
    u, v, depth = view(p)
    order = np.argsort(-depth)  # far first
    c = color[order] if isinstance(color, np.ndarray) and color.ndim > 1 else color
    ax.scatter(u[order], v[order], s=s, c=c, lw=0, rasterized=True, **kw)


def box_edges(pose):
    R, p, d = np.array(pose["rotation"]), np.array(pose["position"]), pose["dims"]
    c = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) / 2
    pts = p + (c * d) @ R.T
    edges = [
        (i, j) for i in range(8) for j in range(i + 1, 8) if bin(i ^ j).count("1") == 1
    ]
    return [pts[[i, j]] for i, j in edges]


def draw_box(ax, pose, **kw):
    for e in box_edges(pose):
        u, v, _ = view(e)
        ax.plot(u, v, solid_capstyle="round", **kw)


seq = pal.SEQUENTIAL
pal.use()
fig, (photo, *axs) = plt.subplots(1, 6, figsize=(pal.WIDTH["double"], 1.3))
titles = ["raw scan", "ground removed", "clusters", "planes", "boxes"]

# camera image of the same scene, for orientation only (not a detector input here)
im = plt.imread(snap / "rgb.png")
H, W = im.shape[:2]
r0, r1, c0, c1 = (int(f * n) for f, n in zip(CROP, (H, H, W, W)))
photo.imshow(im[r0:r1, c0:c1], interpolation="antialiased")
photo.set_title("camera", fontsize=8, pad=2)
photo.axis("off")

# (a) raw: everything the lidar returned inside the window, by height
raw = stage["raw"]
z = np.clip((raw[:, 2] - centre[2] + 1.0) / 2.5, 0, 1)
scatter(axs[0], raw, seq(0.15 + 0.8 * z))

# (b) ground removed: ground pale, above-ground by height above ground
above = stage["above"]
ground = stage["input"]
scatter(axs[1], ground, pal.GRID)
h = np.clip(above[:, 3] / 0.8, 0, 1)
scatter(axs[1], above, seq(0.3 + 0.7 * h), s=0.3)

# (c) clusters: accepted DBSCAN components; other seed points muted
scatter(axs[2], above, pal.MUTED)
cl = stage["clusters"]
for k in np.unique(cl[:, 3]).astype(int):
    scatter(axs[2], cl[cl[:, 3] == k], pal.CATEGORICAL[k % 3], s=0.4)

# (d) planes: final top / side patches, other patches muted
role = {}
for l in final:
    c, r = l["proposal_component"], l["region"]
    role[(c, r, l["top_plane"])] = pal.BLUE
    if l["side_plane"] is not None:
        role[(c, r, l["side_plane"])] = pal.TEAL
pl = stage["planes"]
scatter(axs[3], above, pal.GRID)
key = [tuple(k) for k in pl[:, 3:6].astype(int)]
other = np.array([k not in role for k in key])
scatter(axs[3], pl[other], pal.MUTED, s=0.4)
for k, color in role.items():  # drawn last: on top where patches overlap
    scatter(axs[3], pl[np.array([kk == k for kk in key])], color, s=0.4)

# (e) boxes: rejected candidates dashed, final boxes solid
scatter(axs[4], above, pal.MUTED)
for l in result["raw_lineage"]:
    if l["fate"] != "final":
        draw_box(
            axs[4], l["synthesized_pose"], color=pal.RED, lw=0.35, ls=(0, (2, 1.5))
        )
for p in result["poses"]:
    draw_box(axs[4], p, color=pal.BLUE, lw=0.8)

for ax, t in zip(axs, titles):
    ax.set_title(t, fontsize=8, pad=2)
    ax.set_xlim(-HALF, HALF)
    ax.set_ylim(-HALF * 0.75, HALF * 0.75)  # 4:3, as the photo crop
    ax.set_aspect("equal")
    ax.axis("off")
fig.subplots_adjust(left=0.005, right=0.995, top=0.88, bottom=0.01, wspace=0.1)
n_rej = sum(l["fate"] != "final" for l in result["raw_lineage"])
print(
    f"{snap.name}: {len(result['poses'])} boxes, {n_rej} rejected candidates, "
    f"{len(np.unique(cl[:, 3]))} clusters, {len(pl)} plane points"
)
for ext in ("pdf", "svg", "png"):
    fig.savefig(out / f"fig_detector_stages.{ext}")
