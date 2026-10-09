# cbs_figures — figures for the AiC paper

One palette for plots, diagrams and renders, plus stylised 3D figures built from the
real CAD meshes (crane, PZS100, perception kit, blocks). Not a ROS package
(`COLCON_IGNORE`); plain Python, no build.

```bash
cd src/concrete_block_stack/cbs_figures
PYTHONPATH=$PWD python3 examples/kinematic_sketch.py out   # out/ is gitignored
PYTHONPATH=$PWD python3 examples/pickup_sequence.py out
PYTHONPATH=$PWD python3 examples/palette_demo.py out       # also rewrites latex/cbs_colors.sty
PYTHONPATH=$PWD:<ws>/timber_crane_mujoco_py python3 examples/fig_c3_validation.py out  # Fig. 4b, HydraulicCalib bags
```

Use `PYTHONPATH=$PWD:$PYTHONPATH` if ROS is sourced. Needs numpy, scipy, matplotlib,
vtk, trimesh (all in the devcontainer). The package also reads files from the source
tree, not from an install: `crane_model/crane_model/linkage.py` (loaded alone, without
pinocchio), `crane_model/config/hydraulics.yaml`, and the mesh packages found by walking
`src/`. The first `Scene()` takes a few seconds for that walk. VTK renders offscreen
but needs an X display (`$DISPLAY`); there is no xvfb in the container. `save()` calls
`palette.use()`, which sets matplotlib's global rcParams.

## Paper figures — how to redo them

```bash
./make_paper_figures.sh                 # all, final: 600 dpi, Blender line art -> wiki/diagrams/aic/
./make_paper_figures.sh preview         # all, 150 dpi VTK "cad" look into out/, no Blender, seconds
./make_paper_figures.sh fig4 fig11      # only these
```

| fig | script | inputs | renderer | time |
|---|---|---|---|---|
| 2 (a, b) | `examples/fig_kinematics_pzs.py` | `data/crane_pzs100.urdf` + CAD meshes in `src/` | Blender line art + matplotlib overlay | ~2 min |
| 3 | `examples/fig_overview.py` | block diagram after the ICRA26 overview; components and rates from paper §3-§8 | matplotlib | seconds |
| 4 (b) | `examples/fig_c3_validation.py` | `~/Documents/HydraulicCalib/bags` (read-only), `wiki/diagrams/hydraulic_calibration/c3_full_model.json`, code from `timber_crane_mujoco_py` | matplotlib | ~1 min with the bag cache, ~15 min first run (decodes into `~/.cache/crane_c3_bags`) |
| 5 | `examples/fig_detector_stages.py` | `~/Documents/seibersdorf_dataset/<scan>` (read-only), `concrete_block_detector/config/detector.yaml`; runs the built `concrete_block_detector_snapshot_runner --dump-stages` (needs `colcon build --packages-select concrete_block_detector` on a branch with that flag) | matplotlib | ~10 s |
| 7 | `examples/fig_planner_move.py` | same move (`examples/carry_move.py`): executed TCP path, sway box swept along it (shaded), (b) swing extremes at goal; ROS sourced | Blender line art | ~2 min |
| 8 | `examples/fig_timing_ocp.py` | deployed `crane_planning` Planner offline, the move in `examples/carry_move.py` (first `example_wall` carry, 800 kg, staged blocks as scene); ROS sourced for pinocchio/acados | matplotlib | ~10 s |
| 9 | `examples/fig_timing.py` | timing numbers from `wiki/mpc.md` (hard-coded: $D=T_s=60$ ms, 100 Hz) | matplotlib | seconds |
| 11 (a–f) | `examples/fig_wall_sequence.py` | `example_wall` in `concrete_block_assembly_planning/config/wall_plans.yaml`, staging seed `world_model_seed_pick_place.yaml`, T_world_K0 | Blender line art | ~6 min |

Each writes `.png` (wiki, preview) and `.svg` (Inkscape); Fig. 2 also `.pdf`. Fig. 4a (block
diagram) and Inkscape touch-ups are manual and **lost on regeneration** — keep a note of
them in the figure's wiki entry. Change a pose, camera or label in the script, not in
the SVG, when it should survive the next run. If the plan, URDF or C3 fit changes, rerun the
matching line; the wiki entry names the script, so a figure is never orphaned.

Needs: Blender 4.2 (see below) for the line art; `timber_crane_mujoco_py` checked out at the
workspace root for Fig. 4 (the script puts it on `PYTHONPATH`); `$DISPLAY` for VTK.

## Palette — `cbs_figures.palette`

| name | hex | use |
|---|---|---|
| `BLUE` | `#00669A` | TU blue, slot 1, MPC |
| `RED` | `#BA122B` | ACIN red, slot 2, baseline (PID / FK-only / operator) |
| `OCHRE` | `#B07D0A` | slot 3, model / simulation |
| `TEAL` | `#00897B` | slot 4 |
| `VIOLET` | `#851F83` | slot 5 |
| `INK` / `GRAY` / `MUTED` / `GRID` | | text + measured / reference (dashed) / ghosts / grid |
| `TOLERANCE` | `#F3E6C4` | tolerance band, settle box |
| `AXIS` | x/y/z | frame triads |
| `MATERIAL[...]` | | render colours: machine, tool, sensor, block, block_active, ghost, ground, edge |

Rules:
- Assign categorical slots in order, never cycle. Validated (CVD-safe, ≥3:1 on white)
  for 5 series on lines and bars. Scatter plots, or any plot where two arbitrary series
  can end up side by side: **3 series max**.
- Prefer `ROLE["mpc" | "baseline" | "reference" | "measured" | "model" | "tolerance"]`
  so a role keeps its colour across every figure.
- Continuous data: `cbs_seq` (one hue, light→dark) and `cbs_div` (blue | neutral | red).
  No rainbow maps.
- `palette.use()` sets the matplotlib style: 8 pt serif with CM math, Elsevier widths
  `WIDTH["single" | "onehalf" | "double"]` (90/140/190 mm), 600 dpi, no top/right spines.
- In LaTeX: `\usepackage{cbs_colors}` → `cbsBlue`, `cbsRed`, …, `cbsAxisX`, `cbsMachine`.
  Generated file, regenerate with `palette_demo.py`.

## 3D scenes — `cbs_figures.Scene`

Three looks, same API, same overlay:

| `Scene(look=…)` | renderer | for |
|---|---|---|
| `"lineart"` | Blender Freestyle: flat fills, ink outlines | paper figures (Fig. 2) |
| `"cad"` | VTK, shaded + outlines | fast previews (seconds); tune pose/camera/labels here first |
| `"wire"` | VTK, pastel wire cages, watercolour ground | experimental |

`lineart` needs Blender 4.2 LTS: `$CBS_BLENDER`, else `blender` on PATH, else
`~/.local/opt/blender-*/blender` (portable tarball from download.blender.org, no root).
It renders once per scene and size; a double-column figure at 600 dpi takes about 1 min.
Workflow: iterate with `look="cad"`, render `lineart`, save `.svg`, polish labels in
Inkscape. Example: `examples/fig_kinematics_pzs.py out 150 cad` (preview) and
`… out 600` (final).

All coordinates are in K0 (crane mounting base), in metres. Ground is at `GROUND_Z`
(−1.1407). `q` is the canonical eight
`(q1 slew, q2 boom, q3 arm, q4 telescope, q5 tip, q6 tilt, q7 rotator, q8 tool)`, numbered as in
`wiki/nomenclature.md`
(`robot.JOINTS`). The cylinders, the q5 mimic and the q11 mimic are filled in for you.

```python
from cbs_figures import GROUND_Z, Scene, palette as pal

s = Scene()
q = s.robot.reach([6, 0, GROUND_Z + 1.45], yaw=0.0)  # TCP hanging at a point, gripper yaw
s.ground()
s.crane(q)                                   # style="solid" | "ghost" | "outline"; ghost ignores color
s.block_in_gripper(q, color=pal.MATERIAL["block_active"])
s.block_on_ground(7.0, 1.0, yaw=0.3, course=0)
s.link_frame("K8_tool_center_point", q, r"$\mathcal{K}_8$", length=0.5)  # axis_labels=True for x/y/z
s.joint("theta6_tip_joint", q, r"$q_6$", radius=0.18)   # revolute arc, prismatic arrow
s.path(points, color=pal.BLUE, label="TCP")             # (N,3) K0 points
s.arrow(a, b, label=r"$\Delta p$"); s.label(p, "text", offset=(4, 4))
s.label(s.link_center("K2_boom", q), "boom", offset=(-40, 10), leader=True)  # S-curve leader
s.camera(azimuth=-60, elevation=18)          # also fov=, ortho=, margin=; target= (+ distance=) fixes the view
s.save("fig.pdf", width="double", aspect=0.5)   # .pdf/.svg: raster bodies + vector overlays; dpi=600
```

- `robot.reach(p, yaw, q0=…, tool=q8)`: inverse kinematics for pictures only. q1–q4 and q7 are
  fitted by least squares within the URDF limits, with tip and tilt hanging. `q0`
  selects the branch. For a sequence, seed each step with the previous solution so
  the poses stay continuous.
- `robot.hang(q)` puts tip and tilt at rest, with the rotator axis vertical. This is a
  picture of the equilibrium, not the dynamics.
- `robot.fk(q)` returns a 4×4 pose per link and raises if q2/q3 lie outside the boom
  four-bar's reach; `robot.joint_frame(name, q)` returns the
  joint's origin and axis.
- Frames that are available include `K0_mounting_base`, `K1_slewing_column`, `K2_boom`,
  `K3_arm`, `K4_outer_telescope`, `K5_inner_telescope`, `K6_double_joint_link`,
  `K7_rotator_upper_part`, `K8_tool_center_point`, `livox_frame`, `seyond`,
  `imu15_origin`, `imu16_origin`. Symbol names follow `wiki/nomenclature.md`.
- Callouts: `link_center(link, q)` is the centre of the link's CAD body; `leader=True`
  draws an engineering-book S-curve from the label to it.
- Labels are mathtext. Move a label that collides with the drawing using `offset=(dx, dy)`
  in points. Without `target`, the camera pans and zooms so that all bodies and
  annotations fit. `save()` then scales and shifts the drawing until no label, arrow
  or arc is cut off, keeping the exact page size. A side where the render itself is
  cut (a close-up) is left alone. `save(..., fit=False)` keeps the camera framing exactly (sequences).
- `Scene.feature_angle` (default 60°) sets how many CAD crease lines are drawn. Lower
  it for more detail, raise it for a cleaner look.
- Ghosts are pale and see-through, which suits earlier and later poses. `outline`
  draws edges only.

The block is drawn 0.9×0.6×0.6 m, with two frustum studs (0.15→0.05 m, 0.05 m high,
±0.2 m along the long axis, taken from the paper draft). The recesses underneath are
not modelled. Grasp offset: `TCP_TO_BLOCK` comes from `concrete_block_detector/config/detector.yaml`.

**Description**: `cbs_figures/data/crane_pzs100.urdf` is the crane with the PZS100 and
the perception kit, generated from the current xacro. Regenerate it, do not edit it:

```bash
source install/setup.bash
xacro src/epsilon_crane_description/urdf/crane.urdf.xacro tool:=pzs100_description \
  gazebo:=false load_ros2_control:=false > src/concrete_block_stack/cbs_figures/cbs_figures/data/crane_pzs100.urdf
```

## Checking your figure

Render the PNG and look at it before you hand it over: label collisions, framing,
and frames hidden inside meshes. The PDF warning `'created' timestamp seems very low`
comes from fontTools when it embeds fonts and is harmless.
