#!/usr/bin/env bash
# Regenerate the AiC paper figures into wiki/diagrams/aic/.
#   ./make_paper_figures.sh            all, final (600 dpi, Blender line art)
#   ./make_paper_figures.sh preview    all, 150 dpi VTK "cad" look (seconds, no Blender)
#   ./make_paper_figures.sh fig4 ...   only the named ones: fig2 fig3 fig4 fig5 fig7 fig8 fig9 fig11
# Manual Inkscape edits on the .svg are lost on regeneration: re-apply them.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
ws=$(cd "$here/../../.." && pwd)
dest=$ws/wiki/diagrams/aic
out=$here/out
mode=final
if [[ ${1:-} == preview ]]; then mode=preview; shift; fi
figs=("$@")
[[ ${#figs[@]} -gt 0 ]] || figs=(fig2 fig3 fig4 fig5 fig7 fig8 fig9 fig11)
if [[ $mode == preview ]]; then args=(150 cad); else args=(600); fi
export PYTHONPATH=$here:$ws/timber_crane_mujoco_py${PYTHONPATH:+:$PYTHONPATH}
mkdir -p "$out" "$dest"
cd "$here"
for f in "${figs[@]}"; do
  case $f in
    fig2) python3 examples/fig_kinematics_pzs.py "$out" "${args[@]}"
          names=(fig_kinematics_pzs_a fig_kinematics_pzs_b) ;;
    fig3) python3 examples/fig_overview.py "$out"
          names=(fig_overview) ;;
    fig4) python3 examples/fig_c3_validation.py "$out"  # data plot: dpi/look do not apply
          names=(fig_c3_validation) ;;
    fig5) python3 examples/fig_detector_stages.py "$out"  # needs the built detector runner
          names=(fig_detector_stages) ;;
    fig7) python3 examples/fig_planner_move.py "$out" "${args[@]}"  # ROS sourced
          names=(fig_planner_move fig_planner_move_b) ;;
    fig8) python3 examples/fig_timing_ocp.py "$out"  # needs ROS sourced (planner)
          names=(fig_timing_ocp) ;;
    fig9) python3 examples/fig_timing.py "$out"
          names=(fig_timing) ;;
    fig11) python3 examples/fig_wall_sequence.py "$out" "${args[@]}"
          names=(fig_wall_sequence fig_wall_sequence_{a,b,c,d,e,f}) ;;
    *) echo "unknown figure $f (fig2 fig3 fig4 fig5 fig7 fig8 fig9 fig11)" >&2; exit 1 ;;
  esac
  if [[ $mode == preview ]]; then echo "$f: preview in $out"; continue; fi
  for n in "${names[@]}"; do
    for ext in png svg; do
      if [[ -f $out/$n.$ext ]]; then cp "$out/$n.$ext" "$dest/"; fi
    done
  done
  echo "$f -> $dest"
done
