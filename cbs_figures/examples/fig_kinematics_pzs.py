"""
Fig. 2 (paper draft): crane + PZS100 kinematics, Blender line art + vector overlay.

(a) overview: K0..K5, lidar, q1..q4.  (b) passive pair + tool: K5..K12, IMUs, q5..q8.
Side by side in the paper: (a) 130 mm, (b) 55 mm wide, both 75 mm high.
"""

import sys

from cbs_figures import Scene

out = sys.argv[1] if len(sys.argv) > 1 else "out"
dpi = int(sys.argv[2]) if len(sys.argv) > 2 else 600
look = sys.argv[3] if len(sys.argv) > 3 else "lineart"  # "cad": fast VTK preview
K = r"$\mathcal{K}_{%s}$"


def scene():
    s = Scene(look=look)
    q = s.robot.hang([0.0, 0.8, 0.5, 1.0, 0.0, 1.571, 1.0, 0.3])  # boom up, arm down
    s.crane(q)
    return s, q


# --- (a) overview -------------------------------------------------------------------------
s, q = scene()
P = s.robot.fk(q)


def at(link):
    return P[link][:3, 3]


for link, k, off in [
    ("K0_mounting_base", "0", (6, -12)),
    ("K1_slewing_column", "1", (-18, 6)),
    ("K2_boom", "2", (-20, 4)),
    ("K3_arm", "3", (6, -14)),
    ("K4_outer_telescope", "4", (-4, -16)),
    ("K5_inner_telescope", "5", (8, 2)),
]:
    s.link_frame(link, q, K % k, length=0.55, offset=off)
s.link_frame("livox_frame", q, length=0.4)
s.joint("theta1_slewing_joint", q, "$q_1$", radius=0.45, offset=(-16, -6))
s.joint("theta2_boom_joint", q, "$q_2$", radius=0.3, offset=(10, -12))
s.joint("theta3_arm_joint", q, "$q_3$", radius=0.3, offset=(6, 6))
o, axis = s.robot.joint_frame("q4_big_telescope", q)
mid = (at("K4_outer_telescope") + at("K3_arm")) / 2
s.arrow(mid - 0.4 * axis, mid + 0.4 * axis, label="$q_4$", offset=(-2, 6))
for link, text, off in [
    ("K1_slewing_column", "slewing column", (-34, -14)),
    ("K2_boom", "boom", (-40, 10)),
    ("K3_arm", "arm", (10, 26)),
    ("K5_inner_telescope", "telescope", (14, 24)),
    ("K8_tool_center_point", "PZS100", (34, 4)),
    ("seyond", r"lidar $\mathcal{K}_\mathrm{L}$", (-34, 22)),
]:
    p = s.link_center(link, q)
    s.label(p, text, offset=off, leader=True)
s.camera(azimuth=-70, elevation=14)
for ext in ("pdf", "svg", "png"):
    s.save(f"{out}/fig_kinematics_pzs_a.{ext}", width=130 / 25.4, aspect=0.58, dpi=dpi)

# --- (b) passive pair and tool --------------------------------------------------------------
s, q = scene()
P = s.robot.fk(q)
for link, k, length, off in [
    ("K5_inner_telescope", "5", 0.25, (8, 6)),
    ("K10_left_rail", "10", 0.2, (-22, -6)),
    ("K12_right_rail", "12", 0.2, (6, -8)),
]:
    s.link_frame(link, q, K % k, length=length, offset=off)
s.link_frame("K8_tool_center_point", q, length=0.25)  # label off the plate, on a leader
s.label(P["K8_tool_center_point"][:3, 3], K % "8", offset=(-34, 30), leader=True)
s.link_frame("imu15_origin", q, K % "15", length=0.15, offset=(-26, 4))
s.link_frame("imu16_origin", q, K % "16", length=0.15, offset=(-26, -6))
s.joint("theta6_tip_joint", q, "$q_5$", radius=0.1, offset=(14, 4))
s.joint("theta7_tilt_joint", q, "$q_6$", radius=0.1, offset=(24, 8))
s.joint("theta8_rotator_joint", q, "$q_7$", radius=0.18, offset=(16, -18))
o, axis = s.robot.joint_frame("q9_left_rail_joint", q)
foot = P["K10_left_rail"][:3, 3] + [0, 0, 0.35]
s.arrow(foot, foot + 0.3 * axis, label="$q_8$", offset=(2, 4))
s.camera(
    azimuth=-40,
    elevation=12,
    target=P["K8_tool_center_point"][:3, 3] + [0, 0, 0.15],
    distance=8.5,
)
for ext in ("pdf", "svg", "png"):
    s.save(f"{out}/fig_kinematics_pzs_b.{ext}", width=55 / 25.4, aspect=1.37, dpi=dpi)
