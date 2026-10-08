"""Fig. 2 draft: crane + PZS100 with frames K0..K8, lidar frame, passive pair q5/q6."""

import sys

from cbs_figures import GROUND_Z, Scene

out = sys.argv[1] if len(sys.argv) > 1 else "out"
s = Scene(look=sys.argv[2] if len(sys.argv) > 2 else "cad")
q = s.robot.reach(
    [6.0, 0.0, GROUND_Z + 1.45], yaw=0.0
)  # gripped block just above ground
s.ground()
s.crane(q)
s.block_in_gripper(q, color="block_active")
for link, k in [
    ("K0_mounting_base", "0"),
    ("K1_slewing_column", "1"),
    ("K2_boom", "2"),
    ("K3_arm", "3"),
    ("K5_inner_telescope", "5"),
    ("K8_tool_center_point", "8"),
]:
    s.link_frame(link, q, rf"$\mathcal{{K}}_{k}$", length=0.5)
s.link_frame("livox_frame", q, r"$\mathcal{K}_\mathrm{L}$", length=0.4)
s.joint("theta6_tip_joint", q, r"$q_5$", radius=0.18)
s.joint("theta7_tilt_joint", q, r"$q_6$", radius=0.18)
s.camera(azimuth=-75, elevation=12)
for ext in ("pdf", "png"):
    s.save(f"{out}/kinematic_sketch_{s.look}.{ext}", width="double", aspect=0.5)
