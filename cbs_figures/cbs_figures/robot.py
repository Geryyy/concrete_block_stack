"""
URDF forward kinematics for display: link poses, visual meshes, joint axes.

Numpy only. Cylinder chains closed by `crane_model.linkage`; mimic joints honoured.
Configuration is the canonical eight (hydraulics.yaml `joints`), names or array.
"""

from __future__ import annotations

import os
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np

SRC = Path(__file__).resolve().parents[3]  # <ws>/src
CBS = SRC / "concrete_block_stack"
#: `xacro epsilon_crane_description/urdf/crane.urdf.xacro tool:=pzs100_description gazebo:=false
#: load_ros2_control:=false` -- crane + PZS100 + perception kit. Regenerate, never edit.
DEFAULT_URDF = Path(__file__).resolve().parent / "data/crane_pzs100.urdf"

#: Canonical eight q1..q8, contract order (URDF names). The q5_small_telescope and
#: q11_right_rail URDF joints follow by mimic.
JOINTS = (
    "theta1_slewing_joint",
    "theta2_boom_joint",
    "theta3_arm_joint",
    "q4_big_telescope",
    "theta6_tip_joint",
    "theta7_tilt_joint",
    "theta8_rotator_joint",
    "q9_left_rail_joint",
)
#: K0 sits this far above ground (world_model.yaml), m.
GROUND_Z = -1.1407


def _linkage():
    """crane_model.linkage alone: the package __init__ pulls pinocchio/coal, linkage is numpy."""
    import importlib
    import types

    name = "_cbs_figures_crane_model"
    if name not in sys.modules:
        pkg = types.ModuleType(name)
        pkg.__path__ = [str(CBS / "crane_model/crane_model")]
        sys.modules[name] = pkg
    return importlib.import_module(name + ".linkage")


linkage = _linkage()


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = (
        np.cos(r),
        np.sin(r),
        np.cos(p),
        np.sin(p),
        np.cos(y),
        np.sin(y),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def axis_angle(axis, angle: float) -> np.ndarray:
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K


@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float = -np.inf
    upper: float = np.inf
    mimic: tuple[str, float, float] | None = None


@dataclass
class Visual:
    link: str
    origin: np.ndarray
    mesh: str  # absolute path
    scale: np.ndarray


@lru_cache(maxsize=None)
def _packages() -> dict[str, Path]:
    """Map package name to source dir, from every package.xml under src/."""
    found: dict[str, Path] = {}
    for root, dirs, files in os.walk(SRC):
        dirs[:] = [d for d in dirs if d not in ("build", "install", ".git", "log")]
        if "package.xml" in files:
            m = re.search(
                r"<name>\s*([^<\s]+)\s*</name>", Path(root, "package.xml").read_text()
            )
            if m:
                found.setdefault(m.group(1), Path(root))
    return found


def resolve(uri: str) -> str:
    if uri.startswith("package://"):
        pkg, rel = uri[len("package://") :].split("/", 1)
        return str(_packages()[pkg] / rel)
    return uri.removeprefix("file://")


def _origin(el) -> np.ndarray:
    o = el.find("origin") if el is not None else None
    if o is None:
        return np.eye(4)
    xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
    return transform(xyz, rpy)


class Robot:
    """Kinematic tree of one URDF. `fk(q)` -> {link: 4x4 in K0}."""

    def __init__(self, urdf: str | Path = DEFAULT_URDF):
        root = ET.parse(urdf).getroot()
        self.joints: dict[str, Joint] = {}
        for j in root.findall("joint"):
            lim, mim = j.find("limit"), j.find("mimic")
            ax = j.find("axis")
            self.joints[j.get("name")] = Joint(
                name=j.get("name"),
                type=j.get("type"),
                parent=j.find("parent").get("link"),
                child=j.find("child").get("link"),
                origin=_origin(j),
                axis=np.array([float(v) for v in ax.get("xyz").split()])
                if ax is not None
                else np.array([1.0, 0, 0]),
                lower=float(lim.get("lower", -np.inf)) if lim is not None else -np.inf,
                upper=float(lim.get("upper", np.inf)) if lim is not None else np.inf,
                mimic=(
                    mim.get("joint"),
                    float(mim.get("multiplier", 1)),
                    float(mim.get("offset", 0)),
                )
                if mim is not None
                else None,
            )
        self.visuals: list[Visual] = []
        for link in root.findall("link"):
            for v in link.findall("visual"):
                m = v.find(
                    "geometry/mesh"
                )  # primitives: none worth drawing in this URDF
                if m is not None:
                    scale = np.array(
                        [float(s) for s in m.get("scale", "1 1 1").split()]
                    )
                    self.visuals.append(
                        Visual(
                            link.get("name"),
                            _origin(v),
                            resolve(m.get("filename")),
                            scale,
                        )
                    )
        children = {j.child for j in self.joints.values()}
        self.root = next(
            j.parent for j in self.joints.values() if j.parent not in children
        )
        self._order = self._topological()
        self._geometry = linkage.load_geometry(
            str(CBS / "crane_model/config/hydraulics.yaml")
        )

    def _topological(self) -> list[Joint]:
        out, frontier = [], [self.root]
        by_parent: dict[str, list[Joint]] = {}
        for j in self.joints.values():
            by_parent.setdefault(j.parent, []).append(j)
        while frontier:
            link = frontier.pop()
            for j in by_parent.get(link, []):
                out.append(j)
                frontier.append(j.child)
        return out

    def values(self, q, chains: bool = True) -> dict[str, float]:
        """Every movable joint value: canonical q + mimics + closed cylinder chains."""
        if isinstance(q, dict):
            vals = {k: float(v) for k, v in q.items()}
        else:
            vals = dict(zip(JOINTS, map(float, q)))
        vals.setdefault("q9_left_rail_joint", 0.0)
        if chains:
            with np.errstate(invalid="ignore"):
                cyl = linkage.cylinder_joints(
                    self._geometry, vals.get(JOINTS[1], 0.0), vals.get(JOINTS[2], 0.0)
                )
            if not all(np.isfinite(v) for v in cyl.values()):
                raise ValueError(
                    "q2/q3 outside the boom four-bar's reach, no cylinder pose"
                )
            vals.update(cyl)
        for j in self.joints.values():
            if j.mimic:
                src, mul, off = j.mimic
                vals[j.name] = mul * vals.get(src, 0.0) + off
        return vals

    def fk(self, q, chains: bool = True) -> dict[str, np.ndarray]:
        """Pose of every link in K0; `chains=False` skips the cylinders (IK inner loop)."""
        vals = self.values(q, chains)
        poses = {self.root: np.eye(4)}
        for j in self._order:
            T = poses[j.parent] @ j.origin
            v = vals.get(j.name, 0.0)
            if j.type in ("revolute", "continuous"):
                M = np.eye(4)
                M[:3, :3] = axis_angle(j.axis, v)
                T = T @ M
            elif j.type == "prismatic":
                M = np.eye(4)
                M[:3, 3] = j.axis * v
                T = T @ M
            poses[j.child] = T
        return poses

    def joint_frame(self, name: str, q) -> tuple[np.ndarray, np.ndarray]:
        """Joint origin and axis direction in K0 (the axis the joint moves about/along)."""
        j = self.joints[name]
        T = self.fk(q)[j.parent] @ j.origin
        return T[:3, 3], T[:3, :3] @ (j.axis / np.linalg.norm(j.axis))

    def hang(self, q, branch: bool = True) -> np.ndarray:
        """Return q with tip/tilt (q5, q6 = q[4:6]) at rest: rotator axis (TCP z) straight down."""
        from scipy.optimize import least_squares

        q = np.array(q, float)

        def z(x):
            q[4:6] = x
            return self.fk(q, chains=False)["K8_tool_center_point"][:3, 2]

        seeds = [q[4:6].copy()]
        if branch:
            seeds += [
                q[4:6] + [0, np.pi],
                q[4:6] + [np.pi, 0],
                np.array([0, np.pi / 2]),
            ]
        best = None
        for x0 in seeds:
            x = least_squares(lambda x: z(x)[:2], x0).x
            if best is None or z(x)[2] < z(best)[2]:
                best = x
        q[4:6] = wrap(best)
        return q

    def reach(self, p_tcp, yaw: float = 0.0, q0=None, tool: float = 0.0) -> np.ndarray:
        """
        Solve for the hanging TCP at `p_tcp` (K0) and tool yaw `yaw` about world z.

        Least squares over q1..q4 and q8 inside URDF limits, tip/tilt hanging; `q0` picks
        the branch (default: boom up, arm out). For pictures only.
        """
        from scipy.optimize import least_squares

        q = self.hang(
            q0 if q0 is not None else [0.0, 0.3, 2.0, 0.5, 0, np.pi / 2, 0, 0]
        )
        q[7] = tool
        idx = [0, 1, 2, 3, 6]
        lo = np.array([max(self.joints[JOINTS[i]].lower, -1e3) for i in idx])
        hi = np.array([min(self.joints[JOINTS[i]].upper, 1e3) for i in idx])
        target = np.asarray(p_tcp, float)

        def res(x):
            q[idx] = x
            q[:] = self.hang(q, branch=False)
            T = self.fk(q, chains=False)["K8_tool_center_point"]
            e = np.arctan2(T[1, 0], T[0, 0]) - yaw
            return np.r_[
                T[:3, 3] - target, np.arctan2(np.sin(e), np.cos(e)), T[2, 2] + 1
            ]

        q[idx] = least_squares(
            res, np.clip(q[idx], lo + 1e-6, hi - 1e-6), bounds=(lo, hi)
        ).x
        q[6] = wrap(q[6])
        return self.hang(q, branch=False)
