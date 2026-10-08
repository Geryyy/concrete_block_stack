"""
Stylised 3D figures from the CAD meshes.

VTK renders the bodies, matplotlib draws frames, joints, paths and labels on top as
vectors. Coordinates are K0, metres.
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import vtk
from matplotlib.patches import FancyArrowPatch, PathPatch
from matplotlib.path import Path as MplPath
from matplotlib.transforms import ScaledTranslation
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray, vtk_to_numpy

from . import palette as pal
from .robot import GROUND_Z, Robot, transform

#: Concrete block, m. Studs: frustum 0.15 -> 0.05 square, 0.05 high, on the long axis.
BLOCK = {"size": (0.9, 0.6, 0.6), "stud_x": (-0.2, 0.2), "stud": (0.15, 0.05, 0.05)}
#: Block centre in the TCP frame when gripped (concrete_block_detector/config/detector.yaml).
TCP_TO_BLOCK = transform((0, 0, 0.85), (np.pi, 0, 0))


def _numpy_matrix(m: vtk.vtkMatrix4x4) -> np.ndarray:
    return np.array([[m.GetElement(i, j) for j in range(4)] for i in range(4)])


def _vtk_matrix(T: np.ndarray) -> vtk.vtkMatrix4x4:
    m = vtk.vtkMatrix4x4()
    for i in range(4):
        for j in range(4):
            m.SetElement(i, j, float(T[i, j]))
    return m


def _polydata(vertices: np.ndarray, faces: np.ndarray) -> vtk.vtkPolyData:
    pd = vtk.vtkPolyData()
    pts = vtk.vtkPoints()
    pts.SetData(numpy_to_vtk(np.ascontiguousarray(vertices, float), deep=True))
    pd.SetPoints(pts)
    cells = np.hstack([np.full((len(faces), 1), 3), faces]).astype(np.int64).ravel()
    ca = vtk.vtkCellArray()
    ca.SetCells(len(faces), numpy_to_vtkIdTypeArray(cells, deep=True))
    pd.SetPolys(ca)
    return pd


@lru_cache(maxsize=None)
def _stl(path: str) -> vtk.vtkPolyData:
    r = vtk.vtkSTLReader()
    r.SetFileName(path)
    r.Update()
    return r.GetOutput()


def blender() -> str:
    """Blender executable: $CBS_BLENDER, else `blender` on PATH, else ~/.local/opt/blender-*."""
    import os
    import shutil

    found = os.environ.get("CBS_BLENDER") or shutil.which("blender")
    if not found:
        local = sorted(Path.home().glob(".local/opt/blender-*/blender"))
        found = str(local[-1]) if local else None
    if not found:
        raise FileNotFoundError("look='lineart' needs Blender; set CBS_BLENDER")
    return found


@lru_cache(maxsize=None)
def _cage(poly: vtk.vtkPolyData, reduction: float, angle: float) -> vtk.vtkPolyData:
    """Crease edges of the decimated mesh: facets read, triangulation diagonals do not."""
    clean = vtk.vtkCleanPolyData()  # STL is triangle soup: merge before decimating
    clean.SetInputData(poly)
    clean.SetTolerance(1e-4)
    deci = vtk.vtkQuadricDecimation()
    deci.SetInputConnection(clean.GetOutputPort())
    deci.SetTargetReduction(reduction)
    edges = vtk.vtkFeatureEdges()
    edges.SetInputConnection(deci.GetOutputPort())
    edges.SetFeatureAngle(angle)
    edges.BoundaryEdgesOn()
    edges.NonManifoldEdgesOff()
    edges.ManifoldEdgesOff()
    edges.ColoringOff()
    edges.Update()
    return edges.GetOutput()


@lru_cache(maxsize=None)
def block_mesh() -> vtk.vtkPolyData:
    """Block with its studs, centre at origin. Recesses underneath are not modelled."""
    import trimesh

    sx, sy, sz = BLOCK["size"]
    base, top, h = BLOCK["stud"]
    parts = [trimesh.creation.box((sx, sy, sz))]
    for x in BLOCK["stud_x"]:
        corners = [
            (x + a * w / 2, b * w / 2, sz / 2 + z)
            for w, z in ((base, 0.0), (top, h))
            for a in (-1, 1)
            for b in (-1, 1)
        ]
        parts.append(trimesh.convex.convex_hull(np.array(corners)))
    m = trimesh.util.concatenate(parts)
    return _polydata(m.vertices, m.faces)


class Scene:
    """Collect bodies and annotations, then `save()`. See README for the vocabulary."""

    def __init__(self, robot: Robot | None = None, look: str = "cad"):
        """look: 'cad' (VTK, shaded) | 'wire' (VTK, pastel cages) | 'lineart' (Blender)."""
        self.robot = robot or Robot()
        self.look = look
        #: 'wire' look: fraction of triangles decimated away before drawing the cage
        self.cage_reduction = 0.85
        #: 'wire' look: crease angle, deg, above which a cage edge is drawn
        self.cage_angle = 25.0
        self._bodies: list[tuple[vtk.vtkPolyData, np.ndarray, dict]] = []
        self._marks: list[tuple] = []
        self._ground = None
        #: crease angle, deg, above which an edge is outlined; lower = more CAD detail
        self.feature_angle = 60.0
        self._cam = dict(
            azimuth=-55.0,
            elevation=18.0,
            target=None,
            distance=None,
            fov=18.0,
            ortho=False,
            margin=0.04,
        )

    # --- bodies -------------------------------------------------------------------------
    def mesh(
        self,
        poly: vtk.vtkPolyData,
        T=np.eye(4),
        color="machine",
        style: str = "solid",
        source: str | None = None,
    ):
        """
        Add a body; style 'solid' | 'ghost' (pale, see-through, ignores color) | 'outline'.

        `source`: the STL the poly came from, so 'lineart' (Blender) can load it.
        """
        st = dict(color=color, style=style, source=source)
        self._bodies.append((poly, np.asarray(T, float), st))

    def crane(self, q, style: str = "solid", below: str | None = None):
        """Add the machine at configuration q (canonical eight); `below`: only that joint's subtree."""
        P = self.robot.fk(q)
        links = self.robot.subtree(below) if below else None
        for v in self.robot.visuals:
            if links is not None and v.link not in links:
                continue
            if v.link.startswith(("K8_tool", "K9", "K10", "K11", "K12")):
                c = "tool"
            elif "sensor_description" in v.mesh:
                c = "sensor"
            else:
                c = "machine"
            S = np.diag(np.r_[v.scale, 1.0])
            self.mesh(_stl(v.mesh), P[v.link] @ v.origin @ S, c, style, v.mesh)

    def link_center(self, link: str, q) -> np.ndarray:
        """Centre of a link's CAD body in K0 (bounding-box centre): where a leader points."""
        P = self.robot.fk(q)
        pts = []
        for v in self.robot.visuals:
            if v.link == link:
                b = np.array(_stl(v.mesh).GetBounds()).reshape(3, 2) * v.scale[:, None]
                c = np.array(np.meshgrid(*b)).reshape(3, -1)  # 8 corners
                pts.append((P[link] @ v.origin @ np.r_[c, np.ones((1, 8))])[:3].T)
        if not pts:
            raise KeyError(f"{link} has no visual mesh")
        pts = np.vstack(pts)
        return (pts.min(0) + pts.max(0)) / 2

    def block(self, pose, color="block", style: str = "solid"):
        """pose: 4x4, or (x, y, z, yaw) of the block centre. color: role name or hex."""
        T = np.asarray(pose, float)
        if T.shape != (4, 4):
            T = transform(T[:3], (0, 0, T[3] if len(T) > 3 else 0.0))
        self.mesh(block_mesh(), T, color, style)
        return T

    def block_on_ground(self, x, y, yaw=0.0, course: int = 0, **kw):
        z = GROUND_Z + BLOCK["size"][2] * (course + 0.5)
        return self.block((x, y, z, yaw), **kw)

    def block_in_gripper(self, q, **kw):
        return self.block(self.robot.fk(q)["K8_tool_center_point"] @ TCP_TO_BLOCK, **kw)

    def ground(self, margin: float = 1.5, color=None):
        """Ground plane under everything added, `margin` m beyond it on each side."""
        self._ground = (margin, color)

    def _add_ground(self, ren, b):
        margin, color = self._ground
        margin += 2.5 if self.look == "wire" else 0.0  # wash is a blot inside the plane
        lo, hi = b[:, [0, 2]].min(0) - margin, b[:, [1, 3]].max(0) + margin
        src = vtk.vtkPlaneSource()
        src.SetOrigin(lo[0], lo[1], GROUND_Z)
        src.SetPoint1(hi[0], lo[1], GROUND_Z)
        src.SetPoint2(lo[0], hi[1], GROUND_Z)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(src.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().LightingOff()
        if self.look == "wire":
            from .wash import watercolor

            img = watercolor(color or pal.WASH, n=2048, coverage=0.8)
            rgba = (img * 255).astype(np.uint8)[::-1].reshape(-1, 4)
            data = vtk.vtkImageData()
            data.SetDimensions(img.shape[1], img.shape[0], 1)
            data.GetPointData().SetScalars(numpy_to_vtk(rgba, deep=True))
            tex = vtk.vtkTexture()
            tex.SetInputData(data)
            tex.InterpolateOn()
            tex.MipmapOn()
            actor.SetTexture(tex)
            actor.ForceTranslucentOn()
        else:
            actor.GetProperty().SetColor(*pal.rgb(color or pal.MATERIAL["ground"]))
        ren.AddActor(actor)

    # --- vector annotations (drawn over the render) ---------------------------------------
    def frame(
        self,
        T,
        label: str | None = None,
        length: float = 0.6,
        axis_labels: bool = False,
        offset=(6, 6),
    ):
        """Triad at pose T (4x4); `link_frame` for a link at q."""
        self._marks.append(
            ("frame", np.asarray(T, float), label, length, axis_labels, offset)
        )

    def link_frame(self, link: str, q, label=None, **kw):
        self.frame(self.robot.fk(q)[link], label, **kw)

    def joint(
        self,
        name: str,
        q,
        label: str | None = None,
        radius: float = 0.25,
        color=pal.INK,
        offset=(6, -10),
    ):
        """Revolute: arc arrow about the axis. Prismatic: double arrow along it."""
        o, a = self.robot.joint_frame(name, q)
        kind = self.robot.joints[name].type
        self._marks.append(("joint", o, a, kind, label, radius, color, offset))

    def path(self, points, color=pal.BLUE, lw=1.2, ls="-", label=None, offset=(6, 0)):
        self._marks.append(
            ("path", np.atleast_2d(points), color, lw, ls, label, offset)
        )

    def arrow(self, start, end, color=pal.INK, lw=1.0, label=None, offset=(4, 4)):
        self._marks.append(
            (
                "arrow",
                np.asarray(start, float),
                np.asarray(end, float),
                color,
                lw,
                label,
                offset,
            )
        )

    def label(self, point, text: str, offset=(0, 0), color=pal.INK, **text_kw):
        """Text at a K0 point; pass leader=True for a leader line from text to point."""
        self._marks.append(
            ("label", np.asarray(point, float), text, offset, color, text_kw)
        )

    # --- camera -------------------------------------------------------------------------
    def camera(self, **kw):
        """
        Set azimuth/elevation (deg, about world z), target, distance, fov, ortho, margin.

        Without `target` the view is panned and zoomed to fit bodies and annotations.
        """
        unknown = set(kw) - set(self._cam)
        if unknown:
            raise TypeError(f"unknown camera keys {unknown}")
        self._cam.update(kw)

    # --- output -------------------------------------------------------------------------
    def save(
        self,
        path: str,
        width: str | float = "single",
        aspect: float = 0.75,
        dpi: int = 600,
        fit: bool = True,
    ):
        """
        Write path: .pdf/.svg = vector overlay on raster render, .png = raster.

        fit=False keeps the camera framing exactly (sequences): no rescale to the ink.
        """
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        pal.use()
        w_in = pal.WIDTH[width] if isinstance(width, str) else float(width)
        h_in = w_in * aspect
        W, H = int(w_in * dpi), int(h_in * dpi)
        self._size = (w_in, h_in)
        img, project = self._render(W, H)

        fig = plt.figure(figsize=(w_in, h_in))
        ax = fig.add_axes((0, 0, 1, 1))
        ax.imshow(img, extent=(0, W, H, 0), interpolation="lanczos")
        ax.set_xlim(0, W)
        ax.set_ylim(H, 0)
        ax.axis("off")
        for m in self._marks:
            getattr(self, "_draw_" + m[0])(ax, project, *m[1:])
        for artist in (*ax.texts, *ax.patches, *ax.lines):
            artist.set_clip_on(
                False
            )  # overlays end at the page, not at the render's edge
        level = img / 255.0 if img.dtype == np.uint8 else img
        ink = np.argwhere(
            (level < 0.97).any(-1)
        )  # drawn pixels; white margin may leave page
        (r0, c0), (r1, c1) = ink.min(0), ink.max(0)
        h, w = level.shape[:2]
        # a side where the render is already cut (close-up) does not constrain the fit
        drawn = [
            c0 if c0 > 0 else np.nan,
            r1 + 1 if r1 < h - 1 else np.nan,
            c1 + 1 if c1 < w - 1 else np.nan,
            r0 if r0 > 0 else np.nan,
        ]
        if fit:
            self._fit_text(fig, ax, drawn)
        with plt.rc_context(
            {"savefig.bbox": "standard"}
        ):  # exact page; use() sets "tight"
            fig.savefig(
                path,
                dpi=dpi,
                metadata={"CreationDate": None} if path.endswith(".pdf") else None,
            )
        plt.close(fig)
        return path

    @staticmethod
    def _fit_text(fig, ax, drawn, pad: float = 2.0):
        """
        Scale and shift render + overlay as one block until every overlay is on the page.

        `drawn`: (x0, y0, x1, y1) of the render's non-white pixels, data coordinates;
        nan where the render is cut at its own edge (that side then stays at the axes edge).
        Overlay sizes and offsets are in points and do not scale, so iterate. Page size stays.
        """
        for _ in range(12):
            fig.canvas.draw()
            r = fig.canvas.get_renderer()
            box = ax.get_window_extent(r)
            ink = ax.transData.transform([drawn[:2], drawn[2:]])
            edge = np.array([[box.x0, box.y0], [box.x1, box.y1]])
            ink = np.where(np.isnan(ink), edge, ink)  # cut side: the render's own edge
            boxes = [
                a.get_window_extent(r) for a in (*ax.texts, *ax.patches, *ax.lines)
            ]
            x0 = np.min([ink[0, 0]] + [b.x0 for b in boxes])
            x1 = np.max([ink[1, 0]] + [b.x1 for b in boxes])
            y0 = np.min([ink[0, 1]] + [b.y0 for b in boxes])
            y1 = np.max([ink[1, 1]] + [b.y1 for b in boxes])
            fw, fh, p = fig.bbox.width, fig.bbox.height, pad * fig.dpi / 72
            if (
                x0 >= p - 0.5
                and y0 >= p - 0.5
                and x1 <= fw - p + 0.5
                and y1 <= fh - p + 0.5
            ):
                return
            k = min((fw - 2 * p) / (x1 - x0), (fh - 2 * p) / (y1 - y0))
            if np.isnan(drawn).any():  # close-up: never enlarge, the cut is the framing
                k = min(k, 1.0)
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2  # content centre -> page centre

            def map_x(x):
                return (fw / 2 + (x - cx) * k) / fw

            def map_y(y):
                return (fh / 2 + (y - cy) * k) / fh

            ax.set_position(
                (
                    map_x(box.x0),
                    map_y(box.y0),
                    map_x(box.x1) - map_x(box.x0),
                    map_y(box.y1) - map_y(box.y0),
                )
            )

    def _render(self, W, H):
        ren = vtk.vtkRenderer()
        ren.SetBackground(*pal.rgb(pal.PAPER))
        ren.SetUseDepthPeeling(True)
        ren.SetMaximumNumberOfPeels(8)
        kit = vtk.vtkLightKit()
        kit.SetKeyLightIntensity(0.85)
        kit.AddLightsToRenderer(ren)
        cam = ren.GetActiveCamera()
        actors = []
        for poly, T, st in self._bodies:
            normals = vtk.vtkPolyDataNormals()
            normals.SetInputData(poly)
            normals.SetFeatureAngle(35)
            normals.SplittingOn()
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(normals.GetOutputPort())
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.SetUserMatrix(_vtk_matrix(T))
            prop = actor.GetProperty()
            role = "ghost" if st["style"] == "ghost" else st["color"]
            if self.look == "wire":
                line, fill = pal.WIRE.get(role, (role, None))
                st["line"] = line
                if fill is not None and st["style"] == "solid":
                    prop.SetColor(*pal.rgb(fill))
                    prop.LightingOff()
                    ren.AddActor(actor)
                actors.append((actor, poly, T, st))
                continue
            prop.SetColor(*pal.rgb(pal.MATERIAL.get(role, role)))
            prop.SetAmbient(0.25)
            prop.SetDiffuse(0.75)
            prop.SetSpecular(0.08)
            if st["style"] == "ghost":
                prop.SetOpacity(0.28)
            if st["style"] != "outline":
                ren.AddActor(actor)
            actors.append((actor, poly, T, st))
        bounds = np.array([a.GetBounds() for a, *_ in actors])
        self._place_camera(cam, bounds, W / H)
        M = _numpy_matrix(cam.GetCompositeProjectionTransformMatrix(W / H, -1, 1))

        def project(p):
            """K0 point -> (u, v) image pixels, v down."""
            c = M @ np.r_[np.asarray(p, float), 1.0]
            return np.array([(c[0] / c[3] + 1) * W / 2, (1 - c[1] / c[3]) * H / 2])

        if self.look == "lineart":
            # one Blender run serves every format saved from the same scene and size
            key = (W, H, len(self._bodies), cam.GetPosition(), cam.GetFocalPoint())
            if getattr(self, "_blender_cache", (None,))[0] != key:
                self._blender_cache = (key, self._blender(W, H, cam))
            return self._blender_cache[1], project
        if self._ground:
            self._add_ground(ren, bounds)
        for actor, poly, T, st in actors:
            if self.look == "wire":
                self._cage(ren, cam, actor, poly, T, st)
            else:
                self._outline(ren, cam, actor, poly, T, st["style"])
        ren.ResetCameraClippingRange()

        win = vtk.vtkRenderWindow()
        win.SetOffScreenRendering(1)
        win.SetSize(W, H)
        win.SetMultiSamples(8)
        win.SetAlphaBitPlanes(1)
        win.AddRenderer(ren)
        if self.look == "cad" and hasattr(ren, "SetUseSSAO"):
            ren.SetUseSSAO(True)
            ren.SetSSAORadius(0.15)
            ren.SetSSAOKernelSize(128)
            ren.SetSSAOBlur(True)
            ren.SetSSAOBias(0.02)
        win.Render()
        grab = vtk.vtkWindowToImageFilter()
        grab.SetInput(win)
        grab.Update()
        im = grab.GetOutput()
        w, h, _ = im.GetDimensions()
        img = vtk_to_numpy(im.GetPointData().GetScalars()).reshape(h, w, -1)[::-1]
        win.Finalize()
        return img[..., :3], project

    def _blender(self, W, H, cam) -> np.ndarray:
        """Render the bodies in Blender (see blender_render.py), composite on paper."""
        import json
        import subprocess
        import tempfile

        tmp = Path(tempfile.mkdtemp(prefix="cbs_figures_"))
        bodies = []
        for k, (poly, T, st) in enumerate(self._bodies):
            src = st["source"]
            if src is None:  # generated geometry (blocks): hand Blender an STL
                src = str(tmp / f"body{k}.stl")
                w = vtk.vtkSTLWriter()
                w.SetInputData(poly)
                w.SetFileName(src)
                w.Write()
            fill = pal.LINEART.get(st["color"], st["color"])
            bodies.append(
                dict(mesh=src, matrix=T.tolist(), fill=fill, style=st["style"])
            )
        dpi = W / self._size[0]
        job = dict(
            bodies=bodies,
            width=W,
            height=H,
            out=str(tmp / "render.png"),
            crease_angle=180.0 - self.feature_angle,
            lines={k: (c, w * dpi / 72) for k, (c, w) in pal.LINE_PT.items()},
            camera=dict(
                position=cam.GetPosition(),
                focal=cam.GetFocalPoint(),
                up=cam.GetViewUp(),
                fov=cam.GetViewAngle(),
                ortho=bool(cam.GetParallelProjection()),
                parallel_scale=cam.GetParallelScale(),
            ),
        )
        (tmp / "job.json").write_text(json.dumps(job))
        script = Path(__file__).with_name("blender_render.py")
        cmd = [blender(), "-b", "--factory-startup", "--python", str(script), "--"]
        subprocess.run([*cmd, str(tmp / "job.json")], check=True, capture_output=True)
        rgba = plt.imread(tmp / "render.png")
        paper = np.array(pal.rgb(pal.PAPER))
        return np.clip(
            rgba[..., :3] * rgba[..., 3:] + paper * (1 - rgba[..., 3:]), 0, 1
        )

    def _place_camera(self, cam, bounds, aspect):
        c = self._cam
        lo, hi = bounds[:, ::2].min(0), bounds[:, 1::2].max(0)
        target = np.asarray(
            c["target"] if c["target"] is not None else (lo + hi) / 2, float
        )
        az, el = np.radians(c["azimuth"]), np.radians(c["elevation"])
        view = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        radius = np.linalg.norm(hi - lo) / 2
        fov = c["fov"]
        dist = c["distance"] or radius / np.sin(np.radians(fov) / 2) * 1.02
        cam.SetFocalPoint(*target)
        cam.SetPosition(*(target + dist * view))
        cam.SetViewUp(0, 0, 1)
        cam.SetViewAngle(fov)
        if c["ortho"]:
            cam.ParallelProjectionOn()
            cam.SetParallelScale(radius)
        if c["target"] is None:
            self._fit(cam, bounds, aspect, c["margin"], self._mark_points())

    def _mark_points(self) -> np.ndarray:
        pts = [np.zeros((0, 3))]
        for m in self._marks:
            if m[0] == "frame":
                T, length = m[1], m[3]
                pts.append(T[:3, 3] + np.vstack([np.zeros(3), length * T[:3, :3].T]))
            elif m[0] == "path":
                pts.append(m[1])
            elif m[0] in ("arrow", "joint", "label"):
                pts.append(np.atleast_2d(m[1]))
        return np.vstack(pts)

    @staticmethod
    def _fit(cam, bounds, aspect, margin, extra):
        """Pan + zoom so every body's bounding box and every annotation fill the frame."""
        corners = np.array(
            [
                [b[i], b[2 + j], b[4 + k]]
                for b in bounds
                for i in (0, 1)
                for j in (0, 1)
                for k in (0, 1)
            ]
        )
        corners = np.vstack([corners, extra])
        for _ in range(3):
            M = _numpy_matrix(cam.GetCompositeProjectionTransformMatrix(aspect, -1, 1))
            c = (M @ np.c_[corners, np.ones(len(corners))].T).T
            ndc = c[:, :2] / c[:, 3:4]
            lo, hi = ndc.min(0), ndc.max(0)
            mid = (lo + hi) / 2
            half = max((hi - lo)[0] / 2, (hi - lo)[1] / 2) / (1 - margin)
            # pan: shift focal point and position along the view-plane axes
            fp, pos = np.array(cam.GetFocalPoint()), np.array(cam.GetPosition())
            d = np.linalg.norm(pos - fp)
            up = np.array(cam.GetViewUp())
            fwd = (fp - pos) / d
            right = np.cross(fwd, up)
            right /= np.linalg.norm(right)
            up = np.cross(right, fwd)
            if cam.GetParallelProjection():
                hh = cam.GetParallelScale()
            else:
                hh = d * np.tan(np.radians(cam.GetViewAngle()) / 2)
            shift = right * mid[0] * hh * aspect + up * mid[1] * hh
            cam.SetFocalPoint(*(fp + shift))
            cam.SetPosition(*(pos + shift))
            cam.Zoom(1.0 / half)

    def _cage(self, ren, cam, actor, poly, T, st):
        """Pastel wire cage of the decimated mesh, plus its silhouette in the same colour."""
        ghost = st["style"] == "ghost"
        sil = vtk.vtkPolyDataSilhouette()
        sil.SetInputData(poly)
        sil.SetCamera(cam)
        sil.SetProp3D(actor)
        sources = [(sil.GetOutputPort(), 1.0 if ghost else 1.3)]
        if not ghost:
            sources.append((_cage(poly, self.cage_reduction, self.cage_angle), 0.5))
        for src, width in sources:
            m = vtk.vtkPolyDataMapper()
            if isinstance(src, vtk.vtkPolyData):
                m.SetInputData(src)
            else:
                m.SetInputConnection(src)
            m.SetResolveCoincidentTopologyToPolygonOffset()
            m.SetRelativeCoincidentTopologyLineOffsetParameters(0, -4)
            a = vtk.vtkActor()
            a.SetMapper(m)
            a.SetUserMatrix(_vtk_matrix(T))
            a.GetProperty().SetColor(*pal.rgb(st["line"]))
            a.GetProperty().SetLineWidth(width)
            a.GetProperty().LightingOff()
            if ghost:
                a.GetProperty().SetOpacity(0.6)
            ren.AddActor(a)

    def _outline(self, ren, cam, actor, poly, T, style):
        edge = pal.rgb(pal.MUTED if style == "ghost" else pal.MATERIAL["edge"])
        sil = vtk.vtkPolyDataSilhouette()
        sil.SetInputData(poly)
        sil.SetCamera(cam)
        sil.SetProp3D(actor)
        sil.SetEnableFeatureAngle(True)
        sil.SetFeatureAngle(self.feature_angle)
        sil.BorderEdgesOn()
        m = vtk.vtkPolyDataMapper()
        m.SetInputConnection(sil.GetOutputPort())
        m.SetResolveCoincidentTopologyToPolygonOffset()
        m.SetRelativeCoincidentTopologyLineOffsetParameters(0, -4)
        a = vtk.vtkActor()
        a.SetMapper(m)
        a.SetUserMatrix(_vtk_matrix(T))
        a.GetProperty().SetColor(*edge)
        a.GetProperty().SetLineWidth(1.6 if style == "solid" else 1.0)
        a.GetProperty().LightingOff()
        ren.AddActor(a)

    # --- overlay painters ----------------------------------------------------------------
    @staticmethod
    def _text(ax, uv, text, offset, color=pal.INK, **kw):
        if text:
            ax.annotate(
                text,
                uv,
                xytext=offset,
                textcoords="offset points",
                color=color,
                fontsize=kw.pop("fontsize", 8),
                **kw,
            )

    @staticmethod
    def _arrow2(ax, a, b, color, lw, both=False):
        ax.add_patch(
            FancyArrowPatch(
                a,
                b,
                arrowstyle="<|-|>" if both else "-|>",
                mutation_scale=6,
                color=color,
                lw=lw,
                shrinkA=0,
                shrinkB=0,
                joinstyle="miter",
            )
        )

    def _draw_frame(self, ax, project, T, label, length, axis_labels, offset):
        o = project(T[:3, 3])
        for k, name in enumerate("xyz"):
            e = project(T[:3, 3] + length * T[:3, k])
            self._arrow2(ax, o, e, pal.AXIS[k], 1.0)
            if axis_labels:
                self._text(ax, e, f"${name}$", (2, 2), color=pal.AXIS[k], fontsize=7)
        ax.plot(*o, "o", ms=2.0, color=pal.INK)
        self._text(ax, o, label, offset)

    def _draw_joint(self, ax, project, o, a, kind, label, radius, color, offset):
        a = a / np.linalg.norm(a)
        if kind == "prismatic":
            p0, p1 = project(o), project(o + 2.5 * radius * a)
            self._arrow2(ax, p0, p1, color, 1.0, both=True)
            self._text(ax, p1, label, offset, color=color)
            return
        u = np.cross(a, [0, 0, 1] if abs(a[2]) < 0.9 else [1, 0, 0])
        u /= np.linalg.norm(u)
        v = np.cross(a, u)
        t = np.linspace(0.15, 1.75 * np.pi, 60)
        pts = np.array(
            [project(o + radius * (np.cos(s) * u + np.sin(s) * v)) for s in t]
        )
        tail, head = project(o - 0.8 * radius * a), project(o + 0.8 * radius * a)
        ax.plot(
            [tail[0], head[0]],
            [tail[1], head[1]],
            color=color,
            lw=0.6,
            ls=(0, (6, 2, 1, 2)),
        )
        ax.plot(pts[:-3, 0], pts[:-3, 1], color=color, lw=1.0)
        self._arrow2(ax, pts[-4], pts[-1], color, 1.0)
        self._text(ax, pts[len(pts) // 2], label, offset, color=color)

    def _draw_path(self, ax, project, points, color, lw, ls, label, offset):
        uv = np.array([project(p) for p in points])
        ax.plot(uv[:, 0], uv[:, 1], color=color, lw=lw, ls=ls, solid_capstyle="round")
        self._text(ax, uv[-1], label, offset, color=pal.INK)

    def _draw_arrow(self, ax, project, start, end, color, lw, label, offset):
        a, b = project(start), project(end)
        self._arrow2(ax, a, b, color, lw)
        self._text(ax, b, label, offset)

    def _draw_label(self, ax, project, point, text, offset, color, text_kw):
        text_kw = dict(text_kw)  # the mark is drawn again for every saved format
        if text_kw.pop("leader", False):
            uv = project(point)
            ax.plot(*uv, "o", ms=1.6, color=pal.INK)
            # engineering-book leader: an S that leaves the text level and lands level.
            # Built in inches around the point, so it follows the text when _fit_text
            # rescales the drawing.
            gap = 2.0 if offset[0] < 0 else -2.0
            x0, y0 = (offset[0] + gap) / 72, offset[1] / 72
            bend = 0.55 * -x0
            path = MplPath(
                [(x0, y0), (x0 + bend, y0), (-bend, 0), (0, 0)],
                [MplPath.MOVETO, MplPath.CURVE4, MplPath.CURVE4, MplPath.CURVE4],
            )
            at = ax.figure.dpi_scale_trans + ScaledTranslation(*uv, ax.transData)
            ax.add_patch(
                PathPatch(path, transform=at, fill=False, lw=0.5, color=pal.INK)
            )
            text_kw.setdefault("ha", "right" if offset[0] < 0 else "left")
            text_kw.setdefault("va", "center")
        self._text(ax, project(point), text, offset, color=color, **text_kw)
