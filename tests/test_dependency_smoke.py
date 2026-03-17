from __future__ import annotations

import os
from pathlib import Path

import numpy as np


def test_pinocchio_and_casadi_bindings_smoke() -> None:
    import casadi as ca
    import pinocchio as pin
    from pinocchio import casadi as cpin

    x = ca.SX.sym("x", 2)
    f = ca.Function("dependency_smoke_sum", [x], [x[0] + 2.0 * x[1]])
    value = float(f(np.array([1.0, 3.0])).full().squeeze())

    assert np.isclose(value, 7.0)
    assert pin.__file__
    assert cpin.__file__

    model = cpin.Model()
    assert model.nq == 0
    assert model.nv == 0


def test_acados_template_runtime_smoke() -> None:
    import casadi as ca
    from acados_template import AcadosModel

    acados_source_dir = os.environ.get("ACADOS_SOURCE_DIR")
    assert acados_source_dir, "ACADOS_SOURCE_DIR is not set"

    acados_root = Path(acados_source_dir)
    assert (acados_root / "bin" / "t_renderer").exists()
    assert (acados_root / "lib" / "link_libs.json").exists()

    x = ca.SX.sym("x")
    u = ca.SX.sym("u")
    xdot = ca.SX.sym("xdot")

    model = AcadosModel()
    model.name = "dependency_smoke_model"
    model.x = x
    model.u = u
    model.xdot = xdot
    model.f_expl_expr = -x + u
    model.f_impl_expr = xdot - model.f_expl_expr

    assert model.name == "dependency_smoke_model"
    assert tuple(model.x.shape) == (1, 1)
    assert tuple(model.f_impl_expr.shape) == (1, 1)


def test_open3d_geometry_smoke() -> None:
    import open3d as o3d

    points = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=float,
    )

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)

    bbox = cloud.get_axis_aligned_bounding_box()
    extent = np.asarray(bbox.get_extent(), dtype=float)

    assert len(cloud.points) == 3
    assert np.allclose(extent, np.array([1.0, 1.0, 0.0], dtype=float))
