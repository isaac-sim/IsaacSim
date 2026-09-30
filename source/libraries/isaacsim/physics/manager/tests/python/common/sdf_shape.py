# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Exercise signed-distance-field shape queries against analytic box results.

The scenario queries a grid of mesh cubes in each shape's local frame. The
returned ``[N, maxQ, 4]`` tensor
(``grad.x, grad.y, grad.z, distance`` per point) is checked against the
analytic box SDF.

SDF evaluation is GPU-only in ovphysx (``ovphysx_evaluate_sdf`` requires the
query and the output on the GPU). So the GPU lane runs the real evaluation and
the caller-``out`` guards; the CPU lane asserts the view is unsupported
(``count == 0``) — ovphysx refuses to create a CPU SDF view.
"""

from __future__ import annotations

import os
import sys
from collections.abc import Callable

import isaacsim.physics.manager.impl.tensors as t
import numpy as np
import pytest
from physx_usd_schemas import PhysxSchema

_PARENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT_DIR not in sys.path:
    sys.path.append(_PARENT_DIR)

from _scenario import DeviceParams, GridParams, GridTestBase, SimParams  # noqa: E402
from pxr import Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

# A unit cube centered at the origin, authored as 8 verts / 6 quads — the same
# topology the legacy `physicsUtils.create_mesh_cube` cooks into an SDF.
_CUBE_UNIT_POINTS = [
    (0.5, -0.5, -0.5),
    (0.5, 0.5, -0.5),
    (0.5, 0.5, 0.5),
    (0.5, -0.5, 0.5),
    (-0.5, -0.5, -0.5),
    (-0.5, 0.5, -0.5),
    (-0.5, 0.5, 0.5),
    (-0.5, -0.5, 0.5),
]
_CUBE_FACE_COUNTS = [4, 4, 4, 4, 4, 4]
_CUBE_FACE_INDICES = [0, 1, 2, 3, 1, 5, 6, 2, 3, 2, 6, 7, 0, 3, 7, 4, 1, 0, 4, 5, 5, 4, 7, 6]


def _add_sdf_cube(
    stage: Usd.Stage,
    path: str | Sdf.Path,
    half: float,
    resolution: int = 256,
) -> Usd.Prim:
    """Create a mesh cube with a signed-distance-field collider.

    Args:
        stage: Stage that receives the cube.
        path: Prim path for the cube.
        half: Cube half extent.
        resolution: SDF cooking resolution.

    Returns:
        Authored cube prim.

    """
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    pts = [(p[0] * 2 * half, p[1] * 2 * half, p[2] * 2 * half) for p in _CUBE_UNIT_POINTS]
    mesh.CreatePointsAttr().Set(pts)
    mesh.CreateFaceVertexCountsAttr().Set(_CUBE_FACE_COUNTS)
    mesh.CreateFaceVertexIndicesAttr().Set(_CUBE_FACE_INDICES)
    mesh.CreateExtentAttr().Set([(-half, -half, -half), (half, half, half)])
    prim = mesh.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    mca = UsdPhysics.MeshCollisionAPI.Apply(prim)
    mca.CreateApproximationAttr().Set("sdf")
    sdf_api = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim)
    sdf_api.CreateSdfResolutionAttr().Set(resolution)
    return prim


def _analytic_box_sdf(points: np.ndarray, half: float) -> tuple[np.ndarray, np.ndarray]:
    """Compute signed distances and gradients for an axis-aligned box.

    distance = ||max(|p|-h, 0)|| + min(max(|p|-h), 0); gradient points away from
    the surface outside, and toward the nearest face inside. Points are chosen to
    avoid the surface/center so the gradient is unambiguous.

    Args:
        points: Local-space sample points with shape ``(N, 3)``.
        half: Box half-extent shared by all three axes.

    Returns:
        Signed distances and normalized local-space gradients.

    """
    p = np.asarray(points, dtype=np.float64)
    d = np.abs(p) - half
    outside = np.maximum(d, 0.0)
    onorm = np.linalg.norm(outside, axis=-1)
    inside = np.minimum(np.max(d, axis=-1), 0.0)
    dist = onorm + inside

    grad = np.zeros_like(p)
    for i in range(p.shape[0]):
        if onorm[i] > 1e-9:  # outside: direction of the exterior offset
            grad[i] = np.sign(p[i]) * outside[i] / onorm[i]
        else:  # inside: toward the least-penetrated (nearest) face
            axis = int(np.argmax(d[i]))
            grad[i, axis] = 1.0 if p[i, axis] >= 0 else -1.0
    return dist.astype(np.float32), grad.astype(np.float32)


# Query points in each cube's local frame (half-extent 0.5): a mix of axis /
# diagonal / corner exterior points and near-face interior points. Distances are
# all >= ~0.05 from the surface so both distance and gradient are well-defined.
_LOCAL_QUERY = np.array(
    [
        [0.70, 0.00, 0.00],  # +x exterior
        [0.00, -0.80, 0.00],  # -y exterior
        [0.00, 0.00, 0.90],  # +z exterior
        [-0.75, 0.00, 0.00],  # -x exterior
        [0.60, 0.60, 0.00],  # xy-edge exterior
        [0.55, 0.55, 0.55],  # xyz-corner exterior
        [0.20, 0.05, 0.00],  # interior, nearest +x face
        [0.05, -0.30, 0.10],  # interior, nearest -y face
    ],
    dtype=np.float32,
)


class SdfDistancesAndGradientsCommon(GridTestBase):
    """Scenario that validates SDF values, gradients, and output-buffer guards.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    half = 0.5
    max_q = _LOCAL_QUERY.shape[0]
    cook_steps = 5

    def __init__(self, test_case: object, device_params: DeviceParams) -> None:
        super().__init__(test_case, GridParams(num_envs=3, env_spacing=5.0), SimParams(), device_params)
        _add_sdf_cube(self.stage, self.env_template_path.AppendChild("cube"), self.half)
        self.view = None

    def on_start(self, sim: object) -> None:
        """Create the SDF shape view for all replicated cubes.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.view = sim.create_sdf_shape_view("/envs/*/cube", self.max_q)

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Evaluate SDF queries after collision cooking completes.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        if stepno != self.cook_steps:
            return
        import warp as wp

        tc = self.test_case
        n = int(getattr(self.view, "count", 0))

        # CPU lane: SDF is GPU-only, so ovphysx yields an unsupported view.
        if self.wp_device == "cpu":
            assert n == 0, "SDF evaluation is GPU-only; the CPU view must resolve no shapes"
            self.finish()
            return

        # GPU lane: real evaluation against the analytic box SDF.
        assert n == self.num_envs, "SDF view should resolve one cube per env"

        pts = np.broadcast_to(_LOCAL_QUERY, (n, self.max_q, 3)).copy()
        qp = wp.from_numpy(pts, dtype=wp.float32, device=self.wp_device)
        out = wp.empty((n, self.max_q, 4), dtype=wp.float32, device=self.wp_device)

        res = self.view.get_data("distances-and-gradients", qp, out)
        assert res.size == n * self.max_q * 4, "result must be [N, maxQ, 4]"
        assert str(res.device).startswith("cuda"), "GPU eval must return a CUDA tensor"

        arr = res.numpy().reshape(n, self.max_q, 4)
        exp_dist, exp_grad = _analytic_box_sdf(_LOCAL_QUERY, self.half)
        for i in range(n):
            got_dist = arr[i, :, 3]
            got_grad = arr[i, :, :3]
            assert np.allclose(
                got_dist, exp_dist, atol=0.05
            ), f"env {i} distances mismatch: got {got_dist.tolist()} expected {exp_dist.tolist()}"
            assert np.allclose(
                got_grad, exp_grad, atol=0.1
            ), f"env {i} gradients mismatch: got {got_grad.tolist()} expected {exp_grad.tolist()}"

        self._check_buffer_guards(qp, n)
        self.finish()

    def _check_buffer_guards(self, qp: object, n: int) -> None:
        """Verify rejection of malformed query and output buffers.

        Args:
            qp: Valid GPU query-point buffer used as the control input.
            n: Number of SDF shapes in the view.

        """
        import warp as wp

        tc = self.test_case

        def expect_raises(fn: Callable[[], object], needle: str) -> None:
            try:
                fn()
            except Exception as exc:  # noqa: BLE001 -- asserting the rejection
                assert needle in str(exc), f"unexpected error: {exc!r}"
                return
            pytest.fail(f"expected a raise containing {needle!r}")

        wrong_count = wp.empty((n, self.max_q, 3), dtype=wp.float32, device=self.wp_device)
        expect_raises(lambda: self.view.get_data("distances-and-gradients", qp, wrong_count), "element count mismatch")

        wrong_dtype = wp.empty((n, self.max_q, 4), dtype=wp.float64, device=self.wp_device)
        expect_raises(lambda: self.view.get_data("distances-and-gradients", qp, wrong_dtype), "dtype mismatch")

        cpu_out = wp.empty((n, self.max_q, 4), dtype=wp.float32, device="cpu")
        expect_raises(
            lambda: self.view.get_data("distances-and-gradients", qp, cpu_out), "out buffer must be on the GPU"
        )

        expect_raises(
            lambda: self.view.get_data("distances-and-gradients", qp), "provide a preallocated GPU out buffer"
        )

        good_out = wp.empty((n, self.max_q, 4), dtype=wp.float32, device=self.wp_device)

        cpu_query = wp.from_numpy(_LOCAL_QUERY.copy(), dtype=wp.float32, device="cpu")
        expect_raises(
            lambda: self.view.get_data("distances-and-gradients", cpu_query, good_out),
            "query points must be on the GPU",
        )

        wrong_q_dtype = wp.empty((n, self.max_q, 3), dtype=wp.float64, device=self.wp_device)
        expect_raises(
            lambda: self.view.get_data("distances-and-gradients", wrong_q_dtype, good_out),
            "query points must be float32",
        )

        wrong_q_shape = wp.empty((n, self.max_q, 2), dtype=wp.float32, device=self.wp_device)
        expect_raises(
            lambda: self.view.get_data("distances-and-gradients", wrong_q_shape, good_out),
            "query points must be [N, maxQ, 3]",
        )


class SdfQueryCapacityResizeCommon(SdfDistancesAndGradientsCommon):
    """Scenario that verifies an SDF view adopts the query extent the caller submits.

    ``maxQ`` is a creation parameter of the underlying ovphysx SDF view, so adopting a different one
    means recreating that view. Unlike a contact capacity it cannot be discovered from a failed read:
    it is how many points the caller intends to submit, and it is stated by the query itself.

    The view is deliberately built at one query point -- the default a view built through
    ``create_entity`` starts at -- so the first real query has to grow it.

    Args:
        test_case: Test instance associated with the scenario.
        device_params: Simulation and tensor device selection.

    """

    #: Query-point extent the view is constructed with.
    initial_max_q = 1

    def on_start(self, sim: object) -> None:
        """Create the SDF view at the initial single-point extent.

        Args:
            sim: Entity-view factory for the running simulation.

        """
        self.view = sim.create_sdf_shape_view("/envs/*/cube", self.initial_max_q)

    def _declared_query_points(self) -> int:
        """Read the query extent the view currently declares.

        Returns:
            Second axis of the declared ``[N, maxQ, 4]`` output shape.

        """
        spec = self.view.get_impl_spec("distances-and-gradients", t.ImplKind.Get)
        return int(spec.shape_hint[1])

    def _evaluate(self, query_points: np.ndarray) -> np.ndarray:
        """Evaluate the SDF for a query of arbitrary extent.

        Args:
            query_points: Local-frame query points, shaped ``[Q, 3]``.

        Returns:
            Result reshaped to ``[N, Q, 4]``.

        """
        import warp as wp

        count = int(self.view.count)
        extent = query_points.shape[0]
        points = np.broadcast_to(query_points, (count, extent, 3)).copy()
        query = wp.from_numpy(points, dtype=wp.float32, device=self.wp_device)
        out = wp.empty((count, extent, 4), dtype=wp.float32, device=self.wp_device)
        result = self.view.get_data("distances-and-gradients", query, out)
        return result.numpy().reshape(count, extent, 4)

    def _assert_matches_analytic(self, values: np.ndarray, query_points: np.ndarray, label: str) -> None:
        """Check evaluated distances and gradients against the analytic box SDF.

        Args:
            values: Evaluated ``[N, Q, 4]`` results.
            query_points: Query points the results correspond to.
            label: Phase name used in assertion messages.

        """
        expected_distances, expected_gradients = _analytic_box_sdf(query_points, self.half)
        for env in range(values.shape[0]):
            assert np.allclose(
                values[env, :, 3], expected_distances, atol=0.05
            ), f"{label}: env {env} distances mismatch after resize"
            assert np.allclose(
                values[env, :, :3], expected_gradients, atol=0.1
            ), f"{label}: env {env} gradients mismatch after resize"

    def on_physics_step(self, sim: object, stepno: int, dt: float) -> None:
        """Grow, then shrink, the query extent and verify the results stay correct.

        Args:
            sim: Entity-view factory for the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """
        del sim, dt
        if stepno != self.cook_steps:
            return

        count = int(getattr(self.view, "count", 0))
        if self.wp_device == "cpu":
            assert count == 0, "SDF evaluation is GPU-only; the CPU view must resolve no shapes"
            self.finish()
            return

        assert count == self.num_envs, "SDF view should resolve one cube per env"
        assert self._declared_query_points() == self.initial_max_q

        # A larger query grows the view, and the results are the real SDF rather than a truncation of
        # the single point it was built for.
        grown = self._evaluate(_LOCAL_QUERY)
        assert self._declared_query_points() == _LOCAL_QUERY.shape[0]
        self._assert_matches_analytic(grown, _LOCAL_QUERY, "grown")

        # A smaller query shrinks it again: the extent follows the caller in both directions.
        subset = _LOCAL_QUERY[:2]
        shrunk = self._evaluate(subset)
        assert self._declared_query_points() == subset.shape[0]
        self._assert_matches_analytic(shrunk, subset, "shrunk")

        self.finish()
