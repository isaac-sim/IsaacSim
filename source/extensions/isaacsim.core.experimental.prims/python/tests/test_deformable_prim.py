# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test for deformable prim."""

import re
import sys
from typing import Literal

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.prims import DeformablePrim
from isaacsim.core.experimental.utils.backend import use_backend
from omni.physx.scripts import deformableUtils
from pxr import Usd, UsdGeom, Vt

from .common import (
    check_allclose,
    check_array,
    check_lists,
    cprint,
    draw_choice,
    draw_indices,
    draw_sample,
    parametrize,
)


def _define_trimesh(stage, path):
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.GetPointsAttr().Set(Vt.Vec3fArray([[-1.0, -1.0, 0.0], [1.0, -1.0, 0.0], [-1.0, 1.0, 0.0], [1.0, 1.0, 0.0]]))
    mesh.GetNormalsAttr().Set(Vt.Vec3fArray([[0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0]]))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray([3, 3]))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 3, 0, 3, 2]))
    mesh.GetSubdivisionSchemeAttr().Set("none")
    mesh.GetExtentAttr().Set(Vt.Vec3fArray([[-1.0, -1.0, 0.0], [1.0, 1.0, 0.0]]))


def _define_tetmesh(stage, path):
    """Define a tetrahedron mesh.

    The tetrahedron mesh values are generated as follows:

    .. code-block:: python

        >>> import math
        >>> import pyvista as pv
        >>>
        >>> tetmesh = pv.Tetrahedron(radius=math.sqrt(3))
        >>> bbox = tetmesh.bounds
        >>> print("points:", tetmesh.points.tolist())
        >>> print("normals:", tetmesh.point_normals.tolist())
        >>> print("tetVertexIndices:", tetmesh.surface_indices().tolist())
        >>> print("surfaceFaceVertexIndices:", tetmesh.regular_faces.tolist())
        >>> print("extent:", [[bbox.x_min, bbox.y_min, bbox.z_min], [bbox.x_max, bbox.y_max, bbox.z_max]])
    """
    tetmesh = UsdGeom.TetMesh.Define(stage, path)
    tetmesh.GetPointsAttr().Set(
        Vt.Vec3fArray([[1.0, 1.0, 1.0], [-1.0, 1.0, -1.0], [1.0, -1.0, -1.0], [-1.0, -1.0, 1.0]])
    )
    tetmesh.GetNormalsAttr().Set(
        Vt.Vec3fArray([[0.57, 0.57, 0.57], [-0.57, 0.57, -0.57], [0.57, -0.57, -0.57], [-0.57, -0.57, 0.57]])
    )
    tetmesh.GetTetVertexIndicesAttr().Set(Vt.Vec4iArray([[0, 1, 2, 3]]))
    tetmesh.GetSurfaceFaceVertexIndicesAttr().Set(UsdGeom.TetMesh.ComputeSurfaceFaces(tetmesh, Usd.TimeCode.Default()))
    tetmesh.GetExtentAttr().Set(Vt.Vec3fArray([[-1.0, -1.0, -1.0], [1.0, 1.0, 1.0]]))


def _define_mesh(stage, path):
    """Define a cube mesh.

    The cube mesh values are generated as follows:

    .. code-block:: python

        >>> import pyvista as pv
        >>>
        >>> mesh = pv.Cube(x_length=2, y_length=2, z_length=2)
        >>> bbox = mesh.bounds
        >>> regular_faces = mesh.regular_faces.tolist()
        >>> print("points:", mesh.points.tolist())
        >>> print("normals:", mesh.point_normals.tolist())
        >>> print("faceVertexCounts:", [len(face) for face in regular_faces])
        >>> print("faceVertexIndices:", [item for face  in regular_faces for item in face])
        >>> print("extent:", [[bbox.x_min, bbox.y_min, bbox.z_min], [bbox.x_max, bbox.y_max, bbox.z_max]])
    """
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.GetPointsAttr().Set(
        Vt.Vec3fArray(
            [
                [-1.0, -1.0, -1.0],
                [-1.0, -1.0, 1.0],
                [-1.0, 1.0, 1.0],
                [-1.0, 1.0, -1.0],
                [1.0, -1.0, -1.0],
                [1.0, 1.0, -1.0],
                [1.0, 1.0, 1.0],
                [1.0, -1.0, 1.0],
            ]
        )
    )
    mesh.GetNormalsAttr().Set(
        Vt.Vec3fArray(
            [
                [-0.57, -0.57, -0.57],
                [-0.57, -0.57, 0.57],
                [-0.57, 0.57, 0.57],
                [-0.57, 0.57, -0.57],
                [0.57, -0.57, -0.57],
                [0.57, 0.57, -0.57],
                [0.57, 0.57, 0.57],
                [0.57, -0.57, 0.57],
            ]
        )
    )
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray([4, 4, 4, 4, 4, 4]))
    mesh.GetFaceVertexIndicesAttr().Set(
        Vt.IntArray([0, 1, 2, 3, 4, 5, 6, 7, 0, 4, 7, 1, 3, 2, 6, 5, 0, 3, 5, 4, 1, 7, 6, 2])
    )
    mesh.GetSubdivisionSchemeAttr().Set("none")
    mesh.GetExtentAttr().Set(Vt.Vec3fArray([[-1.0, -1.0, -1.0], [1.0, 1.0, 1.0]]))


async def populate_stage(max_num_prims: int, operation: Literal["wrap", "create"], **kwargs) -> None:
    """Populate stage."""
    deformable_case = kwargs.get("deformable_case")
    assert operation == "wrap", "Other operations except 'wrap' are not supported"
    assert deformable_case in [
        "surface",
        "volume",
        "auto-surface",
        "auto-volume",
    ], f"Invalid deformable case: {deformable_case}"
    # create new stage
    stage = await stage_utils.create_new_stage_async()
    # define prims
    stage_utils.define_prim(f"/World", "Xform")
    stage_utils.define_prim(f"/World/PhysicsScene", "PhysicsScene")
    for i in range(max_num_prims):
        # Mesh (surface): triangular mesh plane with 2 triangles
        if deformable_case == "surface":
            _define_trimesh(stage, f"/World/A_{i}")
            deformableUtils.set_physics_surface_deformable_body(stage, f"/World/A_{i}")
        # TetMesh (volume)
        elif deformable_case == "volume":
            _define_tetmesh(stage, f"/World/A_{i}")
            deformableUtils.set_physics_volume_deformable_body(stage, f"/World/A_{i}")
        # Xform (auto-surface)
        elif deformable_case == "auto-surface":
            stage_utils.define_prim(f"/World/A_{i}", "Xform")
            _define_mesh(stage, f"/World/A_{i}/mesh")
            if not deformableUtils.create_auto_surface_deformable_hierarchy(
                stage_utils.get_current_stage(),
                f"/World/A_{i}",
                simulation_mesh_path=f"/World/A_{i}/simulation_mesh",
                cooking_src_mesh_path=f"/World/A_{i}/mesh",
                cooking_src_simplification_enabled=False,
            ):
                raise ValueError(f"Failed to create auto-surface deformable hierarchy for {f'/World/A_{i}'}")
        # Xform (auto-volume)
        elif deformable_case == "auto-volume":
            stage_utils.define_prim(f"/World/A_{i}", "Xform")
            _define_mesh(stage, f"/World/A_{i}/mesh")
            if not deformableUtils.create_auto_volume_deformable_hierarchy(
                stage_utils.get_current_stage(),
                f"/World/A_{i}",
                simulation_tetmesh_path=f"/World/A_{i}/simulation_mesh",
                collision_tetmesh_path=f"/World/A_{i}/collision_mesh",
                cooking_src_mesh_path=f"/World/A_{i}/mesh",
                simulation_hex_mesh_enabled=True,
                cooking_src_simplification_enabled=kwargs.get("mesh_simplification", True),
            ):
                raise ValueError(f"Failed to create auto-volume deformable hierarchy for {f'/World/A_{i}'}")


class TestDeformablePrim(omni.kit.test.AsyncTestCase):
    """Test deformable prim."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        # auto-volume platform dependent checking
        if sys.platform == "win32":
            self._nnpb = (1350, 1717, 1350)
            self._nepb = (6096, 5395, 6096)
            self._nepb_nms = (5280, -1, 5280)
        else:
            self._nnpb = (1348, 1717, 1348)
            self._nepb = (6120, 5432, 6120)
            self._nepb_nms = (5280, -1, 5280)

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        super().tearDown()

    # --------------------------------------------------------------------

    def check_backend(self, backend, prim):
        """Check backend."""
        if backend == "tensor":
            self.assertTrue(prim.is_physics_tensor_entity_valid(), f"Tensor API should be enabled ({backend})")
        elif backend in ["usd", "usdrt", "fabric"]:
            self.assertFalse(prim.is_physics_tensor_entity_valid(), f"Tensor API should be disabled ({backend})")
        else:
            raise ValueError(f"Invalid backend: {backend}")

    # --------------------------------------------------------------------
    # surface
    # --------------------------------------------------------------------

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],  # "tensor" backend plays the simulation
        instances=["one"],
        operations=["wrap"],
        prim_class=lambda *args, **kwargs: None,
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "surface"},
        max_num_prims=1,
    )
    async def test_surface_runtime_instance_creation(self, prim, num_prims, device, backend):
        """Test surface runtime instance creation."""
        DeformablePrim("/World/A_0")

    @parametrize(
        devices=["cuda"],
        backends=["tensor", "usd"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "surface"},
    )
    async def test_surface_len(self, prim, num_prims, device, backend):
        """Test surface len."""
        self.assertEqual(len(prim), num_prims, f"Invalid DeformablePrim ({num_prims} prims) len")

    @parametrize(
        devices=["cuda"],
        backends=["tensor", "usd"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "surface"},
    )
    async def test_surface_properties_and_getters(self, prim, num_prims, device, backend):
        """Test surface properties and getters."""
        # check backend
        self.check_backend(backend, prim)
        # test cases (properties)
        self.assertEqual(prim.deformable_type, "surface", f"Invalid deformable type")
        # - amount
        self.assertEqual(prim.num_nodes_per_element, 3, f"Invalid num_nodes_per_element")
        self.assertEqual(prim.num_nodes_per_body, (4, 4, 4), f"Invalid num_nodes_per_body")
        self.assertEqual(prim.num_elements_per_body, (2, 2, 2), f"Invalid num_elements_per_body")
        # - paths
        for path in prim.simulation_mesh_paths:
            self.assertTrue(re.match(r"/World/A_[0-9]+$", path), f"Invalid simulation mesh paths: '{path}'")
        for path in prim.collision_mesh_paths:
            self.assertTrue(re.match(r"/World/A_[0-9]+$", path), f"Invalid collision mesh paths: '{path}'")

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "surface"},
    )
    async def test_surface_element_indices(self, prim, num_prims, device, backend):
        """Test surface element indices."""
        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                output = prim.get_element_indices(indices=indices)
            assert len(output) == 3, "Invalid number of outputs"
            check_array(output, shape=(expected_count, 2, 3), dtype=wp.uint32, device=device)

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "surface"},
    )
    async def test_surface_nodal_positions(self, prim, num_prims, device, backend):
        """Test surface nodal positions."""
        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 4, 3), dtype=wp.float32):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_nodal_positions(v0, indices=indices)
                    output = prim.get_nodal_positions(indices=indices)
                assert len(output) == 3, "Invalid number of outputs"
                check_array(output, shape=(expected_count, 4, 3), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output[0], given=(v0,))

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "surface"},
    )
    async def test_surface_nodal_velocities(self, prim, num_prims, device, backend):
        """Test surface nodal velocities."""
        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 4, 3), dtype=wp.float32):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_nodal_velocities(v0, indices=indices)
                    output = prim.get_nodal_velocities(indices=indices)
                check_array(output, shape=(expected_count, 4, 3), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "surface"},
    )
    async def test_surface_nodal_kinematic_position_targets(self, prim, num_prims, device, backend):
        """Test surface nodal kinematic position targets."""
        # check backend
        self.check_backend(backend, prim)
        # test cases (surface does not support nodal kinematic targets)
        self.assertRaises(AssertionError, prim.set_nodal_kinematic_position_targets)

    @parametrize(
        devices=["cuda"],
        backends=["usd"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "surface"},
    )
    async def test_surface_physics_materials(self, prim, num_prims, device, backend):
        """Test surface physics materials."""
        from isaacsim.core.experimental.materials import SurfaceDeformableMaterial

        choices = [
            SurfaceDeformableMaterial("/physics_materials/surface_0", poissons_ratios=[0.1], youngs_moduli=[1000000.0]),
            SurfaceDeformableMaterial("/physics_materials/surface_1", poissons_ratios=[0.6], youngs_moduli=[2000000.0]),
        ]
        # test cases
        # - check the number of applied materials before applying any material
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            output = prim.get_applied_physics_materials()
        number_of_materials = sum(1 for material in output if material is not None)
        assert number_of_materials == 0, f"No material should have been applied. Applied: {number_of_materials}"
        # - by indices
        for indices, expected_count in draw_indices(count=num_prims, step=2, types=[list, np.ndarray, wp.array]):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            count = expected_count
            for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.apply_physics_materials(v0, indices=indices)
                    output = prim.get_applied_physics_materials(indices=indices)
                check_lists(expected_v0, output, predicate=lambda x: x.paths[0])
        # - check the number of applied materials after applying materials by indices
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            output = prim.get_applied_physics_materials()
        number_of_materials = sum(1 for material in output if material is not None)
        assert (
            number_of_materials == count
        ), f"{count} materials should have been applied. Applied: {number_of_materials}"
        # - all
        count = num_prims
        for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
            with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                prim.apply_physics_materials(v0)
                output = prim.get_applied_physics_materials()
            check_lists(expected_v0, output, predicate=lambda x: x.paths[0])
        # - check the number of applied materials after applying materials by all
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            output = prim.get_applied_physics_materials()
        number_of_materials = sum(1 for material in output if material is not None)
        assert (
            number_of_materials == count
        ), f"{count} materials should have been applied. Applied: {number_of_materials}"

    # --------------------------------------------------------------------
    # auto-surface
    # --------------------------------------------------------------------

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],  # "tensor" backend plays the simulation
        instances=["one"],
        operations=["wrap"],
        prim_class=lambda *args, **kwargs: None,
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "auto-surface"},
        max_num_prims=1,
    )
    async def test_auto_surface_runtime_instance_creation(self, prim, num_prims, device, backend):
        """Test auto surface runtime instance creation."""
        DeformablePrim("/World/A_0")

    @parametrize(
        devices=["cuda"],
        backends=["tensor", "usd"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "auto-surface"},
    )
    async def test_auto_surface_len(self, prim, num_prims, device, backend):
        """Test auto surface len."""
        self.assertEqual(len(prim), num_prims, f"Invalid DeformablePrim ({num_prims} prims) len")

    @parametrize(
        devices=["cuda"],
        backends=["tensor", "usd"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "auto-surface"},
    )
    async def test_auto_surface_properties_and_getters(self, prim, num_prims, device, backend):
        """Test auto surface properties and getters."""
        # check backend
        self.check_backend(backend, prim)
        # test cases (properties)
        self.assertEqual(prim.deformable_type, "surface", f"Invalid deformable type")
        # - amount
        self.assertEqual(prim.num_nodes_per_element, 3, f"Invalid num_nodes_per_element")
        self.assertEqual(prim.num_nodes_per_body, (8, 8, 8), f"Invalid num_nodes_per_body")
        self.assertEqual(prim.num_elements_per_body, (12, 12, 12), f"Invalid num_elements_per_body")
        # - paths
        for path in prim.simulation_mesh_paths:
            self.assertTrue(
                re.match(r"/World/A_[0-9]+/simulation_mesh$", path), f"Invalid simulation mesh paths: '{path}'"
            )
        for path in prim.collision_mesh_paths:
            self.assertTrue(
                re.match(r"/World/A_[0-9]+/simulation_mesh$", path), f"Invalid collision mesh paths: '{path}'"
            )

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "auto-surface"},
    )
    async def test_auto_surface_element_indices(self, prim, num_prims, device, backend):
        """Test auto surface element indices."""
        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                output = prim.get_element_indices(indices=indices)
            assert len(output) == 3, "Invalid number of outputs"
            check_array(output, shape=(expected_count, 12, 3), dtype=wp.uint32, device=device)

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "auto-surface"},
    )
    async def test_auto_surface_nodal_positions(self, prim, num_prims, device, backend):
        """Test auto surface nodal positions."""
        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 8, 3), dtype=wp.float32):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_nodal_positions(v0, indices=indices)
                    output = prim.get_nodal_positions(indices=indices)
                assert len(output) == 3, "Invalid number of outputs"
                check_array(output, shape=(expected_count, 8, 3), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output[0], given=(v0,))

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "auto-surface"},
    )
    async def test_auto_surface_nodal_velocities(self, prim, num_prims, device, backend):
        """Test auto surface nodal velocities."""
        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 8, 3), dtype=wp.float32):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_nodal_velocities(v0, indices=indices)
                    output = prim.get_nodal_velocities(indices=indices)
                check_array(output, shape=(expected_count, 8, 3), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "auto-surface"},
    )
    async def test_auto_surface_nodal_kinematic_position_targets(self, prim, num_prims, device, backend):
        """Test auto surface nodal kinematic position targets."""
        # check backend
        self.check_backend(backend, prim)
        # test cases (surface does not support nodal kinematic targets)
        self.assertRaises(AssertionError, prim.set_nodal_kinematic_position_targets)

    @parametrize(
        devices=["cuda"],
        backends=["usd"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "auto-surface"},
    )
    async def test_auto_surface_physics_materials(self, prim, num_prims, device, backend):
        """Test auto surface physics materials."""
        from isaacsim.core.experimental.materials import SurfaceDeformableMaterial

        choices = [
            SurfaceDeformableMaterial("/physics_materials/surface_0", poissons_ratios=[0.1], youngs_moduli=[1000000.0]),
            SurfaceDeformableMaterial("/physics_materials/surface_1", poissons_ratios=[0.6], youngs_moduli=[2000000.0]),
        ]
        # test cases
        # - check the number of applied materials before applying any material
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            output = prim.get_applied_physics_materials()
        number_of_materials = sum(1 for material in output if material is not None)
        assert number_of_materials == 0, f"No material should have been applied. Applied: {number_of_materials}"
        # - by indices
        for indices, expected_count in draw_indices(count=num_prims, step=2, types=[list, np.ndarray, wp.array]):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            count = expected_count
            for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.apply_physics_materials(v0, indices=indices)
                    output = prim.get_applied_physics_materials(indices=indices)
                check_lists(expected_v0, output, predicate=lambda x: x.paths[0])
        # - check the number of applied materials after applying materials by indices
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            output = prim.get_applied_physics_materials()
        number_of_materials = sum(1 for material in output if material is not None)
        assert (
            number_of_materials == count
        ), f"{count} materials should have been applied. Applied: {number_of_materials}"
        # - all
        count = num_prims
        for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
            with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                prim.apply_physics_materials(v0)
                output = prim.get_applied_physics_materials()
            check_lists(expected_v0, output, predicate=lambda x: x.paths[0])
        # - check the number of applied materials after applying materials by all
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            output = prim.get_applied_physics_materials()
        number_of_materials = sum(1 for material in output if material is not None)
        assert (
            number_of_materials == count
        ), f"{count} materials should have been applied. Applied: {number_of_materials}"

    # --------------------------------------------------------------------
    # volume
    # --------------------------------------------------------------------

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],  # "tensor" backend plays the simulation
        instances=["one"],
        operations=["wrap"],
        prim_class=lambda *args, **kwargs: None,
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "volume"},
        max_num_prims=1,
    )
    async def test_volume_runtime_instance_creation(self, prim, num_prims, device, backend):
        """Test volume runtime instance creation."""
        DeformablePrim("/World/A_0")

    @parametrize(
        devices=["cuda"],
        backends=["tensor", "usd"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "volume"},
    )
    async def test_volume_len(self, prim, num_prims, device, backend):
        """Test volume len."""
        self.assertEqual(len(prim), num_prims, f"Invalid DeformablePrim ({num_prims} prims) len")

    @parametrize(
        devices=["cuda"],
        backends=["tensor", "usd"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "volume"},
    )
    async def test_volume_properties_and_getters(self, prim, num_prims, device, backend):
        """Test volume properties and getters."""
        # check backend
        self.check_backend(backend, prim)
        # test cases (properties)
        self.assertEqual(prim.deformable_type, "volume", f"Invalid deformable type")
        # - amount
        self.assertEqual(prim.num_nodes_per_element, 4, f"Invalid num_nodes_per_element")
        self.assertEqual(prim.num_nodes_per_body, (4, 4, 4), f"Invalid num_nodes_per_body")
        self.assertEqual(prim.num_elements_per_body, (1, 1, 1), f"Invalid num_elements_per_body")
        # - paths
        for path in prim.simulation_mesh_paths:
            self.assertTrue(re.match(r"/World/A_[0-9]+$", path), f"Invalid simulation mesh paths: '{path}'")
        for path in prim.collision_mesh_paths:
            self.assertTrue(re.match(r"/World/A_[0-9]+$", path), f"Invalid collision mesh paths: '{path}'")

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "volume"},
    )
    async def test_volume_element_indices(self, prim, num_prims, device, backend):
        """Test volume element indices."""
        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                output = prim.get_element_indices(indices=indices)
            assert len(output) == 3, "Invalid number of outputs"
            check_array(output, shape=(expected_count, 1, 4), dtype=wp.uint32, device=device)

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "volume"},
    )
    async def test_volume_nodal_positions(self, prim, num_prims, device, backend):
        """Test volume nodal positions."""
        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 4, 3), dtype=wp.float32):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_nodal_positions(v0, indices=indices)
                    output = prim.get_nodal_positions(indices=indices)
                assert len(output) == 3, "Invalid number of outputs"
                check_array(output, shape=(expected_count, 4, 3), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output[0], given=(v0,))

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "volume"},
    )
    async def test_volume_nodal_velocities(self, prim, num_prims, device, backend):
        """Test volume nodal velocities."""
        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 4, 3), dtype=wp.float32):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_nodal_velocities(v0, indices=indices)
                    output = prim.get_nodal_velocities(indices=indices)
                check_array(output, shape=(expected_count, 4, 3), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "volume"},
    )
    async def test_volume_nodal_kinematic_position_targets(self, prim, num_prims, device, backend):
        """Test volume nodal kinematic position targets."""
        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for (v0, expected_v0), (v1, expected_v1) in zip(
                draw_sample(shape=(expected_count, 4, 3), dtype=wp.float32),
                draw_sample(shape=(expected_count, 4, 1), dtype=wp.bool),
            ):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_nodal_kinematic_position_targets(v0, v1, indices=indices)
                    output = prim.get_nodal_kinematic_position_targets(indices=indices)
                check_array(output[0], shape=(expected_count, 4, 3), dtype=wp.float32, device=device)
                check_array(output[1], shape=(expected_count, 4, 1), dtype=wp.bool, device=device)
                check_allclose((expected_v0, expected_v1), output, given=(v0, v1))

    @parametrize(
        devices=["cuda"],
        backends=["usd"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "volume"},
    )
    async def test_volume_physics_materials(self, prim, num_prims, device, backend):
        """Test volume physics materials."""
        from isaacsim.core.experimental.materials import VolumeDeformableMaterial

        choices = [
            VolumeDeformableMaterial("/physics_materials/volume_0", poissons_ratios=[0.1], youngs_moduli=[1000000.0]),
            VolumeDeformableMaterial("/physics_materials/volume_1", poissons_ratios=[0.6], youngs_moduli=[2000000.0]),
        ]
        # test cases
        # - check the number of applied materials before applying any material
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            output = prim.get_applied_physics_materials()
        number_of_materials = sum(1 for material in output if material is not None)
        assert number_of_materials == 0, f"No material should have been applied. Applied: {number_of_materials}"
        # - by indices
        for indices, expected_count in draw_indices(count=num_prims, step=2, types=[list, np.ndarray, wp.array]):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            count = expected_count
            for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.apply_physics_materials(v0, indices=indices)
                    output = prim.get_applied_physics_materials(indices=indices)
                check_lists(expected_v0, output, predicate=lambda x: x.paths[0])
        # - check the number of applied materials after applying materials by indices
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            output = prim.get_applied_physics_materials()
        number_of_materials = sum(1 for material in output if material is not None)
        assert (
            number_of_materials == count
        ), f"{count} materials should have been applied. Applied: {number_of_materials}"
        # - all
        count = num_prims
        for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
            with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                prim.apply_physics_materials(v0)
                output = prim.get_applied_physics_materials()
            check_lists(expected_v0, output, predicate=lambda x: x.paths[0])
        # - check the number of applied materials after applying materials by all
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            output = prim.get_applied_physics_materials()
        number_of_materials = sum(1 for material in output if material is not None)
        assert (
            number_of_materials == count
        ), f"{count} materials should have been applied. Applied: {number_of_materials}"

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "volume"},
    )
    async def test_volume_element_rotations(self, prim, num_prims, device, backend):
        """Test volume element rotations."""
        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                output = prim.get_nodal_rotations(indices=indices)
            check_array(output, shape=(expected_count, 1, 4), dtype=wp.float32, device=device)
            identity_quaternions = np.tile([0.0, 0.0, 0.0, 1.0], (expected_count, 1, 1))
            check_allclose(identity_quaternions.astype(np.float32), output)

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],
        operations=["wrap"],
        instances=["one"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "volume"},
    )
    async def test_volume_element_gradients(self, prim, num_prims, device, backend):
        """Test volume element gradients."""
        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            # - get nodal gradients
            with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                output = prim.get_nodal_gradients(indices=indices)
            check_array(output, shape=(expected_count, 1), dtype=wp.mat33, device=device)
            expected_output = np.tile(np.identity(3), (expected_count, 1, 1)).reshape(expected_count, 1, 3, 3)
            check_allclose(expected_output, output)

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],
        operations=["wrap"],
        instances=["one"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "volume"},
    )
    async def test_volume_element_stresses(self, prim, num_prims, device, backend):
        """Test volume element stresses."""
        from isaacsim.core.experimental.materials import VolumeDeformableMaterial

        choices = [
            VolumeDeformableMaterial("/physics_materials/volume_0", poissons_ratios=[0.1], youngs_moduli=[1000000.0]),
            VolumeDeformableMaterial("/physics_materials/volume_1", poissons_ratios=[0.6], youngs_moduli=[2000000.0]),
        ]
        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            # - apply physics materials
            for v0, expected_v0 in draw_choice(shape=(expected_count,), choices=choices):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.apply_physics_materials(v0, indices=indices)
            # - get nodal stresses
            with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                output = prim.get_nodal_stresses(indices=indices)
            check_array(output, shape=(expected_count, 1), dtype=wp.mat33, device=device)
            check_allclose(np.zeros((expected_count, 1, 3, 3)), output)

    # --------------------------------------------------------------------
    # auto-volume
    # --------------------------------------------------------------------

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],  # "tensor" backend plays the simulation
        instances=["one"],
        operations=["wrap"],
        prim_class=lambda *args, **kwargs: None,
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "auto-volume"},
        max_num_prims=1,
    )
    async def test_auto_volume_runtime_instance_creation(self, prim, num_prims, device, backend):
        """Test auto volume runtime instance creation."""
        DeformablePrim("/World/A_0")

    @parametrize(
        devices=["cuda"],
        backends=["tensor", "usd"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "auto-volume"},
    )
    async def test_auto_volume_len(self, prim, num_prims, device, backend):
        """Test auto volume len."""
        self.assertEqual(len(prim), num_prims, f"Invalid DeformablePrim ({num_prims} prims) len")

    @parametrize(
        devices=["cuda"],
        backends=["tensor", "usd"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "auto-volume"},
    )
    async def test_auto_volume_properties_and_getters(self, prim, num_prims, device, backend):
        """Test auto volume properties and getters."""
        # check backend
        self.check_backend(backend, prim)
        # test cases (properties)
        self.assertEqual(prim.deformable_type, "volume", f"Invalid deformable type")
        # - amount
        self.assertEqual(prim.num_nodes_per_element, 4, f"Invalid num_nodes_per_element")
        self.assertEqual(prim.num_nodes_per_body, self._nnpb, f"Invalid num_nodes_per_body")
        self.assertEqual(prim.num_elements_per_body, self._nepb, f"Invalid num_elements_per_body")
        # - paths
        for path in prim.simulation_mesh_paths:
            self.assertTrue(
                re.match(r"/World/A_[0-9]+/simulation_mesh$", path), f"Invalid simulation mesh paths: '{path}'"
            )
        for path in prim.collision_mesh_paths:
            self.assertTrue(
                re.match(r"/World/A_[0-9]+/collision_mesh$", path), f"Invalid collision mesh paths: '{path}'"
            )

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "auto-volume"},
    )
    async def test_auto_volume_element_indices(self, prim, num_prims, device, backend):
        """Test auto volume element indices."""
        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                output = prim.get_element_indices(indices=indices)
            assert len(output) == 3, "Invalid number of outputs"
            check_array(output[0], shape=(expected_count, self._nepb[0], 4), dtype=wp.uint32, device=device)
            check_array(output[1], shape=(expected_count, self._nepb[1], 4), dtype=wp.uint32, device=device)
            check_array(output[2], shape=(expected_count, self._nepb[2], 4), dtype=wp.uint32, device=device)

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "auto-volume"},
    )
    async def test_auto_volume_nodal_positions(self, prim, num_prims, device, backend):
        """Test auto volume nodal positions."""
        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, self._nnpb[0], 3), dtype=wp.float32):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_nodal_positions(v0, indices=indices)
                    output = prim.get_nodal_positions(indices=indices)
                assert len(output) == 3, "Invalid number of outputs"
                check_array(output[0], shape=(expected_count, self._nnpb[0], 3), dtype=wp.float32, device=device)
                check_array(output[1], shape=(expected_count, self._nnpb[1], 3), dtype=wp.float32, device=device)
                check_array(output[2], shape=(expected_count, self._nnpb[2], 3), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output[0], given=(v0,))

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "auto-volume"},
    )
    async def test_auto_volume_nodal_velocities(self, prim, num_prims, device, backend):
        """Test auto volume nodal velocities."""
        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, self._nnpb[0], 3), dtype=wp.float32):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_nodal_velocities(v0, indices=indices)
                    output = prim.get_nodal_velocities(indices=indices)
                check_array(output, shape=(expected_count, self._nnpb[0], 3), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "auto-volume"},
    )
    async def test_auto_volume_nodal_kinematic_position_targets(self, prim, num_prims, device, backend):
        """Test auto volume nodal kinematic position targets."""
        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for (v0, expected_v0), (v1, expected_v1) in zip(
                draw_sample(shape=(expected_count, self._nnpb[0], 3), dtype=wp.float32),
                draw_sample(shape=(expected_count, self._nnpb[0], 1), dtype=wp.bool),
            ):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_nodal_kinematic_position_targets(v0, v1, indices=indices)
                    output = prim.get_nodal_kinematic_position_targets(indices=indices)
                check_array(output[0], shape=(expected_count, self._nnpb[0], 3), dtype=wp.float32, device=device)
                check_array(output[1], shape=(expected_count, self._nnpb[0], 1), dtype=wp.bool, device=device)
                check_allclose((expected_v0, expected_v1), output, given=(v0, v1))

    @parametrize(
        devices=["cuda"],
        backends=["usd"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "auto-volume"},
    )
    async def test_auto_volume_physics_materials(self, prim, num_prims, device, backend):
        """Test auto volume physics materials."""
        from isaacsim.core.experimental.materials import VolumeDeformableMaterial

        choices = [
            VolumeDeformableMaterial("/physics_materials/volume_0", poissons_ratios=[0.1], youngs_moduli=[1000000.0]),
            VolumeDeformableMaterial("/physics_materials/volume_1", poissons_ratios=[0.6], youngs_moduli=[2000000.0]),
        ]
        # test cases
        # - check the number of applied materials before applying any material
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            output = prim.get_applied_physics_materials()
        number_of_materials = sum(1 for material in output if material is not None)
        assert number_of_materials == 0, f"No material should have been applied. Applied: {number_of_materials}"
        # - by indices
        for indices, expected_count in draw_indices(count=num_prims, step=2, types=[list, np.ndarray, wp.array]):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            count = expected_count
            for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.apply_physics_materials(v0, indices=indices)
                    output = prim.get_applied_physics_materials(indices=indices)
                check_lists(expected_v0, output, predicate=lambda x: x.paths[0])
        # - check the number of applied materials after applying materials by indices
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            output = prim.get_applied_physics_materials()
        number_of_materials = sum(1 for material in output if material is not None)
        assert (
            number_of_materials == count
        ), f"{count} materials should have been applied. Applied: {number_of_materials}"
        # - all
        count = num_prims
        for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
            with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                prim.apply_physics_materials(v0)
                output = prim.get_applied_physics_materials()
            check_lists(expected_v0, output, predicate=lambda x: x.paths[0])
        # - check the number of applied materials after applying materials by all
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            output = prim.get_applied_physics_materials()
        number_of_materials = sum(1 for material in output if material is not None)
        assert (
            number_of_materials == count
        ), f"{count} materials should have been applied. Applied: {number_of_materials}"

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "auto-volume", "mesh_simplification": False},
    )
    async def test_auto_volume_element_rotations(self, prim, num_prims, device, backend):
        """Test auto volume element rotations."""
        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                output = prim.get_nodal_rotations(indices=indices)
            check_array(output, shape=(expected_count, self._nepb_nms[0], 4), dtype=wp.float32, device=device)
            identity_quaternions = np.tile([0.0, 0.0, 0.0, 1.0], (expected_count, self._nepb_nms[0], 1))
            check_allclose(identity_quaternions.astype(np.float32), output)

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "auto-volume", "mesh_simplification": False},
    )
    async def test_auto_volume_element_gradients(self, prim, num_prims, device, backend):
        """Test auto volume element gradients."""
        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                output = prim.get_nodal_gradients(indices=indices)
            check_array(output, shape=(expected_count, self._nepb_nms[0]), dtype=wp.mat33, device=device)
            identity_matrices = np.tile(np.identity(3), (expected_count, self._nepb_nms[0], 1, 1))
            check_allclose(identity_matrices.astype(np.float32), output)

    @parametrize(
        devices=["cuda"],
        backends=["tensor"],
        operations=["wrap"],
        prim_class=DeformablePrim,
        prim_class_kwargs={},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"deformable_case": "auto-volume", "mesh_simplification": False},
    )
    async def test_auto_volume_element_stresses(self, prim, num_prims, device, backend):
        """Test auto volume element stresses."""
        from isaacsim.core.experimental.materials import VolumeDeformableMaterial

        choices = [
            VolumeDeformableMaterial("/physics_materials/volume_0", poissons_ratios=[0.1], youngs_moduli=[1000000.0]),
            VolumeDeformableMaterial("/physics_materials/volume_1", poissons_ratios=[0.6], youngs_moduli=[2000000.0]),
        ]
        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            # - apply physics materials
            for v0, expected_v0 in draw_choice(shape=(expected_count,), choices=choices):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.apply_physics_materials(v0, indices=indices)
            # - get nodal stresses
            with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                output = prim.get_nodal_stresses(indices=indices)
            check_array(output, shape=(expected_count, self._nepb_nms[0]), dtype=wp.mat33, device=device)
            check_allclose(np.zeros((expected_count, self._nepb_nms[0], 3, 3)), output, atol=20)
