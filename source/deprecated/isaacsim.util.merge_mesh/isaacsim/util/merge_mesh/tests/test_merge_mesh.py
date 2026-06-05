# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for the mesh merge command and utility."""

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
from pxr import UsdShade

from ..mesh_merger import MeshMerger


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestMergeMesh(omni.kit.test.AsyncTestCase):
    """Test suite for the Mesh Merge Tool functionality."""

    async def setUp(self) -> None:
        """Set up the test environment before each test.

        Creates a new stage with three cube meshes, each with a bound OmniPBR material.
        """
        await omni.usd.get_context().new_stage_async()
        self._stage = omni.usd.get_context().get_stage()
        mtl_created_list = []
        # Add three cubes to the scene
        self.cubes_list = []
        for i in range(3):
            result, path = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube")
            self.cubes_list.append(path)
            # Create a new material using OmniPBR.mdl
            omni.kit.commands.execute(
                "CreateAndBindMdlMaterialFromLibrary",
                mdl_name="OmniPBR.mdl",
                mtl_name="OmniPBR",
                mtl_created_list=mtl_created_list,
            )
            mtl_prim = self._stage.GetPrimAtPath(mtl_created_list[-1])
            # Get the path to the prim
            cube_prim = self._stage.GetPrimAtPath(path)
            # Bind the material to the prim
            cube_mat_shade = UsdShade.Material(mtl_prim)
            UsdShade.MaterialBindingAPI(cube_prim).Bind(cube_mat_shade, UsdShade.Tokens.strongerThanDescendants)

        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Clean up after each test."""
        await omni.kit.app.get_app().next_update_async()

    async def test_startup(self) -> None:
        """Test that the Mesh Merge Tool window loads without errors."""
        window = omni.ui.Workspace.get_window("Mesh Merge Tool")
        self.assertIsNotNone(window)
        window.visible = True
        for frame in range(60):
            await omni.kit.app.get_app().next_update_async()
        window.visible = False

    async def test_basic_merge(self) -> None:
        """Test basic mesh merge functionality without any additional options."""
        mesh_merger = MeshMerger(self._stage)
        mesh_merger.clear_parent_xform = False
        mesh_merger.deactivate_source = False
        mesh_merger.combine_materials = False
        mesh_merger.materials_destination = ""
        mesh_merger.update_selection(selection=self.cubes_list, stage=self._stage)
        mesh_merger.output_mesh = "/Merged/" + str(self._stage.GetPrimAtPath(self.cubes_list[0]).GetName())

        mesh_merger.merge_meshes()
        await omni.kit.app.get_app().next_update_async()
        merged = self._stage.GetPrimAtPath(mesh_merger.output_mesh)
        self.assertTrue(merged.IsValid())

        self.assertEqual(len(merged.GetChildren()), 3)

    async def test_material_combine_merge(self) -> None:
        """Test mesh merge with material combining enabled."""
        mesh_merger = MeshMerger(self._stage)
        mesh_merger.clear_parent_xform = False
        mesh_merger.deactivate_source = False
        mesh_merger.combine_materials = True
        mesh_merger.materials_destination = "/World/Looks2"
        mesh_merger.update_selection(selection=self.cubes_list, stage=self._stage)
        mesh_merger.output_mesh = "/Merged/" + str(self._stage.GetPrimAtPath(self.cubes_list[0]).GetName())

        mesh_merger.merge_meshes()

        merged = self._stage.GetPrimAtPath(mesh_merger.output_mesh)
        self.assertTrue(merged.IsValid())

        self.assertEqual(len(merged.GetChildren()), 3)

        newLooks = self._stage.GetPrimAtPath("/World/Looks2")
        self.assertEqual(len(newLooks.GetChildren()), 3)

    async def test_deactivate_source(self) -> None:
        """Test that source prims are deactivated after merge when the option is enabled."""
        mesh_merger = MeshMerger(self._stage)
        mesh_merger.clear_parent_xform = False
        mesh_merger.deactivate_source = True
        mesh_merger.combine_materials = False
        mesh_merger.materials_destination = "/World/Looks2"
        mesh_merger.update_selection(selection=self.cubes_list, stage=self._stage)
        mesh_merger.output_mesh = "/Merged/" + str(self._stage.GetPrimAtPath(self.cubes_list[0]).GetName())

        mesh_merger.merge_meshes()

        merged = self._stage.GetPrimAtPath(mesh_merger.output_mesh)
        self.assertTrue(merged.IsValid())

        self.assertEqual(len(merged.GetChildren()), 3)

        for src in self.cubes_list:
            prim = self._stage.GetPrimAtPath(src)
            self.assertFalse(prim.IsActive())

    async def test_clear_parent_xform(self) -> None:
        """Test mesh merge with parent transform clearing enabled."""
        mesh_merger = MeshMerger(self._stage)
        mesh_merger.clear_parent_xform = True
        mesh_merger.deactivate_source = True
        mesh_merger.combine_materials = False
        mesh_merger.materials_destination = "/World/Looks2"
        mesh_merger.update_selection(selection=self.cubes_list, stage=self._stage)
        mesh_merger.output_mesh = "/Merged/" + str(self._stage.GetPrimAtPath(self.cubes_list[0]).GetName())

        mesh_merger.merge_meshes()

        merged = self._stage.GetPrimAtPath(mesh_merger.output_mesh)
        self.assertTrue(merged.IsValid())

    async def test_merge_command(self) -> None:
        """Test the MergeMeshesCommand with all options enabled."""
        result, prim = omni.kit.commands.execute(
            "MergeMeshesCommand",
            source=self.cubes_list,
            clear_transform=False,
            deactivate_source=True,
            combine_materials=True,
            materials_destination="/World/Looks",
        )

        merged = self._stage.GetPrimAtPath(prim)
        self.assertTrue(merged.IsValid())
