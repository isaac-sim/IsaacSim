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

"""Validate public API docstring examples for object wrappers.

The suite builds a fresh CPU-backed stage and runs docstring tests for cameras,
ground planes, meshes, geometric shape wrappers, shared base classes, and all
light wrappers exported by this extension.
"""

import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.test.docstring
from isaacsim.core.experimental.objects import (
    Camera,
    Capsule,
    Cone,
    Cube,
    Cylinder,
    CylinderLight,
    DiskLight,
    DistantLight,
    DomeLight,
    GroundPlane,
    Light,
    Mesh,
    Plane,
    RectLight,
    Shape,
    Sphere,
    SphereLight,
)
from isaacsim.core.simulation_manager import SimulationManager


class TestExtensionDocstrings(isaacsim.test.docstring.AsyncDocTestCase):
    """Run doctest coverage for exported object wrapper classes."""

    async def setUp(self) -> None:
        """Create a fresh ``/World`` stage and force CPU physics for doctests."""
        super().setUp()
        # create new stage
        await stage_utils.create_new_stage_async()
        stage_utils.define_prim(f"/World", "Xform")
        # configure simulation
        SimulationManager.set_physics_sim_device("cpu")

    async def tearDown(self) -> None:
        """Finalize the docstring test fixture."""
        super().tearDown()

    # --------------------------------------------------------------------

    async def test_camera_docstrings(self) -> None:
        """Test camera docstrings."""
        await self.assertDocTests(Camera)

    # --------------------------------------------------------------------

    async def test_ground_plane_docstrings(self) -> None:
        """Test ground plane docstrings."""
        await self.assertDocTests(GroundPlane)

    # --------------------------------------------------------------------

    async def test_mesh_docstrings(self) -> None:
        """Test mesh docstrings."""
        await self.assertDocTests(Mesh)

    # --------------------------------------------------------------------

    async def test_capsule_docstrings(self) -> None:
        """Test capsule docstrings."""
        await self.assertDocTests(Capsule)
        await self.assertDocTests(Shape)

    async def test_cone_docstrings(self) -> None:
        """Test cone docstrings."""
        await self.assertDocTests(Cone)
        await self.assertDocTests(Shape)

    async def test_cube_docstrings(self) -> None:
        """Test cube docstrings."""
        await self.assertDocTests(Cube)
        await self.assertDocTests(Shape)

    async def test_cylinder_docstrings(self) -> None:
        """Test cylinder docstrings."""
        await self.assertDocTests(Cylinder)
        await self.assertDocTests(Shape)

    async def test_plane_docstrings(self) -> None:
        """Test plane docstrings."""
        await self.assertDocTests(Plane)
        await self.assertDocTests(Shape)

    async def test_sphere_docstrings(self) -> None:
        """Test sphere docstrings."""
        await self.assertDocTests(Sphere)
        await self.assertDocTests(Shape)

    # --------------------------------------------------------------------

    async def test_cylinder_light_docstrings(self) -> None:
        """Test cylinder light docstrings."""
        await self.assertDocTests(CylinderLight)
        await self.assertDocTests(Light)

    async def test_disk_light_docstrings(self) -> None:
        """Test disk light docstrings."""
        await self.assertDocTests(DiskLight)
        await self.assertDocTests(Light)

    async def test_distant_light_docstrings(self) -> None:
        """Test distant light docstrings."""
        await self.assertDocTests(DistantLight)
        await self.assertDocTests(Light)

    async def test_dome_light_docstrings(self) -> None:
        """Test dome light docstrings."""
        await self.assertDocTests(DomeLight)
        await self.assertDocTests(Light)

    async def test_rect_light_docstrings(self) -> None:
        """Test rect light docstrings."""
        await self.assertDocTests(RectLight)
        await self.assertDocTests(Light)

    async def test_sphere_light_docstrings(self) -> None:
        """Test sphere light docstrings."""
        await self.assertDocTests(SphereLight)
        await self.assertDocTests(Light)
