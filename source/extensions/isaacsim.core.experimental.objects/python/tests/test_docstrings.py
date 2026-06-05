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

"""Test for docstrings."""

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
    """Test extension docstrings."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        # create new stage
        await stage_utils.create_new_stage_async()
        stage_utils.define_prim(f"/World", "Xform")
        # configure simulation
        SimulationManager.set_physics_sim_device("cpu")

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        super().tearDown()

    # --------------------------------------------------------------------

    async def test_camera_docstrings(self):
        """Test camera docstrings."""
        await self.assertDocTests(Camera)

    # --------------------------------------------------------------------

    async def test_ground_plane_docstrings(self):
        """Test ground plane docstrings."""
        await self.assertDocTests(GroundPlane)

    # --------------------------------------------------------------------

    async def test_mesh_docstrings(self):
        """Test mesh docstrings."""
        await self.assertDocTests(Mesh)

    # --------------------------------------------------------------------

    async def test_capsule_docstrings(self):
        """Test capsule docstrings."""
        await self.assertDocTests(Capsule)
        await self.assertDocTests(Shape)

    async def test_cone_docstrings(self):
        """Test cone docstrings."""
        await self.assertDocTests(Cone)
        await self.assertDocTests(Shape)

    async def test_cube_docstrings(self):
        """Test cube docstrings."""
        await self.assertDocTests(Cube)
        await self.assertDocTests(Shape)

    async def test_cylinder_docstrings(self):
        """Test cylinder docstrings."""
        await self.assertDocTests(Cylinder)
        await self.assertDocTests(Shape)

    async def test_plane_docstrings(self):
        """Test plane docstrings."""
        await self.assertDocTests(Plane)
        await self.assertDocTests(Shape)

    async def test_sphere_docstrings(self):
        """Test sphere docstrings."""
        await self.assertDocTests(Sphere)
        await self.assertDocTests(Shape)

    # --------------------------------------------------------------------

    async def test_cylinder_light_docstrings(self):
        """Test cylinder light docstrings."""
        await self.assertDocTests(CylinderLight)
        await self.assertDocTests(Light)

    async def test_disk_light_docstrings(self):
        """Test disk light docstrings."""
        await self.assertDocTests(DiskLight)
        await self.assertDocTests(Light)

    async def test_distant_light_docstrings(self):
        """Test distant light docstrings."""
        await self.assertDocTests(DistantLight)
        await self.assertDocTests(Light)

    async def test_dome_light_docstrings(self):
        """Test dome light docstrings."""
        await self.assertDocTests(DomeLight)
        await self.assertDocTests(Light)

    async def test_rect_light_docstrings(self):
        """Test rect light docstrings."""
        await self.assertDocTests(RectLight)
        await self.assertDocTests(Light)

    async def test_sphere_light_docstrings(self):
        """Test sphere light docstrings."""
        await self.assertDocTests(SphereLight)
        await self.assertDocTests(Light)
