# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Verifies custom Replicator randomizer examples implemented with mathematical distributions and OmniGraph nodes. The tests exercise both direct randomizer execution and graph-based custom randomization."""

from typing import Any

import omni.kit
import omni.usd
from isaacsim.core.experimental.utils.stage import create_new_stage_async


class TestOgnCustomReplicatorRandomizer(omni.kit.test.AsyncTestCase):
    """Test class for demonstrating custom replicator randomization techniques.

    This class provides comprehensive test cases for implementing and using custom randomizers
    within Omniverse Replicator. It demonstrates two approaches: manual randomization using pure
    Python code and automated randomization using Omniverse Graph (OmniGraph) nodes.

    The test class showcases three distinct spatial distribution patterns:

    * Point sampling on sphere surfaces for objects positioned at exact distances
    * Point sampling within sphere volumes for objects distributed throughout 3D spaces
    * Point sampling between concentric spheres for objects in annular regions

    The first test method demonstrates manual randomization by implementing mathematical algorithms
    for uniform spatial distribution. It creates 1,500 primitives (spheres, cubes, cylinders) and
    applies randomized positions and rotations using custom Python functions that ensure proper
    spatial uniformity.

    The second test method demonstrates automated randomization through custom OmniGraph nodes.
    It defines replicator wrapper functions that interface with specialized nodes:
    ``isaacsim.replicator.examples.OgnSampleOnSphere``, ``isaacsim.replicator.examples.OgnSampleInSphere``,
    and ``isaacsim.replicator.examples.OgnSampleBetweenSpheres``. These nodes provide the same
    spatial distribution capabilities as the manual approach but integrate seamlessly with
    Omniverse Replicator's orchestration system.

    Both approaches handle proper mathematical considerations for uniform distribution, including
    cube root scaling for volumetric sampling and cosine theta sampling for spherical coordinates.
    The test validates that custom randomizers can be successfully integrated into Replicator
    workflows while maintaining proper spatial distribution properties.
    """

    async def setUp(self) -> None:
        """Set up the test environment by creating a new stage."""
        await omni.kit.app.get_app().next_update_async()
        await create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Clean up the test environment by closing the stage and waiting for assets to finish loading."""
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def test_custom_randomizer(self) -> Any:
        """Test custom randomizer implementations for distributing prims on sphere surfaces and volumes.

        Creates 500 prims of each type (spheres, cubes, cylinders) and randomizes their positions using
        custom mathematical distribution functions: on sphere surface, within sphere volume, and between
        two concentric spheres.

        Returns:
            None.
        """
        import math
        import random
        from itertools import chain

        import omni.usd
        from pxr import UsdGeom

        # Generate a random 3D point on the surface of a sphere of a given radius.
        def random_point_on_sphere(radius: Any) -> Any:
            # Generate a random direction by spherical coordinates (phi, theta)
            phi = random.uniform(0, 2 * math.pi)
            # Sample costheta to ensure uniform distribution of points on the sphere (surface is proportional to sin(theta))
            costheta = random.uniform(-1, 1)
            theta = math.acos(costheta)

            # Convert from spherical to Cartesian coordinates
            x = radius * math.sin(theta) * math.cos(phi)
            y = radius * math.sin(theta) * math.sin(phi)
            z = radius * math.cos(theta)

            return x, y, z

        # Generate a random 3D point within a sphere of a given radius, ensuring a uniform distribution throughout the volume.
        def random_point_in_sphere(radius: Any) -> Any:
            # Generate a random direction by spherical coordinates (phi, theta)
            phi = random.uniform(0, 2 * math.pi)
            # Sample costheta to ensure uniform distribution of points on the sphere (surface is proportional to sin(theta))
            costheta = random.uniform(-1, 1)
            theta = math.acos(costheta)

            # Scale the radius uniformly within the sphere, applying the cube root to a random value
            # to account for volume's cubic growth with radius (r^3), ensuring spatial uniformity.
            r = radius * (random.random() ** (1 / 3))

            # Convert from spherical to Cartesian coordinates
            x = r * math.sin(theta) * math.cos(phi)
            y = r * math.sin(theta) * math.sin(phi)
            z = r * math.cos(theta)

            return x, y, z

        # Generate a random 3D point between two spheres, ensuring a uniform distribution throughout the volume.
        def random_point_between_spheres(radius1: Any, radius2: Any) -> Any:
            # Ensure radius1 < radius2
            if radius1 > radius2:
                radius1, radius2 = radius2, radius1

            # Generate a random direction by spherical coordinates (phi, theta)
            phi = random.uniform(0, 2 * math.pi)
            # Sample costheta to ensure uniform distribution of points on the sphere (surface is proportional to sin(theta))
            costheta = random.uniform(-1, 1)
            theta = math.acos(costheta)

            # Uniformly distribute points between two spheres by weighting the radius to match volume growth (r^3),
            # ensuring spatial uniformity by taking the cube root of a value between the radii cubed.
            r = (random.uniform(radius1**3, radius2**3)) ** (1 / 3.0)

            # Convert from spherical to Cartesian coordinates
            x = r * math.sin(theta) * math.cos(phi)
            y = r * math.sin(theta) * math.sin(phi)
            z = r * math.cos(theta)

            return x, y, z

        stage = omni.usd.get_context().get_stage()
        prim_count = 500
        prim_scale = 0.1
        rad_in = 0.5
        rad_on = 1.5
        rad_bet1 = 2.5
        rad_bet2 = 3.5

        # Create the default prims
        on_sphere_prims = [stage.DefinePrim(f"/World/sphere_{i}", "Sphere") for i in range(prim_count)]
        in_sphere_prims = [stage.DefinePrim(f"/World/cube_{i}", "Cube") for i in range(prim_count)]
        between_spheres_prims = [stage.DefinePrim(f"/World/cylinder_{i}", "Cylinder") for i in range(prim_count)]

        # Add xformOps and scale to the prims
        for prim in chain(on_sphere_prims, in_sphere_prims, between_spheres_prims):
            if not prim.HasAttribute("xformOp:translate"):
                UsdGeom.Xformable(prim).AddTranslateOp()
            if not prim.HasAttribute("xformOp:scale"):
                UsdGeom.Xformable(prim).AddScaleOp()
            if not prim.HasAttribute("xformOp:rotateXYZ"):
                UsdGeom.Xformable(prim).AddRotateXYZOp()
            prim.GetAttribute("xformOp:scale").Set((prim_scale, prim_scale, prim_scale))

        # Randomize the prims
        for _ in range(10):
            for in_sphere_prim in in_sphere_prims:
                rand_rot = (random.uniform(0, 360), random.uniform(0, 360), random.uniform(0, 360))
                in_sphere_prim.GetAttribute("xformOp:rotateXYZ").Set(rand_rot)
                rand_loc = random_point_in_sphere(rad_in)
                in_sphere_prim.GetAttribute("xformOp:translate").Set(rand_loc)

            for on_sphere_prim in on_sphere_prims:
                rand_rot = (random.uniform(0, 360), random.uniform(0, 360), random.uniform(0, 360))
                on_sphere_prim.GetAttribute("xformOp:rotateXYZ").Set(rand_rot)
                rand_loc = random_point_on_sphere(rad_on)
                on_sphere_prim.GetAttribute("xformOp:translate").Set(rand_loc)

            for between_spheres_prim in between_spheres_prims:
                rand_rot = (random.uniform(0, 360), random.uniform(0, 360), random.uniform(0, 360))
                between_spheres_prim.GetAttribute("xformOp:rotateXYZ").Set(rand_rot)
                rand_loc = random_point_between_spheres(rad_bet1, rad_bet2)
                between_spheres_prim.GetAttribute("xformOp:translate").Set(rand_loc)

    async def test_custom_replicator_randomizer_graph(self) -> Any:
        """Test custom replicator randomizer graph using Omni Replicator framework.

        Creates 50 prims of each type and applies randomization using custom replicator nodes for
        sphere-based distribution patterns. Tests the integration with omni.replicator.core framework.

        Returns:
            None.
        """
        import omni.replicator.core as rep
        from omni.replicator.core.scripts.utils import ReplicatorItem, ReplicatorWrapper, create_node, set_target_prims

        @ReplicatorWrapper
        def on_sphere(
            radius: float = 1.0,
            input_prims: ReplicatorItem | list[str] | None = None,
        ) -> ReplicatorItem:

            node = create_node("isaacsim.replicator.examples.OgnSampleOnSphere", radius=radius)
            if input_prims:
                set_target_prims(node, "inputs:prims", input_prims)
            return node

        @ReplicatorWrapper
        def in_sphere(
            radius: float = 1.0,
            input_prims: ReplicatorItem | list[str] | None = None,
        ) -> ReplicatorItem:

            node = create_node("isaacsim.replicator.examples.OgnSampleInSphere", radius=radius)
            if input_prims:
                set_target_prims(node, "inputs:prims", input_prims)
            return node

        @ReplicatorWrapper
        def between_spheres(
            radius1: float = 0.5,
            radius2: float = 1.0,
            input_prims: ReplicatorItem | list[str] | None = None,
        ) -> ReplicatorItem:

            node = create_node("isaacsim.replicator.examples.OgnSampleBetweenSpheres", radius1=radius1, radius2=radius2)
            if input_prims:
                set_target_prims(node, "inputs:prims", input_prims)
            return node

        prim_count = 50
        prim_scale = 0.1
        rad_in = 0.5
        rad_on = 1.5
        rad_bet1 = 2.5
        rad_bet2 = 3.5

        # Create the default prims
        sphere = rep.create.sphere(count=prim_count, scale=prim_scale)
        cube = rep.create.cube(count=prim_count, scale=prim_scale)
        cylinder = rep.create.cylinder(count=prim_count, scale=prim_scale)

        # Create the randomization graph
        with rep.trigger.on_frame():
            with sphere:
                rep.randomizer.rotation()
                in_sphere(rad_in)

            with cube:
                rep.randomizer.rotation()
                on_sphere(rad_on)

            with cylinder:
                rep.randomizer.rotation()
                between_spheres(rad_bet1, rad_bet2)

        # Preview and call the graph
        await rep.orchestrator.preview_async()
        for _ in range(10):
            await rep.orchestrator.step_async()
