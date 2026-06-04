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

"""Test suite for the OgnWritePhysicsRigidPrimView node in the experimental DR extension."""

import isaacsim.replicator.experimental.domain_randomization as dr
import numpy as np
import omni.graph.core as og
import omni.kit.test
import omni.timeline
import omni.usd
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.core.experimental.utils.stage import create_new_stage_async
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.replicator.experimental.domain_randomization import physics_view as physics


class TestOgnWritePhysicsRigidPrimView(omni.kit.test.AsyncTestCase):
    """Test suite for the OgnWritePhysicsRigidPrimView node in the experimental domain randomization extension.

    Validates randomization of rigid body physics properties including position, orientation,
    velocities, forces, masses, inertias, material properties, and collision offsets using
    a dynamic cube primitive.
    """

    async def setUp(self):
        """Set up the test environment with a physics world, rigid body, and OmniGraph nodes."""
        await create_new_stage_async()

        SimulationManager.setup_simulation()
        await omni.kit.app.get_app().next_update_async()

        physics_scenes = SimulationManager.get_physics_scenes()
        if physics_scenes:
            physics_scenes[0].set_gravity((0, 0, 0))
        await omni.kit.app.get_app().next_update_async()

        self._stage = omni.usd.get_context().get_stage()
        self._controller = og.Controller()
        self._graph = self._controller.create_graph("/World/PushGraph")

        self._rigid_prim_view_node = self._controller.create_node(
            ("rigid_prim_view", self._graph),
            "isaacsim.replicator.domain_randomization.OgnWritePhysicsRigidPrimView",
        )
        self._distribution_node = self._controller.create_node(
            ("uniform", self._graph), "omni.replicator.core.OgnSampleUniform"
        )

        self._iface = omni.timeline.get_timeline_interface()
        self._cube_path = "/World/Cube"
        Cube(self._cube_path)
        GeomPrim(self._cube_path, apply_collision_apis=True)
        self._rb_view = RigidPrim(self._cube_path, masses=1.0)

        await omni.kit.app.get_app().next_update_async()

        self._iface.play()
        await omni.kit.app.get_app().next_update_async()

        dr.physics_view.register_rigid_prim_view(self._rb_view, name="cube")
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        """Clean up the test environment by stopping timeline, clearing state, and closing stage."""
        self._iface.stop()
        dr.physics_view._rigid_prim_views = dict()
        dr.physics_view._rigid_prim_views_initial_values = dict()
        dr.physics_view._rigid_prim_views_reset_values = dict()
        omni.usd.get_context().close_stage()

    async def _setup_random_attribute(self, attribute_name, value):
        """Set up a random attribute for the rigid prim view node with specified value.

        Args:
            attribute_name: Name of the physics attribute to randomize.
            value: Value to assign to the attribute.
        """
        self._distribution_node.get_attribute("inputs:numSamples").set(1)
        self._distribution_node.get_attribute("inputs:lower").set([value])
        self._distribution_node.get_attribute("inputs:upper").set([value])

        self._rigid_prim_view_node.get_attribute("inputs:prims").set("cube")
        self._rigid_prim_view_node.get_attribute("inputs:attribute").set(attribute_name)
        self._rigid_prim_view_node.get_attribute("inputs:indices").set([0])
        self._rigid_prim_view_node.get_attribute("inputs:operation").set("direct")

        n_elem = physics._rigid_prim_views_initial_values["cube"][attribute_name].shape[-1]
        self._rigid_prim_view_node.get_attribute("inputs:values").set_resolved_type(
            og.Type(og.BaseDataType.FLOAT, n_elem, 1)
        )

        self._controller.connect(
            self._distribution_node.get_attribute("outputs:samples"),
            self._rigid_prim_view_node.get_attribute("inputs:values"),
        )
        await self._controller.evaluate(self._graph)

    async def test_randomize_position(self):
        """Test randomization of rigid body position."""
        value = [100, 200, 300]
        await self._setup_random_attribute(attribute_name="position", value=value)
        position, _ = self._rb_view.get_world_poses()
        position = np.asarray(position)
        self.assertTrue(np.all(np.isclose(position, value)))

    async def test_randomize_orientation(self):
        """Test randomization of rigid body orientation."""
        value = [0, np.pi, 0]
        await self._setup_random_attribute(attribute_name="orientation", value=value)
        _, orientation = self._rb_view.get_world_poses()
        orientation = np.asarray(orientation)
        self.assertTrue(np.all(np.isclose(orientation, [0, 0, 1, 0], atol=1e-04)))

    async def test_randomize_linear_velocity(self):
        """Test randomization of rigid body linear velocity."""
        value = [100, 200, 300]
        await self._setup_random_attribute(attribute_name="linear_velocity", value=value)
        linear_velocity, _ = self._rb_view.get_velocities()
        linear_velocity = np.asarray(linear_velocity)
        self.assertTrue(np.all(np.isclose(linear_velocity, value)))

    async def test_randomize_angular_velocity(self):
        """Test randomization of rigid body angular velocity."""
        value = [100, 200, 300]
        await self._setup_random_attribute(attribute_name="angular_velocity", value=value)
        _, angular_velocity = self._rb_view.get_velocities()
        angular_velocity = np.asarray(angular_velocity)
        self.assertTrue(np.all(np.isclose(angular_velocity, value)))

    async def test_randomize_forces(self):
        """Test randomization of forces applied to the rigid body."""
        value = [100, 100, 100]
        await self._setup_random_attribute(attribute_name="force", value=value)

    async def test_randomize_masses(self):
        """Test randomization of rigid body masses (CPU only)."""
        if SimulationManager.get_physics_sim_device() == "cpu":
            physics_view = self._rb_view._physics_rigid_body_view
            value = [100] * len(self._rb_view)
            await self._setup_random_attribute(attribute_name="mass", value=value)
            new_value = np.asarray(physics_view.get_masses())
            self.assertTrue(np.all(np.isclose(new_value, value)))

    async def test_randomize_inertias(self):
        """Test randomization of rigid body inertias (CPU only)."""
        if SimulationManager.get_physics_sim_device() == "cpu":
            physics_view = self._rb_view._physics_rigid_body_view
            inertias = [0.1, 0.1, 0.1] * len(self._rb_view)
            await self._setup_random_attribute(attribute_name="inertia", value=inertias)
            new_value = np.asarray(physics_view.get_inertias())
            diagonal = new_value[:, [0, 4, 8]]
            self.assertTrue(np.all(np.isclose(diagonal.flatten(), inertias)))

    async def test_randomize_material_properties(self):
        """Test randomization of material properties for rigid body prims."""
        physics_view = self._rb_view._physics_rigid_body_view
        value = [0.5] * len(self._rb_view) * 3 * physics_view.max_shapes
        await self._setup_random_attribute(attribute_name="material_properties", value=value)
        new_value = np.asarray(physics_view.get_material_properties()).flatten()
        self.assertTrue(np.all(np.isclose(new_value, value)))

    async def test_randomize_contact_offsets(self):
        """Test randomization of contact offsets for rigid body prims."""
        physics_view = self._rb_view._physics_rigid_body_view
        value = [0.05] * len(self._rb_view) * physics_view.max_shapes
        await self._setup_random_attribute(attribute_name="contact_offset", value=value)
        new_value = np.asarray(physics_view.get_contact_offsets())
        self.assertTrue(np.all(np.isclose(new_value, value)))

    async def test_randomize_rest_offset(self):
        """Test randomization of rest offsets for rigid body prims."""
        physics_view = self._rb_view._physics_rigid_body_view
        value = np.asarray(physics_view.get_contact_offsets()) / 2
        await self._setup_random_attribute(attribute_name="rest_offset", value=value)
        new_value = np.asarray(physics_view.get_rest_offsets())
        self.assertTrue(np.all(np.isclose(new_value, value)))
