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

"""Test suite for validating the OgnWritePhysicsArticulationView node in the experimental DR extension."""

import isaacsim.replicator.experimental.domain_randomization as dr
import numpy as np
import omni.graph.core as og
import omni.kit.test
import omni.timeline
import omni.usd
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.experimental.utils.stage import add_reference_to_stage, create_new_stage_async
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path_async


class TestOgnWritePhysicsArticulationView(omni.kit.test.AsyncTestCase):
    """Test suite for the OgnWritePhysicsArticulationView node in the experimental domain randomization extension.

    Validates randomization of articulation physics properties including joint parameters,
    dynamics properties, and physics simulation settings using a Franka Panda robot.
    """

    async def setUp(self):
        """Set up the test environment with a physics simulation, articulation view, and Franka robot."""
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

        self._articulation_view_node = self._controller.create_node(
            ("articulation_view", self._graph),
            "isaacsim.replicator.domain_randomization.OgnWritePhysicsArticulationView",
        )
        self._distribution_node = self._controller.create_node(
            ("uniform", self._graph), "omni.replicator.core.OgnSampleUniform"
        )

        self._iface = omni.timeline.get_timeline_interface()

        assets_root_path = await get_assets_root_path_async()
        asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        add_reference_to_stage(usd_path=asset_path, path="/World/Franka")

        self._articulation_view = Articulation("/World/Franka")
        await omni.kit.app.get_app().next_update_async()

        self._iface.play()
        await omni.kit.app.get_app().next_update_async()

        dr.physics_view.register_articulation_view(self._articulation_view, name="franka")
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        """Clean up the test environment by stopping the simulation and clearing resources."""
        self._iface.stop()
        dr.physics_view._articulation_views = dict()
        dr.physics_view._articulation_views_initial_values = dict()
        dr.physics_view._articulation_views_reset_values = dict()
        dr.physics_view._current_tendon_properties = dict()
        omni.usd.get_context().close_stage()

    async def _setup_random_attribute(self, attribute_name, value):
        """Set up a random attribute for the articulation view node with the specified value.

        Args:
            attribute_name: Name of the attribute to randomize.
            value: Value to assign to the attribute.
        """
        self._distribution_node.get_attribute("inputs:numSamples").set(1)
        self._distribution_node.get_attribute("inputs:lower").set([value])
        self._distribution_node.get_attribute("inputs:upper").set([value])

        self._articulation_view_node.get_attribute("inputs:prims").set("franka")
        self._articulation_view_node.get_attribute("inputs:attribute").set(attribute_name)
        self._articulation_view_node.get_attribute("inputs:indices").set([0])
        self._articulation_view_node.get_attribute("inputs:operation").set("direct")

        self._controller.connect(
            self._distribution_node.get_attribute("outputs:samples"),
            self._articulation_view_node.get_attribute("inputs:values"),
        )
        await self._controller.evaluate(self._graph)
        await omni.kit.app.get_app().next_update_async()

    async def test_randomize_stiffness(self):
        """Test randomization of joint stiffness values in the articulation view."""
        value = [100, 200, 300, 400, 500, 600, 700, 800, 900]
        await self._setup_random_attribute(attribute_name="stiffness", value=value)
        stiffness = np.asarray(self._articulation_view._physics_articulation_view.get_dof_stiffnesses())
        self.assertTrue(np.all(np.isclose(stiffness, value)))

    async def test_randomize_damping(self):
        """Test randomization of joint damping values in the articulation view."""
        value = [100, 200, 300, 400, 500, 600, 700, 800, 900]
        await self._setup_random_attribute(attribute_name="damping", value=value)
        damping = np.asarray(self._articulation_view._physics_articulation_view.get_dof_dampings())
        self.assertTrue(np.all(np.isclose(damping, value)))

    async def test_randomize_joint_friction(self):
        """Test randomization of joint friction coefficients in the articulation view."""
        value = [100, 200, 300, 400, 500, 600, 700, 800, 900]
        await self._setup_random_attribute(attribute_name="joint_friction", value=value)
        dof_friction = np.asarray(self._articulation_view._physics_articulation_view.get_dof_friction_coefficients())
        self.assertTrue(np.all(np.isclose(dof_friction, value)))

    async def test_randomize_position(self):
        """Test randomization of the root position of the articulation."""
        value = [100, 200, 300]
        await self._setup_random_attribute(attribute_name="position", value=value)
        root_position, _ = self._articulation_view.get_world_poses()
        root_position = np.asarray(root_position)
        self.assertTrue(np.all(np.isclose(root_position, value)))

    async def test_randomize_orientation(self):
        """Test randomization of the root orientation of the articulation."""
        value = [0, np.pi, 0]
        await self._setup_random_attribute(attribute_name="orientation", value=value)
        _, root_orientation = self._articulation_view.get_world_poses()
        root_orientation = np.asarray(root_orientation)
        self.assertTrue(np.all(np.isclose(root_orientation, [0, 0, 1, 0], atol=1e-04)))

    async def test_randomize_velocities(self):
        """Test randomization of the articulation velocities."""
        value = [10] * 6
        await self._setup_random_attribute(attribute_name="velocity", value=value)
        lin_vel, ang_vel = self._articulation_view.get_velocities()
        velocities = np.concatenate([np.asarray(lin_vel), np.asarray(ang_vel)], axis=-1)
        self.assertTrue(np.all(np.isclose(value, velocities, atol=1e-04)))

    async def test_randomize_joint_positions(self):
        """Test randomization of joint positions in the articulation."""
        value = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
        await self._setup_random_attribute(attribute_name="joint_positions", value=value)
        dof_positions = np.asarray(self._articulation_view.get_dof_positions())
        self.assertTrue(np.all(np.isclose(dof_positions, value)))

    async def test_randomize_joint_velocities(self):
        """Test randomization of joint velocities for the articulation view."""
        value = [100, 200, 300, 400, 500, 600, 700, 800, 900]
        await self._setup_random_attribute(attribute_name="joint_velocities", value=value)
        dof_velocities = np.asarray(self._articulation_view.get_dof_velocities())
        self.assertTrue(np.all(np.isclose(dof_velocities, value)))

    async def test_randomize_lower_dof_limits(self):
        """Test randomization of lower degree of freedom limits for the articulation view."""
        value = [-10, -20, -30, -40, -50, -60, -70, -80, -90]
        await self._setup_random_attribute(attribute_name="lower_dof_limits", value=value)
        lower_dof_limits = np.asarray(self._articulation_view._physics_articulation_view.get_dof_limits())[..., 0]
        self.assertTrue(np.all(np.isclose(lower_dof_limits, value)))

    async def test_randomize_upper_dof_limits(self):
        """Test randomization of upper degree of freedom limits for the articulation view."""
        value = [100, 200, 300, 400, 500, 600, 700, 800, 900]
        await self._setup_random_attribute(attribute_name="upper_dof_limits", value=value)
        upper_dof_limits = np.asarray(self._articulation_view._physics_articulation_view.get_dof_limits())[..., 1]
        self.assertTrue(np.all(np.isclose(upper_dof_limits, value)))

    async def test_randomize_max_efforts(self):
        """Test randomization of maximum effort values for the articulation view."""
        value = [100, 200, 300, 400, 500, 600, 700, 800, 900]
        await self._setup_random_attribute(attribute_name="max_efforts", value=value)
        dof_max_forces = np.asarray(self._articulation_view._physics_articulation_view.get_dof_max_forces())
        self.assertTrue(np.all(np.isclose(dof_max_forces, value)))

    async def test_randomize_armature(self):
        """Test randomization of joint armature values for the articulation view."""
        value = [100, 200, 300, 400, 500, 600, 700, 800, 900]
        await self._setup_random_attribute(attribute_name="joint_armatures", value=value)
        new_values = np.asarray(self._articulation_view._physics_articulation_view.get_dof_armatures())
        self.assertTrue(np.all(np.isclose(new_values, value)))

    async def test_randomize_max_velocities(self):
        """Test randomization of maximum velocity values for the articulation view."""
        value = [100, 200, 300, 400, 500, 600, 700, 800, 900]
        await self._setup_random_attribute(attribute_name="joint_max_velocities", value=value)
        new_values = np.asarray(self._articulation_view._physics_articulation_view.get_dof_max_velocities())
        self.assertTrue(np.all(np.isclose(new_values, value)))

    async def test_randomize_joint_efforts(self):
        """Test randomization of joint effort values for the articulation view."""
        value = [100, 200, 300, 400, 500, 600, 700, 800, 900]
        await self._setup_random_attribute(attribute_name="joint_efforts", value=value)

    async def test_randomize_masses(self):
        """Test randomization of body mass values for the articulation view (CPU only)."""
        if SimulationManager.get_physics_sim_device() == "cpu":
            physics_view = self._articulation_view._physics_articulation_view
            value = [100] * len(self._articulation_view) * physics_view.max_links
            await self._setup_random_attribute(attribute_name="body_masses", value=value)
            new_value = np.asarray(physics_view.get_masses())
            self.assertTrue(np.all(np.isclose(new_value, value)))

    async def test_randomize_inertias(self):
        """Test randomization of body inertia values for the articulation view (CPU only)."""
        if SimulationManager.get_physics_sim_device() == "cpu":
            physics_view = self._articulation_view._physics_articulation_view
            inertias = [0.1, 0.1, 0.1] * len(self._articulation_view) * physics_view.max_links
            await self._setup_random_attribute(attribute_name="body_inertias", value=inertias)
            new_value = np.asarray(physics_view.get_inertias())
            diagonal = new_value[:, :, [0, 4, 8]]
            self.assertTrue(np.all(np.isclose(diagonal.flatten(), inertias)))

    async def test_randomize_material_properties(self):
        """Test randomization of material properties for the articulation view."""
        physics_view = self._articulation_view._physics_articulation_view
        value = [0.5] * len(self._articulation_view) * physics_view.max_shapes * 3
        await self._setup_random_attribute(attribute_name="material_properties", value=value)
        new_value = np.asarray(physics_view.get_material_properties()).flatten()
        self.assertTrue(np.all(np.isclose(new_value, value)))

    async def test_randomize_contact_offsets(self):
        """Test randomization of contact offsets for the articulation view."""
        physics_view = self._articulation_view._physics_articulation_view
        value = [0.05] * len(self._articulation_view) * physics_view.max_shapes
        await self._setup_random_attribute(attribute_name="contact_offset", value=value)
        new_value = np.asarray(physics_view.get_contact_offsets())
        self.assertTrue(np.all(np.isclose(new_value, value)))

    async def test_randomize_rest_offset(self):
        """Test randomization of rest offsets for the articulation view."""
        physics_view = self._articulation_view._physics_articulation_view
        value = np.asarray(physics_view.get_contact_offsets()) / 2
        await self._setup_random_attribute(attribute_name="rest_offset", value=value)
        new_value = np.asarray(physics_view.get_rest_offsets())
        self.assertTrue(np.all(np.isclose(new_value, value)))
