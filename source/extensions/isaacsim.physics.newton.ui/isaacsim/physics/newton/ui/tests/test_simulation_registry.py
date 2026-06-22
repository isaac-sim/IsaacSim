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

"""Tests for Newton simulator registration and capability management."""

from omni.kit.test.async_unittest import AsyncTestCase
from omni.physics.core import get_physics_interface, get_physics_simulation_interface
from omni.physics.isaacsimready import get_capability_manager, get_variant_switcher


class NewtonRegistryTests(AsyncTestCase):
    """Tests for Newton extension capability and variant registration."""

    def setUp(self) -> None:
        """Set up test fixtures."""
        # Get the singleton instances
        self._capability_manager = get_capability_manager()
        self._variant_switcher = get_variant_switcher()

    def tearDown(self) -> None:
        """Clean up after tests."""

    def test_capability(self) -> None:
        """Test Newton simulation capability for different schema types.

        Verifies that Newton simulation can handle NewtonSceneAPI and MjcActuator schemas but not PhysxSceneAPI.
        """
        simulation_ids = get_physics_interface().get_simulation_ids()
        sim_found = False

        for i, sim_id in enumerate(simulation_ids):
            if get_physics_interface().get_simulation_name(sim_id) == "Newton":
                sim_found = True
                schema_list = ["NewtonSceneAPI", "MjcActuator", "PhysxSceneAPI"]
                result, capabilities = get_physics_simulation_interface().is_capable_of_simulating(
                    simulation_ids[i], schema_list
                )
                self.assertTrue(result)
                self.assertTrue(len(capabilities) == 3)
                self.assertTrue(capabilities[0])
                self.assertTrue(capabilities[1])
                self.assertTrue(not capabilities[2])

        self.assertTrue(sim_found)

    def test_variant_switch_registry(self) -> None:
        """Test variant switcher registration for Newton simulator.

        Verifies that the variant switcher correctly returns 'mujoco' as the variant for the Newton simulator.
        """
        variant_name = self._variant_switcher.get_variant_for_simulator("Newton")
        self.assertTrue(variant_name == "mujoco")

    def test_capablity_register(self) -> None:
        """Test capability manager registration of schema types.

        Verifies that MjcActuator schema type is properly registered with the capability manager.
        """
        registered_types = self._capability_manager.get_registered_schema_type_names()
        self.assertTrue("MjcActuator" in registered_types)
