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

"""Test for scene registry."""

from isaacsim.core.api.scenes import SceneRegistry

from .common import CoreTestCase


class TestSceneRegistry(CoreTestCase):
    """Test scene registry behavior."""

    def _add_method_names(self) -> tuple[str, ...]:
        return (
            "add_rigid_object",
            "add_rigid_prim_view",
            "add_rigid_contact_view",
            "add_articulated_system",
            "add_articulated_view",
            "add_geometry_object",
            "add_geometry_prim_view",
            "add_robot",
            "add_robot_view",
            "add_xform_view",
            "add_deformable",
            "add_deformable_view",
            "add_deformable_material",
            "add_deformable_material_view",
            "add_cloth",
            "add_cloth_view",
            "add_particle_system",
            "add_particle_system_view",
            "add_particle_material",
            "add_particle_material_view",
            "add_xform",
            "add_sensor",
        )

    async def test_add_methods_reject_duplicate_names(self) -> None:
        """Test every direct add method rejects an existing name."""
        for add_method_name in self._add_method_names():
            with self.subTest(add_method=add_method_name):
                registry = SceneRegistry()
                original = object()
                replacement = object()
                add_method = getattr(registry, add_method_name)

                add_method("duplicate_name", original)

                with self.assertRaises(ValueError) as error:
                    add_method("duplicate_name", replacement)

                self.assertEqual(
                    str(error.exception),
                    "Cannot add the object duplicate_name to the scene since its name is not unique",
                )
                self.assertIs(registry.get_object("duplicate_name"), original)

    async def test_add_methods_reject_names_registered_in_other_categories(self) -> None:
        """Test direct add methods reject names that exist in any registry category."""
        for add_method_name in self._add_method_names()[1:]:
            with self.subTest(add_method=add_method_name):
                registry = SceneRegistry()
                original = object()
                replacement = object()
                registry.add_rigid_object("shared_name", original)

                with self.assertRaises(ValueError):
                    getattr(registry, add_method_name)("shared_name", replacement)

                self.assertIs(registry.get_object("shared_name"), original)
