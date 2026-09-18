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

"""Verify standalone virtual gantry schema registration and authoring."""

import unittest

from pxr import Plug, Usd, UsdGeom


class TestVirtualGantrySchemaStandalone(unittest.TestCase):
    """Verify the virtual gantry schema without an Isaac Sim or Kit runtime."""

    def test_import_registers_schema_plugin(self) -> None:
        """Verify importing the package registers the codeless USD plugin."""
        import isaacsim.robot_setup.virtual_gantry_schema  # noqa: F401

        self.assertIsNotNone(Plug.Registry().GetPluginWithName("VirtualGantrySchema"))

    def test_gantry_prim_is_a_concrete_xformable(self) -> None:
        """The anchor is only usable if the type is concrete and Xform-derived.

        A schema plugin registered after USD has built its prim-type catalog
        gets a TfType but no prim definition, and ``xformOp:translate`` is then
        silently ignored -- the failure this schema extension exists to avoid.
        """
        from isaacsim.robot_setup.virtual_gantry_schema import CreateVirtualGantry

        self.assertTrue(Usd.SchemaRegistry().IsConcrete("IsaacVirtualGantry"))

        stage = Usd.Stage.CreateInMemory()
        prim = CreateVirtualGantry(stage, "/World/VirtualGantry")
        self.assertTrue(prim.IsA(UsdGeom.Xformable))

        UsdGeom.Xformable(prim).AddTranslateOp().Set((0.0, 0.0, 2.0))
        anchor = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
        self.assertAlmostEqual(anchor[2], 2.0)

    def test_attributes_and_relationships_are_authored(self) -> None:
        """Verify the helper authors every gantry property."""
        from isaacsim.robot_setup.virtual_gantry_schema import Attributes, CreateVirtualGantry, Relations

        stage = Usd.Stage.CreateInMemory()
        prim = CreateVirtualGantry(stage, "/World/VirtualGantry")

        for attribute in Attributes:
            self.assertTrue(prim.HasAttribute(attribute.name), attribute.name)
        for relation in Relations:
            self.assertTrue(prim.HasRelationship(relation.name), relation.name)


if __name__ == "__main__":
    unittest.main()
