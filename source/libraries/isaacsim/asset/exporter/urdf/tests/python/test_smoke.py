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

"""Standalone smoke tests for the URDF exporter module."""

from __future__ import annotations

import sys
import unittest

from usd.schema.isaac import robot_schema


class TestSmoke(unittest.TestCase):
    """Import and namespace validation."""

    def test_import_public_api(self) -> None:
        """Verify URDF exporter class is importable."""
        from isaacsim.asset.exporter.urdf import UsdToUrdfConverter

        self.assertTrue(callable(UsdToUrdfConverter))

    def test_import_joint_reader(self) -> None:
        """Verify joint_reader functions are importable."""
        from isaacsim.asset.exporter.urdf.converter.joint_reader import (
            _read_armature,
            _read_physx_friction,
        )

        self.assertTrue(callable(_read_armature))
        self.assertTrue(callable(_read_physx_friction))

    def test_no_omni_modules(self) -> None:
        """No omni.* modules should be loaded after import."""
        from isaacsim.asset.exporter.urdf import UsdToUrdfConverter  # noqa: F401

        omni_mods = [m for m in sys.modules if m.startswith("omni.")]
        self.assertEqual(omni_mods, [], f"Unexpected omni modules: {omni_mods}")


class TestNameOverrides(unittest.TestCase):
    """Exporter naming preserves canonical precedence and legacy compatibility."""

    def test_joint_names(self) -> None:
        """Check fallback, empty and blocked canonical values, and user warnings."""
        import warnings

        from isaacsim.asset.exporter.urdf.converter.transform_utils import get_prim_name
        from pxr import Sdf, Usd

        for canonical, expected, warns in (
            (None, "legacy", True),
            ("new", "new", False),
            ("", "Joint", False),
            (Sdf.ValueBlock(), "Joint", False),
        ):
            with self.subTest(canonical=canonical):
                stage = Usd.Stage.CreateInMemory()
                prim = stage.DefinePrim("/Joint", "PhysicsRevoluteJoint")
                prim.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String).Set("legacy")
                if canonical is not None:
                    prim.CreateAttribute("isaac:NameOverride", Sdf.ValueTypeNames.String).Set(canonical)
                with warnings.catch_warnings(record=True) as caught:
                    warnings.simplefilter("always")
                    self.assertEqual(get_prim_name(prim), expected)
                self.assertEqual(len(caught), int(warns))
                if warns:
                    self.assertIn("legacy support will be removed in a future version", str(caught[0].message))

    def test_link_names_and_display_name(self) -> None:
        """Link metadata is not deprecated and displayName keeps export precedence."""
        import warnings

        from isaacsim.asset.exporter.urdf.converter.transform_utils import get_prim_name
        from pxr import Sdf, Usd

        stage = Usd.Stage.CreateInMemory()
        prim = stage.DefinePrim("/Link", "Xform")
        prim.ApplyAPI(robot_schema.Classes.LINK_API.value)
        prim.ApplyAPI(robot_schema.Classes.JOINT_API.value)
        prim.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String).Set("link")
        prim.CreateAttribute("isaac:NameOverride", Sdf.ValueTypeNames.String).Set("joint")
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            self.assertEqual(get_prim_name(prim), "link")
        self.assertEqual(caught, [])
        prim.SetMetadata("displayName", "display")
        self.assertEqual(get_prim_name(prim), "display")


if __name__ == "__main__":
    unittest.main()
