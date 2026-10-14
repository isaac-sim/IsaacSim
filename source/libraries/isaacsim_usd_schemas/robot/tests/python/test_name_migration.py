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

"""Regression tests for joint name migration and edit-layer ownership."""

from __future__ import annotations

import unittest

from pxr import Sdf, Usd
from usd.schema.isaac import robot_schema
from usd.schema.isaac.robot_schema.utils import UpdateDeprecatedSchemas


class TestJointNameMigration(unittest.TestCase):
    """Canonical authored names take precedence and only owned legacy opinions are removed."""

    def test_values_and_idempotency(self) -> None:
        """Legacy-only, canonical-only and dual values preserve the documented name."""
        for legacy, canonical, expected in (
            ("legacy", None, "legacy"),
            (None, "canonical", "canonical"),
            ("same", "same", "same"),
            ("legacy", "canonical", "canonical"),
            ("legacy", "", ""),
            ("", None, ""),
        ):
            with self.subTest(legacy=legacy, canonical=canonical):
                stage = Usd.Stage.CreateInMemory()
                root = stage.DefinePrim("/Robot", "Xform")
                joint = stage.DefinePrim("/Robot/Joint", "PhysicsRevoluteJoint")
                joint.ApplyAPI(robot_schema.Classes.JOINT_API.value)
                if legacy is not None:
                    joint.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String).Set(legacy)
                if canonical is not None:
                    joint.GetAttribute("isaac:NameOverride").Set(canonical)
                UpdateDeprecatedSchemas(root)
                self.assertFalse(joint.HasAttribute("isaac:nameOverride"))
                self.assertEqual(joint.GetAttribute("isaac:NameOverride").Get(), expected)
                snapshot = stage.GetRootLayer().ExportToString()
                UpdateDeprecatedSchemas(root)
                self.assertEqual(stage.GetRootLayer().ExportToString(), snapshot)

    def test_link_names_are_preserved(self) -> None:
        """A prim carrying the link API owns a valid lowercase name attribute."""
        stage = Usd.Stage.CreateInMemory()
        prim = stage.DefinePrim("/Link", "Xform")
        prim.ApplyAPI(robot_schema.Classes.LINK_API.value)
        prim.ApplyAPI(robot_schema.Classes.JOINT_API.value)
        prim.GetAttribute("isaac:nameOverride").Set("link_name")
        before = stage.GetRootLayer().ExportToString()
        UpdateDeprecatedSchemas(prim)
        self.assertEqual(stage.GetRootLayer().ExportToString(), before)

    def test_reference_opinions_are_preserved(self) -> None:
        """Root-layer migration never edits referenced layers, even for a local override."""
        source = Usd.Stage.CreateInMemory()
        joint = source.DefinePrim("/Joint", "PhysicsRevoluteJoint")
        source.SetDefaultPrim(joint)
        joint.ApplyAPI(robot_schema.Classes.JOINT_API.value)
        joint.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String).Set("legacy")
        stage = Usd.Stage.CreateInMemory()
        target = stage.DefinePrim("/Referenced")
        target.GetReferences().AddReference(source.GetRootLayer().identifier)
        snapshot = source.GetRootLayer().ExportToString()
        UpdateDeprecatedSchemas(target)
        self.assertEqual(source.GetRootLayer().ExportToString(), snapshot)
        self.assertFalse(stage.GetRootLayer().GetAttributeAtPath("/Referenced.isaac:NameOverride"))
        target.GetAttribute("isaac:nameOverride").Set("local")
        UpdateDeprecatedSchemas(target)
        self.assertEqual(target.GetAttribute("isaac:NameOverride").Get(), "local")
        self.assertFalse(stage.GetRootLayer().GetAttributeAtPath("/Referenced.isaac:nameOverride"))
        self.assertEqual(source.GetRootLayer().ExportToString(), snapshot)
        # The weaker legacy attribute remains composed until its owning asset is migrated.
        self.assertEqual(target.GetAttribute("isaac:nameOverride").Get(), "legacy")

    def test_local_declaration_does_not_capture_reference(self) -> None:
        """A value-less local declaration must not freeze a referenced name."""
        source = Usd.Stage.CreateInMemory()
        joint = source.DefinePrim("/Joint", "PhysicsRevoluteJoint")
        joint.ApplyAPI(robot_schema.Classes.JOINT_API.value)
        legacy = joint.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String)
        legacy.Set("referenced")
        stage = Usd.Stage.CreateInMemory()
        target = stage.DefinePrim("/Target")
        target.GetReferences().AddReference(source.GetRootLayer().identifier, "/Joint")
        target.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String)
        UpdateDeprecatedSchemas(target)
        self.assertFalse(target.GetAttribute("isaac:NameOverride").HasAuthoredValueOpinion())
        legacy.Set("updated")
        self.assertEqual(target.GetAttribute("isaac:nameOverride").Get(), "updated")

    def test_migration_reads_edit_target_value(self) -> None:
        """A stronger opinion must not be copied into a weaker edit layer."""
        stage = Usd.Stage.CreateInMemory()
        joint = stage.DefinePrim("/Joint", "PhysicsRevoluteJoint")
        joint.ApplyAPI(robot_schema.Classes.JOINT_API.value)
        joint.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String).Set("strong")
        layer = Sdf.Layer.CreateAnonymous()
        stage.GetRootLayer().subLayerPaths.append(layer.identifier)
        with Usd.EditContext(stage, layer):
            joint.GetAttribute("isaac:nameOverride").Set("owned")
            UpdateDeprecatedSchemas(joint)
        self.assertEqual(layer.GetAttributeAtPath("/Joint.isaac:NameOverride").default, "owned")
        self.assertEqual(joint.GetAttribute("isaac:nameOverride").Get(), "strong")

    def test_blocked_and_time_sampled_names(self) -> None:
        """Migrate owned blocks and retain animated legacy names without losing samples."""
        stage = Usd.Stage.CreateInMemory()
        joint = stage.DefinePrim("/Joint", "PhysicsRevoluteJoint")
        joint.ApplyAPI(robot_schema.Classes.JOINT_API.value)
        legacy = joint.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String)
        legacy.Block()
        UpdateDeprecatedSchemas(joint)
        self.assertIsInstance(
            stage.GetRootLayer().GetAttributeAtPath("/Joint.isaac:NameOverride").default, Sdf.ValueBlock
        )
        joint.RemoveProperty("isaac:NameOverride")
        legacy = joint.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String)
        legacy.Set("animated", 1)
        before = stage.GetRootLayer().ExportToString()
        UpdateDeprecatedSchemas(joint)
        self.assertEqual(stage.GetRootLayer().ExportToString(), before)
