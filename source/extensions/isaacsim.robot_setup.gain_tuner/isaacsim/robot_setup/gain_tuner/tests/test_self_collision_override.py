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

"""Unit tests for the test-time self-collision override on GainTuner."""

from __future__ import annotations

from types import SimpleNamespace

import omni.kit.test
from isaacsim.robot_setup.gain_tuner.gains_tuner import (
    _NEWTON_ARTICULATION_ROOT_API,
    _NEWTON_SELF_COLLISION_ATTR,
    _SELF_COLLISION_ATTR,
    GainTuner,
)
from pxr import PhysxSchema, Usd, UsdPhysics


class TestSelfCollisionOverride(omni.kit.test.AsyncTestCase):
    """Disabling self-collisions for a test run without editing the user's asset."""

    def _tuner_on_articulation_root(self) -> tuple[GainTuner, Usd.Stage, Usd.Prim]:
        """Bind a GainTuner to an articulation root carrying only UsdPhysics.

        Returns:
            The tuner, the stage that keeps the prim alive, and the root prim.
        """
        stage = Usd.Stage.CreateInMemory()
        prim = stage.DefinePrim("/World/Robot", "Xform")
        UsdPhysics.ArticulationRootAPI.Apply(prim)

        tuner = GainTuner()
        tuner._articulation = SimpleNamespace(prims=[prim])
        return tuner, stage, prim

    async def test_override_applies_to_a_robot_without_the_physx_schema(self) -> None:
        """A robot imported with only UsdPhysics still gets self-collisions off.

        Such a robot carries no ``physxArticulation:enabledSelfCollisions``
        property, and ``GetAttribute`` answers that with an invalid attribute
        rather than raising, so the override has to apply the schema itself.
        """
        tuner, _stage, prim = self._tuner_on_articulation_root()
        self.assertFalse(prim.GetAttribute(_SELF_COLLISION_ATTR), "precondition: property is absent")

        self.assertIsNone(tuner.apply_self_collision_override())

        self.assertTrue(tuner.self_collision_override_active())
        self.assertFalse(prim.GetAttribute(_SELF_COLLISION_ATTR).Get())

    async def test_override_disables_the_newton_property_too(self) -> None:
        """Newton leads its resolver chain with its own property, so it is authored.

        A robot whose asset authors ``newton:selfCollisionEnabled`` would keep
        self-collisions on under Newton if only the PhysX property were written.
        """
        tuner, _stage, prim = self._tuner_on_articulation_root()

        self.assertIsNone(tuner.apply_self_collision_override())

        newton_attr = prim.GetAttribute(_NEWTON_SELF_COLLISION_ATTR)
        self.assertTrue(newton_attr, f"'{_NEWTON_SELF_COLLISION_ATTR}' must be authored")
        self.assertFalse(newton_attr.Get())

    async def test_restore_drops_the_newton_property_too(self) -> None:
        """Both engines' properties go away together with the override layer."""
        tuner, _stage, prim = self._tuner_on_articulation_root()
        tuner.apply_self_collision_override()

        tuner.restore_self_collision_override()

        self.assertFalse(
            prim.GetAttribute(_NEWTON_SELF_COLLISION_ATTR),
            "restore must leave no trace of the Newton override on the robot",
        )

    async def test_override_holds_over_an_asset_that_enables_newton_self_collisions(self) -> None:
        """A session-layer opinion outranks the robot's own authored value."""
        tuner, _stage, prim = self._tuner_on_articulation_root()
        prim.ApplyAPI(_NEWTON_ARTICULATION_ROOT_API)
        prim.GetAttribute(_NEWTON_SELF_COLLISION_ATTR).Set(True)

        self.assertIsNone(tuner.apply_self_collision_override())

        self.assertFalse(prim.GetAttribute(_NEWTON_SELF_COLLISION_ATTR).Get())

        tuner.restore_self_collision_override()
        self.assertTrue(prim.GetAttribute(_NEWTON_SELF_COLLISION_ATTR).Get(), "the asset's own value comes back")

    async def test_override_does_not_author_into_the_root_layer(self) -> None:
        """The opinions land on the session layer, leaving the user's asset untouched.

        A diagnostic test run must not leave the robot carrying a schema or a
        value it did not ship with, nor dirty the layer with edits to save.
        """
        tuner, stage, prim = self._tuner_on_articulation_root()

        tuner.apply_self_collision_override()

        root = stage.GetRootLayer()
        for attr_name in (_SELF_COLLISION_ATTR, _NEWTON_SELF_COLLISION_ATTR):
            self.assertIsNone(
                root.GetAttributeAtPath(f"/World/Robot.{attr_name}"),
                f"'{attr_name}' must not be authored into the user's layer",
            )
        applied = root.GetPrimAtPath("/World/Robot").GetInfo("apiSchemas").GetAddedOrExplicitItems()
        for api in ("PhysxArticulationAPI", "NewtonArticulationRootAPI"):
            self.assertNotIn(api, applied, "the override must not apply an API schema to the user's layer")
        self.assertFalse(prim.GetAttribute(_SELF_COLLISION_ATTR).Get(), "the engine still composes the override")

    async def test_restore_drops_the_override(self) -> None:
        """Restoring returns the composed value to the asset's own."""
        tuner, _stage, prim = self._tuner_on_articulation_root()
        tuner.apply_self_collision_override()

        tuner.restore_self_collision_override()

        self.assertFalse(tuner.self_collision_override_active())
        self.assertFalse(
            prim.GetAttribute(_SELF_COLLISION_ATTR),
            "restore must leave no trace of the override on the robot",
        )

    async def test_restore_keeps_a_robot_that_was_already_disabled_disabled(self) -> None:
        """A robot shipped with self-collisions off is not silently turned back on."""
        tuner, _stage, prim = self._tuner_on_articulation_root()
        PhysxSchema.PhysxArticulationAPI.Apply(prim)
        prim.GetAttribute(_SELF_COLLISION_ATTR).Set(False)

        tuner.apply_self_collision_override()
        tuner.restore_self_collision_override()

        self.assertFalse(prim.GetAttribute(_SELF_COLLISION_ATTR).Get())

    async def test_restore_targets_the_prim_the_override_was_applied_to(self) -> None:
        """A robot swap mid-test cannot leak the override onto the new robot.

        The restore targets the prim the override was applied to, not whichever
        articulation happens to be bound when the restore runs.
        """
        tuner, stage, first = self._tuner_on_articulation_root()
        tuner.apply_self_collision_override()

        second = stage.DefinePrim("/World/OtherRobot", "Xform")
        UsdPhysics.ArticulationRootAPI.Apply(second)
        tuner._articulation = SimpleNamespace(prims=[second])

        tuner.restore_self_collision_override()

        self.assertFalse(first.GetAttribute(_SELF_COLLISION_ATTR), "the original robot must be restored")
        self.assertFalse(second.GetAttribute(_SELF_COLLISION_ATTR), "the new robot must not be touched")

    async def test_reset_drops_a_live_override(self) -> None:
        """A stage close or hot reload mid-test does not leave the override armed."""
        tuner, _stage, prim = self._tuner_on_articulation_root()
        tuner.apply_self_collision_override()

        tuner.reset()

        self.assertFalse(tuner.self_collision_override_active())
        self.assertFalse(prim.GetAttribute(_SELF_COLLISION_ATTR))

    async def test_applying_twice_still_restores_the_asset_value(self) -> None:
        """A second apply must not capture the override as the value to restore."""
        tuner, _stage, prim = self._tuner_on_articulation_root()
        PhysxSchema.PhysxArticulationAPI.Apply(prim)
        prim.GetAttribute(_SELF_COLLISION_ATTR).Set(True)

        tuner.apply_self_collision_override()
        tuner.apply_self_collision_override()
        tuner.restore_self_collision_override()

        self.assertTrue(prim.GetAttribute(_SELF_COLLISION_ATTR).Get())

    async def test_a_stronger_session_opinion_is_reported_not_silently_ignored(self) -> None:
        """Authoring is not the same as taking effect, so the result is read back.

        A direct session-layer opinion outranks the override's sublayer. Reporting
        success there would leave the test running with self-collisions on while
        the UI says otherwise.
        """
        tuner, stage, prim = self._tuner_on_articulation_root()
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            PhysxSchema.PhysxArticulationAPI.Apply(prim)
            prim.GetAttribute(_SELF_COLLISION_ATTR).Set(True)

        reason = tuner.apply_self_collision_override()

        self.assertIsNotNone(reason)
        self.assertIn("stronger opinion", reason)
        self.assertFalse(tuner.self_collision_override_active())
        self.assertTrue(prim.GetAttribute(_SELF_COLLISION_ATTR).Get())

    async def test_a_rejected_override_leaves_no_sublayer_behind(self) -> None:
        """A refusal leaves no sublayer attached to the stage."""
        tuner, stage, prim = self._tuner_on_articulation_root()
        before = len(stage.GetSessionLayer().subLayerPaths)
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            PhysxSchema.PhysxArticulationAPI.Apply(prim)
            prim.GetAttribute(_SELF_COLLISION_ATTR).Set(True)

        self.assertIsNotNone(tuner.apply_self_collision_override())

        self.assertEqual(len(stage.GetSessionLayer().subLayerPaths), before)

    async def test_restore_detaches_after_the_robot_is_deleted(self) -> None:
        """Deleting the robot mid-test must not strand the override on the stage."""
        tuner, stage, prim = self._tuner_on_articulation_root()
        self.assertIsNone(tuner.apply_self_collision_override())
        attached = len(stage.GetSessionLayer().subLayerPaths)
        stage.RemovePrim(prim.GetPath())

        tuner.restore_self_collision_override()

        self.assertEqual(len(stage.GetSessionLayer().subLayerPaths), attached - 1)
        self.assertFalse(tuner.self_collision_override_active())

    async def test_restore_without_an_override_is_a_no_op(self) -> None:
        """Cancelling before the override was applied must not touch the asset."""
        tuner, _stage, prim = self._tuner_on_articulation_root()

        tuner.restore_self_collision_override()

        self.assertFalse(prim.GetAttribute(_SELF_COLLISION_ATTR))

    async def test_no_robot_reports_a_reason(self) -> None:
        """With nothing bound the caller gets a reason to surface, not a crash."""
        tuner = GainTuner()
        self.assertIsNotNone(tuner.apply_self_collision_override())
