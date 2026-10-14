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

"""Durable payload transaction regressions."""

import asyncio
import hashlib
import itertools
import tempfile
from pathlib import Path
from unittest.mock import patch

import omni.kit.test
import omni.usd
from isaacsim.robot_setup.assembler.robot_assembler import AssemblyStatus, RobotAssembler
from isaacsim.robot_setup.assembler.session_layer_util import (
    start_assembly_session_sublayer,
    stop_assembly_session_sublayer,
)
from pxr import Sdf, Usd, UsdPhysics


class TestAssemblySession(omni.kit.test.AsyncTestCase):
    """Exercise assemblysession regressions."""

    async def test_failed_finish_preserves_queued_root_edit(self) -> None:
        """Keep unrelated queued edits outside the assembly rollback window."""
        with tempfile.TemporaryDirectory() as directory:
            filename = str(Path(directory) / "robot.usda")
            stage = Usd.Stage.CreateNew(filename)
            stage.SetDefaultPrim(stage.DefinePrim("/Robot", "Xform"))
            # The mount frames carry the Rigid Body API so the assembler can bind its fixed
            # joint to them. Without it the assembly is rejected before the save transaction
            # under test is ever reached.
            UsdPhysics.RigidBodyAPI.Apply(stage.DefinePrim("/Robot/Mount", "Xform"))
            stage.DefinePrim("/Robot/Attachment", "Xform")
            UsdPhysics.RigidBodyAPI.Apply(stage.DefinePrim("/Robot/Attachment/Mount", "Xform"))
            stage.GetRootLayer().Save()
            await omni.usd.get_context().open_stage_async(filename)
            stage = omni.usd.get_context().get_stage()
            assembler = RobotAssembler()
            queued_edit = None
            task = None
            try:
                assembler.begin_assembly(
                    stage, "/Robot", "/Robot/Mount", "/Robot/Attachment", "/Robot/Attachment/Mount", "Tools", "New"
                )

                def edit_unrelated_prim() -> None:
                    with Usd.EditContext(stage, stage.GetRootLayer()):
                        stage.DefinePrim("/UnrelatedUserEdit", "Xform")

                with patch(
                    "isaacsim.robot_setup.assembler.robot_assembler.save_assembly_session_sublayer",
                    side_effect=RuntimeError("disk full"),
                ):
                    task = assembler.finish_assemble()
                    queued_edit = asyncio.get_running_loop().call_soon(edit_unrelated_prim)
                    with self.assertRaisesRegex(RuntimeError, "disk full"):
                        await task
                self.assertTrue(stage.GetPrimAtPath("/UnrelatedUserEdit"))
                self.assertTrue(stage.GetRootLayer().GetPrimAtPath("/Robot/Attachment"))
                self.assertEqual(assembler._status, AssemblyStatus.ASSEMBLING)
            finally:
                if queued_edit is not None:
                    queued_edit.cancel()
                assembler.reset()
                await omni.kit.app.get_app().next_update_async()
                await omni.usd.get_context().new_stage_async()
                assembler = None
                task = None
                stage = None

    def test_saved_payload_cancel_and_replace(self) -> None:
        """Verify saved payload cancel and replace."""
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "saved.usda"
            payload = Usd.Stage.CreateNew(str(target))
            payload.SetDefaultPrim(payload.DefinePrim("/Robot", "Xform"))
            payload.DefinePrim("/Robot/Saved", "Xform")
            payload.GetRootLayer().Save()
            digest = hashlib.sha256(target.read_bytes()).hexdigest()
            stage = Usd.Stage.CreateInMemory()
            stage.DefinePrim("/Robot", "Xform").GetReferences().AddReference(str(target))
            for save in (False, True):
                working = start_assembly_session_sublayer(stage, str(target))
                stage.SetEditTarget(working)
                stage.DefinePrim("/Robot/New", "Xform")
                self.assertEqual(hashlib.sha256(target.read_bytes()).hexdigest(), digest)
                stop_assembly_session_sublayer(stage, str(target), save)
                self.assertTrue(stage.GetPrimAtPath("/Robot/Saved"))
                self.assertEqual(bool(stage.GetPrimAtPath("/Robot/New")), save)
                self.assertFalse(list(Path(directory).glob("*.assembly-*")))
                # Parse the bytes on disk into a new layer, bypassing USD's
                # identifier cache and Kit's process-local Python setup.
                disk_layer = Sdf.Layer.CreateAnonymous("reopened.usda")
                self.assertTrue(disk_layer.ImportFromString(target.read_text()))
                disk_stage = Usd.Stage.Open(disk_layer)
                self.assertTrue(disk_stage.GetPrimAtPath("/Robot/Saved"))
                self.assertEqual(bool(disk_stage.GetPrimAtPath("/Robot/New")), save)
                if not save:
                    self.assertEqual(hashlib.sha256(target.read_bytes()).hexdigest(), digest)
            self.assertNotEqual(hashlib.sha256(target.read_bytes()).hexdigest(), digest)
            # Import into an independent layer rather than observing cached state.
            fresh = Usd.Stage.Open(str(target))
            fresh.GetRootLayer().Reload(force=True)
            self.assertTrue(fresh.GetPrimAtPath("/Robot/Saved"))
            self.assertTrue(fresh.GetPrimAtPath("/Robot/New"))

    def test_first_cancel_and_missing_session(self) -> None:
        """Verify first cancel and missing session."""
        with tempfile.TemporaryDirectory() as directory:
            target = str(Path(directory) / "new.usda")
            stage = Usd.Stage.CreateInMemory()
            stop_assembly_session_sublayer(stage, target)
            layer = start_assembly_session_sublayer(stage, target)
            stage.SetEditTarget(layer)
            stage.DefinePrim("/Unsaved")
            stop_assembly_session_sublayer(stage, target)
            stop_assembly_session_sublayer(stage, target)
            self.assertFalse(Path(target).exists())
            self.assertEqual(list(Path(directory).iterdir()), [])
            self.assertEqual(stage.GetEditTarget().GetLayer(), stage.GetRootLayer())

    def test_duplicate_begin_preserves_active_session(self) -> None:
        """Verify duplicate begin preserves active session."""
        with tempfile.TemporaryDirectory() as directory:
            target = str(Path(directory) / "new.usda")
            stage = Usd.Stage.CreateInMemory()
            layer = start_assembly_session_sublayer(stage, target)
            with self.assertRaisesRegex(RuntimeError, "already active"):
                start_assembly_session_sublayer(stage, target)
            self.assertIn(layer.identifier, stage.GetSessionLayer().subLayerPaths)
            stop_assembly_session_sublayer(stage, target)

    async def test_variant_override_and_failed_finish_can_retry_or_cancel(self) -> None:
        """Hide the previous assembly transiently and retain a failed save for cancellation."""
        for retry, prior_selection in itertools.product((False, True), (None, "Old", "None", "")):
            with tempfile.TemporaryDirectory() as directory:
                filename = str(Path(directory) / "robot.usda")
                stage = Usd.Stage.CreateNew(filename)
                robot = stage.DefinePrim("/Robot", "Xform")
                stage.SetDefaultPrim(robot)
                # See the note in ``test_failed_finish_preserves_queued_root_edit``: the mount
                # frames need the Rigid Body API for the assembly to be accepted.
                UsdPhysics.RigidBodyAPI.Apply(stage.DefinePrim("/Robot/Mount", "Xform"))
                stage.DefinePrim("/Robot/Attachment", "Xform")
                UsdPhysics.RigidBodyAPI.Apply(stage.DefinePrim("/Robot/Attachment/Mount", "Xform"))
                variants = robot.GetVariantSets().AddVariantSet("Tools")
                variants.AddVariant("None")
                variants.AddVariant("Old")
                variants.SetVariantSelection("Old")
                with variants.GetVariantEditContext():
                    stage.DefinePrim("/Robot/PreviousAssembly", "Xform")
                stage.GetRootLayer().Save()
                await omni.usd.get_context().open_stage_async(filename)
                stage = omni.usd.get_context().get_stage()
                session_variant = None
                if prior_selection is not None:
                    with Usd.EditContext(stage, stage.GetSessionLayer()):
                        session_variant = stage.GetPrimAtPath("/Robot").GetVariantSet("Tools")
                        if prior_selection == "":
                            session_variant.BlockVariantSelection()
                        else:
                            session_variant.SetVariantSelection(prior_selection)
                assembler = RobotAssembler()
                payload = None
                working = None
                task = None
                root = None
                try:
                    assembler.begin_assembly(
                        stage, "/Robot", "/Robot/Mount", "/Robot/Attachment", "/Robot/Attachment/Mount", "Tools", "New"
                    )
                    self.assertFalse(stage.GetPrimAtPath("/Robot/PreviousAssembly"))
                    self.assertEqual(stage.GetRootLayer().GetPrimAtPath("/Robot").variantSelections["Tools"], "Old")
                    root_before = stage.GetRootLayer().ExportToString()
                    working = assembler._assembly_layer
                    with patch(
                        "isaacsim.robot_setup.assembler.robot_assembler.save_assembly_session_sublayer",
                        side_effect=RuntimeError("disk full"),
                    ):
                        task = assembler.finish_assemble()
                        self.assertIs(task, assembler.finish_assemble())
                        self.assertEqual(assembler._status, AssemblyStatus.ASSEMBLING)
                        with self.assertRaisesRegex(RuntimeError, "disk full"):
                            await task
                    self.assertEqual(assembler._status, AssemblyStatus.ASSEMBLING)
                    self.assertEqual(stage.GetRootLayer().ExportToString(), root_before)
                    self.assertIn(working.identifier, stage.GetSessionLayer().subLayerPaths)
                    self.assertEqual(stage.GetEditTarget().GetLayer(), working)
                    self.assertFalse(stage.GetPrimAtPath("/Robot/PreviousAssembly"))
                    if retry:
                        await assembler.finish_assemble()
                        self.assertEqual(assembler._status, AssemblyStatus.IDLE)
                        self.assertEqual(
                            stage.GetPrimAtPath("/Robot").GetVariantSet("Tools").GetVariantSelection(),
                            "New" if prior_selection is None else prior_selection,
                        )
                        root = stage.GetRootLayer()
                        self.assertEqual(
                            list(root.GetPrimAtPath("/Robot{Tools=New}").payloadList.prependedItems),
                            [Sdf.Payload("payloads/Tools/New.usd")],
                        )
                        for path in ("/Robot", "/Robot{Tools=Old}", "/Robot{Tools=None}"):
                            self.assertFalse(root.GetPrimAtPath(path).payloadList.GetAppliedItems())
                        payload = Usd.Stage.Open(str(Path(directory) / "payloads/Tools/New.usd"))
                        self.assertEqual(payload.GetDefaultPrim().GetPath(), Sdf.Path("/Robot"))
                        self.assertTrue(payload.GetPrimAtPath("/Robot/Attachment"))
                    else:
                        assembler.cancel_assembly()
                        self.assertEqual(
                            bool(stage.GetPrimAtPath("/Robot/PreviousAssembly")), prior_selection in (None, "Old")
                        )
                    self.assertEqual(
                        stage.GetSessionLayer().GetPrimAtPath("/Robot").variantSelections.get("Tools"),
                        prior_selection,
                    )
                    self.assertFalse(list(Path(directory).rglob("*.assembly-*")))
                    await omni.kit.app.get_app().next_update_async()
                    await omni.kit.app.get_app().next_update_async()
                finally:
                    if assembler._status == AssemblyStatus.ASSEMBLING:
                        assembler.cancel_assembly()
                        await omni.kit.app.get_app().next_update_async()
                    await omni.usd.get_context().new_stage_async()
                    # Binary USD payloads stay memory-mapped while a stage or
                    # layer owner survives. Windows cannot unlink those files.
                    # Drop both reopened and composed stages before TempDir exits;
                    # keep cleanup errors enabled so CI catches ownership leaks.
                    assembler.reset()
                    assembler = None
                    task = None
                    payload = None
                    root = None
                    working = None
                    variants = None
                    session_variant = None
                    robot = None
                    stage = None
                    await omni.kit.app.get_app().next_update_async()
                    self.assertIsNone(Sdf.Layer.Find(str(Path(directory) / "payloads/Tools/New.usd")))
