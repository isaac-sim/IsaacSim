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

"""Tests for the robot assembler assembly and disassembly workflows."""

import inspect
from pathlib import Path

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
from isaacsim.core.experimental.objects import SphereLight
from isaacsim.core.experimental.prims import Articulation
from isaacsim.robot_setup.assembler import RobotAssembler
from isaacsim.robot_setup.assembler.robot_assembler import AssemblyStatus
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will
# make it auto-discoverable by omni.kit.test
class TestRobotAssembler(omni.kit.test.AsyncTestCase):
    """Test the robot assembler assembly, cancellation, and finishing workflows."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test environment with robot assembler and stage."""
        self._physics_fps = 60
        self._physics_dt = 1 / self._physics_fps  # duration of physics frame in seconds

        self._timeline = omni.timeline.get_timeline_interface()

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("isaacsim.robot_setup.assembler")

        self._robot_assembler = RobotAssembler()

        await stage_utils.create_new_stage_async()

        self.stage = omni.usd.get_context().get_stage()
        self.stage.DefinePrim(Sdf.Path("/World"), "Xform")

        await self._prepare_stage()

        await app_utils.update_app_async()

    # After running each test
    async def tearDown(self) -> None:
        """Tear down test environment and reset the assembler."""
        self._timeline.stop()
        self._robot_assembler.reset()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await app_utils.update_app_async()
        await app_utils.update_app_async()

    async def _create_light(self) -> None:
        sphere_light = SphereLight("/World/SphereLight", radii=2.0, positions=[6.5, 0, 12])
        sphere_light.set_intensities(100000)

    def assertListsSame(self, l1: list, l2: list) -> None:  # noqa: N802
        """Assert that two lists contain the same elements regardless of order.

        Args:
            l1: First list to compare.
            l2: Second list to compare.
        """
        for item in l1:
            self.assertTrue(item in l2, f"{l1}, {l2}")

        self.assertTrue(len(l2) == len(l1), f"{l1}, {l2}")

    async def _prepare_stage(self) -> None:
        # Set settings to ensure deterministic behavior
        # Initialize the robot
        # Play the timeline

        self._timeline.stop()
        await self._create_light()
        assets_root_path = await get_assets_root_path_async()
        stage_utils.add_reference_to_stage(
            assets_root_path + "/Isaac/Robots_Multiphysics/UniversalRobots/ur10e/ur10e.usda", "/World/ur10e"
        )
        stage_utils.add_reference_to_stage(
            assets_root_path + "/Isaac/Robots/WonikRobotics/AllegroHand/allegro_hand_instanceable.usd",
            "/World/allegro_hand",
        )

        self._robot_base = "/World/ur10e"
        # ``flange`` is the tool mount frame the extension UI offers for this robot. It is not
        # itself a rigid body, but it is nested under the ``wrist_3_link`` rigid body, which is
        # what the assembler's fixed joint binds to. The previously used ``ee_link`` has no rigid
        # body anywhere in its ancestry, so the joint could not bind and the assembly silently
        # dropped every attachment DOF.
        self._robot_base_mount = "/wrist_3_link/flange"
        self._robot_attach = "/World/allegro_hand"
        self._robot_attach_mount = "/allegro_mount"

        await app_utils.update_app_async()

    def apply_rotation(self, axis: list, angle: float) -> None:
        """Apply a rotation to the attachment robot prim.

        Args:
            axis: Rotation axis vector.
            angle: Rotation angle in degrees.
        """
        prim = self.stage.GetPrimAtPath(self._robot_assembler._attachment_robot_prim)

        xformable = UsdGeom.Xformable(prim)
        old_matrix = xformable.GetLocalTransformation()
        old_rotation = old_matrix.ExtractRotation()
        rotation = Gf.Rotation(axis, angle)
        new_matrix = Gf.Matrix4d().SetRotateOnly(rotation) * old_matrix

        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=prim.GetPath(),
            old_transform_matrix=old_matrix,
            new_transform_matrix=new_matrix,
        )

    async def test_robot_assembler_begin_assembly(self) -> None:
        """Test beginning a robot assembly with rotation adjustments."""
        self._robot_assembler.begin_assembly(
            self.stage,
            self._robot_base,
            self._robot_base + self._robot_base_mount,
            self._robot_attach,
            self._robot_attach + self._robot_attach_mount,
            "Gripper",
            "allegro_hand",
        )

        self.apply_rotation([0, 0, 1], -90)
        self.apply_rotation([0, 1, 0], -90)

        await self._assert_pose()

    async def test_robot_assembler_cancel_assembly(self) -> None:
        """Test canceling a robot assembly restores original pose."""
        self._robot_assembler.begin_assembly(
            self.stage,
            self._robot_base,
            self._robot_base + self._robot_base_mount,
            self._robot_attach,
            self._robot_attach + self._robot_attach_mount,
            "Gripper",
            "allegro_hand",
        )
        self._robot_assembler.cancel_assembly()
        await app_utils.update_app_async(steps=5)
        attachment_mount_pose = omni.usd.get_world_transform_matrix(
            self.stage.GetPrimAtPath(self._robot_attach + self._robot_attach_mount)
        )
        self.assertAlmostEqual(np.linalg.norm(attachment_mount_pose - np.eye(4)), 0.0, 2)

    async def test_robot_assembler_cancel_twice(self) -> None:
        """Test canceling assembly twice in succession."""
        for i in range(2):
            await self.test_robot_assembler_cancel_assembly()

    async def test_robot_assembler_cancel_on_idle_instance_is_noop(self) -> None:
        """Regression: ``cancel_assembly`` on a fresh instance must be a no-op.

        Calling it on a never-started instance must not raise
        ``AttributeError: '_assembly_identifier'``.

        Reproduces the failure mode where calling ``cancel_assembly`` (or any
        flow that reaches it via ``reset()``) before ``begin_assembly`` has
        ever run crashed because the assembly-sublayer state attributes were
        only initialized inside ``begin_assembly``.
        """
        fresh_assembler = RobotAssembler()
        # Must not raise.
        fresh_assembler.cancel_assembly()
        # Allow the async cleanup scheduled by cancel_assembly to settle.
        await app_utils.update_app_async(steps=2)
        # A second cancel on the now-reset instance must also be safe.
        fresh_assembler.cancel_assembly()
        await app_utils.update_app_async(steps=2)
        # Direct reset on a fresh instance must likewise be safe.
        RobotAssembler().reset()

    async def _wait_for_stage_load(self) -> None:
        """Wait until every referenced asset has finished loading."""
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await app_utils.update_app_async()
        await app_utils.update_app_async(steps=2)

    async def test_resolve_joint_body_path(self) -> None:
        """Test that joint body targets resolve to the nearest rigid body at or above them."""
        # ``flange`` is a plain frame nested under the ``wrist_3_link`` rigid body.
        self.assertEqual(
            self._robot_assembler.resolve_joint_body_path(self._robot_base + "/wrist_3_link/flange"),
            self._robot_base + "/wrist_3_link",
        )
        # A rigid body resolves to itself.
        self.assertEqual(
            self._robot_assembler.resolve_joint_body_path(self._robot_base + "/wrist_3_link"),
            self._robot_base + "/wrist_3_link",
        )
        # ``ee_link`` is a top level frame with no rigid body anywhere in its ancestry.
        self.assertEqual(self._robot_assembler.resolve_joint_body_path(self._robot_base + "/ee_link"), "")
        self.assertEqual(self._robot_assembler.resolve_joint_body_path(""), "")

    async def test_robot_assembler_rejects_mount_frame_without_rigid_body(self) -> None:
        """Regression: a mount frame the fixed joint cannot bind to must raise, not assemble.

        ``ee_link`` has no rigid body at or above it. Assembling onto it used to succeed
        silently: the fixed joint was created inert, the attachment robot's Articulation Root
        API was removed anyway, and the resulting articulation exposed only the base robot's
        DOFs with nothing in the log naming the joint or the failed merge.
        """
        with self.assertRaises(ValueError):
            self._robot_assembler.begin_assembly(
                self.stage,
                self._robot_base,
                self._robot_base + "/ee_link",
                self._robot_attach,
                self._robot_attach + self._robot_attach_mount,
                "Gripper",
                "allegro_hand",
            )

        # Rejection must happen before any stage edit, so no assembly is left in flight.
        self.assertEqual(self._robot_assembler._status, AssemblyStatus.IDLE)

        # An empty mount frame is likewise rejected rather than silently auto-created.
        with self.assertRaises(ValueError):
            self._robot_assembler.begin_assembly(
                self.stage,
                self._robot_base,
                self._robot_base + self._robot_base_mount,
                self._robot_attach,
                "",
                "Gripper",
                "allegro_hand",
            )

    async def test_robot_assembler_merged_articulation_contains_attachment_dofs(self) -> None:
        """Regression: the assembled robot's articulation must contain the attachment's DOFs.

        The assembler removes the attachment robot's Articulation Root API and relies on the
        fixed joint it creates to fold the attachment into the base articulation. When that
        joint cannot bind, the merge yields only the base robot's DOFs while every other
        success signal, including the single remaining Articulation Root API, still looks
        correct. The attachment robot is also left welded in place, so pose assertions cannot
        detect the loss; the DOFs of the merged articulation have to be inspected directly.

        The physics view is created only after the assembly is complete. Views registered
        before the assembly stay subscribed to physics events and are re-evaluated against the
        restructured stage on the next play, which crashes the USD layer lookup.
        """
        await self._wait_for_stage_load()

        self._robot_assembler.begin_assembly(
            self.stage,
            self._robot_base,
            self._robot_base + self._robot_base_mount,
            self._robot_attach,
            self._robot_attach + self._robot_attach_mount,
            "Gripper",
            "allegro_hand",
        )
        await app_utils.update_app_async(steps=5)
        self._robot_assembler.assemble()
        await app_utils.update_app_async(steps=5)
        self._robot_assembler.finish_assemble()
        await app_utils.update_app_async(steps=10)

        # Exactly one articulation root must remain, and it must be the base robot's.
        self.assertEqual(
            self._robot_assembler.get_articulation_root_api_path(self._robot_attach),
            "",
            "attachment robot kept its own articulation root, so the robots were not merged",
        )

        self._timeline.play()
        await app_utils.update_app_async(steps=30)

        articulation = Articulation(self._robot_assembler.get_articulation_root_api_path(self._robot_base))
        dof_paths = [path for paths in articulation.dof_paths for path in paths]

        base_dof_paths = [path for path in dof_paths if path.startswith(self._robot_base)]
        attach_dof_paths = [path for path in dof_paths if path.startswith(self._robot_attach)]

        self.assertGreater(len(base_dof_paths), 0, "merged articulation contains no base robot DOFs")
        self.assertGreater(
            len(attach_dof_paths),
            0,
            "Merged articulation contains none of the attachment robot's DOFs. The fixed joint "
            f"did not fold {self._robot_attach} into the base articulation. Merged DOFs: {dof_paths}",
        )
        self.assertEqual(
            len(dof_paths),
            len(base_dof_paths) + len(attach_dof_paths),
            f"merged articulation exposes unexpected DOFs: {dof_paths}",
        )

        self._timeline.stop()
        await app_utils.update_app_async()

    async def _assert_pose(self) -> None:
        robot_base_pose = omni.usd.get_world_transform_matrix(
            self.stage.GetPrimAtPath(self._robot_base + self._robot_base_mount)
        )
        attachment_mount_pose = omni.usd.get_world_transform_matrix(
            self.stage.GetPrimAtPath(self._robot_attach + self._robot_attach_mount)
        )

        dist = np.linalg.norm(robot_base_pose.ExtractTranslation() - attachment_mount_pose.ExtractTranslation())
        self.assertAlmostEqual(float(dist), 0.0, 2)

    async def _assert_assembled(self) -> None:
        self._robot_assembler.assemble()

        attachment_root_joint = self.stage.GetPrimAtPath(self._robot_attach + "/root_joint")
        self.assertFalse(attachment_root_joint.IsActive())

        await self._assert_pose()

        self._timeline.play()
        await app_utils.update_app_async(steps=20)
        await self._assert_pose()

    async def test_articulated_attachment_requires_base_articulation(self) -> None:
        """Reject an articulated attachment when only its own root is present."""
        base_path = "/World/plain_base"
        attach_path = base_path + "/tool"
        base = self.stage.DefinePrim(base_path, "Xform")
        attachment = self.stage.DefinePrim(attach_path, "Xform")
        UsdPhysics.RigidBodyAPI.Apply(base)
        UsdPhysics.RigidBodyAPI.Apply(attachment)
        root_joint = UsdPhysics.FixedJoint.Define(self.stage, attach_path + "/root_joint")
        root_joint.CreateBody1Rel().SetTargets([Sdf.Path(attach_path)])
        UsdPhysics.ArticulationRootAPI.Apply(root_joint.GetPrim())

        # The generic subtree lookup finds the nested attachment root; it is not a base articulation.
        self.assertEqual(self._robot_assembler.get_articulation_root_api_path(base_path), str(root_joint.GetPath()))
        before = self.stage.GetRootLayer().ExportToString()
        with self.assertRaisesRegex(ValueError, "no active articulation root"):
            self._robot_assembler.begin_assembly(
                self.stage, base_path, base_path, attach_path, attach_path, "Gripper", "tool"
            )
        self.assertEqual(self._robot_assembler._status, AssemblyStatus.IDLE)
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), before)
        with self.assertRaisesRegex(ValueError, "no active articulation root"):
            self._robot_assembler._assemble_rigid_bodies(base_path, attach_path, base_path, attach_path)

        self.assertTrue(root_joint.GetPrim().IsActive())
        self.assertTrue(root_joint.GetPrim().HasAPI(UsdPhysics.ArticulationRootAPI))
        self.assertFalse(self.stage.GetPrimAtPath(attach_path + "/AssemblerFixedJoint").IsValid())

        UsdPhysics.ArticulationRootAPI.Apply(base)
        assembled = self._robot_assembler._assemble_rigid_bodies(base_path, attach_path, base_path, attach_path)
        self.assertEqual(assembled.fixed_joint.GetBody0Rel().GetTargets(), [Sdf.Path(base_path)])
        self.assertEqual(assembled.fixed_joint.GetBody1Rel().GetTargets(), [Sdf.Path(attach_path)])
        self.assertFalse(root_joint.GetPrim().IsActive())
        self.assertTrue(base.HasAPI(UsdPhysics.ArticulationRootAPI))

    async def test_ur10e_robotiq_requires_active_physics_variant(self) -> None:
        """Reject inactive physics, then assemble and actuate all five mimic followers."""
        self.assertEqual(Path(inspect.getfile(RobotAssembler)).resolve().parent, Path(__file__).resolve().parent.parent)
        await stage_utils.create_new_stage_async()
        self.stage = omni.usd.get_context().get_stage()
        self.stage.SetDefaultPrim(self.stage.DefinePrim("/World", "Xform"))
        assets_root = await get_assets_root_path_async()
        samples = assets_root + "/Isaac/Samples/Rigging/Manipulator/import_manipulator"
        stage_utils.add_reference_to_stage(samples + "/ur10e/ur/ur.usda", "/ur")
        stage_utils.add_reference_to_stage(samples + "/robotiq_2f_140/robotiq_2f_140.usd", "/ur/robotiq_2f_140")
        await app_utils.update_app_async(steps=5)
        await self._wait_for_stage_load()

        base = self.stage.GetPrimAtPath("/ur")
        root_joint = self.stage.GetPrimAtPath("/ur/robotiq_2f_140/root_joint")
        base_mount = (
            "/ur/Geometry/world/base_link/base_link_inertia/shoulder_link/upper_arm_link/"
            "forearm_link/wrist_1_link/wrist_2_link/wrist_3_link"
        )
        attach_mount = "/ur/robotiq_2f_140/robotiq_arg2f_base_link"
        variants = base.GetVariantSet("Physics")
        # An explicit empty selection also blocks any selection authored in the referenced asset.
        variants.BlockVariantSelection()
        before = self.stage.GetRootLayer().ExportToString()
        with self.assertRaisesRegex(ValueError, "no active articulation root"):
            self._robot_assembler.begin_assembly(
                self.stage, "/ur", base_mount, "/ur/robotiq_2f_140", attach_mount, "Gripper", "robotiq"
            )
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), before)
        self.assertTrue(root_joint.IsActive())

        self.assertTrue(variants.SetVariantSelection("physx"))
        self._robot_assembler.begin_assembly(
            self.stage, "/ur", base_mount, "/ur/robotiq_2f_140", attach_mount, "Gripper", "robotiq"
        )
        self.apply_rotation(Gf.Vec3d(0, 0, 1), 90)
        # Changing Physics while adjusting the assembly must also fail before removing the gripper root.
        with Usd.EditContext(self.stage, self.stage.GetRootLayer()):
            variants.BlockVariantSelection()
        with self.assertRaisesRegex(ValueError, "no active articulation root"):
            self._robot_assembler.assemble()
        self.assertTrue(root_joint.IsActive())
        with Usd.EditContext(self.stage, self.stage.GetRootLayer()):
            variants.SetVariantSelection("physx")
        self._robot_assembler.assemble()
        self.assertFalse(root_joint.IsActive())
        roots = [str(p.GetPath()) for p in Usd.PrimRange(base) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        self.assertEqual(roots, ["/ur/Geometry/world"])

        # Give the input finger drive deterministic gains, without modifying the referenced asset.
        drive = UsdPhysics.DriveAPI.Get(self.stage.GetPrimAtPath("/ur/robotiq_2f_140/joints/finger_joint"), "angular")
        self.assertTrue(drive)
        drive.CreateStiffnessAttr().Set(1000.0)
        drive.CreateDampingAttr().Set(100.0)
        drive.CreateTargetPositionAttr().Set(0.0)
        app_utils.play()
        await app_utils.update_app_async(steps=30)
        articulation = Articulation("/ur/Geometry/world")
        names = articulation.dof_names
        followers = [
            "left_inner_finger_joint",
            "left_inner_knuckle_joint",
            "right_inner_knuckle_joint",
            "right_outer_knuckle_joint",
            "right_inner_finger_joint",
        ]
        expected = followers + [
            "finger_joint",
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.assertEqual(set(names), set(expected))
        start = articulation.get_dof_positions().numpy().copy()
        articulation.set_dof_position_targets([0.3], dof_indices=[names.index("finger_joint")])
        await app_utils.update_app_async(steps=120)
        end = articulation.get_dof_positions().numpy()
        self.assertTrue(np.isfinite(end).all())
        self.assertAlmostEqual(float(end[0, names.index("finger_joint")]), 0.3, delta=0.03)
        for name in followers:
            index = names.index(name)
            self.assertGreater(abs(float(end[0, index] - start[0, index])), 0.03, name)
        base_pose = omni.usd.get_world_transform_matrix(self.stage.GetPrimAtPath(base_mount))
        attach_pose = omni.usd.get_world_transform_matrix(self.stage.GetPrimAtPath(attach_mount))
        distance = (base_pose.ExtractTranslation() - attach_pose.ExtractTranslation()).GetLength()
        self.assertLess(distance, 0.01)
        carb.log_info(
            f"UR10e/Robotiq simulation: {len(names)} DOFs, finger={end[0, names.index('finger_joint')]:.4f}, "
            f"all five mimic followers moved, mount separation={distance:.6f} m"
        )
        app_utils.stop()
        await app_utils.update_app_async()
        self._robot_assembler.cancel_assembly()
        await app_utils.update_app_async(steps=5)

    async def test_robot_assembler_assemble(self) -> None:
        """Test assembling robots and verifying pose during simulation."""
        await self.test_robot_assembler_begin_assembly()
        await self._assert_assembled()
        self._timeline.stop()
        await app_utils.update_app_async()

    async def test_robot_assembler_assemble_twice(self) -> None:
        """Test assembling robots twice with a cancel in between."""
        await self.test_robot_assembler_assemble()
        self._robot_assembler.cancel_assembly()
        await app_utils.update_app_async(steps=5)
        await self.test_robot_assembler_assemble()

    async def test_robot_assembler_finish_assembly(self) -> None:
        """Test finishing assembly and verifying pose persists after timeline stop."""
        await self.test_robot_assembler_assemble()
        await self._robot_assembler.finish_assemble()
        await app_utils.update_app_async(steps=10)
        attachment_root_joint = self.stage.GetPrimAtPath(self._robot_attach + "/root_joint")
        self.assertFalse(attachment_root_joint.IsActive())

        await self._assert_pose()

        self._timeline.play()
        await app_utils.update_app_async(steps=20)
        await self._assert_pose()

        self._timeline.stop()
        await app_utils.update_app_async()
        await self._assert_pose()
