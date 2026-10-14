# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Physics integration tests for velocity locomotion."""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import patch

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.backend as backend_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
import omni.timeline
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim, XformPrim
from isaacsim.replicator.teleop import LocomotionController, LocomotionDriveMode
from pxr import Gf, PhysxSchema, UsdPhysics

_CUBE_PATH = "/World/VelocityBase"


class TestLocomotionVelocity(omni.kit.test.AsyncTestCase):
    """Drive a dynamic rigid body through the live PhysX tensor backend."""

    async def setUp(self) -> None:
        """Create a zero-gravity stage with one dynamic cube."""
        await stage_utils.create_new_stage_async()
        stage_utils.define_prim("/World", "Xform")
        scene_prim = stage_utils.define_prim("/World/PhysicsScene", "PhysicsScene")
        scene = UsdPhysics.Scene(scene_prim)
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr(0.0)
        Cube(_CUBE_PATH, sizes=0.2, positions=(0.0, 0.0, 1.0))
        GeomPrim(_CUBE_PATH, apply_collision_apis=True)
        RigidPrim(_CUBE_PATH, masses=[1.0])
        await app_utils.update_app_async()
        self._controller = LocomotionController()

    async def tearDown(self) -> None:
        """Stop physics and release the stage."""
        if self._controller.is_running:
            self._controller.disable()
        omni.timeline.get_timeline_interface().stop()
        await app_utils.update_app_async()
        stage_utils.close_stage()
        await app_utils.update_app_async()

    async def test_velocity_mode_moves_dynamic_rigid_body(self) -> None:
        """Command forward velocity and verify PhysX integrates base motion."""
        _rigid_prim, moved_x = await self._move_velocity_base()

        self.assertGreater(moved_x, 0.05, "Velocity locomotion should move the dynamic base along local +X")

    async def test_velocity_disable_stops_without_restoring_pose(self) -> None:
        """Disabling velocity mode stops at the physics-integrated pose."""
        rigid_prim, moved_x = await self._move_velocity_base()

        self._controller.disable()
        linear_velocities, angular_velocities = rigid_prim.get_velocities()
        stopped_position = self._position_x(rigid_prim)

        self.assertGreater(stopped_position, 0.04, "Velocity disable must not restore the enable-time pose")
        self.assertAlmostEqual(stopped_position, moved_x, delta=0.02)
        self.assertLess(float(np.linalg.norm(linear_velocities.numpy())), 1e-4)
        self.assertLess(float(np.linalg.norm(angular_velocities.numpy())), 1e-4)

    async def test_velocity_writes_ignore_transform_backend(self) -> None:
        """Fabric pose selection must not leak into Tensor/USD-only velocity writes."""
        controller = self._controller
        controller.set_prim_path(_CUBE_PATH)
        controller.set_drive_mode(LocomotionDriveMode.VELOCITY)
        ok, message = controller.enable()
        self.assertTrue(ok, message)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(5):
            await app_utils.update_app_async()

        rigid_prim = controller._ensure_rigid_prim()  # noqa: SLF001
        self.assertIsNotNone(rigid_prim)
        self.assertTrue(rigid_prim.is_physics_tensor_entity_valid())
        world_pose = rigid_prim.get_world_poses()
        get_current_backend = backend_utils.get_current_backend

        def require_supported_backend(supported: list[str], **_kwargs: object) -> str:
            return get_current_backend(supported, raise_on_unsupported=True)

        with (
            patch("isaacsim.replicator.teleop._backend.get_teleop_backend", return_value="fabric"),
            patch.object(RigidPrim, "get_world_poses", return_value=world_pose),
            patch.object(backend_utils, "get_current_backend", side_effect=require_supported_backend),
        ):
            controller._apply_velocity(1.0, 0.0, 0.0, 0.0, False)  # noqa: SLF001
            linear_velocities, _ = rigid_prim.get_velocities()
            self.assertGreater(float(np.linalg.norm(linear_velocities.numpy())), 0.1)
            controller.stop_motion()

        linear_velocities, angular_velocities = rigid_prim.get_velocities()
        self.assertLess(float(np.linalg.norm(linear_velocities.numpy())), 1e-4)
        self.assertLess(float(np.linalg.norm(angular_velocities.numpy())), 1e-4)

    def test_teleport_disable_retains_restore_behavior(self) -> None:
        """Disabling teleport mode continues to restore its enable-time pose."""
        teleport_path = "/World/TeleportBase"
        stage_utils.define_prim(teleport_path, "Xform")
        base = XformPrim(teleport_path)
        controller = self._controller
        controller.set_prim_path(teleport_path)
        controller.set_drive_mode(LocomotionDriveMode.TELEPORT)
        controller.set_linear_step(0.1)
        ok, message = controller.enable()
        self.assertTrue(ok, message)

        left_controller, right_controller = self._forward_inputs()
        controller.update(left_controller, right_controller)
        self.assertGreater(self._position_x(base), 0.05)

        controller.disable()
        self.assertAlmostEqual(self._position_x(base), 0.0, delta=1e-5)

    async def _move_velocity_base(self) -> tuple[RigidPrim, float]:
        """Enable velocity mode and return its live handle after measurable motion."""
        controller = self._controller
        controller.set_prim_path(_CUBE_PATH)
        controller.set_drive_mode(LocomotionDriveMode.VELOCITY)
        controller.set_linear_speed(1.0)
        ok, message = controller.enable()
        self.assertTrue(ok, message)
        self.assertEqual(controller.effective_drive_mode, LocomotionDriveMode.VELOCITY)

        left_controller, right_controller = self._forward_inputs()

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(5):
            await app_utils.update_app_async()

        rigid_prim = controller._rigid_prim  # noqa: SLF001
        moved_x = 0.0
        for _ in range(120):
            controller.update(left_controller, right_controller)
            await app_utils.update_app_async()
            rigid_prim = controller._rigid_prim  # noqa: SLF001
            if rigid_prim is None or not rigid_prim.valid:
                continue
            moved_x = self._position_x(rigid_prim)
            if moved_x > 0.05:
                break

        self.assertIsNotNone(rigid_prim)
        return rigid_prim, moved_x

    @staticmethod
    def _forward_inputs() -> tuple[SimpleNamespace, SimpleNamespace]:
        """Create controller snapshots that command local forward motion."""
        inputs = SimpleNamespace(
            thumbstick_x=0.0,
            thumbstick_y=1.0,
            primary_click=False,
            secondary_click=False,
        )
        left_controller = SimpleNamespace(inputs=inputs)
        right_controller = SimpleNamespace(inputs=SimpleNamespace(**vars(inputs)))
        return left_controller, right_controller

    @staticmethod
    def _position_x(prim: RigidPrim | XformPrim) -> float:
        """Read the first prim's world-space X position."""
        positions, _orientations = prim.get_world_poses()
        position_array = positions.numpy() if hasattr(positions, "numpy") else np.asarray(positions)
        return float(position_array.reshape(-1, 3)[0, 0])

    def test_enable_restores_missing_physx_rigid_body_api(self) -> None:
        """Velocity activation restores the optional PhysX schema when absent."""
        prim = stage_utils.get_current_stage().GetPrimAtPath(_CUBE_PATH)
        prim.RemoveAPI(PhysxSchema.PhysxRigidBodyAPI)
        self.assertFalse(prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI))

        self._controller.set_prim_path(_CUBE_PATH)
        self._controller.set_drive_mode(LocomotionDriveMode.VELOCITY)
        ok, message = self._controller.enable()

        self.assertTrue(ok, message)
        self.assertTrue(prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI))

    def test_drive_mode_schema_query(self) -> None:
        """AUTO selects teleport for a plain Xform and velocity for a dynamic body."""
        teleport_path = "/World/TeleportBase"
        stage_utils.define_prim(teleport_path, "Xform")
        self._controller.set_prim_path(teleport_path)
        ok, message = self._controller.validate()
        self.assertTrue(ok, message)
        self.assertEqual(self._controller.effective_drive_mode, LocomotionDriveMode.TELEPORT)

        self._controller.set_prim_path(_CUBE_PATH)
        ok, message = self._controller.validate()
        self.assertTrue(ok, message)
        self.assertEqual(self._controller.effective_drive_mode, LocomotionDriveMode.VELOCITY)
