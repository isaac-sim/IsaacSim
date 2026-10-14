# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Physics-based (sim) validation of HolonomicController on real robots.

Kaya is a 3-wheel OMNI base (roller angle 90); O3dyn is a 4-wheel MECANUM base
(non-90 roller angles).  Each test builds the controller from the robot's
authored wheel geometry, commands a constant body twist, drives the wheels
open-loop, and checks the base moves as commanded.
"""

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.robot_motion.controllers as ctrl
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.kit.test
import omni.usd
import warp as wp
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path_async

KAYA_WHEEL_JOINTS = ["axle_0_joint", "axle_1_joint", "axle_2_joint"]
O3DYN_WHEEL_JOINTS = ["wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"]
DT = 1.0 / 60.0


class TestHolonomicControllerSim(omni.kit.test.AsyncTestCase):
    """End-to-end physics tests: the controller must actually drive a real robot."""

    async def setUp(self) -> None:
        """Create a fresh stage with a ground plane and resolve the assets root."""
        self._assets_root = await get_assets_root_path_async()
        await stage_utils.create_new_stage_async()
        stage_utils.set_stage_up_axis("Z")
        stage_utils.set_stage_units(meters_per_unit=1.0)
        GroundPlane("/World/GroundPlane")

    async def tearDown(self) -> None:
        """Stop the timeline and drain any pending stage loads."""
        app_utils.stop()
        await app_utils.update_app_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await app_utils.update_app_async()

    async def _build_kaya_controller(self):
        """Load the Kaya omni robot and build a controller from its authored geometry.

        Returns:
            ``(articulation, HolonomicController)``. Skips the test if assets are absent.
        """
        if self._assets_root is None:
            self.skipTest("Isaac assets root is not available; skipping sim test.")

        from isaacsim.robot.experimental.wheeled_robots.robots import HolonomicRobotUsdSetup

        stage_utils.add_reference_to_stage(
            usd_path=self._assets_root + "/Isaac/Robots/NVIDIA/Kaya/kaya.usd",
            path="/World/Kaya",
        )
        kaya = Articulation(
            "/World/Kaya",
            positions=[0.0, 0.0, 0.02],
            orientations=[1.0, 0.0, 0.0, 0.0],
            reset_xform_op_properties=True,
        )
        await app_utils.update_app_async()

        setup = HolonomicRobotUsdSetup(
            robot_prim_path="/World/Kaya",
            com_prim_path="/World/Kaya/base_link/control_offset",
        )
        wheel_radius, wheel_positions, wheel_orientations, _angles, wheel_axis, up_axis = (
            setup.get_holonomic_controller_params()
        )

        SimulationManager.setup_simulation(dt=DT, device="cpu")
        app_utils.play()
        await app_utils.update_app_async(steps=5)

        controller = ctrl.HolonomicController(
            robot_joint_space=list(kaya.dof_names),
            wheel_joint_names=KAYA_WHEEL_JOINTS,
            wheel_radius=wheel_radius,
            wheel_positions=wheel_positions,
            wheel_orientations=wheel_orientations,
            mecanum_angles=[90.0, 90.0, 90.0],
            wheel_axis=wheel_axis,
            rotation_direction=up_axis,
        )
        return kaya, controller

    async def _build_o3dyn_controller(self):
        """Load the O3dyn mecanum robot and build a controller from its authored geometry.

        Returns:
            ``(articulation, HolonomicController)``. Skips the test if assets are absent.
        """
        if self._assets_root is None:
            self.skipTest("Isaac assets root is not available; skipping sim test.")

        from isaacsim.robot.experimental.wheeled_robots.robots import HolonomicRobotUsdSetup

        stage_utils.add_reference_to_stage(
            usd_path=self._assets_root + "/Isaac/Robots_Multiphysics/Fraunhofer/O3dyn/o3dyn.usda",
            path="/World/O3dyn",
        )
        o3dyn = Articulation(
            "/World/O3dyn",
            positions=[0.0, 0.0, 0.3],
            orientations=[1.0, 0.0, 0.0, 0.0],
            reset_xform_op_properties=True,
        )
        await app_utils.update_app_async()

        setup = HolonomicRobotUsdSetup(robot_prim_path="/World/O3dyn", com_prim_path="/World/O3dyn/base_link")
        wheel_radius, wheel_positions, wheel_orientations, mecanum_angles, wheel_axis, up_axis = (
            setup.get_holonomic_controller_params()
        )
        self.assertEqual(
            len(wheel_positions),
            len(O3DYN_WHEEL_JOINTS),
            "O3dyn mecanum-wheel metadata does not match the expected wheel count.",
        )

        SimulationManager.setup_simulation(dt=DT, device="cpu")
        app_utils.play()
        await app_utils.update_app_async(steps=5)

        wheel_joint_names = list(setup.get_articulation_controller_params())
        controller = ctrl.HolonomicController(
            robot_joint_space=list(o3dyn.dof_names),
            wheel_joint_names=wheel_joint_names,
            wheel_radius=wheel_radius,
            wheel_positions=wheel_positions,
            wheel_orientations=wheel_orientations,
            mecanum_angles=mecanum_angles,
            wheel_axis=wheel_axis,
            rotation_direction=up_axis,
        )
        return o3dyn, controller

    def _apply_wheel_command(
        self,
        robot: Articulation,
        controller: ctrl.HolonomicController,
        vx: float,
        vy: float,
        wz: float,
    ) -> None:
        """Compute and apply wheel velocity targets for a command-site twist."""
        site = "control_point"
        setpoint = mg.RobotState(
            sites=mg.SpatialState.from_name(
                spatial_space=[site],
                linear_velocities=([site], wp.array([[vx, vy, 0.0]], dtype=wp.float32)),
                angular_velocities=([site], wp.array([[0.0, 0.0, wz]], dtype=wp.float32)),
            )
        )
        desired = controller.forward(mg.RobotState(), setpoint, 0.0)
        self.assertIsNotNone(desired, "controller returned no command")
        robot.set_dof_velocity_targets(
            desired.joints.velocities,
            dof_indices=robot.get_dof_indices(desired.joints.velocity_names),
        )

    @staticmethod
    def _xy(robot: Articulation) -> np.ndarray:
        """Return the base world position (x, y)."""
        return robot.get_world_poses()[0].numpy()[0][:2].copy()

    # ── Kaya (omni) ───────────────────────────────────────────────────────────

    async def test_kaya_drives_forward(self) -> None:
        """A forward twist drives the Kaya base forward in +x."""
        kaya, controller = await self._build_kaya_controller()
        self._apply_wheel_command(kaya, controller, 0.4, 0.0, 0.0)
        await app_utils.update_app_async(steps=30)
        start = self._xy(kaya)
        await app_utils.update_app_async(steps=60)
        delta = self._xy(kaya) - start
        self.assertGreater(delta[0], 0.05, f"base did not translate forward: delta={delta}")
        self.assertGreater(delta[0], 2.0 * abs(delta[1]), f"lateral drift dominates: delta={delta}")

    async def test_kaya_drives_diagonally(self) -> None:
        """An equal forward+lateral twist drives the Kaya base along the +x/+y diagonal."""
        kaya, controller = await self._build_kaya_controller()
        self._apply_wheel_command(kaya, controller, 0.3, 0.3, 0.0)
        await app_utils.update_app_async(steps=30)
        start = self._xy(kaya)
        await app_utils.update_app_async(steps=60)
        delta = self._xy(kaya) - start
        self.assertGreater(delta[0], 0.05, f"no forward motion: delta={delta}")
        self.assertGreater(delta[1], 0.05, f"no lateral motion: delta={delta}")
        self.assertGreater(delta[0], 0.4 * delta[1], f"not diagonal (x too small): delta={delta}")
        self.assertGreater(delta[1], 0.4 * delta[0], f"not diagonal (y too small): delta={delta}")

    async def test_kaya_rotates_in_place(self) -> None:
        """A pure yaw twist spins the Kaya base (positive yaw rate) without translating."""
        kaya, controller = await self._build_kaya_controller()
        self._apply_wheel_command(kaya, controller, 0.0, 0.0, 1.0)
        await app_utils.update_app_async(steps=30)
        start = self._xy(kaya)
        _lin, ang = kaya.get_velocities()
        ang_z = float(ang.numpy()[0][2])
        await app_utils.update_app_async(steps=60)
        delta = self._xy(kaya) - start
        self.assertGreater(ang_z, 0.5, f"yaw rate too low / wrong sign: {ang_z}")
        self.assertLess(ang_z, 1.5, f"yaw rate too high: {ang_z}")
        self.assertLess(float(np.linalg.norm(delta)), 0.15, f"base translated while rotating: delta={delta}")

    # ── O3dyn (mecanum) ───────────────────────────────────────────────────────

    async def test_o3dyn_drives_forward(self) -> None:
        """A forward twist drives the O3dyn mecanum base forward in +x."""
        o3dyn, controller = await self._build_o3dyn_controller()
        self._apply_wheel_command(o3dyn, controller, 0.4, 0.0, 0.0)
        await app_utils.update_app_async(steps=30)
        start = self._xy(o3dyn)
        await app_utils.update_app_async(steps=60)
        delta = self._xy(o3dyn) - start
        self.assertGreater(delta[0], 0.05, f"base did not translate forward: delta={delta}")
        self.assertGreater(delta[0], 2.0 * abs(delta[1]), f"lateral drift dominates: delta={delta}")

    async def test_o3dyn_strafes_sideways(self) -> None:
        """A pure lateral twist strafes the O3dyn mecanum base in +y (the mecanum hallmark)."""
        o3dyn, controller = await self._build_o3dyn_controller()
        self._apply_wheel_command(o3dyn, controller, 0.0, 0.4, 0.0)
        await app_utils.update_app_async(steps=30)
        start = self._xy(o3dyn)
        await app_utils.update_app_async(steps=60)
        delta = self._xy(o3dyn) - start
        self.assertGreater(delta[1], 0.05, f"base did not strafe sideways: delta={delta}")
        self.assertGreater(delta[1], 2.0 * abs(delta[0]), f"forward drift dominates: delta={delta}")

    async def test_o3dyn_rotates_in_place(self) -> None:
        """A pure yaw twist spins the O3dyn base (positive yaw rate) without translating."""
        o3dyn, controller = await self._build_o3dyn_controller()
        self._apply_wheel_command(o3dyn, controller, 0.0, 0.0, 1.0)
        await app_utils.update_app_async(steps=30)
        start = self._xy(o3dyn)
        _lin, ang = o3dyn.get_velocities()
        ang_z = float(ang.numpy()[0][2])
        await app_utils.update_app_async(steps=60)
        delta = self._xy(o3dyn) - start
        self.assertGreater(ang_z, 0.5, f"yaw rate too low / wrong sign: {ang_z}")
        self.assertLess(ang_z, 1.5, f"yaw rate too high: {ang_z}")
        self.assertLess(float(np.linalg.norm(delta)), 0.25, f"base translated while rotating: delta={delta}")
