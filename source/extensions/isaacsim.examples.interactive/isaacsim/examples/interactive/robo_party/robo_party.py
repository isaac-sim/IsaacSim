"""Robo party sample combining Franka stacking, UR10 stacking, Kaya, and Jetbot."""

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

import isaacsim.core.experimental.utils.app as app_utils
import numpy as np
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
from isaacsim.examples.base.base_sample_experimental import BaseSample
from isaacsim.robot.experimental.manipulators.examples.franka.stacking import Stacking as FrankaStacking
from isaacsim.robot.experimental.manipulators.examples.universal_robots.stacking import Stacking as UR10Stacking

# Wheeled robots (Kaya, Jetbot): isaacsim.robot.experimental.wheeled_robots
# Extension: source/extensions/isaacsim.robot.experimental.wheeled_robots
from isaacsim.robot.experimental.wheeled_robots.controllers import (
    DifferentialController,
    HolonomicController,
)
from isaacsim.robot.experimental.wheeled_robots.robots import (
    HolonomicRobotUsdSetup,
    WheeledRobot,
)
from isaacsim.storage.native import get_assets_root_path


class RoboParty(BaseSample):
    """Interactive sample: Franka stacking + UR10 stacking + Kaya (holonomic) + Jetbot (differential)."""

    def __init__(self) -> None:
        super().__init__()
        self._stacking: FrankaStacking | None = None
        self._ur10_stacking: UR10Stacking | None = None
        self._kaya: WheeledRobot | None = None
        self._jetbot: WheeledRobot | None = None
        self._holonomic_controller: HolonomicController | None = None
        self._differential_controller: DifferentialController | None = None
        self._physics_callback_id: int | None = None
        self._is_executing = False
        self._party_step = 0

    def setup_scene(self) -> None:
        """Set up the scene: Franka stacking, UR10 stacking, Kaya, and Jetbot."""
        # Franka stacking
        self._stacking = FrankaStacking(
            robot_path="/World/robot_0",
            cube_positions=[
                np.array([0.3, 0.3, 0.025]),
                np.array([0.3, -0.3, 0.025]),
            ],
            offset=np.array([0.0, -2.0, 0.0]),
            robot_name="Franka",
        )
        self._stacking.setup_scene()

        # UR10 stacking
        self._ur10_stacking = UR10Stacking(
            robot_path="/World/robot_1",
            cube_positions=[
                np.array([0.25, 0.25, 0.025]),
                np.array([0.2, -0.2, 0.025]),
            ],
            offset=np.array([0.0, 0.0, 0.0]),
            robot_name="UR",
        )
        self._ur10_stacking.setup_scene()

        assets_root = get_assets_root_path()
        if assets_root is None:
            return

        # Kaya (holonomic)
        kaya_path = "/World/Kaya"
        kaya_usd = assets_root + "/Isaac/Robots/NVIDIA/Kaya/kaya.usd"
        self._kaya = WheeledRobot(
            paths=kaya_path,
            wheel_dof_names=["axle_0_joint", "axle_1_joint", "axle_2_joint"],
            usd_path=kaya_usd,
            positions=np.array([-1.0, 0.0, 0.0]),
        )

        # Jetbot (differential)
        jetbot_path = "/World/Jetbot"
        jetbot_usd = assets_root + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
        self._jetbot = WheeledRobot(
            paths=jetbot_path,
            wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
            usd_path=jetbot_usd,
            positions=np.array([-1.5, -1.5, 0.0]),
        )

    async def setup_post_load(self) -> None:
        """Build controllers for Kaya and Jetbot after scene is loaded."""
        # View so Franka/UR10 stackings and Jetbot/Kaya are all visible (target slightly toward wheeled robots)
        ViewportManager.set_camera_view(eye=[10.0, 0.0, 5.0], target=[0.0, -2.0, 0.0], camera="/OmniverseKit_Persp")

        if self._kaya is not None:
            kaya_setup = HolonomicRobotUsdSetup(
                robot_prim_path="/World/Kaya",
                com_prim_path="/World/Kaya/base_link/control_offset",
            )
            (
                wheel_radius,
                wheel_positions,
                wheel_orientations,
                mecanum_angles,
                wheel_axis,
                up_axis,
            ) = kaya_setup.get_holonomic_controller_params()
            self._holonomic_controller = HolonomicController(
                wheel_radius=wheel_radius,
                wheel_positions=wheel_positions,
                wheel_orientations=wheel_orientations,
                mecanum_angles=mecanum_angles,
                wheel_axis=wheel_axis,
                up_axis=up_axis,
            )

        if self._jetbot is not None:
            self._differential_controller = DifferentialController(
                wheel_radius=0.03,
                wheel_base=0.1125,
            )

    async def setup_pre_reset(self) -> None:
        """Remove physics callback and reset state."""
        self._remove_physics_callback()
        if self._stacking is not None:
            self._stacking.reset()
        if self._ur10_stacking is not None:
            self._ur10_stacking.reset()
        self._is_executing = False
        self._party_step = 0

    async def setup_post_reset(self) -> None:
        """After reset: reset Franka and UR10 to default pose."""
        if self._stacking is not None:
            self._stacking.reset_robot()
        if self._ur10_stacking is not None:
            self._ur10_stacking.reset_robot()

    async def setup_post_clear(self) -> None:
        """Clear all references."""
        self._remove_physics_callback()
        self._stacking = None
        self._ur10_stacking = None
        self._kaya = None
        self._jetbot = None
        self._holonomic_controller = None
        self._differential_controller = None
        self._is_executing = False
        self._party_step = 0

    def physics_cleanup(self) -> None:
        """Clean up physics callback and state."""
        self._remove_physics_callback()
        self._stacking = None
        self._ur10_stacking = None
        self._kaya = None
        self._jetbot = None
        self._holonomic_controller = None
        self._differential_controller = None
        self._is_executing = False
        self._party_step = 0

    def _remove_physics_callback(self) -> None:
        if self._physics_callback_id is not None:
            SimulationManager.deregister_callback(self._physics_callback_id)
            self._physics_callback_id = None

    def _party_physics_callback(self, dt: float, context: object) -> None:
        """Run stacking and time-based wheeled robot commands.

        Args:
            dt: Time delta for the physics step.
            context: Physics simulation context.
        """
        if not self._is_executing:
            return

        # Franka stacking
        if self._stacking is not None and not self._stacking.is_done():
            self._stacking.forward()

        # UR10 stacking
        if self._ur10_stacking is not None and not self._ur10_stacking.is_done():
            self._ur10_stacking.forward()

        # Time-based commands for Kaya and Jetbot (same idea as original robo_party)
        if self._party_step < 500:
            kaya_cmd = np.array([0.2, 0.0, 0.0])
            jetbot_cmd = np.array([0.1, 0.0])
        elif self._party_step < 1000:
            kaya_cmd = np.array([0.0, 0.2, 0.0])
            jetbot_cmd = np.array([0.0, np.pi / 10])
        elif self._party_step < 1500:
            kaya_cmd = np.array([0.0, 0.0, 0.06])
            jetbot_cmd = np.array([0.1, 0.0])
        else:
            kaya_cmd = np.array([0.0, 0.0, 0.0])
            jetbot_cmd = np.array([0.0, 0.0])

        if self._holonomic_controller is not None and self._kaya is not None:
            velocities = self._holonomic_controller.forward(kaya_cmd)
            self._kaya.apply_wheel_actions(velocities)
        if self._differential_controller is not None and self._jetbot is not None:
            velocities = self._differential_controller.forward(jetbot_cmd)
            self._jetbot.apply_wheel_actions(velocities)

        self._party_step += 1

    async def _on_start_party_event_async(self) -> None:
        """Start the party: register physics callback and play."""
        if self._is_executing:
            return
        if self._stacking is None and self._ur10_stacking is None:
            return
        self._is_executing = True
        self._party_step = 0
        self._physics_callback_id = SimulationManager.register_callback(
            self._party_physics_callback, event=SimulationEvent.PHYSICS_POST_STEP
        )
        app_utils.play()
        await app_utils.update_app_async()
