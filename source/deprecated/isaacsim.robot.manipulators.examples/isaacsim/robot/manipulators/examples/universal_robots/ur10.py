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

"""UR10 robot wrapper with gripper and motion generation support."""

from __future__ import annotations

import carb
import numpy as np
from isaacsim.core.api.robots.robot import Robot
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
from isaacsim.storage.native import get_assets_root_path


class UR10(Robot):
    """Universal Robots UR10 robot arm with optional surface gripper.

    Args:
        prim_path: USD prim path for the robot.
        name: Name identifier for the robot.
        usd_path: Path to custom USD file.
        position: Initial position of the robot.
        orientation: Initial orientation as quaternion.
        end_effector_prim_name: Name of the end effector prim.
        attach_gripper: Whether to attach a gripper.
        gripper_usd: Path to gripper USD or "default".

    Raises:
        NotImplementedError: If custom gripper USD is specified but not supported.
    """

    def __init__(
        self,
        prim_path: str,
        name: str = "ur10_robot",
        usd_path: str | None = None,
        position: np.ndarray | None = None,
        orientation: np.ndarray | None = None,
        end_effector_prim_name: str | None = None,
        attach_gripper: bool = False,
        gripper_usd: str | None = "default",
    ) -> None:
        prim = get_prim_at_path(prim_path)
        self._end_effector = None
        self._gripper = None
        self._end_effector_prim_name = end_effector_prim_name
        if not prim.IsValid():
            if usd_path:
                prim = add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            else:
                assets_root_path = get_assets_root_path()
                if assets_root_path is None:
                    carb.log_error("Could not find Isaac Sim assets folder")
                    return
                usd_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
                prim = add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
            if self._end_effector_prim_name is None:
                self._end_effector_prim_path = prim_path + "/ee_link"
            else:
                self._end_effector_prim_path = prim_path + "/" + end_effector_prim_name
        else:
            # TODO: change this
            if self._end_effector_prim_name is None:
                self._end_effector_prim_path = prim_path + "/ee_link"
            else:
                self._end_effector_prim_path = prim_path + "/" + end_effector_prim_name
        super().__init__(
            prim_path=prim_path, name=name, position=position, orientation=orientation, articulation_controller=None
        )
        self._gripper_usd = gripper_usd
        if attach_gripper:
            if gripper_usd == "default":
                assets_root_path = get_assets_root_path()
                if assets_root_path is None:
                    carb.log_error("Could not find Isaac Sim assets folder")
                    return
                prim.GetVariantSet("Gripper").SetVariantSelection("Short_Suction")
                self._gripper = SurfaceGripper(
                    end_effector_prim_path=self._end_effector_prim_path,
                    surface_gripper_path=self._end_effector_prim_path + "/SurfaceGripper",
                )
            elif gripper_usd is None:
                carb.log_warn("Not adding a gripper usd, the gripper already exists in the ur10 asset")
                self._gripper = SurfaceGripper(
                    end_effector_prim_path=self._end_effector_prim_path,
                    surface_gripper_path=self._end_effector_prim_path + "/SurfaceGripper",
                )
            else:
                raise NotImplementedError
        self._attach_gripper = attach_gripper
        return

    @property
    def attach_gripper(self) -> bool:
        """Whether a gripper is attached to the robot.

        Returns:
            True if gripper is attached, False otherwise.
        """
        return self._attach_gripper

    @property
    def end_effector(self) -> SingleRigidPrim:
        """The robot's end effector prim.

        Returns:
            The end effector as a SingleRigidPrim.
        """
        return self._end_effector

    @property
    def gripper(self) -> SurfaceGripper:
        """The surface gripper controller attached to the robot.

        Returns:
            The surface gripper controller.
        """
        return self._gripper

    def initialize(self, physics_sim_view: object = None) -> None:
        """Initialize the robot and its components.

        Args:
            physics_sim_view: Physics simulation view for initialization.
        """
        super().initialize(physics_sim_view)
        if self._attach_gripper:
            self._gripper.initialize(physics_sim_view=physics_sim_view, articulation_num_dofs=self.num_dof)
        self._end_effector = SingleRigidPrim(prim_path=self._end_effector_prim_path, name=self.name + "_end_effector")
        self.disable_gravity()
        self._end_effector.initialize(physics_sim_view)
        return

    def post_reset(self) -> None:
        """Reset callback for end effector and gripper."""
        Robot.post_reset(self)
        self._end_effector.post_reset()
        self._gripper.post_reset()
        return
