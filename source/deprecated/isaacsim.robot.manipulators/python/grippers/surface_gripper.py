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

"""Provides a high-level interface for controlling surface grippers such as suction cups in Isaac Sim."""

import carb
import omni.kit.app
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.grippers.gripper import Gripper
from isaacsim.robot.surface_gripper import _surface_gripper as surface_gripper


class SurfaceGripper(Gripper):
    """Provides high level functions to set/ get properties and actions of a surface gripper.

    (a suction cup for example).

    Args:
        end_effector_prim_path: Prim path of the Prim that corresponds to the gripper root/ end effector.
        surface_gripper_path: Prim path of the surface gripper.
    """

    def __init__(
        self,
        end_effector_prim_path: str,
        surface_gripper_path: str,
    ) -> None:
        Gripper.__init__(self, end_effector_prim_path=end_effector_prim_path)
        self._surface_gripper_interface = surface_gripper.acquire_surface_gripper_interface()
        self._surface_gripper_path = surface_gripper_path
        return

    def initialize(
        self, physics_sim_view: omni.physics.tensors.SimulationView = None, articulation_num_dofs: int = None
    ) -> None:
        """Create a physics simulation view if not passed and creates a rigid prim view using physX tensor api.

        This needs to be called after each hard reset (i.e stop + play on the timeline) before interacting with any
        of the functions of this class.

        Args:
            physics_sim_view: Current physics simulation view.
            articulation_num_dofs: Number of DOFs of the articulation.
        """
        Gripper.initialize(self, physics_sim_view=physics_sim_view)
        self._articulation_num_dofs = articulation_num_dofs

        if self._default_state is None:
            self._default_state = not self.is_closed()
        return

    def close(self) -> None:
        """Applies actions to the articulation that closes the gripper (ex: to hold an object)."""
        if not self.is_closed():
            self._surface_gripper_interface.close_gripper(self._surface_gripper_path)
        if not self.is_closed():
            carb.log_warn("gripper didn't close successfully")
        return

    def open(self) -> None:
        """Applies actions to the articulation that opens the gripper (ex: to release an object held)."""
        self._surface_gripper_interface.open_gripper(self._surface_gripper_path)
        if not self.is_open():
            carb.log_warn("gripper didn't open successfully")

        return

    def update(self) -> None:
        """Updates the gripper state."""
        # self._virtual_gripper.update()
        return

    def is_closed(self) -> bool:
        """Whether the gripper is in a closed state.

        Returns:
            True if the gripper is closed.
        """
        return (
            self._surface_gripper_interface.get_gripper_status(self._surface_gripper_path)
            == surface_gripper.GripperStatus.Closed
        )

    def is_open(self) -> bool:
        """Whether the gripper is in an open state.

        Returns:
            True if the gripper is open.
        """
        return (
            self._surface_gripper_interface.get_gripper_status(self._surface_gripper_path)
            == surface_gripper.GripperStatus.Open
        )

    def set_default_state(self, opened: bool) -> None:
        """Sets the default state of the gripper.

        Args:
            opened: True if the surface gripper should start in an opened state. False otherwise.
        """
        self._default_state = opened
        return

    def get_default_state(self) -> dict:
        """Gets the default state of the gripper.

        Returns:
            Key is "opened" and value would be true if the surface gripper should start in an opened state.
            False otherwise.
        """
        return {"opened": self._default_state}

    def post_reset(self) -> None:
        """Resets the gripper to its default state."""
        Gripper.post_reset(self)
        if self._default_state:  # means opened is true
            self.open()
        else:
            self.close()
        return

    def forward(self, action: str) -> ArticulationAction:
        """Calculates the ArticulationAction for all of the articulation joints that corresponds to "open".

        or "close" actions.

        Args:
            action: "open" or "close" as an abstract action.

        Raises:
            Exception: If articulation_num_dofs is not set during initialization.
            Exception: If action is not "open" or "close".

        Returns:
            Articulation action to be passed to the articulation itself (includes all joints of the articulation).
        """
        if self._articulation_num_dofs is None:
            raise Exception(
                "Num of dofs of the articulation needs to be passed to initialize in order to use this method"
            )
        if action == "open":
            self.open()
        elif action == "close":
            self.close()
        else:
            raise Exception(f"action {action} is not defined for SurfaceGripper")
        return ArticulationAction(joint_positions=[None] * self._articulation_num_dofs)
