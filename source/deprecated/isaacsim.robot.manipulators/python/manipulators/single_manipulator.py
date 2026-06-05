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

"""High level wrapper for a manipulator with a single end effector and optional gripper."""

from collections.abc import Sequence
from typing import Optional

import omni.kit.app
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
from isaacsim.robot.manipulators.grippers.gripper import Gripper
from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper


class SingleManipulator(SingleArticulation):
    """Provides high level functions to set/ get properties and actions of a manipulator with a single end effector.

    and optionally a gripper.

    Args:
        prim_path: Prim path of the Prim to encapsulate or create.
        end_effector_prim_path: End effector prim path to be used to track the rigid body that corresponds
            to the end effector. One of the following args can be specified only:
            end_effector_prim_name or end_effector_prim_path.
        name: Shortname to be used as a key by Scene class. Note: needs to be unique if the
            object is added to the Scene.
        position: Position in the world frame of the prim. shape is (3, ).
        translation: Translation in the local frame of the prim
            (with respect to its parent prim). shape is (3, ).
        orientation: Quaternion orientation in the world/ local frame of the prim
            (depends if translation or position is specified).
            quaternion is scalar-first (w, x, y, z). shape is (4, ).
        scale: Local scale to be applied to the prim's dimensions. shape is (3, ).
        visible: Set to false for an invisible prim in the stage while rendering.
        gripper: Gripper to be used with the manipulator.
    """

    def __init__(
        self,
        prim_path: str,
        end_effector_prim_path: str,
        name: str = "single_manipulator",
        position: Optional[Sequence[float]] = None,
        translation: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = None,
        visible: Optional[bool] = None,
        gripper: Gripper = None,
    ) -> None:
        self._end_effector_prim_path = end_effector_prim_path
        self._gripper = gripper
        self._end_effector = None
        SingleArticulation.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            articulation_controller=None,
        )
        return

    @property
    def end_effector(self) -> SingleRigidPrim:
        """End effector of the manipulator.

        Returns:
            End effector that can be used to get its world pose, angular velocity, etc.
        """
        return self._end_effector

    @property
    def gripper(self) -> Gripper:
        """Gripper of the manipulator.

        Returns:
            Gripper that can be used to open or close the gripper, get its world pose or angular velocity, etc.
        """
        return self._gripper

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """Create a physics simulation view if not passed and creates an articulation view using physX tensor api.

        This needs to be called after each hard reset (i.e stop + play on the timeline) before interacting with any
        of the functions of this class.

        Args:
            physics_sim_view: Current physics simulation view.
        """
        SingleArticulation.initialize(self, physics_sim_view=physics_sim_view)
        self._end_effector = SingleRigidPrim(prim_path=self._end_effector_prim_path, name=self.name + "_end_effector")
        self._end_effector.initialize(physics_sim_view)
        if isinstance(self._gripper, ParallelGripper):
            self._gripper.initialize(
                physics_sim_view=physics_sim_view,
                articulation_apply_action_func=self.apply_action,
                get_joint_positions_func=self.get_joint_positions,
                set_joint_positions_func=self.set_joint_positions,
                dof_names=self.dof_names,
            )
        if isinstance(self._gripper, SurfaceGripper):
            self._gripper.initialize(physics_sim_view=physics_sim_view, articulation_num_dofs=self.num_dof)
        return

    def post_reset(self) -> None:
        """Resets the manipulator, the end effector and the gripper to its default state."""
        SingleArticulation.post_reset(self)
        self._end_effector.post_reset()
        if self._gripper is not None:
            self._gripper.post_reset()
        return
