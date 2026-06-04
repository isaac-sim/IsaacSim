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

"""High-level API for creating and controlling robots as articulations in Isaac Sim."""

from __future__ import annotations

from collections.abc import Sequence

from isaacsim.core.api.controllers.articulation_controller import ArticulationController
from isaacsim.core.prims import SingleArticulation


class Robot(SingleArticulation):
    """Implementation (on ``SingleArticulation`` class) to deal with an articulation prim as a robot.

    .. warning::

        The robot (articulation) object must be initialized in order to be able to operate on it.
        See the ``initialize`` method for more details.

    Args:
        prim_path: Prim path of the Prim to encapsulate or create.
        name: Shortname to be used as a key by Scene class.
            Note: needs to be unique if the object is added to the Scene.
        position: Position in the world frame of the prim. shape is (3, ).
        translation: Translation in the local frame of the prim
            (with respect to its parent prim). shape is (3, ).
        orientation: Quaternion orientation in the world/ local frame of the prim
            (depends if translation or position is specified).
            quaternion is scalar-first (w, x, y, z). shape is (4, ).
        scale: Local scale to be applied to the prim's dimensions. shape is (3, ).
        visible: Set to false for an invisible prim in the stage while rendering.
        articulation_controller: A custom ArticulationController which
            inherits from it.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.stage as stage_utils
        >>> from isaacsim.core.api.robots import Robot
        >>>
        >>> usd_path = "/home/<user>/Documents/Assets/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        >>> prim_path = "/World/envs/env_0/panda"
        >>>
        >>> # load the Franka Panda robot USD file
        >>> stage_utils.add_reference_to_stage(usd_path, prim_path)
        >>>
        >>> # wrap the prim as a robot (articulation)
        >>> prim = Robot(prim_path=prim_path, name="franka_panda")
        >>> print(prim)
        <isaacsim.core.api.robots.robot.Robot object at 0x7fdd4875a1d0>

    """

    def __init__(
        self,
        prim_path: str,
        name: str = "robot",
        position: Sequence[float] | None = None,
        translation: Sequence[float] | None = None,
        orientation: Sequence[float] | None = None,
        scale: Sequence[float] | None = None,
        visible: bool | None = None,
        articulation_controller: ArticulationController | None = None,
    ) -> None:
        SingleArticulation.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            articulation_controller=articulation_controller,
        )
        self._sensors = []
        return

    def post_reset(self) -> None:
        """Reset the robot to its default state.

        .. note::

            For a robot, in addition to configuring the root prim's default position and spatial orientation
            (defined via the ``set_default_state`` method), the joint's positions, velocities, and efforts
            (defined via the ``set_joints_default_state`` method) are imposed

        Example:

        .. code-block:: python

            >>> prim.post_reset()

        """
        SingleArticulation.post_reset(self)
