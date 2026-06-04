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

"""Handles multiple robot (articulation) prims efficiently through regex-based selection for batch operations."""

from __future__ import annotations

import numpy as np
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.prims import Articulation

torch = import_module("torch")


class RobotView(Articulation):
    """Implementation (on ``Articulation`` class) to deal with articulation prims as robots.

    This class wraps all matching articulations found at the regex provided at the ``prim_paths_expr`` argument

    .. warning::

        The robot (articulation) view object must be initialized in order to be able to operate on it.
        See the ``initialize`` method for more details.

    Args:
        prim_paths_expr: prim paths regex to encapsulate all prims that match it.
            example: "/World/Env[1-5]/Franka" will match /World/Env1/Franka,
            /World/Env2/Franka, etc.
            (a non regex prim path can also be used to encapsulate one rigid prim).
        name: shortname to be used as a key by Scene class.
            Note: needs to be unique if the object is added to the Scene.
        positions: default positions in the world frame of the prims.
            shape is (N, 3).
        translations: default translations in the local frame of the prims
            (with respect to its parent prims). shape is (N, 3).
        orientations: default quaternion orientations in the world/ local frame of the prims
            (depends if translation or position is specified).
            quaternion is scalar-first (w, x, y, z). shape is (N, 4).
        scales: local scales to be applied to
            the prim's dimensions in the view. shape is (N, 3).
        visibilities: set to false for an invisible prim in
            the stage while rendering. shape is (N,).

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.stage as stage_utils
        >>> from isaacsim.core.cloner import GridCloner
        >>> from isaacsim.core.api.robots import RobotView
        >>> from pxr import UsdGeom
        >>>
        >>> usd_path = "/home/<user>/Documents/Assets/Robots/FrankaRobotics/FrankaPanda/frankas.usd"
        >>> env_zero_path = "/World/envs/env_0"
        >>> num_envs = 5
        >>>
        >>> # load the Franka Panda robot USD file
        >>> stage_utils.add_reference_to_stage(usd_path, prim_path=f"{env_zero_path}/panda")  # /World/envs/env_0/panda
        >>>
        >>> # clone the environment (num_envs)
        >>> cloner = GridCloner(spacing=1.5)
        >>> cloner.define_base_env(env_zero_path)
        >>> UsdGeom.Xform.Define(stage_utils.get_current_stage(), env_zero_path)
        >>> cloner.clone(source_prim_path=env_zero_path, prim_paths=cloner.generate_paths("/World/envs/env", num_envs))
        >>>
        >>> # wrap all robots
        >>> prims = RobotView(prim_paths_expr="/World/envs/env.*/panda", name="franka_panda_view")
        >>> print(prims)
        <isaacsim.core.api.robots.robot_view.RobotView object at 0x7f12785a5fc0>

    """

    def __init__(
        self,
        prim_paths_expr: str,
        name: str = "robot_view",
        positions: np.ndarray | torch.Tensor | None = None,
        translations: np.ndarray | torch.Tensor | None = None,
        orientations: np.ndarray | torch.Tensor | None = None,
        scales: np.ndarray | torch.Tensor | None = None,
        visibilities: np.ndarray | torch.Tensor | None = None,
    ) -> None:
        Articulation.__init__(
            self,
            prim_paths_expr=prim_paths_expr,
            name=name,
            positions=positions,
            translations=translations,
            orientations=orientations,
            scales=scales,
            visibilities=visibilities,
        )
        self._sensors = []
        return

    def post_reset(self) -> None:
        """Reset the robots to their default states.

        .. note::

            For the robots, in addition to configuring the root prim's default positions and spatial orientations
            (defined via the ``set_default_state`` method), the joint's positions, velocities, and efforts
            (defined via the ``set_joints_default_state`` method) and the joint's stiffness and dampings
            (defined via the ``set_gains`` method) are imposed

        Example:

        .. code-block:: python

            >>> prims.post_reset()

        """
        Articulation.post_reset(self)
        return
