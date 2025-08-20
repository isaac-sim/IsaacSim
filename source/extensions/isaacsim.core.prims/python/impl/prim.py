# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import re
from typing import List, Optional, Union

import carb
import numpy as np
import omni.kit.app
import torch
import warp as wp
from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager
from isaacsim.core.utils.prims import find_matching_prim_paths, get_prim_at_path
from pxr import Usd


class Prim(object):
    def __init__(self, prim_paths_expr: str, name: str = "prim_view") -> None:
        if not isinstance(prim_paths_expr, list):
            prim_paths_expr = [prim_paths_expr]
        self._prim_paths = []
        self._callbacks = []
        for prim_path_expression in prim_paths_expr:
            self._prim_paths = self._prim_paths + find_matching_prim_paths(prim_path_expression)
        self._is_valid = True
        if len(self._prim_paths) == 0:
            raise Exception(
                "Prim path expression {} is invalid, a prim matching the expression needs to created before wrapping it as view".format(
                    prim_paths_expr
                )
            )
        self._name = name
        self._count = len(self._prim_paths)
        self._prims = []
        self._regex_prim_paths = prim_paths_expr
        for prim_path in self._prim_paths:
            self._prims.append(get_prim_at_path(prim_path))
        self._backend = SimulationManager.get_backend()
        self._device = SimulationManager.get_physics_sim_device()
        self._backend_utils = SimulationManager._get_backend_utils()
        self._callbacks.append(
            SimulationManager.register_callback(self._on_physics_ready, event=IsaacEvents.PHYSICS_READY)
        )
        self._callbacks.append(SimulationManager.register_callback(self._on_post_reset, event=IsaacEvents.POST_RESET))
        self._callbacks.append(
            SimulationManager.register_callback(self._on_prim_deletion, event=IsaacEvents.PRIM_DELETION)
        )
        return

    def __del__(self):
        self.destroy()

    def destroy(self):
        for callback_id in self._callbacks:
            SimulationManager.deregister_callback(callback_id)
        self._callbacks = []
        self._prims = []
        self._prim_paths = []
        self._count = 0
        self._is_valid = False

    @property
    def prim_paths(self) -> List[str]:
        """
        Returns:
            List[str]: list of prim paths in the stage encapsulated in this view.

        Example:

        .. code-block:: python

            >>> prims.prim_paths
            ['/World/envs/env_0', '/World/envs/env_1', '/World/envs/env_2', '/World/envs/env_3', '/World/envs/env_4']
        """
        return self._prim_paths

    @property
    def name(self) -> str:
        """
        Returns:
            str: name given to the prims view when instantiating it.
        """
        return self._name

    @property
    def count(self) -> int:
        """
        Returns:
            int: Number of prims encapsulated in this view.

        Example:

        .. code-block:: python

            >>> prims.count
            5
        """
        return self._count

    @property
    def prims(self) -> List[Usd.Prim]:
        """
        Returns:
            List[Usd.Prim]: List of USD Prim objects encapsulated in this view.

        Example:

        .. code-block:: python

            >>> prims.prims
            [Usd.Prim(</World/envs/env_0>), Usd.Prim(</World/envs/env_1>), Usd.Prim(</World/envs/env_2>),
             Usd.Prim(</World/envs/env_3>), Usd.Prim(</World/envs/env_4>)]
        """
        return self._prims

    @property
    def initialized(self) -> bool:
        """Check if prim view is initialized

        Returns:
            bool: True if the view object was initialized (after the first call of .initialize()). False otherwise.

        Example:

        .. code-block:: python

            >>> # given an initialized articulation view
            >>> prims.initialized
            True
        """
        return SimulationManager.get_physics_sim_view() is not None

    def post_reset(self) -> None:
        """Reset the prims to its default state

        Example:

        .. code-block:: python

            >>> prims.post_reset()
        """
        self._on_post_reset(None)
        return

    def is_valid(self, indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None) -> bool:
        """Check that all prims have a valid USD Prim

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Returns:
            bool: True if all prim paths specified in the view correspond to a valid prim in stage. False otherwise.

        Example:

        .. code-block:: python

            >>> prims.is_valid()
            True
        """
        return self._is_valid

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """Create a physics simulation view if not passed and set other properties using the PhysX tensor API

        .. note::

            For this particular class, calling this method will do nothing

        Args:
            physics_sim_view (omni.physics.tensors.SimulationView, optional): current physics simulation view. Defaults to None.

        Example:

        .. code-block:: python

            >>> prims.initialize()
        """
        self._on_physics_ready(None)
        return

    def _on_prim_deletion(self, prim_path):
        # TODO: regex matching in c++
        if prim_path == "/":
            self._is_valid = False
            for callback_id in self._callbacks:
                SimulationManager.deregister_callback(callback_id)
            self._callbacks = []
            return
        for regex_prim_paths in self._regex_prim_paths:
            result = re.match(
                pattern="^" + "/".join(regex_prim_paths.split("/")[: prim_path.count("/") + 1]) + "$", string=prim_path
            )
            if result:
                self._is_valid = False
                for callback_id in self._callbacks:
                    SimulationManager.deregister_callback(callback_id)
                self._callbacks = []
                return
        return

    def _on_physics_ready(self, event):
        self._backend = SimulationManager.get_backend()
        self._device = SimulationManager.get_physics_sim_device()
        self._backend_utils = SimulationManager._get_backend_utils()
        return

    def _on_post_reset(self, event) -> None:
        return

    def _remove_callbacks(self) -> None:
        for callback_id in self._callbacks:
            SimulationManager.deregister_callback(callback_id)
        self._callbacks = []
        return
