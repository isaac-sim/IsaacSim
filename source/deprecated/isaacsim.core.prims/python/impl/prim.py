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

"""Provides a wrapper class for USD prims that offers a unified interface for managing collections of prims in Isaac Sim."""

from __future__ import annotations

import re

import numpy as np
import omni.kit.app
import warp as wp
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager
from isaacsim.core.utils.prims import find_matching_prim_paths, get_prim_at_path
from pxr import Usd

torch = import_module("torch")


class Prim(object):
    """A wrapper for USD prims that provides a unified interface for managing collections of prims in Isaac Sim.

    This class encapsulates one or more USD prims identified by path expressions and provides common functionality
    for prim management, validation, and lifecycle callbacks. It automatically handles prim discovery through pattern
    matching and maintains references to the underlying USD prims.

    The class integrates with Isaac Sim's simulation manager to receive callbacks for physics initialization,
    post-reset events, and prim deletion events. It serves as a base class for more specialized prim views.

    Args:
        prim_paths_expr: USD prim path or path expression to match prims in the stage. Supports wildcard patterns
            for matching multiple prims.
        name: Identifier name for this prim view instance.

    Raises:
        Exception: If no prims match the provided path expression.

    Example:

    .. code-block:: python

        # Create a view for a single prim
        prim = Prim("/World/my_prim")

        # Create a view for multiple prims using wildcards
        prims = Prim("/World/envs/env_*", name="environment_prims")

        # Access prim properties
        print(f"Found {prims.count} prims")
        print(f"Prim paths: {prims.prim_paths}")
    """

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
                f"Prim path expression {prim_paths_expr} is invalid, a prim matching the expression needs to created before wrapping it as view"
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

    def __del__(self) -> None:
        """Clean up the Prim instance by destroying it."""
        self.destroy()

    def destroy(self) -> None:
        """Clean up and invalidate the prim view by deregistering callbacks and clearing internal state."""
        for callback_id in self._callbacks:
            SimulationManager.deregister_callback(callback_id)
        self._callbacks = []
        self._prims = []
        self._prim_paths = []
        self._count = 0
        self._is_valid = False

    @property
    def prim_paths(self) -> list[str]:
        """List of prim paths in the stage encapsulated in this view.

        Returns:
            List of prim paths in the stage encapsulated in this view.

        Example:

        .. code-block:: python

            >>> prims.prim_paths
            ['/World/envs/env_0', '/World/envs/env_1', '/World/envs/env_2', '/World/envs/env_3', '/World/envs/env_4']
        """
        return self._prim_paths

    @property
    def name(self) -> str:
        """Name given to the prims view when instantiating it.

        Returns:
            Name given to the prims view when instantiating it.
        """
        return self._name

    @property
    def count(self) -> int:
        """Number of prims encapsulated in this view.

        Returns:
            Number of prims encapsulated in this view.

        Example:

        .. code-block:: python

            >>> prims.count
            5
        """
        return self._count

    @property
    def prims(self) -> list[Usd.Prim]:
        """List of USD Prim objects encapsulated in this view.

        Returns:
            List of USD Prim objects encapsulated in this view.

        Example:

        .. code-block:: python

            >>> prims.prims
            [Usd.Prim(</World/envs/env_0>), Usd.Prim(</World/envs/env_1>), Usd.Prim(</World/envs/env_2>),
             Usd.Prim(</World/envs/env_3>), Usd.Prim(</World/envs/env_4>)]
        """
        return self._prims

    @property
    def initialized(self) -> bool:
        """Whether the prim view is initialized.

        Returns:
            True if the view object was initialized (after the first call of .initialize()). False otherwise.

        Example:

        .. code-block:: python

            >>> # given an initialized articulation view
            >>> prims.initialized
            True
        """
        return SimulationManager.get_physics_sim_view() is not None

    def post_reset(self) -> None:
        """Reset the prims to its default state.

        Example:

        .. code-block:: python

            >>> prims.post_reset()
        """
        self._on_post_reset(None)
        return

    def is_valid(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None) -> bool:
        """Check that all prims have a valid USD Prim.

        Args:
            indices: Indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                If None, all prims in the view are queried.

        Returns:
            True if all prim paths specified in the view correspond to a valid prim in stage. False otherwise.

        Example:

        .. code-block:: python

            >>> prims.is_valid()
            True
        """
        return self._is_valid

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """Create a physics simulation view if not passed and set other properties using the PhysX tensor API.

        .. note::

            For this particular class, calling this method will do nothing

        Args:
            physics_sim_view: Current physics simulation view.

        Example:

        .. code-block:: python

            >>> prims.initialize()
        """
        self._on_physics_ready(None)
        return

    def _on_prim_deletion(self, prim_path: str) -> None:
        """Handle prim deletion events.

        Invalidates the view and removes callbacks when a matching prim is deleted from the stage.

        Args:
            prim_path: Path of the deleted prim.
        """
        # TODO: regex matching in c++
        if prim_path == "/":
            self._is_valid = False
            for callback_id in self._callbacks:
                SimulationManager.deregister_callback(callback_id)
            self._callbacks = []
            return
        for regex_prim_paths in self._regex_prim_paths:
            truncated_parts = regex_prim_paths.split("/")[: prim_path.count("/") + 1]
            safe_pattern = "/".join(part.replace(".*", "[^/]*") for part in truncated_parts)
            try:
                result = re.match(pattern="^" + safe_pattern + "$", string=prim_path)
            except re.error:
                result = None
            if result:
                self._is_valid = False
                for callback_id in self._callbacks:
                    SimulationManager.deregister_callback(callback_id)
                self._callbacks = []
                return
        return

    def _on_physics_ready(self, event: object) -> None:
        """Handle physics ready events.

        Updates backend references when physics simulation is ready.

        Args:
            event: The physics ready event.
        """
        self._backend = SimulationManager.get_backend()
        self._device = SimulationManager.get_physics_sim_device()
        self._backend_utils = SimulationManager._get_backend_utils()
        return

    def _on_post_reset(self, event: object) -> None:
        """Handle post-reset events.

        Called after simulation reset to reinitialize prim state.

        Args:
            event: The post-reset event.
        """
        return

    def _remove_callbacks(self) -> None:
        """Remove all registered simulation callbacks.

        Deregisters callbacks for physics ready, post-reset, and prim deletion events.
        """
        for callback_id in self._callbacks:
            SimulationManager.deregister_callback(callback_id)
        self._callbacks = []
        return
