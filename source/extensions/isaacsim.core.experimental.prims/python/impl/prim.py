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

"""Base wrapper class for managing USD prims with support for regex path matching and batch operations."""

from __future__ import annotations

import re
from abc import ABC
from typing import Any

import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import warp as wp
from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager
from pxr import Sdf, Usd

_MSG_PRIM_NOT_VALID = (
    "Instance is not valid. This is most likely because some of the wrapped prims have been removed from the stage, "
    "or because the instance has been deleted"
)

_MSG_PHYSICS_TENSOR_ENTITY_NOT_INITIALIZED = (
    "Instance's physics tensor entity is not initialized. Play the simulation/timeline to initialize it"
)
_MSG_PHYSICS_TENSOR_ENTITY_NOT_VALID = (
    "Instance's physics tensor entity is not valid. Play the simulation/timeline to re-initialize it"
)


class Prim(ABC):
    """Base wrapper class to manage USD prims.

    Creates a wrapper over one or more USD prims in the stage.
    The prims are specified using paths that can include regular expressions for matching multiple prims.

    Args:
        paths: Single path or list of paths to USD prims. Can include regular expressions for matching multiple prims.
        resolve_paths: Whether to resolve the given paths (true) or use them as is (false).

    Raises:
        ValueError: If no prims are found matching the specified path(s).
        AssertionError: If specified paths correspond to non-existing prims.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.prims import Prim
        >>>
        >>> # given a USD stage with the prims: /World/prim_0, /World/prim_1, and /World/prim_2
        >>> # - create wrapper over single prim
        >>> prim = Prim("/World/prim_0")  # doctest: +NO_CHECK
        >>> # - create wrapper over multiple prims using regex
        >>> prims = Prim("/World/prim_.*")  # doctest: +NO_CHECK
    """

    def __init__(self, paths: str | list[str], *, resolve_paths: bool = True) -> None:
        self._is_valid = True
        self._device = wp.get_device(SimulationManager.get_physics_sim_device())
        # get prim paths
        self._raw_paths = [paths] if isinstance(paths, str) else paths
        if resolve_paths:
            self._paths, nonexistent_paths = self.resolve_paths(self._raw_paths)
            assert (
                not nonexistent_paths
            ), f"Specified paths must correspond to existing prims: {', '.join(nonexistent_paths)}"
        else:
            self._paths = self._raw_paths
        # get prims
        stage = stage_utils.get_current_stage(backend="usd")
        self._prims = [stage.GetPrimAtPath(path) for path in self._paths]
        # register internal callbacks
        self._callback_ids = getattr(self, "_callback_ids", [])  # avoid attribute overwriting in multiple inheritance
        self._callback_ids.extend(
            [
                SimulationManager.register_callback(self._on_physics_ready, event=IsaacEvents.PHYSICS_READY),
                SimulationManager.register_callback(self._on_prim_deletion, event=IsaacEvents.PRIM_DELETION),
            ]
        )

    def __del__(self) -> None:
        """Clean up instance by deregistering callbacks and resetting internal state."""
        # deregister callbacks
        self._deregister_callbacks()
        # invalidate prim
        self._is_valid = False
        # reset internal properties
        self._device = None
        self._raw_paths = []
        self._prims = []
        self._paths = []

    def __len__(self) -> int:
        """Get the number of prims encapsulated by the wrapper.

        Returns:
            Number of prims in the wrapper.

        Example:

        .. code-block:: python

            >>> len(prims)
            3
        """
        return len(self._paths)

    """
    Properties.
    """

    @property
    def paths(self) -> list[str]:
        """Prim paths in the stage encapsulated by the wrapper.

        Returns:
            List of prim paths as strings.

        Example:

        .. code-block:: python

            >>> prims.paths
            ['/World/prim_0', '/World/prim_1', '/World/prim_2']
        """
        return self._paths

    @property
    def prims(self) -> list[Usd.Prim]:
        """USD Prim objects encapsulated by the wrapper.

        Returns:
            List of USD Prim objects.

        Example:

        .. code-block:: python

            >>> prims.prims
            [Usd.Prim(</World/prim_0>), Usd.Prim(</World/prim_1>), Usd.Prim(</World/prim_2>)]
        """
        return self._prims

    @property
    def valid(self) -> bool:
        """Whether all prims in the wrapper are valid.

        Returns:
            True if all prim paths specified in the wrapper correspond to valid prims in stage, False otherwise.

        Example:

        .. code-block:: python

            >>> prims.valid
            True
        """
        return self._is_valid

    """
    Static methods.
    """

    @staticmethod
    def ensure_api(prims: list[Usd.Prim], api: type, *args: Any, **kwargs: Any) -> list[type["UsdAPISchemaBase"]]:
        """Ensure that all prims have the specified API schema applied.

        Backends: :guilabel:`usd`.

        If a prim doesn't have the API schema, it will be applied.
        If it already has it, the existing API schema will be returned.

        Args:
            prims: List of USD Prims to ensure API schema on.
            api: The API schema type to ensure.
            *args: Additional positional arguments passed to API schema when applying it.
            **kwargs: Additional keyword arguments passed to API schema when applying it.

        Returns:
            List of API schema objects, one for each input prim.

        Example:

        .. code-block:: python

            >>> import isaacsim.core.experimental.utils.prim as prim_utils
            >>> from pxr import UsdPhysics
            >>> from isaacsim.core.experimental.prims import Prim
            >>>
            >>> # given a USD stage with 3 prims at paths /World/prim_0, /World/prim_1, /World/prim_2,
            >>> # ensure all prims have physics API schema
            >>> usd_prims = [prim_utils.get_prim_at_path(f"/World/prim_{i}") for i in range(3)]
            >>> physics_apis = Prim.ensure_api(usd_prims, UsdPhysics.RigidBodyAPI)
        """
        return [api(prim, *args, **kwargs) if prim.HasAPI(api) else api.Apply(prim, *args, **kwargs) for prim in prims]

    @staticmethod
    def resolve_paths(paths: str | list[str], raise_on_mixed_paths: bool = True) -> tuple[list[str], list[str]]:
        """Resolve paths to prims in the stage to get existing and non-existing paths.

        Backends: :guilabel:`usd`.

        Args:
            paths: Single path or list of paths to USD prims. Paths may contain regular expressions to match multiple prims.
            raise_on_mixed_paths: Whether to raise an error if resulting paths are mixed or invalid.

        Returns:
            Two-elements tuple. 1) List of existing paths. 2) List of non-existing paths.

        Raises:
            ValueError: If resulting paths are mixed or invalid and ``raise_on_mixed_paths`` is True.
        """
        existing_paths, nonexistent_paths, invalid_paths = [], [], []
        paths = [paths] if isinstance(paths, str) else paths
        for path in paths:
            result = prim_utils.find_matching_prim_paths(path)
            # existing paths, it could be a regex or a single path
            if result:
                existing_paths.append(result)
            # non-existing paths, check for valid path string
            elif Sdf.Path.IsValidPathString(path):
                nonexistent_paths.append(path)
            # invalid paths (for logging)
            else:
                invalid_paths.append(path)
        # validate results
        if raise_on_mixed_paths:
            if existing_paths and nonexistent_paths:
                raise ValueError(
                    "Specified paths include both existing and non-existing prims.\n"
                    "Ensure all the paths correspond to existing or non-existing prims only.\n"
                    f"Given paths: {paths}\n"
                    f"Existing paths: {existing_paths}\n"
                    f"Non-existing paths: {nonexistent_paths}\n"
                )
            elif not existing_paths and not nonexistent_paths:
                raise ValueError(
                    "Specified paths are invalid. Possible reasons:\n"
                    " - A regex is used but the stage does not contain any prims matching the regex\n"
                    " - A specified path is malformed or contains invalid characters\n"
                    f"Given paths: {paths}\n"
                    f"Invalid paths: {invalid_paths}\n"
                )
            elif existing_paths and len(existing_paths) != len(paths):
                raise ValueError(
                    "Specified paths are invalid. Possible reasons:\n"
                    " - A regex is used but the stage does not contain any prims matching the regex\n"
                    " - A specified path is malformed or contains invalid characters\n"
                    f"Given paths: {paths}\n"
                    f"Existing paths: {existing_paths}\n"
                    f"Invalid paths: {invalid_paths}\n"
                )
            elif nonexistent_paths and len(nonexistent_paths) != len(paths):
                raise ValueError(
                    "Specified paths are invalid. Possible reasons:\n"
                    " - A regex is used but the stage does not contain any prims matching the regex\n"
                    " - A specified path is malformed or contains invalid characters\n"
                    f"Given paths: {paths}\n"
                    f"Non-existing paths: {nonexistent_paths}\n"
                    f"Invalid paths: {invalid_paths}\n"
                )
        existing_paths = [path for paths in existing_paths for path in paths]
        return existing_paths, nonexistent_paths

    """
    Internal methods.
    """

    def _deregister_callbacks(self) -> None:
        """Deregister all internal callbacks."""
        for callback_id in getattr(self, "_callback_ids", []):
            SimulationManager.deregister_callback(callback_id)
        self._callback_ids = []

    """
    Internal callbacks.
    """

    def _on_physics_ready(self, event: object) -> None:
        """Handle physics ready event.

        Args:
            event: The physics ready event.
        """
        self._device = wp.get_device(SimulationManager.get_physics_sim_device())

    def _on_prim_deletion(self, prim_path: str) -> None:
        """Handle prim deletion event.

        Args:
            prim_path: Path of the deleted prim.
        """
        if prim_path == "/":
            self._deregister_callbacks()
            self._is_valid = False
            return
        for path in self._raw_paths:
            if re.match(pattern=f'^{"/".join(path.split("/")[: prim_path.count("/") + 1])}$', string=prim_path):
                self._deregister_callbacks()
                self._is_valid = False
                return
