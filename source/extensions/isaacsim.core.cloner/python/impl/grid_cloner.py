# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Grid cloner module."""

from __future__ import annotations

__all__ = ["GridCloner"]


import numpy as np
from isaacsim.core.cloner import Cloner
from pxr import Gf, Usd, UsdGeom


class GridCloner(Cloner):
    """A specialized Cloner class that automatically generates clones in a grid pattern.

    This class extends :class:`Cloner` to provide automatic grid-based positioning
    of clones, simplifying the creation of environments arranged in a regular grid.

    Args:
        spacing: Spacing between clones in the grid.
        num_per_row: Number of clones to place in a row. Defaults to sqrt(num_clones)
            if set to -1.
        stage: USD stage where source prim and clones are added to.
            Defaults to the current stage from the USD context.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.cloner import GridCloner
        >>>
        >>> cloner = GridCloner(spacing=2.0)
        >>> cloner.define_base_env("/World/envs")
        >>> prim_paths = cloner.generate_paths("/World/envs/env", 9)
        >>> positions = cloner.clone(
        ...     source_prim_path="/World/envs/env_0",
        ...     prim_paths=prim_paths,
        ... )

    .. code-block:: python

        >>> from isaacsim.core.cloner import GridCloner
        >>>
        >>> # Create a grid cloner with 2.0 spacing
        >>> cloner = GridCloner(spacing=2.0)
        >>>
        >>> # Create a grid cloner with 3 clones per row
        >>> cloner = GridCloner(spacing=1.5, num_per_row=3)
    """

    def __init__(self, spacing: float, num_per_row: int = -1, stage: Usd.Stage = None) -> None:
        self._spacing = spacing
        self._num_per_row = num_per_row

        self._positions = None
        self._orientations = None
        self._cached_num_clones = None

        Cloner.__init__(self, stage)

    def get_clone_transforms(
        self,
        num_clones: int,
        position_offsets: np.ndarray = None,
        orientation_offsets: np.ndarray = None,
    ) -> tuple[list, list]:
        """Compute the positions and orientations of clones in a grid.

        Args:
            num_clones: Number of clones.
            position_offsets: Positions to be applied as local translations on top of
                computed clone position. Defaults to None, no offset will be applied.
            orientation_offsets: Orientations to be applied as local rotations for each
                clone as quaternions (w, x, y, z). Defaults to None, no offset will be applied.

        Returns:
            A tuple containing:
                - positions: Computed positions of all clones.
                - orientations: Computed orientations of all clones as quaternions (w, x, y, z).

        Raises:
            ValueError: If the dimension of position_offsets does not match num_clones.
            ValueError: If the dimension of orientation_offsets does not match num_clones.

        Example:

        .. code-block:: python

            >>> import numpy as np
            >>> from isaacsim.core.cloner import GridCloner
            >>>
            >>> cloner = GridCloner(spacing=2.0)
            >>> positions, orientations = cloner.get_clone_transforms(num_clones=4)
            >>> len(positions)
            4
        """
        # check if inputs are valid
        if position_offsets is not None:
            if len(position_offsets) != num_clones:
                raise ValueError("Dimension mismatch between position_offsets and prim_paths!")
            # convert to numpy array
            # - convert from torch (without explicit importing it)
            try:
                position_offsets = position_offsets.detach().cpu().numpy()
            except Exception:
                pass
            # - convert from other types
            if not isinstance(position_offsets, np.ndarray):
                position_offsets = np.asarray(position_offsets)
        if orientation_offsets is not None:
            if len(orientation_offsets) != num_clones:
                raise ValueError("Dimension mismatch between orientation_offsets and prim_paths!")
            # convert to numpy array
            # - convert from torch (without explicit importing it)
            try:
                orientation_offsets = orientation_offsets.detach().cpu().numpy()
            except Exception:
                pass
            # - convert from other types
            if not isinstance(orientation_offsets, np.ndarray):
                orientation_offsets = np.asarray(orientation_offsets)

        if num_clones < 0:
            raise ValueError("num_clones must be non-negative")

        use_cache = position_offsets is None and orientation_offsets is None

        if (
            use_cache
            and self._positions is not None
            and self._orientations is not None
            and self._cached_num_clones == num_clones
        ):
            return self._positions, self._orientations

        if num_clones == 0:
            positions = []
            orientations = []
            if use_cache:
                self._positions = positions
                self._orientations = orientations
                self._cached_num_clones = num_clones
            return positions, orientations

        num_per_row = int(np.sqrt(num_clones)) if self._num_per_row == -1 else self._num_per_row
        if num_per_row <= 0:
            raise ValueError("num_per_row must be positive")

        num_rows = np.ceil(num_clones / num_per_row)
        num_cols = np.ceil(num_clones / num_rows)

        row_offset = 0.5 * self._spacing * (num_rows - 1)
        col_offset = 0.5 * self._spacing * (num_cols - 1)

        positions = []
        orientations = []

        for i in range(num_clones):
            # compute transform
            row = i // num_cols
            col = i % num_cols
            x = row_offset - row * self._spacing
            y = col * self._spacing - col_offset

            up_axis = UsdGeom.GetStageUpAxis(self._stage)
            position = [x, y, 0] if up_axis == UsdGeom.Tokens.z else [x, 0, y]
            orientation = Gf.Quatd.GetIdentity()

            if position_offsets is not None:
                translation = position_offsets[i] + position
            else:
                translation = position

            if orientation_offsets is not None:
                orientation = (
                    Gf.Quatd(orientation_offsets[i][0].item(), Gf.Vec3d(orientation_offsets[i][1:].tolist()))
                    * orientation
                )

            orientation = [
                orientation.GetReal(),
                orientation.GetImaginary()[0],
                orientation.GetImaginary()[1],
                orientation.GetImaginary()[2],
            ]

            positions.append(translation)
            orientations.append(orientation)

        if use_cache:
            self._positions = positions
            self._orientations = orientations
            self._cached_num_clones = num_clones

        return positions, orientations

    def clone(
        self,
        source_prim_path: str,
        prim_paths: list[str],
        position_offsets: np.ndarray = None,
        orientation_offsets: np.ndarray = None,
        replicate_physics: bool = False,
        base_env_path: str = None,
        root_path: str = None,
        copy_from_source: bool = False,
        enable_env_ids: bool = False,
        clone_in_fabric: bool = False,
    ) -> list:
        """Create clones in a grid pattern with automatically computed positions.

        Args:
            source_prim_path: Path of the source object.
            prim_paths: List of destination paths.
            position_offsets: Positions to be applied as local translations on top of
                computed clone position. Defaults to None, no offset will be applied.
            orientation_offsets: Orientations to be applied as local rotations for each
                clone as quaternions (w, x, y, z). Defaults to None, no offset will be applied.
            replicate_physics: Uses omni.physics replication. This will replicate physics
                properties directly for paths beginning with root_path and skip physics
                parsing for anything under the base_env_path.
            base_env_path: Path to namespace for all environments. Required if
                replicate_physics=True and define_base_env() was not called.
            root_path: Prefix path for each environment. Required if replicate_physics=True
                and generate_paths() was not called.
            copy_from_source: Setting this to False will inherit all clones from the source
                prim; any changes made to the source prim will be reflected in the clones.
                Setting this to True will make copies of the source prim when creating new
                clones; changes to the source prim will not be reflected in clones.
                Defaults to False. Note that setting this to True will take longer to execute.
            enable_env_ids: Setting this enables co-location of clones in physics with
                automatic filtering of collisions between clones.
            clone_in_fabric: Whether to perform cloning operations in Fabric for improved
                performance.

        Returns:
            Computed positions of all clones.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.cloner import GridCloner
            >>>
            >>> cloner = GridCloner(spacing=2.0)
            >>> cloner.define_base_env("/World/envs")
            >>> prim_paths = cloner.generate_paths("/World/envs/env", 9)
            >>> positions = cloner.clone(
            ...     source_prim_path="/World/envs/env_0",
            ...     prim_paths=prim_paths,
            ... )
            >>> len(positions)
            9
        """
        num_clones = len(prim_paths)

        positions, orientations = self.get_clone_transforms(num_clones, position_offsets, orientation_offsets)

        super().clone(
            source_prim_path=source_prim_path,
            prim_paths=prim_paths,
            positions=positions,
            orientations=orientations,
            replicate_physics=replicate_physics,
            base_env_path=base_env_path,
            root_path=root_path,
            copy_from_source=copy_from_source,
            enable_env_ids=enable_env_ids,
            clone_in_fabric=clone_in_fabric,
        )

        return positions
