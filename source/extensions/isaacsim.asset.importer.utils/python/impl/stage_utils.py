# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Stage helpers for asset importer utilities."""

from pxr import Usd, UsdUtils

__all__ = [
    "save_stage",
    "open_stage",
    "get_stage_id",
]


def save_stage(stage: Usd.Stage, usd_path: str) -> bool:
    """Save a stage to a USD file.

    Backends: :guilabel:`usd`.

    Args:
        stage: USD stage to save.
        usd_path: USD file path to save the stage to.

    Returns:
        Whether the stage was saved successfully.

    Raises:
        ValueError: If the target path is not a supported USD file or the stage is invalid.

    Example:

    .. code-block:: python

        >>> import os
        >>> import tempfile
        >>> import isaacsim.asset.importer.utils.stage_utils as stage_utils
        >>>
        >>> # save the stage to a USD file
        >>> stage = Usd.Stage.CreateInMemory()
        >>> usd_path = os.path.join(tempfile.gettempdir(), "test.usd")
        >>> stage_utils.save_stage(stage, usd_path)
        True
    """
    if not Usd.Stage.IsSupportedFile(usd_path):
        raise ValueError(f"The file ({usd_path}) is not USD open-able")
    if stage is None:
        raise ValueError("Stage must be valid to save")
    result = stage.Export(usd_path)
    return result


def open_stage(usd_path: str) -> Usd.Stage:
    """Open a USD file as a standalone USD stage.

    Backends: :guilabel:`usd`.

    Args:
        usd_path: USD file path to open.

    Returns:
        Opened USD stage instance.

    Raises:
        ValueError: If the USD file does not exist or is not a valid (shallow check).

    Example:

    .. code-block:: python

        >>> import isaacsim.asset.importer.utils.stage_utils as stage_utils
        >>> from isaacsim.storage.native import get_assets_root_path
        >>>
        >>> # open a USD file
        >>> stage = stage_utils.open_stage(
        ...     get_assets_root_path() + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        ... )
        >>> stage
        Usd.Stage.Open(rootLayer=Sdf.Find('...'), ...)
    """
    if not Usd.Stage.IsSupportedFile(usd_path):
        raise ValueError(f"The file ({usd_path}) is not USD open-able")
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise ValueError(f"Failed to open stage: {usd_path}")
    return stage


def get_stage_id(stage: Usd.Stage) -> int:
    """Get the stage ID of a USD stage.

    Backends: :guilabel:`usd`.

    Args:
        stage: The stage to get the ID of.

    Returns:
        The stage ID.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>>
        >>> stage = stage_utils.get_current_stage()
        >>> stage_utils.get_stage_id(stage)  # doctest: +NO_CHECK
        9223006
    """
    stage_cache = UsdUtils.StageCache.Get()
    stage_id = stage_cache.GetId(stage).ToLongInt()
    if stage_id < 0:
        stage_id = stage_cache.Insert(stage).ToLongInt()
    return stage_id
