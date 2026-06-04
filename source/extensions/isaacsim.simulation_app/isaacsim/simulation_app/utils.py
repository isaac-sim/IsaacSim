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

"""Utility functions for SimulationApp including settings, stage, and livesync operations."""

from __future__ import annotations

from typing import Any

import carb
from omni.kit.usd import layers


def set_carb_setting(carb_settings: carb.settings.ISettings, setting: str, value: Any) -> None:
    """Convenience function to set settings.

    Arguments:
        carb_settings: Carb settings interface.
        setting: Name of setting to change.
        value: New value for the setting.

    Raises:
        TypeError: If the type of value does not match setting type.
    """
    if isinstance(value, str):
        carb_settings.set_string(setting, value)
    elif isinstance(value, bool):
        carb_settings.set_bool(setting, value)
    elif isinstance(value, int):
        carb_settings.set_int(setting, value)
    elif isinstance(value, float):
        carb_settings.set_float(setting, value)
    else:
        raise TypeError(f"Value of type {type(value)} is not supported.")


def open_stage(usd_path: str) -> bool:
    """Open the given usd file and replace currently opened stage.

    Args:
        usd_path: Path to open.

    Returns:
        True if the stage was opened successfully, False otherwise.
    """
    import omni.usd
    from pxr import Usd

    if not Usd.Stage.IsSupportedFile(usd_path):
        raise ValueError("Only USD files can be loaded with this method")
    usd_context = omni.usd.get_context()
    usd_context.disable_save_to_recent_files()
    result = omni.usd.get_context().open_stage(usd_path)
    usd_context.enable_save_to_recent_files()
    return result


def create_new_stage() -> Usd.Stage:
    """Create a new empty USD stage.

    Returns:
        The newly created USD stage.
    """
    import omni.usd

    return omni.usd.get_context().new_stage()


def save_stage(usd_path: str) -> bool:
    """Save usd file to path, it will be overwritten with the current stage.

    Args:
        usd_path: Path to save the current stage to.

    Returns:
        True if the stage was saved successfully, False otherwise.
    """
    import omni.usd
    from pxr import Usd

    if not Usd.Stage.IsSupportedFile(usd_path):
        raise ValueError("Only USD files can be saved with this method")
    result = omni.usd.get_context().save_as_stage(usd_path)
    return result


def set_livesync_stage(usd_path: str, enable: bool) -> bool:
    """Enable or disable live sync for a USD stage.

    Args:
        usd_path: Path to enable live sync for, will be overwritten with the current stage.
        enable: True to enable livesync, False to disable livesync.

    Returns:
        True if the operation succeeded, False otherwise.
    """
    # TODO: Check that the provided usd_path exists
    if save_stage(usd_path):
        if enable:
            usd_path_split = usd_path.split("/")
            live_session = layers.get_live_syncing().find_live_session_by_name(usd_path, "Default")
            if live_session is None:
                live_session = layers.get_live_syncing().create_live_session(name="Default")
            result = layers.get_live_syncing().join_live_session(live_session)
            return True
        else:
            layers.get_live_syncing().stop_live_session(usd_path)
            return True
    else:
        return False


def is_stage_loading() -> bool:
    """Convenience function to see if any files are being loaded.

    Returns:
        True if loading, False otherwise.
    """
    import omni.usd

    context = omni.usd.get_context()
    if context is None:
        return False
    else:
        _, _, loading = context.get_stage_loading_status()
        return loading > 0
