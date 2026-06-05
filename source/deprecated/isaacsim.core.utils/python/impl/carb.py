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

# python

"""Deprecated Carbonite utility functions."""

from typing import Any

# omniverse
import carb


def set_carb_setting(carb_settings: carb.settings.ISettings, setting: str, value: Any) -> None:
    """Convenience to set the carb settings.

    Args:
        carb_settings: The interface to carb settings.
        setting: Name of setting to change.
        value: New value for the setting.

    Raises:
        TypeError: If the type of value does not match setting type.

    Example:

    .. code-block:: python

        >>> import carb
        >>> import isaacsim.core.utils.carb as carb_utils
        >>>
        >>> settings = carb.settings.get_settings()
        >>> carb_utils.set_carb_setting(settings, "/physics/updateToUsd", True)
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


def get_carb_setting(carb_settings: carb.settings.ISettings, setting: str) -> Any:
    """Convenience function to get settings.

    Args:
        carb_settings: The interface to carb settings.
        setting: Name of setting to change.

    Returns:
        Value for the setting.

    Example:

    .. code-block:: python

        >>> import carb
        >>> import isaacsim.core.utils.carb as carb_utils
        >>>
        >>> settings = carb.settings.get_settings()
        >>> carb_utils.get_carb_setting(settings, "/physics/updateToUsd")
        False
    """
    return carb_settings.get(setting)
