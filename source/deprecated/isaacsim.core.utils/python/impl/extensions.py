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

"""Deprecated extension utility functions."""

import omni.kit.app


def get_extension_id(extension_name: str) -> str:
    """Get extension id for a loaded extension.

    Args:
        extension_name: name of the extension

    Returns:
        Full extension id

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.extensions as extensions_utils
        >>>
        >>> extensions_utils.get_extension_id("omni.kit.window.stage")
        omni.kit.window.stage-2.4.3
    """
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    return extension_manager.get_enabled_extension_id(extension_name)


def get_extension_path(ext_id: str) -> str:
    """Get extension path for a loaded extension by its full id.

    Args:
        ext_id: full id of extension

    Returns:
        Path to loaded extension root directory

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.extensions as extensions_utils
        >>>
        >>> ext_id = extensions_utils.get_extension_id("omni.kit.window.stage")
        >>> extensions_utils.get_extension_path(ext_id)
        /home/user/.local/share/ov/pkg/isaac_sim-<version>/kit/exts/omni.kit.window.stage
    """
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    return extension_manager.get_extension_path(ext_id)


def get_extension_path_from_name(extension_name: str) -> str:
    """Get extension path for a loaded extension by its name.

    Args:
        extension_name: name of the extension

    Returns:
        Path to loaded extension root directory

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.extensions as extensions_utils
        >>>
        >>> extensions_utils.get_extension_path_from_name("omni.kit.window.stage")
        /home/user/.local/share/ov/pkg/isaac_sim-<version>/kit/exts/omni.kit.window.stage
    """
    ext_id = get_extension_id(extension_name)
    if not ext_id:
        raise RuntimeError(f"Extension '{extension_name}' is not enabled or could not be found.")
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    return extension_manager.get_extension_path(ext_id)


def enable_extension(extension_name: str) -> bool:
    """Load an extension from the extension manager.

    Args:
        extension_name: name of the extension

    Returns:
        True if extension could be loaded, False otherwise

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.extensions as extensions_utils
        >>>
        >>> extensions_utils.enable_extension("omni.kit.window.stage")
        True
    """
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    return extension_manager.set_extension_enabled_immediate(extension_name, True)


def disable_extension(extension_name: str) -> bool:
    """Unload an extension.

    Args:
        extension_name: name of the extension

    Returns:
        True if extension could be unloaded, False otherwise

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.extensions as extensions_utils
        >>>
        >>> extensions_utils.disable_extension("omni.kit.window.stage")
        True
    """
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    return extension_manager.set_extension_enabled_immediate(extension_name, False)
