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

"""Extension module."""

from __future__ import annotations

import carb
import isaacsim.core.experimental.utils.app as app_utils
import omni.ext

from .. import _prims_reader

_reader_interface = None
_default_provider_extension = "isaacsim.core.experimental.primdata"
_provider_setting_path = "/exts/isaacsim.core.experimental.prims/prim_data_reader_provider_extension"


def _get_provider_extension_name() -> str:
    """Resolve the configured prim data reader provider extension name.

    Returns:
        The provider extension name.
    """
    settings = carb.settings.get_settings()
    configured_name = settings.get(_provider_setting_path) if settings is not None else None
    if isinstance(configured_name, str) and configured_name.strip():
        return configured_name.strip()
    return _default_provider_extension


def _ensure_provider_enabled() -> None:
    """Enable the configured provider extension if it is currently disabled."""
    provider_extension = _get_provider_extension_name()
    if not provider_extension:
        return
    try:
        if app_utils.is_extension_enabled(provider_extension):
            return
        app_utils.enable_extension(provider_extension)
    except Exception as error:
        carb.log_warn(f"Failed to enable prim data reader provider '{provider_extension}': {error}")


def _acquire_reader_interface() -> object | None:
    """Acquire and cache IPrimDataReader, enabling provider extension if needed.

    Returns:
        The acquired IPrimDataReader interface, or None if unavailable.
    """
    global _reader_interface
    if _reader_interface is not None:
        return _reader_interface

    _ensure_provider_enabled()
    _reader_interface = _prims_reader.acquire_prim_data_reader_interface()
    return _reader_interface


def get_prim_data_reader() -> object | None:
    """Get the IPrimDataReader Carbonite interface singleton.

    Returns:
        The acquired IPrimDataReader interface, or None if the extension has not started.
    """
    return _acquire_reader_interface()


class Extension(omni.ext.IExt):
    """Extension lifecycle handler for the C++ prim data reader plugin."""

    def on_startup(self, ext_id: str) -> None:
        """Acquire the IPrimDataReader Carbonite interface on extension load.

        Args:
            ext_id: The extension identifier.
        """
        _acquire_reader_interface()

    def on_shutdown(self) -> None:
        """Release the IPrimDataReader Carbonite interface on extension unload."""
        global _reader_interface
        if _reader_interface is not None:
            _prims_reader.release_prim_data_reader_interface(_reader_interface)
            _reader_interface = None
