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

"""Test for startup contract."""

import carb
import omni.kit.app
import omni.kit.test

_DEFAULT_PROVIDER = "isaacsim.core.experimental.primdata"
_PROVIDER_SETTING = "/exts/isaacsim.core.experimental.prims/prim_data_reader_provider_extension"


def _resolve_provider_name() -> str:
    """Resolve the provider extension name using the same logic as the implementation."""
    settings = carb.settings.get_settings()
    configured = settings.get(_PROVIDER_SETTING) if settings is not None else None
    if isinstance(configured, str) and configured.strip():
        return configured.strip()
    return _DEFAULT_PROVIDER


class TestStartupContract(omni.kit.test.AsyncTestCase):
    """Verify that isaacsim.core.experimental.prims eagerly loads the prim data reader provider on startup.

    This test runs in a dedicated [[test]] section for process-level isolation.
    It must never call get_prim_data_reader() or _ensure_provider_enabled() so
    that it only validates the on_startup eager-load path.
    """

    async def test_provider_extension_enabled_at_startup(self):
        """The resolved provider extension must be enabled by on_startup, not by on-demand loading."""
        provider = _resolve_provider_name()
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self.assertTrue(
            ext_manager.is_extension_enabled(provider),
            f"Provider extension '{provider}' should be enabled after isaacsim.core.experimental.prims startup",
        )

    async def test_manager_interface_acquirable_at_startup(self):
        """IPrimDataReaderManager must be registered and acquirable after on_startup."""
        from isaacsim.core.experimental.prims import _prims_reader

        manager = _prims_reader.acquire_prim_data_reader_manager_interface()
        self.assertIsNotNone(
            manager,
            "acquire_prim_data_reader_manager_interface() should return non-None after startup",
        )
        _prims_reader.release_prim_data_reader_manager_interface(manager)
