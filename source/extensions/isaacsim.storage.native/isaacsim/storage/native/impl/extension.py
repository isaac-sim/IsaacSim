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

"""Extension for Omniverse client authentication in Isaac Sim storage."""

import os

import carb.settings
import omni.client
import omni.ext

ASSET_ROOT_ENV_VAR = "ISAACSIM_ASSET_ROOT"


class Extension(omni.ext.IExt):
    """Isaac Sim storage native extension for Omniverse client authentication and asset root configuration.

    This extension handles two concerns:
    1. Overrides the default asset root path from the ISAACSIM_ASSET_ROOT environment variable.
    2. Registers an authentication callback for the Omniverse client when ETM_ACTIVE is set.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension.

        Applies ISAACSIM_ASSET_ROOT env var override (if set), then registers an
        authentication callback if ETM_ACTIVE is set.

        Args:
            ext_id: The extension ID.
        """
        self._auth_cb = None

        asset_root = os.getenv(ASSET_ROOT_ENV_VAR)
        if asset_root:
            carb.settings.get_settings().set_string("/persistent/isaac/asset_root/default", asset_root.rstrip("/"))
            carb.log_info(f"Overriding asset root from {ASSET_ROOT_ENV_VAR}: {asset_root}")

        if os.getenv("ETM_ACTIVE"):
            self._auth_cb = omni.client.register_authentication_callback(self._authenticate)

    def _authenticate(self, prefix: str) -> tuple[str, str] | None:
        """Authentication callback for Omniverse client.

        Retrieves credentials from ISAACSIM_OMNI_USER and ISAACSIM_OMNI_PASS
        environment variables.

        Args:
            prefix: URL prefix for authentication.

        Returns:
            Tuple of (username, password) if credentials are available, None otherwise.
        """
        omniuser = os.getenv("ISAACSIM_OMNI_USER")
        omnipass = os.getenv("ISAACSIM_OMNI_PASS")

        if omniuser and omnipass:
            return (omniuser, omnipass)
        return None

    def on_shutdown(self) -> None:
        """Clean up resources when the extension is shut down."""
        self._auth_cb = None
