# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Omniverse extension entry point for physics sensor OmniGraph nodes.

This extension provides OmniGraph nodes; sensor backends and lifecycle
are in isaacsim.sensors.experimental.physics.
"""

__all__ = []

import omni

from ..bindings._physics_sensor_nodes import acquire_interface, release_interface


class Extension(omni.ext.IExt):
    """Omniverse extension for physics sensor OmniGraph nodes.

    No startup/shutdown work required; nodes use backends from
    isaacsim.sensors.experimental.physics.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension when it is loaded.

        Args:
            ext_id: Extension identifier assigned by Omniverse.
        """
        self._interface = None

        # Acquire the plugin interface so OGN node registration lifetime matches extension lifetime.
        self._interface = acquire_interface()

    def on_shutdown(self) -> None:
        """Clean up when the extension is unloaded."""
        if self._interface is not None:
            release_interface(self._interface)
            self._interface = None
