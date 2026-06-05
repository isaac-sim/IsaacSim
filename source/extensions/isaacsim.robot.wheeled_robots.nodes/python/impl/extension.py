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

"""Extension module that provides OmniGraph nodes for wheeled robot controllers."""

from __future__ import annotations

import omni.ext
from isaacsim.robot.wheeled_robots.nodes.bindings._wheeled_robots_nodes import (
    acquire_interface,
    release_interface,
)


class Extension(omni.ext.IExt):
    """Extension class for the isaacsim.robot.wheeled_robots.nodes extension.

    Loads the native plugin to register OmniGraph nodes for wheeled robot controllers
    (differential, holonomic, Ackermann, path planning, etc.).
    """

    def on_startup(self, ext_id: str) -> None:
        """Start the extension by acquiring the native interface.

        Args:
            ext_id: The extension identifier.

        """
        self._interface = acquire_interface()

    def on_shutdown(self) -> None:
        """Shut down the extension by releasing the native interface."""
        release_interface(self._interface)
