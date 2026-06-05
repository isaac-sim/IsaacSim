# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Extension module for Isaac Sim core rendering management capabilities including rendering events and viewport management."""

import omni.ext

from .rendering_manager import RenderingManager


class Extension(omni.ext.IExt):
    """Extension class for the isaacsim.core.rendering_manager extension.

    This extension provides core rendering management capabilities for Isaac Sim, including
    rendering event handling and viewport management functionality. It makes the RenderingManager,
    ViewportManager, and RenderingEvent classes available for managing rendering operations and
    viewport interactions in Isaac Sim applications.
    """

    def on_startup(self, ext_id: str) -> None:
        """Called when the extension starts up.

        Args:
            ext_id: The extension identifier.
        """

    def on_shutdown(self) -> None:
        """Called when the extension shuts down.

        Deregisters all rendering callbacks to clean up resources.
        """
        RenderingManager.deregister_all_callbacks()
