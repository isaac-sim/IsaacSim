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

"""Isaac Sim Occupancy Map extension that provides core functionality for generating 2D and 3D occupancy maps from USD stages."""

import omni.ext

from ..bindings import _omap


class Extension(omni.ext.IExt):
    """Isaac Sim Occupancy Map extension.

    This extension provides the core functionality for generating 2D and 3D occupancy maps
    from USD stages. It initializes the occupancy map interface on startup and ensures
    proper cleanup on shutdown.

    The extension is used in conjunction with the UI extension (isaacsim.asset.gen.omap.ui)
    to provide a complete occupancy map generation workflow.
    """

    def on_startup(self, ext_id: str) -> None:
        """Called when the extension is enabled.

        Acquires the occupancy map interface which provides access to the C++ backend
        for occupancy map generation.

        Args:
            ext_id: The unique identifier for this extension instance.
        """
        self._interface = _omap.acquire_omap_interface()

    def on_shutdown(self) -> None:
        """Called when the extension is disabled.

        Releases the occupancy map interface and cleans up any resources.
        """
        _omap.release_omap_interface(self._interface)
