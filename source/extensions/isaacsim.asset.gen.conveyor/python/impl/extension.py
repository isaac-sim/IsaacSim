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

"""Extension module for conveyor asset generation capabilities within Isaac Sim."""

__all__ = []

import omni.ext
from isaacsim.asset.gen.conveyor.bindings._isaacsim_asset_gen_conveyor import acquire_interface as _acquire
from isaacsim.asset.gen.conveyor.bindings._isaacsim_asset_gen_conveyor import release_interface as _release


class Extension(omni.ext.IExt):
    """Extension for the isaacsim.asset.gen.conveyor package.

    This extension provides conveyor asset generation capabilities within Isaac Sim. It manages the
    acquisition and release of the conveyor interface during the extension lifecycle.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initializes the extension by acquiring the conveyor asset generation interface.

        Args:
            ext_id: The extension identifier.
        """
        self.__interface = _acquire()

    def on_shutdown(self) -> None:
        """Cleans up the extension by releasing the conveyor asset generation interface."""
        _release(self.__interface)
        self.__interface = None
