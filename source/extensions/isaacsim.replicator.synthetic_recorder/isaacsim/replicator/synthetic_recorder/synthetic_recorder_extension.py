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

"""Extension for recording synthetic data in Isaac Sim through the Replicator framework."""

import omni.ext
from omni.kit.menu.utils import MenuHelperExtensionFull

from .synthetic_recorder_window import SyntheticRecorderWindow


class SyntheticRecorderExtension(omni.ext.IExt, MenuHelperExtensionFull):
    """Extension for recording synthetic data in Isaac Sim.

    This extension provides a user interface for recording synthetic sensor data and annotations
    through the Replicator framework. It adds a "Synthetic Data Recorder" window accessible
    from the Tools/Replicator menu, enabling users to configure and manage synthetic data
    recording sessions for machine learning workflows.
    """

    WINDOW_NAME = "Synthetic Data Recorder"
    """The name displayed in the window title bar."""
    MENU_GROUP = "Tools/Replicator"
    """The menu group path where the extension appears in the application menu."""

    def on_startup(self, ext_id: str) -> None:
        """Called when the Synthetic Data Recorder extension is starting up.

        Adds the Synthetic Data Recorder window to the Tools/Replicator menu group.

        Args:
            ext_id: The extension identifier.
        """
        # Add the menu item
        self.menu_startup(
            lambda: SyntheticRecorderWindow(SyntheticRecorderExtension.WINDOW_NAME, ext_id),
            SyntheticRecorderExtension.WINDOW_NAME,
            SyntheticRecorderExtension.WINDOW_NAME,
            SyntheticRecorderExtension.MENU_GROUP,
        )

    def on_shutdown(self) -> None:
        """Called when the Synthetic Data Recorder extension is shutting down.

        Removes the menu items created during startup.
        """
        self.menu_shutdown()
