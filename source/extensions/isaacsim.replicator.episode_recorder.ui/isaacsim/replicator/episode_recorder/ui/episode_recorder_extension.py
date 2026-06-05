# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Extension entry point for the Episode Recorder UI window."""

import omni.ext
from omni.kit.menu.utils import MenuHelperExtensionFull, MenuItemOrder

from .episode_recorder_window import EpisodeRecorderWindow


class EpisodeRecorderUIExtension(omni.ext.IExt, MenuHelperExtensionFull):
    """Extension class that registers the Episode Recorder menu entry."""

    WINDOW_NAME = "Episode Recorder"
    MENU_GROUP = "Tools/Replicator"

    def on_startup(self, ext_id: str) -> None:
        self.menu_startup(
            lambda: EpisodeRecorderWindow(title=self.WINDOW_NAME),
            self.WINDOW_NAME,
            self.WINDOW_NAME,
            self.MENU_GROUP,
            argv={"appear_after": ["Teleop", "Synthetic Data Recorder", MenuItemOrder.LAST]},
        )

    def on_shutdown(self) -> None:
        self.menu_shutdown()
