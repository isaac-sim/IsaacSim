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

"""Extension module for robot manipulator functionality in Isaac Sim."""

import omni.ext


class Extension(omni.ext.IExt):
    """Extension class for the isaacsim.robot.manipulators extension.

    This extension provides functionality for robot manipulator systems within Isaac Sim.
    It handles the initialization and management of manipulator-related components and services.
    """

    def on_startup(self, ext_id: str) -> None:
        """Called when the extension is starting up.

        Args:
            ext_id: The unique identifier of the extension being started.
        """

    def on_shutdown(self) -> None:
        """Called when the extension is shutting down."""
