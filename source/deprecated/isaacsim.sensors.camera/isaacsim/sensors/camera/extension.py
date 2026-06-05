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

"""Extension providing camera sensor functionality for Isaac Sim simulation environment."""

import omni.ext

EXTENSION_NAME = "Isaac Sensor"


class Extension(omni.ext.IExt):
    """Extension class for the isaacsim.sensors.camera extension.

    This extension provides camera sensor functionality for Isaac Sim, enabling the creation and management
    of camera sensors within the simulation environment.
    """

    def on_startup(self, ext_id: str) -> None:
        """Called when the Isaac Sensor extension starts up.

        Args:
            ext_id: The extension identifier.

        """

    def on_shutdown(self) -> None:
        """Called when the Isaac Sensor extension shuts down."""
