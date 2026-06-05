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

"""ROS 2 Bridge extension implementation."""

import carb
import omni.ext


class ROS2BridgeExtension(omni.ext.IExt):
    """ROS 2 Bridge Extension - Extension for ROS 2 integration.

    This extension brings together all ROS 2 extensions required for the ROS 2 bridge extension.
    """

    def on_startup(self, ext_id: str) -> None:
        """Called when the extension starts up.

        Logs the startup of the ROS 2 Bridge extension.

        Args:
            ext_id: The extension identifier.
        """
        carb.log_info("Starting ROS 2 Bridge extension")

    def on_shutdown(self) -> None:
        """Called when the extension shuts down.

        Logs the shutdown of the ROS 2 Bridge extension.
        """
        carb.log_info("Shutting down ROS 2 Bridge extension")
