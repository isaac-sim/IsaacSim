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

"""Extension that provides UCX integration capabilities for Isaac Sim."""

import carb
import omni.ext


class UCXBridgeExtension(omni.ext.IExt):
    """UCX Bridge Extension - Extension for UCX integration.

    This extension brings together all UCX extensions required for the UCX bridge extension.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize the UCX Bridge extension.

        Args:
            ext_id: Extension identifier provided by the extension manager.
        """
        carb.log_info("Starting UCX Bridge extension")

    def on_shutdown(self) -> None:
        """Clean up the UCX Bridge extension resources."""
        carb.log_info("Shutting down UCX Bridge extension")
