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

"""Extension entry point for isaacsim.replicator.teleop."""

import omni.ext

from ._backend import reset_teleop_backend


class Extension(omni.ext.IExt):
    """Isaac Sim teleop extension entry point."""

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension when it is loaded.

        Args:
            ext_id: Extension identifier provided by the extension manager.
        """

    def on_shutdown(self) -> None:
        """Clean up resources when the extension is unloaded."""
        reset_teleop_backend()
