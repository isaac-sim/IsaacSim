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

"""Core URDF Exporter extension (API only, no UI).

UI components are provided by isaacsim.asset.exporter.urdf.ui.
"""

import omni.ext


class Extension(omni.ext.IExt):
    """Core URDF Exporter extension.

    Provides the USD-to-URDF conversion API without UI components.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize the URDF exporter extension."""

    def on_shutdown(self) -> None:
        """Clean up the URDF exporter extension."""
