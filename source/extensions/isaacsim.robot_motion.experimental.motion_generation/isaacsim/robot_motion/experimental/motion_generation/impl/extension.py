# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Kit lifecycle adapter for the motion generation library."""

import omni.ext

from ._logging import configure_logging_channel

_EXTENSION_LOGGING_CHANNEL = "isaacsim.robot_motion.experimental.motion_generation"


class Extension(omni.ext.IExt):
    """Configure the motion generation library for use as a Kit extension."""

    def on_startup(self, _ext_id: str) -> None:
        """Configure the extension-specific logging channel."""
        configure_logging_channel(_EXTENSION_LOGGING_CHANNEL)
