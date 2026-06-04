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

"""Extension module that provides support for wheeled robot functionality in Isaac Sim.

.. deprecated::
    This extension is deprecated. Use :mod:`isaacsim.robot.experimental.wheeled_robots` for
    controllers and robots, and :mod:`isaacsim.robot.wheeled_robots.nodes` for OmniGraph nodes.
"""

import omni.ext
from isaacsim.robot.wheeled_robots.bindings._isaacsim_robot_wheeled_robots import (
    acquire_wheeled_robots_interface,
    release_wheeled_robots_interface,
)


class Extension(omni.ext.IExt):
    """Extension class for the deprecated isaacsim.robot.wheeled_robots extension."""

    def on_startup(self, ext_id: str) -> None:
        """Start up the wheeled robots extension by acquiring the wheeled robots interface.

        Args:
            ext_id: The extension identifier.
        """
        self.__interface = acquire_wheeled_robots_interface()

    def on_shutdown(self) -> None:
        """Shut down the wheeled robots extension by releasing the wheeled robots interface."""
        release_wheeled_robots_interface(self.__interface)
