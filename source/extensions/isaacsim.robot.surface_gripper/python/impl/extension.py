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

"""Surface gripper extension implementation for robotic applications in Isaac Sim."""

import gc

import omni

from .. import _surface_gripper

EXTENSION_NAME = "Surface Gripper"


class Extension(omni.ext.IExt):
    """Extension class for the isaacsim.robot.surface_gripper extension.

    This extension provides surface gripper functionality for robotic applications in Isaac Sim.
    It manages the surface gripper interface, which enables robots to interact with objects
    through surface-based gripping mechanisms. The extension handles the acquisition and
    release of the surface gripper interface during the extension lifecycle.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initializes the Surface Gripper extension.

        Args:
            ext_id: The extension ID provided by the Omniverse Kit SDK.
        """
        self._sg = _surface_gripper.acquire_surface_gripper_interface()

    def on_shutdown(self) -> None:
        """Cleans up the Surface Gripper extension resources."""
        _surface_gripper.release_surface_gripper_interface(self._sg)

        gc.collect()
