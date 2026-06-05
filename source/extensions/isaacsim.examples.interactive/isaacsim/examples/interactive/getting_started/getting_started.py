# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Basic example demonstrating the fundamental structure and lifecycle of Isaac Sim samples."""

from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.examples.base.base_sample_experimental import BaseSample


class GettingStarted(BaseSample):
    """A basic example that demonstrates the fundamental structure of Isaac Sim samples.

    This class serves as an entry-level tutorial for understanding how to create and structure Isaac Sim examples.
    It inherits from BaseSample and provides the minimal implementation required for a functional sample, making it
    an ideal starting point for users new to Isaac Sim development.

    The sample sets up a simple scene with a default camera view positioned to provide a clear perspective of the
    scene origin. All lifecycle methods are implemented but left empty, allowing users to understand the sample
    framework without complex scene setup or physics interactions.

    This example is particularly useful for:
    - Understanding the BaseSample class structure and required methods
    - Learning the sample lifecycle (setup_scene, setup_post_load, etc.)
    - Serving as a template for creating new custom samples
    - Getting familiar with basic camera positioning and viewport management
    """

    def __init__(self) -> None:
        super().__init__()

    def setup_scene(self) -> None:
        """Sets up the scene for the getting started sample."""

    async def setup_post_load(self) -> None:
        """Sets up the scene after loading by configuring the camera view position."""
        ViewportManager.set_camera_view(eye=[5.0, 2.0, 2.5], target=[0.00, 0.00, 0.00], camera="/OmniverseKit_Persp")

    async def setup_pre_reset(self) -> None:
        """Performs setup tasks before the scene is reset."""

    async def setup_post_reset(self) -> None:
        """Performs setup tasks after the scene is reset."""

    async def setup_post_clear(self) -> None:
        """Performs setup tasks after the scene is cleared."""

    def physics_cleanup(self) -> None:
        """Cleans up physics-related resources."""
