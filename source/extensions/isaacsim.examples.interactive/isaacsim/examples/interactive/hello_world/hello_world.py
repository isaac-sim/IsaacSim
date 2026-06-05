# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""A basic Isaac Sim example module that demonstrates fundamental scene setup and lifecycle management."""

import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.examples.base.base_sample_experimental import BaseSample
from isaacsim.storage.native import get_assets_root_path

# Note: checkout the required tutorials at https://docs.isaacsim.omniverse.nvidia.com/latest/index.html


class HelloWorld(BaseSample):
    """A basic Isaac Sim example that demonstrates fundamental scene setup.

    This class serves as an introductory example for users learning Isaac Sim development. It inherits from BaseSample
    and implements the essential methods required for a complete Isaac Sim sample, including scene setup with a ground
    plane environment.

    The example creates a minimal physics-enabled scene with a grid-based ground plane, providing a foundation that
    can be extended with additional objects, robots, or simulation elements. It demonstrates the standard lifecycle
    methods used in Isaac Sim samples for initialization, loading, resetting, and cleanup operations.
    """

    def __init__(self) -> None:
        super().__init__()

    def setup_scene(self) -> None:
        """Set up the scene by adding a ground plane environment for physics simulation."""
        # Add ground plane environment for physics simulation
        ground_plane = stage_utils.add_reference_to_stage(
            usd_path=get_assets_root_path() + "/Isaac/Environments/Grid/default_environment.usd",
            path="/World/ground",
        )

    async def setup_post_load(self) -> None:
        """Set up operations to be performed after the world is loaded."""

    async def setup_pre_reset(self) -> None:
        """Set up operations to be performed before the world is reset."""

    async def setup_post_reset(self) -> None:
        """Set up operations to be performed after the world is reset."""

    def world_cleanup(self) -> None:
        """Clean up the world and release resources."""
