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

"""Interactive sample demonstrating gamepad control of a Kaya robot in Isaac Sim."""

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.usd
from isaacsim.examples.base.base_sample_experimental import BaseSample
from isaacsim.storage.native import get_assets_root_path
from pxr import Sdf


class KayaGamepad(BaseSample):
    """Interactive sample demonstrating gamepad control of a Kaya robot.

    This sample sets up a complete scene with a Kaya robot that can be controlled using a gamepad controller.
    The robot is loaded with pre-configured action graphs that handle gamepad input for movement control.
    The scene includes a ground plane environment and proper lighting for visualization.

    The sample creates:
    - A Kaya robot with gamepad controller integration
    - A default grid environment as the ground plane
    - A dome light for scene illumination

    This demonstrates how to create interactive robotic simulations where users can directly control
    robots using standard gaming controllers, making it useful for teleoperation scenarios,
    testing robot behaviors, and interactive demonstrations.
    """

    def __init__(self) -> None:
        super().__init__()
        self._kaya_prim_path = "/kaya"

    def setup_scene(self) -> None:
        """Sets up the simulation scene with Kaya robot, ground plane, and lighting.

        Loads the Kaya robot with gamepad controller, adds a ground plane environment,
        and configures dome lighting for the scene.
        """
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        # add kaya robot rigged with gamepad controller
        kaya_ogn_usd = assets_root_path + "/Isaac/Robots/NVIDIA/Kaya/kaya_ogn_gamepad.usd"
        stage = omni.usd.get_context().get_stage()
        graph_prim = stage.DefinePrim(self._kaya_prim_path, "Xform")
        graph_prim.GetReferences().AddReference(kaya_ogn_usd)

        # add ground plane using experimental API
        ground_plane = stage_utils.add_reference_to_stage(
            usd_path=get_assets_root_path() + "/Isaac/Environments/Grid/default_environment.usd",
            path="/World/ground_plane",
        )

        # add dome light
        dome_light = stage.DefinePrim("/World/DomeLight", "DomeLight")
        dome_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(450.0)

    async def setup_post_load(self) -> None:
        """Called after the scene is loaded."""

    async def setup_pre_reset(self) -> None:
        """Called before world reset."""

    async def setup_post_reset(self) -> None:
        """Called after world reset."""

    async def setup_post_clear(self) -> None:
        """Called after clearing the scene."""

    def physics_cleanup(self) -> None:
        """Function called when extension shutdowns and starts again, (hot reloading feature)."""
