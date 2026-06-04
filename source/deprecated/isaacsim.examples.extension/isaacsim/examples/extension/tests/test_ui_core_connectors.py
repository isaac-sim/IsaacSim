"""Tests for the UI core connectors (LoadButton, ResetButton)."""

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

# This import is included for visualization of UI elements as demonstrated in testXYPlotWrapper
import asyncio

import numpy as np
import omni.kit.test
import omni.kit.ui_test as ui_test
import omni.timeline
import omni.ui as ui
from isaacsim.core.api.objects.cuboid import FixedCuboid, VisualCuboid
from isaacsim.core.api.world import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import (
    add_reference_to_stage,
    create_new_stage,
    create_new_stage_async,
    update_stage_async,
)
from isaacsim.examples.extension.core_connectors import LoadButton, ResetButton
from isaacsim.storage.native import get_assets_root_path


async def wait_until_stage_loaded(timeout_s: float = 20.0) -> None:
    """Wait until the stage finishes loading or timeout is reached.

    Args:
        timeout_s: Maximum time in seconds to wait for the stage to load.
    """

    async def _wait() -> None:
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    await asyncio.wait_for(_wait(), timeout=timeout_s)


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestUICoreConnectors(omni.kit.test.AsyncTestCase):
    """Test cases for the UI core connector widgets."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up by creating a fresh stage and timeline."""
        World.clear_instance()
        self._timeline = omni.timeline.get_timeline_interface()
        await create_new_stage_async()
        await update_stage_async()

    # After running each test
    async def tearDown(self) -> None:
        """Tear down by stopping timeline and waiting for assets to load."""
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await update_stage_async()
        World.clear_instance()

    async def _create_window(self, title: str, width: int, height: int) -> object:
        window = ui.Window(
            title=title,
            width=width,
            height=height,
            visible=True,
            dockPreference=ui.DockPreference.LEFT_BOTTOM,
        )
        await update_stage_async()
        return window

    async def test_load_button(self) -> None:
        """Test that the LoadButton triggers setup_scene and setup_post_load callbacks."""
        window_title = "UI_Widget_Wrapper_Test_Window_LoadButton_Test"
        width = 500
        height = 200
        window = await self._create_window(window_title, width, height)

        self.setup_scene_called = False
        self.setup_post_load_called = False

        def setup_scene_fn() -> None:
            self.setup_scene_called = True

            robot_prim_path = "/ur10e"
            path_to_robot_usd = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"

            create_new_stage()
            add_reference_to_stage(path_to_robot_usd, robot_prim_path)

            art = SingleArticulation(robot_prim_path)
            cuboid = FixedCuboid(
                "/Scenario/cuboid", position=np.array([0.3, 0.3, 0.5]), size=0.05, color=np.array([255, 0, 0])
            )

            world = World.instance()
            world.scene.add(art)
            world.scene.add(cuboid)

        loaded = asyncio.Event()

        def setup_post_load_fn() -> None:
            self.setup_post_load_called = True
            loaded.set()

            # Assert that the timeline is paused when this callback is called
            self.assertFalse(self._timeline.is_playing() or self._timeline.is_stopped())

        with window.frame:
            load_button = LoadButton(
                "LoadButton", "LOAD", setup_scene_fn=setup_scene_fn, setup_post_load_fn=setup_post_load_fn
            )
        await update_stage_async()
        button = ui_test.find(f"{window_title}//Frame/Frame[0]/HStack[0]/Button[0]")
        await button.click()
        await update_stage_async()

        # Wait for asset streaming to finish with timeout, then await post-load completion deterministically
        await wait_until_stage_loaded(timeout_s=20.0)
        await asyncio.wait_for(loaded.wait(), timeout=10.0)

        self.assertTrue(self.setup_scene_called)
        self.assertTrue(self.setup_post_load_called)

    async def test_reset_button(self) -> None:
        """Test that the ResetButton triggers pre_reset and post_reset callbacks."""
        window_title = "UI_Widget_Wrapper_Test_Window_ResetButton_Test"
        width = 500
        height = 200
        window = await self._create_window(window_title, width, height)

        self.pre_reset_called = False
        self.post_reset_called = False

        def pre_reset_fn() -> None:
            self.pre_reset_called = True

            self.assertTrue(np.all(self.cuboid.get_world_pose()[0] == np.array([-1, 0, 0])))

        def post_reset_fn() -> None:
            self.post_reset_called = True

            # Assert that the timeline is paused when this callback is called
            self.assertFalse(self._timeline.is_playing() or self._timeline.is_stopped())

            self.assertTrue(np.all(self.cuboid.get_world_pose()[0] == np.array([1, 0, 0])))

        with window.frame:
            reset_button = ResetButton("ResetButton", "RESET", pre_reset_fn=pre_reset_fn, post_reset_fn=post_reset_fn)

        world = World()
        await world.initialize_simulation_context_async()

        self.cuboid = VisualCuboid("/cuboid", size=0.1, position=np.array([1, 0, 0]))
        world.scene.add(self.cuboid)

        self._timeline.play()
        await update_stage_async()
        self.cuboid.set_world_pose(np.array([-1, 0, 0]))

        button = ui_test.find(f"{window_title}//Frame/Frame[0]/HStack[0]/Button[0]")
        await button.click()
        await update_stage_async()

        # The ResetButton resets the Core World asynchronously, so it can take some time to get to the setup_post_load_fn
        await asyncio.sleep(0.25)

        self.assertTrue(self.pre_reset_called)
        self.assertTrue(self.post_reset_called)
