# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import asyncio

import carb
import numpy as np
import omni.kit.test

# from omni.kit.test_suite.helpers import StageEventHandler
import omni.kit.ui_test as ui_test
import omni.timeline
import omni.ui as ui


class TestConveyorBuilderUI(omni.kit.test.AsyncTestCase):
    async def setup(self):
        # wait for material to be preloaded so create menu is complete & menus don't rebuild during tests
        await omni.kit.material.library.get_mdl_list_async()
        await ui_test.human_delay()
        await omni.kit.app.get_app().next_update_async()

        # TODO: get omni.kit.test_suite.
        # self._stage_event_handler = StageEventHandler("omni.kit.stage_templates")

    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()

        pass

    async def test_loading(self):
        await omni.usd.get_context().new_stage_async()
        menu_widget = ui_test.get_menubar()
        try:
            await menu_widget.find_menu("Tools").click(human_delay_speed=50)
            await menu_widget.find_menu("Conveyor Track Builder").click(human_delay_speed=50)
        except:
            carb.log_warn("Could not run test because carb::windowing is not available")
