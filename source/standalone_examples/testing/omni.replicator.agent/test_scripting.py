# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Verifies that the behavior scripting extension can apply the Omni scripting API schema to a newly created USD prim in a headless SimulationApp session."""

from isaacsim import SimulationApp

simulation_app = SimulationApp()

import isaacsim.core.experimental.utils.app as app_utils

simulation_app.update()

app_utils.enable_extension("omni.behavior.scripting.core")

import omni.usd
from omni.behavior.scripting.core import ApplyScriptingAPICommand
from pxr import OmniScriptingSchema


async def work() -> None:
    """Create a prim and verify the scripting API is applied."""
    # Create new prim and attach python scripting api.
    await omni.usd.get_context().new_stage_async("tmp")
    stage = omni.usd.get_context().get_stage()
    stage.DefinePrim("/test")
    ApplyScriptingAPICommand(paths=["/test"]).do()

    # Test
    prim = stage.GetPrimAtPath("/test")
    assert prim.HasAPI(OmniScriptingSchema.OmniScriptingAPI)


simulation_app.run_coroutine(work())

simulation_app.close()
