# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test physics fetch_results for deadlock detection."""

from isaacsim import SimulationApp

kit = SimulationApp()

import omni
import omni.physics.core
from pxr import PhysxSchema, UsdPhysics, UsdUtils

kit.update()

stage = omni.usd.get_context().get_stage()
stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())

kit.update()


def test_callback(step, context):
    """Print a message when the physics step callback fires."""
    print("callback")


print("Start test")
physics_sim_interface = omni.physics.core.get_physics_simulation_interface()
# Commenting out the following line will prevent the deadlock
physics_timer_callback = physics_sim_interface.subscribe_physics_on_step_events(
    pre_step=False, order=0, on_update=test_callback
)

# In Isaac Sim we run the following to "warm up" physics without simulating forward in time
physics_sim_interface.initialize(stage_id)
physics_sim_interface.simulate(1.0 / 60.0, 0.0)
print("Simulate Done")

print("Finish Test")
kit.update()
kit.close()  # Cleanup application
