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

import asyncio

import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import omni.kit.commands
import omni.kit.test
import omni.timeline
import usdrt.Sdf
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdPhysics

from .common import step_simulation


class TestEffortSensorOgn(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await stage_utils.create_new_stage_async()
        physics_rate = 60
        SimulationManager.setup_simulation(dt=1.0 / physics_rate)
        self._timeline = omni.timeline.get_timeline_interface()
        await self.setUp_environment()
        await self.setup_ogn()

    async def tearDown(self):
        if self._timeline.is_playing():
            self._timeline.stop()
        SimulationManager.invalidate_physics()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            # print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        pass

    async def setUp_environment(self):

        assets_root_path = get_assets_root_path()

        asset_path = assets_root_path + "/Isaac/Robots/IsaacSim/SimpleArticulation/simple_articulation.usd"
        stage_utils.add_reference_to_stage(usd_path=asset_path, path="/Articulation")
        arm_joint = "/Articulation/Arm/RevoluteJoint"
        arm_prim = prim_utils.get_prim_at_path(arm_joint)
        joint = UsdPhysics.RevoluteJoint(arm_prim)
        joint.CreateAxisAttr("Y")

    async def setup_ogn(self):
        self.graph_path = "/TestGraph"

        if prim_utils.get_prim_at_path(self.graph_path).IsValid():
            stage_utils.delete_prim(self.graph_path)

        keys = og.Controller.Keys
        try:
            og.Controller.edit(
                {"graph_path": self.graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadEffortNode", "isaacsim.sensors.physics.IsaacReadEffortSensor"),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ReadEffortNode.inputs:execIn"),
                    ],
                },
            )
        except Exception as e:
            print(e)

    # verifying force value and sensor time are non-zero in valid case
    async def test_valid_effort_sensor_ogn(self):
        og.Controller.set(
            og.Controller.attribute(self.graph_path + "/ReadEffortNode.inputs:prim"),
            [usdrt.Sdf.Path("/Articulation/Arm/RevoluteJoint")],
        )
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await step_simulation(0.5)
        effort_value = og.Controller.attribute(self.graph_path + "/ReadEffortNode.outputs:value").get()
        self.assertNotEqual(effort_value, 0.0)

        sensor_time = og.Controller.attribute(self.graph_path + "/ReadEffortNode.outputs:sensorTime").get()
        self.assertNotEqual(sensor_time, 0.0)

    async def test_invalid_effort_sensor_ogn(self):
        """Outputs remain zero when no prim is set."""
        self._timeline.play()
        await step_simulation(0.5)

        effort_value = og.Controller.attribute(self.graph_path + "/ReadEffortNode.outputs:value").get()
        self.assertEqual(effort_value, 0.0)

        sensor_time = og.Controller.attribute(self.graph_path + "/ReadEffortNode.outputs:sensorTime").get()
        self.assertEqual(sensor_time, 0.0)

    async def test_effort_sensor_stop_play_lifecycle(self):
        """Outputs reset to defaults after stop/play cycle and recover valid data."""
        og.Controller.set(
            og.Controller.attribute(self.graph_path + "/ReadEffortNode.inputs:prim"),
            [usdrt.Sdf.Path("/Articulation/Arm/RevoluteJoint")],
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await step_simulation(1.0)

        effort_value = og.Controller.attribute(self.graph_path + "/ReadEffortNode.outputs:value").get()
        self.assertNotEqual(effort_value, 0.0, "Should have non-zero effort before stop")

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await step_simulation(1.0)

        effort_value_after = og.Controller.attribute(self.graph_path + "/ReadEffortNode.outputs:value").get()
        self.assertNotEqual(effort_value_after, 0.0, "Should recover valid effort after stop/play")

        sensor_time_after = og.Controller.attribute(self.graph_path + "/ReadEffortNode.outputs:sensorTime").get()
        self.assertNotEqual(sensor_time_after, 0.0, "Should have non-zero time after stop/play")

    async def test_effort_sensor_deprecated_enabled_input_ignored(self):
        """Setting the deprecated 'enabled' input to False should not affect outputs."""
        og.Controller.set(
            og.Controller.attribute(self.graph_path + "/ReadEffortNode.inputs:prim"),
            [usdrt.Sdf.Path("/Articulation/Arm/RevoluteJoint")],
        )
        og.Controller.set(
            og.Controller.attribute(self.graph_path + "/ReadEffortNode.inputs:enabled"),
            False,
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await step_simulation(1.0)

        effort_value = og.Controller.attribute(self.graph_path + "/ReadEffortNode.outputs:value").get()
        self.assertNotEqual(effort_value, 0.0, "Deprecated enabled=False should be ignored")
