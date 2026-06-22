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

"""Verifies contact sensor OmniGraph nodes for valid and invalid inputs, stop-play lifecycle behavior, output reset, and nonzero readings on an Ant scene."""

import asyncio
from typing import Any

import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import omni.kit.test
import omni.timeline
import usdrt.Sdf
from isaacsim.core.experimental.objects import Cube, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.experimental.physics import Contact
from pxr import PhysxSchema

from .common import setup_ant_scene, step_simulation


class TestContactSensorOgn(omni.kit.test.AsyncTestCase):
    """Exercise the contact sensor read node on a simple cube-ground scene."""

    async def setUp(self) -> None:
        """Create the cube contact sensor scene and its OmniGraph reader node."""
        await stage_utils.create_new_stage_async()
        physics_rate = 60
        SimulationManager.setup_simulation(dt=1.0 / physics_rate)
        self._timeline = omni.timeline.get_timeline_interface()
        await self.setup_environment()
        await self.setup_ogn()

    async def tearDown(self) -> None:
        """Stop playback, invalidate physics, and wait for stage loading to finish."""
        if self._timeline.is_playing():
            self._timeline.stop()
        SimulationManager.invalidate_physics()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            # print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()

    async def setup_environment(self) -> None:
        """Create a falling cube with collision, mass, contact report API, and contact sensor."""
        GroundPlane("/World/GroundPlane", positions=[0.0, 0.0, 0.0])
        Cube("/World/Cube", sizes=1.0, positions=[0.0, 0.0, 1.0])
        GeomPrim("/World/Cube", apply_collision_apis=True)
        RigidPrim("/World/Cube", masses=[1.0])
        contact_report_api = PhysxSchema.PhysxContactReportAPI.Apply(prim_utils.get_prim_at_path("/World/Cube"))
        contact_report_api.CreateThresholdAttr().Set(0)
        Contact.create(
            "/World/Cube/contact_sensor",
            max_threshold=10000000,
        )

    async def setup_ogn(self) -> None:
        """Create an OnPlaybackTick-driven IsaacReadContactSensor graph for the cube sensor."""
        self.graph_path = "/TestGraph"
        self.prim_path = "/World/Cube/contact_sensor"

        if prim_utils.get_prim_at_path(self.graph_path).IsValid():
            stage_utils.delete_prim(self.graph_path)

        keys = og.Controller.Keys
        try:
            og.Controller.edit(
                {"graph_path": self.graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadContactNode", "isaacsim.sensors.physics.IsaacReadContactSensor"),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ReadContactNode.inputs:execIn"),
                    ],
                    keys.SET_VALUES: [
                        (
                            "ReadContactNode.inputs:csPrim",
                            [usdrt.Sdf.Path("/World/Cube/contact_sensor")],
                        ),
                    ],
                },
            )
        except Exception as e:
            print(e)

    # verifying force value and sensor time are non-zero in valid case
    async def test_valid_contact_sensor_ogn(self) -> None:
        """Verify non-zero outputs for a valid contact sensor prim."""
        # must play, stop, and play simulation for force value to be properly recorded
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        await step_simulation(1.0)

        force_value = og.Controller.attribute(self.graph_path + "/ReadContactNode.outputs:value").get()
        self.assertNotEqual(force_value, 0.0)

        sensor_time = og.Controller.attribute(self.graph_path + "/ReadContactNode.outputs:sensorTime").get()
        self.assertNotEqual(sensor_time, 0.0)

    async def test_contact_sensor_stop_play_lifecycle(self) -> None:
        """Outputs recover valid data after a stop/play cycle."""
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await step_simulation(1.0)

        force_value = og.Controller.attribute(self.graph_path + "/ReadContactNode.outputs:value").get()
        self.assertNotEqual(force_value, 0.0, "Should have non-zero force before stop")

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await step_simulation(1.0)

        force_after = og.Controller.attribute(self.graph_path + "/ReadContactNode.outputs:value").get()
        self.assertNotEqual(force_after, 0.0, "Should recover valid force after stop/play")

    async def test_invalid_contact_sensor_ogn(self) -> None:
        """Verify outputs are zero for an invalid contact sensor prim."""
        og.Controller.set(
            og.Controller.attribute(self.graph_path + "/ReadContactNode.inputs:csPrim"), [usdrt.Sdf.Path("/World/Cube")]
        )
        self._timeline.play()
        await step_simulation(0.5)
        force_val = og.Controller.attribute(self.graph_path + "/ReadContactNode.outputs:value").get()
        self.assertEqual(force_val, 0.0)

        sensor_time = og.Controller.attribute(self.graph_path + "/ReadContactNode.outputs:sensorTime").get()
        self.assertEqual(sensor_time, 0.0)


class TestContactSensorOgnWithAnt(omni.kit.test.AsyncTestCase):
    """OGN contact sensor tests using the ant robot scene."""

    async def setUp(self) -> None:
        """Load the Ant scene used by contact sensor node output-reset tests."""
        self._physics_rate = 60
        self._timeline = omni.timeline.get_timeline_interface()
        self._ant_config = await setup_ant_scene(self._physics_rate)
        self._stage = stage_utils.get_current_stage()

    async def tearDown(self) -> None:
        """Stop playback and clear physics state after Ant contact node tests."""
        await omni.kit.app.get_app().next_update_async()
        if self._timeline.is_playing():
            self._timeline.stop()
        SimulationManager.invalidate_physics()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()

    @property
    def leg_paths(self) -> Any:
        """Return Ant lower-arm link paths used as contact sensor parents."""
        return self._ant_config.leg_paths

    @property
    def sensor_offsets(self) -> Any:
        """Return per-leg contact sensor translation offsets for the Ant scene."""
        return self._ant_config.sensor_offsets

    @property
    def color(self) -> Any:
        """Return the per-leg contact sensor debug colors for the Ant scene."""
        return self._ant_config.colors

    async def _add_sensor_prims(self) -> None:
        """Helper to add contact sensors to ant legs."""
        for i in range(4):
            sensor = Contact.create(
                f"{self.leg_paths[i]}/sensor",
                min_threshold=0,
                max_threshold=10000000,
                color=self.color[i],
                radius=0.12,
                translations=self.sensor_offsets[i],
            )
            self.assertIsNotNone(sensor)

    def _setup_contact_sensor_ogn_graph(self, sensor_path: str, graph_path: str = "/controller_graph") -> Any:
        """Create the OGN graph that reads contact sensor outputs.

        Args:
            sensor_path: Contact sensor prim path to read.
            graph_path: OmniGraph path to create.

        Returns:
            Created contact sensor reader node.
        """
        if self._stage.GetPrimAtPath(graph_path).IsValid():
            stage_utils.delete_prim(graph_path)

        _, (_, test_node), _, _ = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("IsaacReadContactSensor", "isaacsim.sensors.physics.IsaacReadContactSensor"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("IsaacReadContactSensor.inputs:csPrim", [usdrt.Sdf.Path(sensor_path)]),
                ],
                og.Controller.Keys.CONNECT: [("OnPlaybackTick.outputs:tick", "IsaacReadContactSensor.inputs:execIn")],
            },
        )
        return test_node

    async def test_node_outputs_reset(self) -> None:
        """Ensure OGN node outputs reset after playback stops."""
        sensor = Contact.create(
            f"{self.leg_paths[0]}/sensor",
            min_threshold=0,
            max_threshold=10000000,
            color=self.color[0],
            radius=0.12,
            translations=self.sensor_offsets[0],
        )
        self.assertIsNotNone(sensor)

        await omni.kit.app.get_app().next_update_async()

        test_node = self._setup_contact_sensor_ogn_graph(self.leg_paths[0] + "/sensor")

        await omni.kit.app.get_app().next_update_async()

        out_in_contact = og.Controller.attribute("outputs:inContact", test_node)
        out_value = og.Controller.attribute("outputs:value", test_node)

        first = True
        for i in range(5):
            self._timeline.play()

            await omni.kit.app.get_app().next_update_async()

            curr_in_contact = og.DataView.get(out_in_contact)
            curr_value = og.DataView.get(out_value)

            await omni.kit.app.get_app().next_update_async()

            if first:
                init_in_contact = curr_in_contact
                init_value = curr_value

            await step_simulation(1.0)

            self._timeline.stop()
            await omni.kit.app.get_app().next_update_async()

            if not first:
                self.assertEqual(init_in_contact, curr_in_contact)
                self.assertEqual(init_value, curr_value)
            first = False

        self._timeline.stop()

    async def test_node_nonzero_outputs(self) -> None:
        """Ensure OGN node reports non-zero outputs during contact."""
        await self._add_sensor_prims()
        await omni.kit.app.get_app().next_update_async()

        test_node = self._setup_contact_sensor_ogn_graph(self.leg_paths[0] + "/sensor")

        await omni.kit.app.get_app().next_update_async()

        out_in_contact = og.Controller.attribute("outputs:inContact", test_node)
        out_value = og.Controller.attribute("outputs:value", test_node)

        self._timeline.play()

        await step_simulation(1.0)

        for i in range(10):
            await omni.kit.app.get_app().next_update_async()
            self.assertNotEqual(og.DataView.get(out_in_contact), 0)
            self.assertNotEqual(og.DataView.get(out_value), 0)

        self._timeline.stop()
