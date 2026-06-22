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

"""Verify ROS 2 QoS profile node output.

Covers default, custom, preset, reliability, history, durability, depth,
time-policy, JSON, and publisher-connected cases.
"""

import json
from typing import Any

import omni.graph.core as og
import omni.kit.test
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase

NODE_TYPE = "isaacsim.ros2.bridge.ROS2QoSProfile"


class TestROS2QoSProfile(ROS2TestCase):
    """Verify ROS 2 QoS profile JSON emitted by the QoSProfile node."""

    async def setUp(self) -> None:
        """Create a fresh stage for QoSProfile node evaluation."""
        await super().setUp()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Run shared ROS 2 cleanup after QoSProfile node tests."""
        await super().tearDown()

    async def _create_qos_graph(self, graph_path: Any, set_values: Any = None) -> Any:
        """Create a graph with a QoS Profile node.

        Args:
            graph_path: OmniGraph path to create.
            set_values: Optional node input values to set.

        Returns:
            Created graph and QoS node.
        """
        edit_spec = {
            og.Controller.Keys.CREATE_NODES: [
                ("QoSProfile", NODE_TYPE),
            ],
        }
        if set_values:
            edit_spec[og.Controller.Keys.SET_VALUES] = set_values

        graph, nodes, _, _ = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "push"},
            edit_spec,
        )
        return graph, nodes[0]

    async def _evaluate_and_get_output(self, graph: Any, qos_node: Any) -> Any:
        """Evaluate the graph and return the parsed QoS profile JSON from the node output.

        Args:
            graph: Graph to evaluate.
            qos_node: QoS node to read.

        Returns:
            Parsed QoS profile output.
        """
        await omni.kit.app.get_app().next_update_async()
        await og.Controller.evaluate(graph)
        await omni.kit.app.get_app().next_update_async()

        qos_json_str = og.Controller.attribute("outputs:qosProfile", qos_node).get()
        self.assertTrue(qos_json_str, "QoS profile output should not be empty")
        return json.loads(qos_json_str)

    async def test_default_profile_output(self) -> None:
        """Verify the node produces valid JSON output with default input values."""
        graph, qos_node = await self._create_qos_graph("/TestGraph")
        profile = await self._evaluate_and_get_output(graph, qos_node)

        self.assertEqual(profile["history"], "keepLast")
        self.assertEqual(profile["depth"], 10)
        self.assertEqual(profile["reliability"], "reliable")
        self.assertEqual(profile["durability"], "volatile")
        self.assertAlmostEqual(profile["deadline"], 0.0)
        self.assertAlmostEqual(profile["lifespan"], 0.0)
        self.assertEqual(profile["liveliness"], "systemDefault")
        self.assertAlmostEqual(profile["leaseDuration"], 0.0)

    async def test_output_json_has_all_required_fields(self) -> None:
        """Verify the output JSON contains all fields expected by downstream nodes."""
        graph, qos_node = await self._create_qos_graph("/TestGraph")
        profile = await self._evaluate_and_get_output(graph, qos_node)

        required_fields = [
            "history",
            "depth",
            "reliability",
            "durability",
            "deadline",
            "lifespan",
            "liveliness",
            "leaseDuration",
        ]
        for field in required_fields:
            self.assertIn(field, profile, f"Output JSON missing required field: {field}")

    async def test_custom_profile_values(self) -> None:
        """Verify custom QoS policy inputs are correctly serialized to JSON."""
        graph, qos_node = await self._create_qos_graph(
            "/TestGraph",
            set_values=[
                ("QoSProfile.inputs:history", "keepAll"),
                ("QoSProfile.inputs:depth", 20),
                ("QoSProfile.inputs:reliability", "bestEffort"),
                ("QoSProfile.inputs:durability", "transientLocal"),
                ("QoSProfile.inputs:deadline", 1.5),
                ("QoSProfile.inputs:lifespan", 2.0),
                ("QoSProfile.inputs:liveliness", "automatic"),
                ("QoSProfile.inputs:leaseDuration", 3.0),
                ("QoSProfile.inputs:createProfile", "Custom"),
            ],
        )
        profile = await self._evaluate_and_get_output(graph, qos_node)

        self.assertEqual(profile["history"], "keepAll")
        self.assertEqual(profile["depth"], 20)
        self.assertEqual(profile["reliability"], "bestEffort")
        self.assertEqual(profile["durability"], "transientLocal")
        self.assertAlmostEqual(profile["deadline"], 1.5)
        self.assertAlmostEqual(profile["lifespan"], 2.0)
        self.assertEqual(profile["liveliness"], "automatic")
        self.assertAlmostEqual(profile["leaseDuration"], 3.0)

    async def test_best_effort_reliability(self) -> None:
        """Verify bestEffort reliability is correctly serialized."""
        graph, qos_node = await self._create_qos_graph(
            "/TestGraph",
            set_values=[
                ("QoSProfile.inputs:reliability", "bestEffort"),
                ("QoSProfile.inputs:createProfile", "Custom"),
            ],
        )
        profile = await self._evaluate_and_get_output(graph, qos_node)

        self.assertEqual(profile["reliability"], "bestEffort")

    async def test_keep_all_history(self) -> None:
        """Verify keepAll history policy is correctly serialized."""
        graph, qos_node = await self._create_qos_graph(
            "/TestGraph",
            set_values=[
                ("QoSProfile.inputs:history", "keepAll"),
                ("QoSProfile.inputs:createProfile", "Custom"),
            ],
        )
        profile = await self._evaluate_and_get_output(graph, qos_node)

        self.assertEqual(profile["history"], "keepAll")

    async def test_transient_local_durability(self) -> None:
        """Verify transientLocal durability is correctly serialized."""
        graph, qos_node = await self._create_qos_graph(
            "/TestGraph",
            set_values=[
                ("QoSProfile.inputs:durability", "transientLocal"),
                ("QoSProfile.inputs:createProfile", "Custom"),
            ],
        )
        profile = await self._evaluate_and_get_output(graph, qos_node)

        self.assertEqual(profile["durability"], "transientLocal")

    async def test_nonzero_time_policies(self) -> None:
        """Verify nonzero deadline, lifespan, and leaseDuration values."""
        graph, qos_node = await self._create_qos_graph(
            "/TestGraph",
            set_values=[
                ("QoSProfile.inputs:deadline", 0.5),
                ("QoSProfile.inputs:lifespan", 1.0),
                ("QoSProfile.inputs:leaseDuration", 2.5),
                ("QoSProfile.inputs:createProfile", "Custom"),
            ],
        )
        profile = await self._evaluate_and_get_output(graph, qos_node)

        self.assertAlmostEqual(profile["deadline"], 0.5)
        self.assertAlmostEqual(profile["lifespan"], 1.0)
        self.assertAlmostEqual(profile["leaseDuration"], 2.5)

    async def test_preset_sensor_data_profile(self) -> None:
        """Verify selecting the Sensor Data preset applies expected policy values."""
        graph, qos_node = await self._create_qos_graph(
            "/TestGraph",
            set_values=[
                ("QoSProfile.inputs:createProfile", "Sensor Data"),
            ],
        )
        profile = await self._evaluate_and_get_output(graph, qos_node)

        self.assertEqual(profile["history"], "keepLast")
        self.assertEqual(profile["depth"], 5)
        self.assertEqual(profile["reliability"], "bestEffort")
        self.assertEqual(profile["durability"], "volatile")
        self.assertEqual(profile["liveliness"], "systemDefault")
        self.assertAlmostEqual(profile["deadline"], 0.0)
        self.assertAlmostEqual(profile["lifespan"], 0.0)
        self.assertAlmostEqual(profile["leaseDuration"], 0.0)

    async def test_preset_system_default_profile(self) -> None:
        """Verify selecting the System Default preset applies expected policy values."""
        graph, qos_node = await self._create_qos_graph(
            "/TestGraph",
            set_values=[
                ("QoSProfile.inputs:createProfile", "System Default"),
            ],
        )
        profile = await self._evaluate_and_get_output(graph, qos_node)

        self.assertEqual(profile["history"], "systemDefault")
        self.assertEqual(profile["depth"], 0)
        self.assertEqual(profile["reliability"], "systemDefault")
        self.assertEqual(profile["durability"], "systemDefault")
        self.assertEqual(profile["liveliness"], "systemDefault")
        self.assertAlmostEqual(profile["deadline"], 0.0)
        self.assertAlmostEqual(profile["lifespan"], 0.0)
        self.assertAlmostEqual(profile["leaseDuration"], 0.0)

    async def test_preset_services_profile_matches_default(self) -> None:
        """Verify selecting the Services preset applies default policy values."""
        graph, qos_node = await self._create_qos_graph(
            "/TestGraph",
            set_values=[
                ("QoSProfile.inputs:createProfile", "Services"),
            ],
        )
        profile = await self._evaluate_and_get_output(graph, qos_node)

        self.assertEqual(profile["history"], "keepLast")
        self.assertEqual(profile["depth"], 10)
        self.assertEqual(profile["reliability"], "reliable")
        self.assertEqual(profile["durability"], "volatile")
        self.assertEqual(profile["liveliness"], "systemDefault")
        self.assertAlmostEqual(profile["deadline"], 0.0)
        self.assertAlmostEqual(profile["lifespan"], 0.0)
        self.assertAlmostEqual(profile["leaseDuration"], 0.0)

    async def test_preset_application_does_not_flip_create_profile_to_custom(self) -> None:
        """Verify preset-driven policy changes keep createProfile at the selected preset."""
        graph, qos_node = await self._create_qos_graph("/TestGraph")

        create_profile_attr = og.Controller.attribute("inputs:createProfile", qos_node)
        create_profile_attr.set("Sensor Data")
        await omni.kit.app.get_app().next_update_async()
        await og.Controller.evaluate(graph)
        await omni.kit.app.get_app().next_update_async()

        self.assertEqual(create_profile_attr.get(), "Sensor Data")

    async def test_output_is_valid_json(self) -> None:
        """Verify the raw output string is valid JSON."""
        graph, qos_node = await self._create_qos_graph("/TestGraph")
        await omni.kit.app.get_app().next_update_async()
        await og.Controller.evaluate(graph)
        await omni.kit.app.get_app().next_update_async()

        qos_json_str = og.Controller.attribute("outputs:qosProfile", qos_node).get()
        try:
            parsed = json.loads(qos_json_str)
            self.assertIsInstance(parsed, dict)
        except json.JSONDecodeError:
            self.fail(f"Output is not valid JSON: {qos_json_str}")

    async def test_depth_is_integer_in_json(self) -> None:
        """Verify depth is serialized as an integer, not a float."""
        graph, qos_node = await self._create_qos_graph("/TestGraph")
        profile = await self._evaluate_and_get_output(graph, qos_node)

        self.assertIsInstance(profile["depth"], int)

    async def test_connected_to_publisher(self) -> None:
        """Verify the QoS profile node can be connected to a publisher node."""
        graph_path = "/TestGraph"
        graph, nodes, _, _ = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("QoSProfile", NODE_TYPE),
                    ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                    ("QoSProfile.outputs:qosProfile", "PublishClock.inputs:qosProfile"),
                ],
            },
        )

        exception_caught = False
        try:
            self._timeline.play()
            await self.simulate_until_condition(lambda: False, max_frames=2)
        except Exception:
            exception_caught = True
        finally:
            self._timeline.stop()

        self.assertFalse(exception_caught, "QoS profile connected to publisher should not throw an exception")
