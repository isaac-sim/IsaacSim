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

"""Verifies the TimeSplitter OmniGraph node decomposes time values into component outputs. Covers graph execution and output propagation for split time fields."""

import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.test


class TestTimeSplitter(ogts.OmniGraphTestCase):
    """Verify TimeSplitter output fields for several numeric input types."""

    async def setUp(self) -> None:
        """Set up  test environment, to be torn down when done."""
        await omni.usd.get_context().new_stage_async()

    # ----------------------------------------------------------------------
    async def tearDown(self) -> None:
        """Get rid of temporary data used by the test."""
        # await omni.kit.stage_templates.new_stage_async()

    # ----------------------------------------------------------------------
    async def test_time_splitter(self) -> None:
        """Verify seconds, milliseconds, microseconds, and nanoseconds are split from time inputs."""
        constant_tuples = [
            ("omni.graph.nodes.ConstantDouble", 1.012345678, (1, 12, 12345, 12345678)),
            ("omni.graph.nodes.ConstantFloat", 2.910123456, (2, 910, 910123, 910123348)),  # precision lost (ns)
            ("omni.graph.nodes.ConstantHalf", 3.789012345, (3, 789, 789062, 789062500)),  # precision lost (ms, ns)
            ("omni.graph.nodes.ConstantInt", 10, (10, 0, 0, 0)),
            ("omni.graph.nodes.ConstantInt64", 20, (20, 0, 0, 0)),
            ("omni.graph.nodes.ConstantUInt", 30, (30, 0, 0, 0)),
            ("omni.graph.nodes.ConstantUInt64", 40, (40, 0, 0, 0)),
        ]

        # walk through the graph, testing each constant
        graph_count = 0
        for node, time, values in constant_tuples:
            with self.subTest(f"{node}-{graph_count}"):
                graph_count = graph_count + 1
                controller = og.Controller()
                keys = controller.Keys
                graph, (const, time_splitter), _, _ = controller.edit(
                    f"/World/Graph{graph_count}",
                    {
                        keys.CREATE_NODES: [
                            ("Constant", node),
                            ("TimeSplitter", "isaacsim.core.nodes.IsaacTimeSplitter"),
                        ],
                        keys.SET_VALUES: [("Constant.inputs:value", time)],
                        keys.CONNECT: [("Constant.inputs:value", "TimeSplitter.inputs:time")],
                    },
                )
                await controller.evaluate(graph)
                await controller.evaluate(graph)
                seconds = time_splitter.get_attribute("outputs:seconds").get()
                milliseconds = time_splitter.get_attribute("outputs:milliseconds").get()
                microseconds = time_splitter.get_attribute("outputs:microseconds").get()
                nanoseconds = time_splitter.get_attribute("outputs:nanoseconds").get()
                print(f"{node}: {time} -> {seconds}|{milliseconds}|{microseconds}|{nanoseconds}")
                self.assertEqual(values[0], seconds)
                self.assertEqual(values[1], milliseconds)
                self.assertEqual(values[2], microseconds)
                self.assertEqual(values[3], nanoseconds)
