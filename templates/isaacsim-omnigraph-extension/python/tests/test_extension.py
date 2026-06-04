# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Unit tests for {{extension_name}}."""

from __future__ import annotations

import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.app
import omni.kit.test


class TestExtension(ogts.OmniGraphTestCase):
    """Test cases for {{title}}."""

    async def test_extension_loaded(self) -> None:
        """Verify the extension is enabled and accessible."""
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self.assertTrue(ext_manager.is_extension_enabled("{{extension_name}}"))

    async def test_python_node_registered(self) -> None:
        """Verify the Python OGN node type is registered."""
        all_types = og.get_registered_nodes()
        self.assertIn("{{extension_name}}.ExamplePython", all_types)

    async def test_cpp_node_registered(self) -> None:
        """Verify the C++ OGN node type is registered."""
        all_types = og.get_registered_nodes()
        self.assertIn("{{extension_name}}.ExampleCpp", all_types)

    async def test_cpp_node_compute(self) -> None:
        """Verify the C++ OGN node computes correctly."""
        keys = og.Controller.Keys
        graph_path = "/TestGraph_ExampleCpp"

        (graph, nodes, _, _) = og.Controller.edit(
            graph_path,
            {
                keys.CREATE_NODES: [
                    ("cpp_node", "{{extension_name}}.ExampleCpp"),
                ],
                keys.SET_VALUES: [
                    ("cpp_node.inputs:value", 21.0),
                ],
            },
        )

        await og.Controller.evaluate(graph)
        result = og.Controller.get(og.Controller.attribute("outputs:result", nodes[0]))
        self.assertAlmostEqual(result, 42.0)

    async def test_python_node_compute(self) -> None:
        """Verify the Python OGN node computes correctly."""
        keys = og.Controller.Keys
        graph_path = "/TestGraph_ExamplePython"

        (graph, nodes, _, _) = og.Controller.edit(
            graph_path,
            {
                keys.CREATE_NODES: [
                    ("py_node", "{{extension_name}}.ExamplePython"),
                ],
                keys.SET_VALUES: [
                    ("py_node.inputs:value", 21.0),
                ],
            },
        )

        await og.Controller.evaluate(graph)
        result = og.Controller.get(og.Controller.attribute("outputs:result", nodes[0]))
        self.assertAlmostEqual(result, 42.0)
