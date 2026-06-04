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

"""Test OmniGraph node input and output value propagation."""

import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp()

import omni

simulation_app.update()
omni.usd.get_context().new_stage()
simulation_app.update()

import omni.graph.core as og

keys = og.Controller.Keys
graph, (tick_node, test_node, str_node), _, _ = og.Controller.edit(
    {"graph_path": "/controller_graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnTick"),
            ("IsaacTest", "isaacsim.core.nodes.IsaacTestNode"),
            ("TestStr", "omni.graph.nodes.ConstantString"),
        ],
        keys.SET_VALUES: [("TestStr.inputs:value", "Hello"), ("OnTick.inputs:onlyPlayback", False)],  # always tick
        keys.CONNECT: [
            ("OnTick.outputs:tick", "IsaacTest.inputs:execIn"),
            ("TestStr.inputs:value", "IsaacTest.inputs:input"),
        ],
    },
)

input_attr = og.Controller.attribute("inputs:value", str_node)
output_attr = og.Controller.attribute("outputs:output", test_node)

simulation_app.update()
value = og.DataView.get(output_attr)
print(value)
if value != "Hello":
    print("[fatal] Output does not equal Hello")
    sys.exit(1)
simulation_app.update()
og.DataView.set(input_attr, "Goodbye")
simulation_app.update()
value = og.DataView.get(output_attr)
print(value)
if value != "Goodbye":
    print("[fatal] Output does not equal Goodbye")
    sys.exit(1)

simulation_app.update()
# Cleanup application
simulation_app.close()
