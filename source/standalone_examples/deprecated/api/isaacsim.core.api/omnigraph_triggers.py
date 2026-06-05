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

"""Demonstrate differences between Push and Action graphs and manual graph triggering."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"renderer": "RealTimePathTracing", "headless": True})

import omni.graph.core as og
from isaacsim.core.api import SimulationContext

"""
This script demonstrates how Push and Action graphs differ, and how to trigger graphs manually.

"""


## build the Push graph with a printout that says "Push Graph Running"
try:
    keys = og.Controller.Keys
    push_graph, _, _, _ = og.Controller.edit(
        {
            "graph_path": "/Push_Graph",
            "evaluator_name": "push",
        },
        {
            keys.CREATE_NODES: [
                ("string", "omni.graph.nodes.ConstantString"),
                ("print", "omni.graph.ui_nodes.PrintText"),
            ],
            keys.SET_VALUES: [
                ("string.inputs:value", "Push Graph Running"),
                ("print.inputs:logLevel", "Warning"),
            ],
            keys.CONNECT: [
                ("string.inputs:value", "print.inputs:text"),
            ],
        },
    )
except Exception as e:
    print(e)
    simulation_app.close()
    exit()

## build an Action graph with a printout that says "Action Graph Running"
try:
    keys = og.Controller.Keys
    action_graph, _, _, _ = og.Controller.edit(
        {
            "graph_path": "/Action_Graph",
            "evaluator_name": "execution",
        },
        {
            keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnTick"),  # action graph needs a trigger
                ("string", "omni.graph.nodes.ConstantString"),
                ("print", "omni.graph.ui_nodes.PrintText"),
            ],
            keys.SET_VALUES: [
                ("string.inputs:value", "Action Graph Running"),
                ("print.inputs:logLevel", "Warning"),
            ],
            keys.CONNECT: [
                ("string.inputs:value", "print.inputs:text"),
                ("tick.outputs:tick", "print.inputs:execIn"),
            ],
        },
    )
except Exception as e:
    print(e)
    simulation_app.close()
    exit()


# let the application run but not simulating (i.e. no physics running). Equivalent to open the app but not pressing "play"
# expected output: only Push Graph ran
print("Starting just the app. Expected output: only Push Graph ran")
for frame in range(10):
    simulation_app.update()

print("ADDING SIMULATION, expected output: both Push Graph and Action Graph ran")
# initiate the simulation (pressed "play")
# expected output: both Push Graph and Action Graph ran
simulation_context = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)
simulation_context.initialize_physics()
simulation_context.play()
for frame in range(10):
    simulation_app.update()


# make both Push and Action Graph OnDemand Only so we can trigger them manually
# default pipeline stage is og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION
push_graph.change_pipeline_stage(og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND)
action_graph.change_pipeline_stage(og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND)

# do the same as before
# expected output: neither graph runs because neither are called explicitly
print("SWITCHED pipelinestage, expected output: neither graph runs because neither are called explicitly")
for frame in range(10):
    simulation_app.update()


print("Manually trigger graphs, expected output: push graph print 2x in 20 frames, action graph printed 4x")
# explicitly calls the push graph every 10 frames
# expected output: push graph print twice in 20 frames, action graph printed 4x
for frame in range(20):
    simulation_app.update()  # still updates every frame doing whatever is needed
    if frame % 10 == 0:
        og.Controller.evaluate_sync(push_graph)
    if frame % 5 == 0:
        og.Controller.evaluate_sync(action_graph)

# add the evaluation of an action graph as part of the physics callback,
# expected output: action graph prints all 10 frames,
print("Trigger a Graph in physics callback, expected output: action graph prints all 10 frames")
simulation_context.add_physics_callback("physics callback", lambda x: og.Controller.evaluate_sync(action_graph))
for frame in range(10):
    simulation_app.update()

# add the evaluation of a push graph as part of the rendering callback (after already having the action graph in physics callback)
# expected output: push and action graph both print all 10 frames
print("trigger push graph in rendering callback,expected output: push and action graph both print all 10 frames ")
simulation_context.add_render_callback("render callback", lambda x: og.Controller.evaluate_sync(push_graph))
for frame in range(10):
    simulation_app.update()


# separately step rendering and physics
# expected output: push graph prints 10 times (every 2 frames), action graph prints 2 times (every 10 frames)
print(
    "separate rendering and physics stepping. expected output: push graph prints 10 times (every 2 frames), action graph prints 2 times (every 10 frames)"
)
for frame in range(20):
    if frame % 2 == 0:
        simulation_context.render()  # only render, no physics

    if frame % 10 == 0:
        simulation_context.step(render=False)  # only physics, no render


# shutdown
simulation_context.stop()
simulation_app.close()
