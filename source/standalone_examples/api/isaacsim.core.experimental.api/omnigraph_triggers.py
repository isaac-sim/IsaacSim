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

"""Demonstrate Push and Action graph differences and manual graph triggering using the experimental API.

This example demonstrates how Push and Action graphs differ, and how to trigger graphs manually using the Isaac Sim
core (experimental) API.

The example serves to illustrate the following concepts:
- The difference between Push graphs and Action graphs in OmniGraph.
- How to create and configure graphs programmatically.
- How to change graph pipeline stages (e.g., to OnDemand).
- How to manually trigger graph evaluation.
- How to register graph evaluation in physics and rendering callbacks.
- How to separately step rendering and physics.

The source code is organized into 3 main sections:
1. Command-line argument parsing and SimulationApp launch (common to all standalone examples).
2. Graph creation and configuration.
3. Example logic demonstrating different graph triggering methods.
"""

from __future__ import annotations

# Parse any command-line arguments specific to the standalone application (only known arguments).
import argparse

# 1. --------------------------------------------------------------------


parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, _ = parser.parse_known_args()

# Launch the `SimulationApp` (see DEFAULT_LAUNCHER_CONFIG for available configuration):
# https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.simulation_app/docs/index.html
from isaacsim import SimulationApp

simulation_app = SimulationApp({"renderer": "RealTimePathTracing", "headless": True})

# Any Omniverse level imports must occur after the `SimulationApp` class is instantiated (because APIs are provided
# by the extension/runtime plugin system, it must be loaded before they will be available to import).
import isaacsim.core.experimental.utils.app as app_utils
import omni.graph.core as og
from isaacsim.core.rendering_manager import RenderingEvent, RenderingManager
from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager

# 2. --------------------------------------------------------------------

# Build the Push graph (`push_graph`) with a printout that says "Push Graph Running".
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

# Build an Action graph (`action_graph`) with a printout that says "Action Graph Running".
try:
    keys = og.Controller.Keys
    action_graph, _, _, _ = og.Controller.edit(
        {
            "graph_path": "/Action_Graph",
            "evaluator_name": "execution",
        },
        {
            keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnTick"),  # - Action graph needs a trigger
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

# 3. --------------------------------------------------------------------

# Let the application run but not simulating (i.e., no physics running). Equivalent to opening the app but not
# pressing "play". Expected output: only Push Graph ran.
print("Starting just the app. Expected output: only Push Graph ran")
for frame in range(10):
    simulation_app.update()
    if args.test is True:
        break

# Initiate the simulation (pressed "play"). Expected output: both Push Graph and Action Graph ran.
print("Adding simulation, expected output: both Push Graph and Action Graph ran")
SimulationManager.set_physics_dt(1.0 / 60.0)
app_utils.play()
for frame in range(10):
    simulation_app.update()
    if args.test is True:
        break

# Make both `push_graph` and `action_graph` OnDemand Only so we can trigger them manually.
# - Default pipeline stage is `og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION`.
push_graph.change_pipeline_stage(og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND)
action_graph.change_pipeline_stage(og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND)

# Do the same as before. Expected output: neither graph runs because neither are called explicitly.
print("Switched pipelinestage, expected output: neither graph runs because neither are called explicitly")
for frame in range(10):
    simulation_app.update()
    if args.test is True:
        break

# Explicitly call `push_graph` every 10 frames and `action_graph` every 5 frames.
# - Expected output: `push_graph` prints twice in 20 frames, `action_graph` printed 4x.
print("Manually trigger graphs, expected output: push graph print 2x in 20 frames, action graph printed 4x")
for frame in range(20):
    simulation_app.update()  # - Still updates every frame doing whatever is needed
    if frame % 10 == 0:
        og.Controller.evaluate_sync(push_graph)
    if frame % 5 == 0:
        og.Controller.evaluate_sync(action_graph)
    if args.test is True:
        break

# Add the evaluation of `action_graph` as part of the physics callback.
# - Expected output: `action_graph` prints all 10 frames.
print("Trigger a Graph in physics callback, expected output: action graph prints all 10 frames")
SimulationManager.register_callback(
    lambda dt, context: og.Controller.evaluate_sync(action_graph), event=IsaacEvents.POST_PHYSICS_STEP
)
for frame in range(10):
    simulation_app.update()
    if args.test is True:
        break

# Add the evaluation of `push_graph` as part of the rendering callback (after already having `action_graph` in
# physics callback). Expected output: `push_graph` and `action_graph` both print all 10 frames.
print("Trigger push graph in rendering callback,expected output: push and action graph both print all 10 frames ")
RenderingManager.register_callback(
    RenderingEvent.NEW_FRAME, callback=lambda event: og.Controller.evaluate_sync(push_graph)
)
for frame in range(10):
    simulation_app.update()
    if args.test is True:
        break

# Separately step rendering and physics.
# - Expected output: `push_graph` prints 10 times (every 2 frames), `action_graph` prints 2 times (every 10 frames).
print(
    "Separate rendering and physics stepping. expected output: push graph prints 10 times (every 2 frames), action graph prints 2 times (every 10 frames)"
)
for frame in range(20):
    if frame % 2 == 0:
        RenderingManager.render()  # - Only render, no physics

    if frame % 10 == 0:
        SimulationManager.step()  # - Only physics, no render

    if args.test is True:
        break

# Shutdown
app_utils.stop()

# Close the `SimulationApp`.
simulation_app.close()
