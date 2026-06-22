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

"""Drive an actuated robot from OmniGraph with Newton actuators.

Companion script for the Newton Actuators "Drive an Actuated Robot from
OmniGraph" tutorial.  Loads a Franka, authors Newton actuators onto it,
builds the example Action Graph programmatically, and keeps the kit window
open so the user can inspect the graph live in
``Window > Graph Editors > Action Graph``.

In a typical workflow you would author this graph by hand in the Action
Graph editor — see the OmniGraph tutorial.  This script exists to let a
user reproduce the example graph quickly, without driving the editor.

Run with:

    ./python.sh standalone_examples/api/isaacsim.core.experimental.actuators/newton_actuators_omnigraph_example.py

The simulation runs until the kit window is closed.
"""

from __future__ import annotations

# ============================================================================
# 1. Parse arguments and launch Simulation App (non-headless so the user can
#    open the graph editor)
# ============================================================================
import argparse

_parser = argparse.ArgumentParser(description="Newton actuators OmniGraph tutorial example.")
_parser.add_argument(
    "--test",
    action="store_true",
    help="Run a fixed number of simulation steps and exit (used for CI testing).",
)
args, _ = _parser.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import omni.kit.app
import omni.timeline
import omni.ui as ui
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.storage.native import get_assets_root_path_async

# Enable the Action Graph editor so the user can inspect the built graph live.
# Equivalent to ticking ``Window > Graph Editors > Action Graph`` in the GUI.
enable_extension("omni.graph.window.action")
simulation_app.update()

FRANKA_USD_REL_PATH = "Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
GRAPH_PATH = "/ActionGraph"


# ============================================================================
# 2. Open the Franka asset and add a PhysicsScene
# ============================================================================
async def setup_stage_with_franka() -> str:
    """Open the Franka USD asset directly as the active stage.

    Returns:
        USD path of the articulation root prim under the active stage.
    """
    assets_root_path = await get_assets_root_path_async()
    franka_url = f"{assets_root_path}/{FRANKA_USD_REL_PATH}"

    franka_path = "/World/Robot"

    await stage_utils.create_new_stage_async(template="default stage")
    stage_utils.add_reference_to_stage(usd_path=franka_url, path=franka_path)

    stage_utils.define_prim("/PhysicsScene", "PhysicsScene")
    await omni.kit.app.get_app().next_update_async()
    return franka_path


# ============================================================================
# 3. Author NewtonActuator prims onto the Franka arm joints
# ============================================================================
def author_actuators_on_franka(franka_path: str) -> None:
    """Author one PD actuator with effort clamping on each Franka arm joint."""
    from isaacsim.core.experimental.actuators import (
        MaxEffortClampingConfig,
        PDControlConfig,
        add_actuator,
    )
    from isaacsim.core.experimental.prims import Articulation

    JOINT_PARAMS = {
        "panda_joint1": {"kp": 64.0, "kd": 8.0, "max_effort": 1000.0},
        "panda_joint2": {"kp": 64.0, "kd": 8.0, "max_effort": 1000.0},
        "panda_joint3": {"kp": 64.0, "kd": 8.0, "max_effort": 1000.0},
        "panda_joint4": {"kp": 64.0, "kd": 8.0, "max_effort": 1000.0},
        "panda_joint5": {"kp": 64.0, "kd": 8.0, "max_effort": 1000.0},
        "panda_joint6": {"kp": 64.0, "kd": 8.0, "max_effort": 1000.0},
        "panda_joint7": {"kp": 64.0, "kd": 8.0, "max_effort": 1000.0},
    }

    for joint_name, p in JOINT_PARAMS.items():
        add_actuator(
            franka_path,
            target_names=joint_name,
            name=f"{joint_name}_actuator",
            controller=PDControlConfig(kp=p["kp"], kd=p["kd"]),
            clamping=[MaxEffortClampingConfig(max_effort=p["max_effort"])],
        )

    # Add armature for additional stability.
    articulation = Articulation(franka_path)
    articulation.set_dof_armatures(0.1)


# ============================================================================
# 4. Build the example Action Graph
# ============================================================================
# Franka home pose: 7 arm joints followed by 2 finger joints (open).
HOME_POSE = [0.0, -1.3, 0.0, -2.87, 0.0, 2.0, 0.75, 0.04, 0.04]


def build_action_graph(franka_path: str) -> None:
    """Build the example Action Graph.

    Layout::

        OnPlaybackTick ─┬──> ArticulationActuators.execIn
                        └──> ArticulationController.execIn

        RobotPath ──────┬──> ArticulationActuators.robotPath
                        └──> ArticulationController.robotPath

        Pose0 ──> TargetPositions.input0 ─┐
        Pose1 ──> TargetPositions.input1  │
        ...                               ├──> ArticulationController.positionCommand
        Pose8 ──> TargetPositions.input8 ─┘

    The robot prim path is provided by an external **Constant String** node
    (``RobotPath``) so it lives as data in the graph rather than as a literal
    on each consumer.  An **Articulation Controller** node is wired in so DOF
    targets can be authored from the graph: one **Constant Double** per DOF
    feeds a **Construct Array** that aggregates the per-DOF targets into a
    ``double[]`` ``positionCommand``.
    """
    keys = og.Controller.Keys

    pose_node_names = [f"Pose{i}" for i in range(len(HOME_POSE))]

    create_nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("RobotPath", "omni.graph.nodes.ConstantString"),
        ("TargetPositions", "omni.graph.nodes.ConstructArray"),
        (
            "ArticulationActuators",
            "isaacsim.core.experimental.actuators.ArticulationActuators",
        ),
        (
            "ArticulationController",
            "isaacsim.core.nodes.IsaacArticulationController",
        ),
        *[(name, "omni.graph.nodes.ConstantDouble") for name in pose_node_names],
    ]

    # ConstructArray exposes input0 by default with an unresolved type that
    # only resolves via an incoming connection.  Add input1..input(N-1) as
    # additional double-typed slots so each per-DOF constant has a place to
    # connect into.
    create_attributes = [(f"TargetPositions.inputs:input{i}", "double") for i in range(1, len(HOME_POSE))]

    set_values = [
        ("RobotPath.inputs:value", franka_path),
        ("TargetPositions.inputs:arrayType", "double[]"),
        ("TargetPositions.inputs:arraySize", len(HOME_POSE)),
        *[(f"{name}.inputs:value", q) for name, q in zip(pose_node_names, HOME_POSE)],
    ]

    connections = [
        ("OnPlaybackTick.outputs:tick", "ArticulationActuators.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
        ("RobotPath.inputs:value", "ArticulationActuators.inputs:robotPath"),
        ("RobotPath.inputs:value", "ArticulationController.inputs:robotPath"),
        ("TargetPositions.outputs:array", "ArticulationController.inputs:positionCommand"),
        # Wire each per-DOF Constant Double into the corresponding ConstructArray slot.
        # The connection to input0 is what resolves its union type to "double".
        *[(f"{name}.inputs:value", f"TargetPositions.inputs:input{i}") for i, name in enumerate(pose_node_names)],
    ]

    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: create_nodes,
            keys.CREATE_ATTRIBUTES: create_attributes,
            keys.SET_VALUES: set_values,
            keys.CONNECT: connections,
        },
    )

    print(f"Built action graph at {GRAPH_PATH}.")


# ============================================================================
# Entry point
# ============================================================================
def main() -> None:
    """Build the graph, start simulation, and keep the app running."""
    SimulationManager.set_physics_dt(1.0 / 60.0)
    franka_path = simulation_app.run_coroutine(setup_stage_with_franka())
    author_actuators_on_franka(franka_path)
    build_action_graph(franka_path)

    # Open the Action Graph editor so the graph is visible without the user
    # having to navigate Window > Graph Editors > Action Graph manually.
    ui.Workspace.show_window("Action Graph", True)
    simulation_app.update()

    # Start the simulation and keep the kit app running so the user can
    # inspect or modify the graph live in the editor.
    omni.timeline.get_timeline_interface().play()
    print(
        f"\nGraph {GRAPH_PATH} is driving the robot.  Open "
        "Window > Graph Editors > Action Graph to inspect it.  "
        "Close the kit window to exit."
    )
    if args.test:
        for _ in range(5):
            simulation_app.update()
    else:
        while simulation_app.is_running():
            simulation_app.update()


if __name__ == "__main__":
    main()
    simulation_app.close()
