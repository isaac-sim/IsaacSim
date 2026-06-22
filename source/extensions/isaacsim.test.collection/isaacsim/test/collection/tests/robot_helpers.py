# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Helper functions for robot simulation tests."""

import omni.graph.core as og
import omni.kit.app
import usdrt.Sdf
from isaacsim.core.experimental.prims import Articulation


async def init_robot_sim(art_path: str, graph_path: str = "/ActionGraph") -> None:
    """Initialize robot simulation by resetting pose and velocities.

    Creates an articulation at the given path, resets its position, orientation,
    and velocities, then resets the differential controller inputs.

    Args:
        art_path: USD path to the robot articulation prim.
        graph_path: USD path to the OmniGraph containing the controller.
    """
    art = Articulation(art_path)
    # Wait for physics to be ready (replaces _articulation_view.initialize())
    await omni.kit.app.get_app().next_update_async()
    # reset position and orientation (wxyz format for experimental API)
    art.set_world_poses(positions=[[0, 0, 0.1]], orientations=[[1, 0, 0, 0]])
    # reset velocities
    art.set_velocities(linear_velocities=[[0, 0, 0]], angular_velocities=[[0, 0, 0]])
    # reset controller
    og.Controller.attribute(graph_path + "/DifferentialController.inputs:linearVelocity").set(0)
    og.Controller.attribute(graph_path + "/DifferentialController.inputs:angularVelocity").set(0)
    # wait for robot to drop
    for i in range(10):
        await omni.kit.app.get_app().next_update_async()

    return


def setup_robot_og(
    graph_path: str,
    lwheel_name: str,
    rwheel_name: str,
    robot_path: str,
    wheel_rad: float,
    wheel_dist: float,
) -> object:
    """Set up OmniGraph for differential drive robot control.

    Creates an action graph with playback tick, differential controller,
    articulation controller, and odometry computation nodes.

    Args:
        graph_path: USD path where the graph will be created.
        lwheel_name: Name of the left wheel joint.
        rwheel_name: Name of the right wheel joint.
        robot_path: USD path to the robot prim.
        wheel_rad: Wheel radius in meters.
        wheel_dist: Distance between wheels in meters.

    Returns:
        Tuple of (graph, odom_node) where graph is the created OmniGraph
        and odom_node is the odometry computation node.
    """
    keys = og.Controller.Keys
    graph, nodes, _, _ = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("DifferentialController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("computeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "DifferentialController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "computeOdom.inputs:execIn"),
                ("DifferentialController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ],
            keys.SET_VALUES: [
                ("ArticulationController.inputs:jointNames", [lwheel_name, rwheel_name]),
                ("ArticulationController.inputs:robotPath", robot_path),
                ("DifferentialController.inputs:wheelRadius", wheel_rad),
                ("DifferentialController.inputs:wheelDistance", wheel_dist),
                ("computeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(robot_path)]),
            ],
        },
    )

    return graph, nodes[3]
