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

"""Per-robot namespaced ROS 2 bridge action graph factory.

Creates an OmniGraph action graph at `/ROS2_Bridge_<robot_name>` that publishes
namespaced odometry + TF and subscribes to a namespaced `cmd_vel` topic. Use one
graph per robot when running multi-robot scenarios (Nav2, fleet RL).

Reference scenario in the repo:
    source/standalone_examples/api/isaacsim.ros2.bridge/carter_multiple_robot_navigation.py

Use the current `isaacsim.ros2.bridge.*` node names; the legacy
`omni.isaac.ros2_bridge.*` namespace will not load on Kit 110.
"""

import omni.graph.core as og


def create_ros2_bridge_for_robot(robot_name: str, robot_prim_path: str):
    """Create a namespaced ROS 2 bridge action graph for a single robot."""
    graph_path = f"/ROS2_Bridge_{robot_name}"
    ns = f"/{robot_name}"
    keys = og.Controller.Keys

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ],
            keys.SET_VALUES: [
                ("PublishOdom.inputs:topicName", f"{ns}/odom"),
                ("PublishOdom.inputs:chassisPrim", robot_prim_path),
                ("PublishOdom.inputs:frameId", f"{robot_name}/odom"),
                ("PublishOdom.inputs:childFrameId", f"{robot_name}/base_link"),
                ("PublishTF.inputs:topicName", "/tf"),
                ("PublishTF.inputs:targetPrims", [robot_prim_path]),
                ("SubscribeTwist.inputs:topicName", f"{ns}/cmd_vel"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishOdom.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
            ],
        },
    )
