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

"""Test suite for validating ROS 2 transform listener functionality with Isaac Sim."""

import os

import carb
import omni.graph.core as og
import omni.kit.app
import omni.kit.test
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Sdf


class TestTransformListener(omni.kit.test.AsyncTestCase):
    """Test suite for validating ROS 2 transform listener functionality with Isaac Sim.

    This test case verifies the integration between Isaac Sim and ROS 2 transform (tf) system by:

    1. Loading a Franka Panda robot into the stage
    2. Setting up an OmniGraph to publish transform tree data to ROS 2
    3. Initializing the transform listener plugin to receive and process tf messages
    4. Running a simulation and spinning the listener to capture transforms
    5. Validating that all expected robot link frames are received
    6. Verifying transform values and parent-child relationships between frames

    The test ensures that the isaacsim.ros2.tf_viewer extension correctly captures and maintains the complete
    transform hierarchy of a robot, including positions, orientations, and frame relationships. It validates both
    the structural completeness of the transform tree (all expected frames present) and the correctness of
    specific transform values.
    """

    # Before running each test
    async def setUp(self) -> None:
        """Sets up the test environment by creating a new stage and initializing the timeline interface."""
        await stage_utils.create_new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()

    # After running each test
    async def tearDown(self) -> None:
        """Cleans up the test environment by releasing the timeline interface."""
        self._timeline = None

    # ----------------------------------------------------------------------
    async def test_transform_listener(self) -> None:
        """Tests the ROS2 transform listener functionality with a Franka Panda robot.

        Verifies that the transform listener correctly receives and processes transform data published via ROS2 /tf
        topic. The test adds a Franka Panda robot to the stage, creates an action graph to publish transform tree
        data, initializes the transform listener plugin, runs the simulation, and validates that all expected frames,
        transforms, and parent-child relations are correctly captured.
        """
        # add robot
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        robot = stage_utils.add_reference_to_stage(usd_path=asset_path, path="/World/panda")
        robot.GetVariantSet("Gripper").SetVariantSelection("Default")
        robot.GetVariantSet("Mesh").SetVariantSelection("Performance")

        # define graph to publish /tf
        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("PublishTransformTree", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PublishTransformTree.inputs:topicName", "tf"),
                    ("PublishTransformTree.inputs:targetPrims", [Sdf.Path("/World/panda")]),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishTransformTree.inputs:execIn"),
                ],
            },
        )
        subscriber_node = new_nodes[-1]

        # load plugin
        ext_path = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
        print(ext_path)
        carb.get_framework().load_plugins(
            loaded_file_wildcards=["isaacsim.ros2.tf_viewer.plugin"],
            search_paths=[os.path.abspath(os.path.join(ext_path, "bin"))],
        )

        # load the transform listener
        from .. import _transform_listener as module

        interface = module.acquire_transform_listener_interface()
        interface.initialize(os.environ.get("ROS_DISTRO", "").lower())

        # run the simulation
        self._timeline.play()
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()
            interface.spin()

        frames, transforms, relations = interface.get_transforms("panda_link0")
        # print(frames, transforms, relations)

        interface.finalize()
        module.release_transform_listener_interface(interface)

        # check frames
        gt_frames = [
            "panda_link0",
            "panda_link1",
            "panda_link2",
            "panda_link3",
            "panda_link4",
            "panda_link5",
            "panda_link6",
            "panda_link7",
            "panda_hand",
            "panda_leftfinger",
            "panda_rightfinger",
        ]
        for frame in gt_frames:
            self.assertIn(frame, frames)

        # check transforms
        self.assertTupleEqual(transforms.get("panda_link0", tuple()), ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))

        # check relations
        gt_relations = [
            ("panda_link1", "panda_link0"),
            ("panda_link2", "panda_link1"),
            ("panda_link3", "panda_link2"),
            ("panda_link4", "panda_link3"),
            ("panda_link5", "panda_link4"),
            ("panda_link6", "panda_link5"),
            ("panda_link7", "panda_link6"),
            ("panda_hand", "panda_link7"),
            ("panda_leftfinger", "panda_hand"),
            ("panda_rightfinger", "panda_hand"),
        ]
        for relation in gt_relations:
            self.assertIn(relation, relations)
