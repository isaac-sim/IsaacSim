# SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from re import I

import carb
import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.kit.test
import omni.kit.usd
import usdrt.Sdf
from isaacsim.core.nodes.scripts.utils import set_target_prims
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.physics import simulate_async
from isaacsim.core.utils.stage import add_reference_to_stage, open_stage_async
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Sdf, UsdGeom
from usd.schema.isaac import robot_schema

from .common import ROS2TestCase, add_cube, add_franka, get_qos_profile


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRos2PoseTree(ROS2TestCase):
    # Before running each test
    async def setUp(self):
        await super().setUp()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        pass

    # After running each test
    async def tearDown(self):
        await super().tearDown()

    async def test_pose_tree(self):
        import rclpy
        from tf2_msgs.msg import TFMessage

        await add_franka()
        await add_cube("/cube", 0.75, (2.00, 0, 0.75))

        self._tf_data = None
        self._tf_data_prev = None

        def tf_callback(data: TFMessage):
            self._tf_data = data

        node = rclpy.create_node("tf_tester")
        tf_sub = node.create_subscription(TFMessage, "/tf_test", tf_callback, get_qos_profile())

        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishTF.inputs:topicName", "/tf_test"),
                        (
                            "PublishTF.inputs:targetPrims",
                            [
                                usdrt.Sdf.Path("/panda"),
                                usdrt.Sdf.Path("/cube"),
                                usdrt.Sdf.Path("/panda/panda_hand/geometry"),
                                usdrt.Sdf.Path("/panda/panda_hand"),
                            ],
                        ),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        # checks
        self.assertEqual(len(self._tf_data.transforms), 14)  # there are 12 items in the tree.
        self.assertEqual(self._tf_data.transforms[12].header.frame_id, "world")  # check cube's parent is world

        # the pose of panda_hand (a rigid body) should match the pose of the geometry xform (non rigid body) child.
        self.assertEqual(
            self._tf_data.transforms[12].transform.translation, self._tf_data.transforms[13].transform.translation
        )
        # print(self._tf_data.transforms)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        spin()
        self._tf_data_prev = self._tf_data
        self._tf_data = None

        # add a parent prim
        set_target_prims(
            primPath="/ActionGraph/PublishTF", inputName="inputs:parentPrim", targetPrimPaths=["/panda/panda_link0"]
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        # checks
        self.assertEqual(
            self._tf_data.transforms[0].header.frame_id, "panda_link0"
        )  # check the first link's parent is panda_link0
        self.assertEqual(
            self._tf_data.transforms[0].child_frame_id, "panda_link1"
        )  # check the child of the first link is not panda_link0

        self._timeline.stop()
        spin()
        pass

    async def test_duplicate_names_tree(self):
        import rclpy
        from tf2_msgs.msg import TFMessage

        await add_franka()

        await add_cube("/cube0/cube", 0.75, (2.00, 0, 0.75))
        await add_cube("/cube1/cube", 0.75, (3.00, 0, 0.75))
        await add_cube("/cube2/cube", 0.75, (4.00, 0, 0.75))

        stage = omni.usd.get_context().get_stage()

        cube2 = stage.GetPrimAtPath("/cube2/cube")

        cube2.CreateAttribute(robot_schema.Attributes.NAME_OVERRIDE.name, Sdf.ValueTypeNames.String, True).Set(
            "Cube_override"
        )

        self._tf_data = None
        self._tf_data_prev = None

        def tf_callback(data: TFMessage):
            self._tf_data = data

        node = rclpy.create_node("tf_tester")
        tf_sub = node.create_subscription(TFMessage, "/tf_test", tf_callback, 10)

        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishTF.inputs:topicName", "/tf_test"),
                        (
                            "PublishTF.inputs:targetPrims",
                            [
                                usdrt.Sdf.Path("/panda"),
                                usdrt.Sdf.Path("/cube0/cube"),
                                usdrt.Sdf.Path("/cube1/cube"),
                                usdrt.Sdf.Path("/cube2/cube"),
                            ],
                        ),
                        (
                            "PublishTF.inputs:parentPrim",
                            [usdrt.Sdf.Path("/panda")],
                        ),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        spin()
        self._tf_data_prev = self._tf_data
        self._tf_data = None

        # add a parent prim
        set_target_prims(
            primPath="/ActionGraph/PublishTF", inputName="inputs:parentPrim", targetPrimPaths=["/cube0/cube"]
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        # checks
        self.assertEqual(self._tf_data.transforms[11].header.frame_id, "cube")  # check the base is cube
        self.assertEqual(
            self._tf_data.transforms[11].child_frame_id, "cube1_cube"
        )  # check the second cube got auto-set to cube_01
        self.assertEqual(
            self._tf_data.transforms[12].child_frame_id, "Cube_override"
        )  # check the third cube got the right override frame ID

        self._timeline.stop()
        spin()
        pass

    async def test_frame_name_override(self):
        import rclpy
        from tf2_msgs.msg import TFMessage

        stage = omni.usd.get_context().get_stage()
        dome_light = stage.DefinePrim("/World/DomeLight", "DomeLight")
        dome_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(500.0)

        # Create two Franka robots at different paths
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/panda1")
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/panda2")

        stage = omni.usd.get_context().get_stage()

        # Verify robots were created
        panda1 = stage.GetPrimAtPath("/World/panda1")
        panda2 = stage.GetPrimAtPath("/World/panda2")

        self.assertTrue(panda1.IsValid(), "First robot not created successfully")
        self.assertTrue(panda2.IsValid(), "Second robot not created successfully")

        # Set position of second robot
        XFormPrim(
            "/World/panda2",
            positions=np.array([[1.5, 0.0, 0.0]]),
        )

        stage = omni.usd.get_context().get_stage()
        self._tf_data = None

        def tf_callback(data: TFMessage):
            self._tf_data = data

        node = rclpy.create_node("tf_tester")
        self._tf_sub = node.create_subscription(TFMessage, "/tf_test", tf_callback, 10)

        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishTF.inputs:topicName", "/tf_test"),
                        (
                            "PublishTF.inputs:targetPrims",
                            [
                                usdrt.Sdf.Path("/World/panda1"),
                                usdrt.Sdf.Path("/World/panda2"),
                            ],
                        ),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        # Run the simulation which will trigger the CARB_LOG_WARN when processing the duplicate "base_link" names
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await simulate_async(1, 60, spin)

        self._timeline.stop()
        spin()

        # Verify transforms were published with unique names
        self.assertIsNotNone(self._tf_data)

        # Creating dict to store all frame IDs from both robots
        original_frames = set()
        renamed_frames = set()

        franka_links = ["panda_link0", "panda_link1", "panda_link2", "panda_hand"]

        # Collect frame IDs from TF message
        frame_ids = []
        for transform in self._tf_data.transforms:
            frame_ids.append(transform.child_frame_id)

        # Check for original and renamed frames
        for link in franka_links:
            if link in frame_ids:
                original_frames.add(link)

            renamed_pattern = "World_panda2_" + link
            for frame_id in frame_ids:
                if renamed_pattern in frame_id:
                    renamed_frames.add(frame_id)

        # Verify we found original frames
        self.assertTrue(len(original_frames) > 0, f"No original frames found. All frames: {frame_ids}")

        # Verify we found renamed frames
        self.assertTrue(len(renamed_frames) > 0, f"No renamed frames found. All frames: {frame_ids}")

        # Verify for each original frame, there's a corresponding renamed frame
        for link in franka_links:
            if link in original_frames:
                renamed_exists = False
                expected_renamed = "World_panda2_" + link
                for renamed in renamed_frames:
                    if expected_renamed in renamed:
                        renamed_exists = True
                        break

                self.assertTrue(renamed_exists, f"Original frame {link} should have a renamed frames")
