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

import carb
import omni.graph.core as og
import omni.kit.test
import omni.kit.usd
import usdrt.Sdf
from isaacsim.core.utils.stage import open_stage_async
from isaacsim.storage.native import get_assets_root_path_async
from pxr import UsdGeom

from .common import ROS2TestCase, add_cube, add_franka


class TestPrimValidation(ROS2TestCase):

    async def setUp(self):
        # before each test
        await super().setUp()
        await omni.usd.get_context().new_stage_async()
        self._stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        # after each test
        await super().tearDown()

    async def test_joint_state_valid_prim(self):
        # test OgnROS2PublishJointState with valid target prim
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        usd_path = assets_root_path + "/Isaac/Robots/IsaacSim/SimpleArticulation/articulation_3_joints.usd"
        (result, error) = await open_stage_async(usd_path)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(result)

        # Create graph with valid target prim
        graph_path = "/TestGraph"
        exception_caught = False
        try:
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path("/Articulation")]),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    ],
                },
            )

            # Start simulation to trigger node compute
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()

        except Exception as e:
            exception_caught = True
            carb.log_info(f"Exception caught in valid prim test: {e}")
        finally:
            self._timeline.stop()

        # Verify that valid prim validation did not throw an exception
        self.assertFalse(exception_caught, "Valid prim should not throw an exception")

    async def test_joint_state_invalid_prim(self):
        # test OgnROS2PublishJointState with invalid target prim
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        usd_path = assets_root_path + "/Isaac/Robots/IsaacSim/SimpleArticulation/articulation_3_joints.usd"
        (result, error) = await open_stage_async(usd_path)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(result)

        # Create graph with invalid target prim
        graph_path = "/TestGraph"
        exception_caught = False
        try:
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path("/NonExistentPrim")]),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    ],
                },
            )

            # Start simulation to trigger node compute - this should fail with validation error
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()

        except Exception as e:
            exception_caught = True
            carb.log_info(f"Exception caught for invalid prim: {e}")
        finally:
            self._timeline.stop()

        # Verify that invalid prim validation was handled gracefully (no exception thrown)
        self.assertFalse(exception_caught, "Invalid prim should be handled gracefully without throwing exception")

    async def test_joint_state_empty_prim(self):
        # test OgnROS2PublishJointState with empty target prim
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        usd_path = assets_root_path + "/Isaac/Robots/IsaacSim/SimpleArticulation/articulation_3_joints.usd"
        (result, error) = await open_stage_async(usd_path)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(result)

        # Create graph with empty target prim
        graph_path = "/TestGraph"
        exception_caught = False
        try:
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishJointState.inputs:targetPrim", []),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    ],
                },
            )

            # Start simulation to trigger node compute - this should fail with validation error
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()

        except Exception as e:
            exception_caught = True
            carb.log_info(f"Exception caught for empty prim: {e}")
        finally:
            self._timeline.stop()

        # Verify that empty prim validation was handled gracefully (no exception thrown)
        self.assertFalse(exception_caught, "Empty prim should be handled gracefully without throwing exception")

    async def test_transform_tree_valid_prims(self):
        # test OgnROS2PublishTransformTree with valid target prims
        await add_franka()
        await add_cube("/cube1", 0.5, (1.0, 0, 0.5))
        await add_cube("/cube2", 0.5, (2.0, 0, 0.5))
        await omni.kit.app.get_app().next_update_async()

        # Create graph with valid target prims
        graph_path = "/TestGraph"
        exception_caught = False
        try:
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("PublishTransformTree", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        (
                            "PublishTransformTree.inputs:targetPrims",
                            [
                                usdrt.Sdf.Path("/panda"),
                                usdrt.Sdf.Path("/cube1"),
                                usdrt.Sdf.Path("/cube2"),
                            ],
                        ),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishTransformTree.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishTransformTree.inputs:timeStamp"),
                    ],
                },
            )

            # Start simulation to trigger node compute
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()

        except Exception as e:
            exception_caught = True
            carb.log_info(f"Exception caught in valid prims test: {e}")
        finally:
            self._timeline.stop()

        # Verify that valid prims validation did not throw an exception
        self.assertFalse(exception_caught, "Valid prims should not throw an exception")

    async def test_transform_tree_invalid_prims(self):
        # test OgnROS2PublishTransformTree with some invalid target prims
        await add_franka()
        await add_cube("/cube1", 0.5, (1.0, 0, 0.5))
        await omni.kit.app.get_app().next_update_async()

        # Create graph with mix of valid and invalid target prims
        graph_path = "/TestGraph"
        exception_caught = False
        try:
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("PublishTransformTree", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        (
                            "PublishTransformTree.inputs:targetPrims",
                            [
                                usdrt.Sdf.Path("/panda"),
                                usdrt.Sdf.Path("/cube1"),
                                usdrt.Sdf.Path("/NonExistentPrim"),  # This is invalid
                            ],
                        ),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishTransformTree.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishTransformTree.inputs:timeStamp"),
                    ],
                },
            )

            # Start simulation to trigger node compute - this should fail with validation error
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()

        except Exception as e:
            exception_caught = True
            carb.log_info(f"Exception caught for invalid prims: {e}")
        finally:
            self._timeline.stop()

        # Verify that invalid prims validation was handled gracefully (no exception thrown)
        self.assertFalse(exception_caught, "Invalid prims should be handled gracefully without throwing exception")

    async def test_transform_tree_empty_prims(self):
        # test OgnROS2PublishTransformTree with empty target prims
        await add_franka()
        await omni.kit.app.get_app().next_update_async()

        # Create graph with empty target prims
        graph_path = "/TestGraph"
        exception_caught = False
        try:
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("PublishTransformTree", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishTransformTree.inputs:targetPrims", []),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishTransformTree.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishTransformTree.inputs:timeStamp"),
                    ],
                },
            )

            # Start simulation to trigger node compute - this should fail with validation error
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()

        except Exception as e:
            exception_caught = True
            carb.log_info(f"Exception caught for empty prims: {e}")
        finally:
            self._timeline.stop()

        # Verify that empty prims validation was handled gracefully (no exception thrown)
        self.assertFalse(exception_caught, "Empty prims should be handled gracefully without throwing exception")

    async def test_transform_tree_all_invalid_prims(self):
        # test OgnROS2PublishTransformTree with all invalid target prims
        await add_franka()
        await omni.kit.app.get_app().next_update_async()

        # Create graph with all invalid target prims
        graph_path = "/TestGraph"
        exception_caught = False
        try:
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("PublishTransformTree", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        (
                            "PublishTransformTree.inputs:targetPrims",
                            [
                                usdrt.Sdf.Path("/NonExistentPrim1"),
                                usdrt.Sdf.Path("/NonExistentPrim2"),
                                usdrt.Sdf.Path("/NonExistentPrim3"),
                            ],
                        ),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishTransformTree.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishTransformTree.inputs:timeStamp"),
                    ],
                },
            )

            # Start simulation to trigger node compute - this should fail with validation error
            self._timeline.play()
            await omni.kit.app.get_app().next_update_async()

        except Exception as e:
            exception_caught = True
            carb.log_info(f"Exception caught for all invalid prims: {e}")
        finally:
            self._timeline.stop()

        # Verify that all invalid prims validation was handled gracefully (no exception thrown)
        self.assertFalse(exception_caught, "All invalid prims should be handled gracefully without throwing exception")

    async def test_joint_state_with_multiple_articulations(self):
        # test OgnROS2PublishJointState with multiple articulations to ensure it works with valid prims
        stage = omni.usd.get_context().get_stage()

        # Create first articulation
        art1_prim = stage.DefinePrim("/Articulation1", "Xform")
        UsdGeom.Xform(art1_prim)

        # Create second articulation
        art2_prim = stage.DefinePrim("/Articulation2", "Xform")
        UsdGeom.Xform(art2_prim)

        await omni.kit.app.get_app().next_update_async()

        # Test with first articulation
        graph_path = "/TestGraph"
        exception_caught = False
        try:
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path("/Articulation1")]),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    ],
                },
            )

        except Exception as e:
            exception_caught = True
            carb.log_info(f"Exception caught in multiple articulations test: {e}")

        # Verify that valid articulation prim validation did not throw an exception
        self.assertFalse(exception_caught, "Valid articulation prim should not throw an exception")
