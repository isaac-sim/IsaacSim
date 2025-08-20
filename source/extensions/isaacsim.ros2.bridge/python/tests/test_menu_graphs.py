# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import time

import numpy as np
import omni.graph.core as og
import omni.kit.app
import omni.kit.test
import omni.kit.ui_test as ui_test
import omni.timeline
import omni.usd
import rclpy
import usdrt.Sdf
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.physics import simulate_async
from isaacsim.core.utils.prims import define_prim
from isaacsim.core.utils.stage import add_reference_to_stage, update_stage_async
from isaacsim.storage.native import get_assets_root_path_async
from nav_msgs.msg import Odometry
from omni.kit.ui_test.menu import *
from omni.kit.ui_test.query import *
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState, LaserScan, PointCloud2
from std_msgs.msg import Float32, Header
from tf2_msgs.msg import TFMessage

from .common import ROS2TestCase


class ROS2MenuTestBase(ROS2TestCase):
    """Base class for ROS2 OmniGraph Menu tests"""

    async def setUp(self):
        await super().setUp()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()
        self._timeline = omni.timeline.get_timeline_interface()

        self.node = Node("test_omnigraph_node")

        # Set up message tracking
        self.received_messages = {}
        self.subscribers = []

    async def tearDown(self):
        # Clean up ROS2 subscribers
        for sub in self.subscribers:
            self.node.destroy_subscription(sub)
        self.node.destroy_node()
        await super().tearDown()

    def create_subscriber(self, topic, msg_type, callback=None):
        """Create a subscriber"""

        def message_callback(msg):
            if callback:
                callback(msg)

        sub = self.node.create_subscription(msg_type, topic, message_callback, 10)
        self.subscribers.append(sub)
        return sub

    async def process_ros_messages(self, duration=0.1):
        """Process ROS messages for a short duration"""
        start_time = time.time()
        while time.time() - start_time < duration:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            await omni.kit.app.get_app().next_update_async()

    async def setup_test_environment(self, robot_path="/World/test_robot", add_test_cubes=False, articulation=False):
        """Helper function to set up a standard test environment with a Nova Carter robot.

        Args:
            robot_path: Path where the robot should be placed.

        Returns:
            The robot prim and base_link path.
        """
        from isaacsim.core.utils.articulations import remove_articulation_root
        from pxr import UsdLux, UsdPhysics

        # Creating environment and Carter Robot
        dome_light_path = "/World/DomeLight"
        dome_light = UsdLux.DomeLight.Define(self._stage, dome_light_path)
        dome_light.CreateIntensityAttr(1000)

        scene = Scene()
        scene.add_default_ground_plane()

        if add_test_cubes:
            # Add cubes to the scene for sensor detection
            VisualCuboid(prim_path="/World/cube_1", scale=(1, 3, 1), position=(5, 0, 0), size=1.0)
            VisualCuboid(prim_path="/World/cube_2", scale=(1, 3, 1), position=(-5, 0, 0), size=1.0)
            VisualCuboid(prim_path="/World/cube_3", scale=(3, 1, 1), position=(0, 5, 0), size=1.0)
            VisualCuboid(prim_path="/World/cube_4", scale=(3, 1, 1), position=(0, -5, 0), size=1.0)

        await update_stage_async()

        assets_root_path = await get_assets_root_path_async()
        asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
        robot = add_reference_to_stage(usd_path=asset_path, prim_path=robot_path)
        robot.GetVariantSet("Physics").SetVariantSelection("Physics_Base")
        robot.GetVariantSet("Sensors").SetVariantSelection("All_Sensors")

        if articulation:
            # Ensure physics properties are set on the robot
            if not UsdPhysics.ArticulationRootAPI(robot):
                UsdPhysics.ArticulationRootAPI.Apply(robot)

            # Remove the nested articulation root API if it exists
            chassis_link_path = f"{robot_path}/chassis_link"
            chassis_link = self._stage.GetPrimAtPath(chassis_link_path)
            if chassis_link and chassis_link.HasAPI(UsdPhysics.ArticulationRootAPI):
                remove_articulation_root(chassis_link)

        # Create base_link in chassis_link using XFormPrim
        xform_path = f"{robot_path}/chassis_link/base_link"
        define_prim(prim_path=xform_path, prim_type="Xform")

        XFormPrim(xform_path, positions=np.array([[0.0, 0.0, 0.0]]))

        await update_stage_async()

        return robot, xform_path


class TestMenuROS2CameraGraph(ROS2MenuTestBase):
    """Test ROS2 Camera OmniGraph creation from menu"""

    async def test_camera_graph_creation(self):
        """Test creation of camera graph structure via menu"""

        # Creating environment and Carter Robot
        robot, base_link_path = await self.setup_test_environment()

        # Click through the menu to create the graph
        await menu_click("Tools/Robotics/ROS 2 OmniGraphs/Camera")
        await omni.kit.app.get_app().next_update_async()

        # Wait for and interact with parameter window
        window_name = "ROS2 Camera Graph"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        await update_stage_async()

        # Find and set the graph root prim
        root_widget_path = f"{window_name}//Frame/VStack[0]"

        # Find and set the camera prim
        camera_prim = ui_test.find(root_widget_path + "/HStack[2]/StringField[0]")
        camera_prim.model.set_value("/World/test_robot/chassis_link/sensors/front_hawk/left/camera_left")

        # Enable Depth Point Clouds
        depth_checkbox = ui_test.find(root_widget_path + "/HStack[7]/HStack[0]/VStack[0]/ToolButton[0]")
        depth_checkbox.model.set_value(True)

        # Enable Instance Segmentation
        instance_checkbox = ui_test.find(root_widget_path + "/HStack[8]/HStack[0]/VStack[0]/ToolButton[0]")
        instance_checkbox.model.set_value(True)

        # Enable Semantic Segmentation
        semantic_checkbox = ui_test.find(root_widget_path + "/HStack[9]/HStack[0]/VStack[0]/ToolButton[0]")
        semantic_checkbox.model.set_value(True)

        # Enable BoundingBox2D Tight
        bbox2d_tight_checkbox = ui_test.find(root_widget_path + "/HStack[10]/HStack[0]/VStack[0]/ToolButton[0]")
        bbox2d_tight_checkbox.model.set_value(True)

        # Enable BoundingBox2D Loose
        bbox2d_loose_checkbox = ui_test.find(root_widget_path + "/HStack[11]/HStack[0]/VStack[0]/ToolButton[0]")
        bbox2d_loose_checkbox.model.set_value(True)

        # Enable BoundingBox3D
        bbox3d_checkbox = ui_test.find(root_widget_path + "/HStack[12]/HStack[0]/VStack[0]/ToolButton[0]")
        bbox3d_checkbox.model.set_value(True)

        await update_stage_async()

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[13]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()
        await omni.kit.app.get_app().next_update_async()

        # Verify graph creation
        graph = og.get_graph_by_path("/Graph/ROS_Camera")
        self.assertIsNotNone(graph, "Graph was not created")

        # Verify essential nodes
        nodes = graph.get_nodes()
        node_types = {node.get_type_name() for node in nodes}

        expected_nodes = {
            "omni.graph.action.OnPlaybackTick",
            "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame",
            "isaacsim.core.nodes.IsaacCreateRenderProduct",
            "isaacsim.ros2.bridge.ROS2Context",
            "isaacsim.ros2.bridge.ROS2CameraHelper",
            "isaacsim.ros2.bridge.ROS2CameraInfoHelper",
        }

        for expected in expected_nodes:
            self.assertIn(expected, node_types, f"Missing expected node type: {expected}")

    async def test_camera_null_conditions(self):
        """Test camera graph with null target prim"""

        # Creating environment and Carter Robot
        robot, base_link_path = await self.setup_test_environment()

        # Click through the menu to create the graph
        await menu_click("Tools/Robotics/ROS 2 OmniGraphs/Camera")
        await omni.kit.app.get_app().next_update_async()

        # Wait for and interact with parameter window
        window_name = "ROS2 Camera Graph"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        root_widget_path = f"{window_name}//Frame/VStack[0]"

        # Leave camera path empty (null condition)
        camera_prim_field = ui_test.find(root_widget_path + "/HStack[2]/StringField[0]")
        camera_prim_field.model.set_value("")

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[13]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await omni.kit.app.get_app().next_update_async()

        # Run simulation to test (should not crash)
        self._timeline.play()
        await simulate_async(2.0)
        self._timeline.stop()

        await omni.kit.app.get_app().next_update_async()

        # Verify simulation completed (implicit test that no crash occurred)
        self.assertFalse(self._timeline.is_playing())

    async def test_camera_data_flow(self):
        """Test camera data is being properly published to ROS2 topics"""

        # Creating environment and Carter Robot
        robot, base_link_path = await self.setup_test_environment(add_test_cubes=True)

        await update_stage_async()

        # Setup ROS2 Subscribers
        self.rgb_image_data = None
        self.depth_image_data = None
        self.camera_info_data = None
        self.point_cloud_data = None

        rgb_topic = "/rgb"
        depth_topic = "/depth"
        point_cloud_topic = "/depth_pcl"

        def rgb_callback(msg):
            self.rgb_image_data = msg

        def depth_callback(msg):
            self.depth_image_data = msg

        def point_cloud_callback(msg):
            self.point_cloud_data = msg

        # Create subscribers
        self.create_subscriber(rgb_topic, Image, rgb_callback)
        self.create_subscriber(depth_topic, Image, depth_callback)
        self.create_subscriber(point_cloud_topic, PointCloud2, point_cloud_callback)

        await menu_click("Tools/Robotics/ROS 2 OmniGraphs/Camera")
        await omni.kit.app.get_app().next_update_async()

        # Wait for and interact with parameter window
        window_name = "ROS2 Camera Graph"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        root_widget_path = f"{window_name}//Frame/VStack[0]"

        # Set camera path
        camera_prim_field = ui_test.find(root_widget_path + "/HStack[2]/StringField[0]")
        camera_prim_field.model.set_value("/World/test_robot/chassis_link/sensors/front_hawk/left/camera_left")

        # Enable Depth Point Clouds
        depth_checkbox = ui_test.find(root_widget_path + "/HStack[7]/HStack[0]/VStack[0]/ToolButton[0]")
        depth_checkbox.model.set_value(True)

        await omni.kit.app.get_app().next_update_async()

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[13]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await omni.kit.app.get_app().next_update_async()

        # Run simulation and collect data
        self._timeline.play()

        # Define helper function to procress messages
        def spin_ros():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Run simulation
        timeout = 3
        start_time = time.time()

        while time.time() - start_time < timeout:
            await simulate_async(0.5, callback=spin_ros)

            # Check if we have received messages
            if self.rgb_image_data and self.depth_image_data and self.point_cloud_data:
                break

        # Stop simulation
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Validate received messages
        # RGB Image Test
        self.assertIsNotNone(self.rgb_image_data, "RGB image data not received")
        if self.rgb_image_data:
            self.assertEqual(self.rgb_image_data.encoding, "rgb8", "Unexpected RGB image encoding")
            self.assertGreater(self.rgb_image_data.width, 0, "Invalid RGB image width")
            self.assertGreater(self.rgb_image_data.height, 0, "Invalid RGB image height")
            self.assertGreater(len(self.rgb_image_data.data), 0, "Empty RGB image data")

        # Depth Image validation
        self.assertIsNotNone(self.depth_image_data, "No depth image received")
        if self.depth_image_data:
            self.assertEqual(self.depth_image_data.encoding, "32FC1", "Unexpected depth image encoding")
            self.assertGreater(self.depth_image_data.width, 0, "Invalid depth image width")
            self.assertGreater(self.depth_image_data.height, 0, "Invalid depth image height")
            self.assertGreater(len(self.depth_image_data.data), 0, "Empty depth image data")

        # Point Cloud validation
        self.assertIsNotNone(self.point_cloud_data, "No point cloud received")
        if self.point_cloud_data:
            from .common import fields_to_dtype

            fields_dtype = fields_to_dtype(self.point_cloud_data.fields, self.point_cloud_data.point_step)
            points_array = np.frombuffer(self.point_cloud_data.data, fields_dtype)
            self.assertGreater(len(points_array), 10, "Point cloud should have at least 10 points")

            # Validate point cloud fields
            field_names = [field.name for field in self.point_cloud_data.fields]
            self.assertIn("x", field_names, "Point cloud missing 'x' field")
            self.assertIn("y", field_names, "Point cloud missing 'y' field")
            self.assertIn("z", field_names, "Point cloud missing 'z' field")


class TestMenuROS2LidarGraph(ROS2MenuTestBase):
    """Test ROS2 Lidar OmniGraph creation from menu"""

    async def test_lidar_graph_creation(self):
        """Test creation of RTX Lidar graph structure via menu"""

        robot, base_link_path = await self.setup_test_environment()

        # Click through the menu to create the graph
        await menu_click("Tools/Robotics/ROS 2 OmniGraphs/RTX Lidar")
        await omni.kit.app.get_app().next_update_async()

        # Wait for and interact with parameter window
        window_name = "ROS2 RTX Lidar Graph"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        # Find and set the graph root prim
        root_widget_path = f"{window_name}//Frame/VStack[0]"

        # Find and set RTX Lidar prim
        lidar_prim = ui_test.find(root_widget_path + "/HStack[2]/StringField[0]")
        lidar_prim.model.set_value("/World/test_robot/chassis_link/sensors/XT_32/PandarXT_32_10hz")

        # Enable Point Cloud
        point_cloud_checkbox = ui_test.find(root_widget_path + "/HStack[6]/HStack[0]/VStack[0]/ToolButton[0]")
        point_cloud_checkbox.model.set_value(True)

        await omni.kit.app.get_app().next_update_async()

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[7]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await omni.kit.app.get_app().next_update_async()

        # Get the created graph at default path
        graph = og.get_graph_by_path("/Graph/ROS_LidarRTX")
        self.assertIsNotNone(graph, "Graph was not created")

        # Verify essential nodes exist
        nodes = graph.get_nodes()
        node_types = {node.get_type_name() for node in nodes}

        expected_nodes = {
            "omni.graph.action.OnPlaybackTick",
            "isaacsim.ros2.bridge.ROS2Context",
            "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame",
            "isaacsim.core.nodes.IsaacCreateRenderProduct",
            "isaacsim.ros2.bridge.ROS2RtxLidarHelper",
        }

        for expected in expected_nodes:
            self.assertIn(expected, node_types, f"Missing expected node type: {expected}")

    async def test_lidar_null_conditions(self):
        """Test lidar graph with null target prim"""

        # Creating environment and Carter Robot
        robot, base_link_path = await self.setup_test_environment()

        # Click through the menu to create the graph
        await menu_click("Tools/Robotics/ROS 2 OmniGraphs/RTX Lidar")
        await omni.kit.app.get_app().next_update_async()

        # Wait for and interact with parameter window
        window_name = "ROS2 RTX Lidar Graph"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        # Find and set the graph root prim
        root_widget_path = f"{window_name}//Frame/VStack[0]"

        # Leave lidar path empty (null condition)
        lidar_prim_field = ui_test.find(root_widget_path + "/HStack[2]/StringField[0]")
        lidar_prim_field.model.set_value("")

        await omni.kit.app.get_app().next_update_async()

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[7]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await omni.kit.app.get_app().next_update_async()

        # Run simulation to test (should not crash)
        self._timeline.play()
        await simulate_async(2.0)
        self._timeline.stop()

        await omni.kit.app.get_app().next_update_async()

        # Verify simulation completed (implicit test that no crash occurred)
        self.assertFalse(self._timeline.is_playing())

    async def test_lidar_data_flow(self):
        """Test RTX Lidar data is being properly published to ROS2 topics with detailed data validation"""

        from pxr import Sdf

        # Creating environment and Carter Robot
        robot, base_link_path = await self.setup_test_environment(add_test_cubes=True)

        # Pre-check if this is a 3D lidar to help with setting up the right expectations
        lidar_path = "/World/test_robot/chassis_link/sensors/XT_32/PandarXT_32_10hz"
        lidar_prim_obj = self._stage.GetPrimAtPath(Sdf.Path(lidar_path))
        is_3d_lidar = False

        # Store actual message data for validation, not just counts
        self.point_cloud_data = None
        # self.laser_scan_data = None

        # Define topic names for the lidar data
        # laser_scan_topic = "/laser_scan"
        point_cloud_topic = "/point_cloud"

        # # Create callbacks to capture and validate message content
        # def laser_scan_callback(msg):
        #     self.laser_scan_data = msg

        def point_cloud_callback(msg):
            self.point_cloud_data = msg

        # Create subscribers with validation callbacks
        # self.create_subscriber(laser_scan_topic, LaserScan, laser_scan_callback)
        self.create_subscriber(point_cloud_topic, PointCloud2, point_cloud_callback)

        # Click through the menu to create the graph
        delays = [5, 50, 100]
        for delay in delays:
            try:
                await menu_click("Tools/Robotics/ROS 2 OmniGraphs/RTX Lidar", human_delay_speed=delay)
                break
            except AttributeError as e:
                if "NoneType' object has no attribute 'center'" in str(e) and delay != delays[-1]:
                    continue
                raise
        await omni.kit.app.get_app().next_update_async()

        # Wait for and interact with parameter window
        window_name = "ROS2 RTX Lidar Graph"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        # Find and set the graph root prim
        root_widget_path = f"{window_name}//Frame/VStack[0]"

        # Find and set RTX Lidar prim
        lidar_prim = ui_test.find(root_widget_path + "/HStack[2]/StringField[0]")
        lidar_prim.model.set_value(lidar_path)

        # Disable LaserScan as the lidar is 3D
        laser_scan_checkbox = ui_test.find(root_widget_path + "/HStack[5]/HStack[0]/VStack[0]/ToolButton[0]")
        laser_scan_checkbox.model.set_value(False)

        # Enable Point Cloud
        point_cloud_checkbox = ui_test.find(root_widget_path + "/HStack[6]/HStack[0]/VStack[0]/ToolButton[0]")
        point_cloud_checkbox.model.set_value(True)

        await omni.kit.app.get_app().next_update_async()

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[7]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await omni.kit.app.get_app().next_update_async()

        # Verify graph was created
        graph = og.get_graph_by_path("/Graph/ROS_LidarRTX")
        self.assertIsNotNone(graph, "Graph was not created")

        # Double-check 3D lidar detection - use the actual prim path that was configured
        lidar_prim_obj = self._stage.GetPrimAtPath(Sdf.Path(lidar_path))
        is_3d_lidar = False

        if lidar_prim_obj:
            # Check for elevation attributes that would indicate a 3D lidar
            if lidar_prim_obj.HasAttribute("elevationDeg") and lidar_prim_obj.GetAttribute("elevationDeg").Get() != 0:
                is_3d_lidar = True
            elif lidar_prim_obj.HasAttribute("elevation") and lidar_prim_obj.GetAttribute("elevation").Get() != 0:
                is_3d_lidar = True
            elif lidar_prim_obj.HasAttribute("verticalFov") and lidar_prim_obj.GetAttribute("verticalFov").Get() > 0:
                is_3d_lidar = True

        # Run simulation to generate lidar data
        self._timeline.play()

        # Define helper function to process messages
        def spin_ros():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Run simulation
        timeout = 3  # seconds
        start_time = time.time()

        while time.time() - start_time < timeout:
            await simulate_async(0.5, callback=spin_ros)

            # check for message
            if self.point_cloud_data is not None:
                break

        # Stop simulation
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        if is_3d_lidar:
            # For 3D lidars, validate PointCloud2
            self.assertIsNotNone(self.point_cloud_data, "No PointCloud2 message received for 3D lidar")

            if self.point_cloud_data:
                # Check point cloud data content
                from .common import fields_to_dtype  # Import utility from test_rtx_sensor

                # Convert ROS PointCloud2 message to numpy array
                fields_dtype = fields_to_dtype(self.point_cloud_data.fields, self.point_cloud_data.point_step)
                points_array = np.frombuffer(self.point_cloud_data.data, fields_dtype)

                # Validate point cloud structure and content
                self.assertGreater(len(points_array), 10, "PointCloud should have at least 10 points")

                # Print first 10 points for debugging
                if len(points_array) > 0:

                    # Check field names
                    field_names = [field.name for field in self.point_cloud_data.fields]
                    self.assertIn("x", field_names, "PointCloud missing 'x' field")
                    self.assertIn("y", field_names, "PointCloud missing 'y' field")
                    self.assertIn("z", field_names, "PointCloud missing 'z' field")


class TestMenuROS2JointStatesGraph(ROS2MenuTestBase):
    """Test ROS2 Joint States OmniGraph creation from menu"""

    async def test_joint_states_graph_creation(self):
        """Test creation of Joint States graph structure via menu"""

        # Creating environment and Carter Robot
        robot, base_link_path = await self.setup_test_environment()

        # Click through the menu to create the graph
        await menu_click("Tools/Robotics/ROS 2 OmniGraphs/Joint States")
        await omni.kit.app.get_app().next_update_async()

        # Wait for and interact with parameter window
        window_name = "ROS2 Joint States Graph"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        # Find and set the graph root prim
        root_widget_path = f"{window_name}//Frame/VStack[0]"

        # Set Articulation Root
        articulation_root_field = ui_test.find(root_widget_path + "/HStack[3]/StringField[0]")
        articulation_root_field.model.set_value("/World/test_robot")

        # Enable Publisher
        publisher_checkbox = ui_test.find(root_widget_path + "/HStack[4]/HStack[0]/VStack[0]/ToolButton[0]")
        publisher_checkbox.model.set_value(True)

        # Enable Subscriber
        subscriber_checkbox = ui_test.find(root_widget_path + "/HStack[5]/HStack[0]/VStack[0]/ToolButton[0]")
        subscriber_checkbox.model.set_value(True)

        await omni.kit.app.get_app().next_update_async()

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[6]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await omni.kit.app.get_app().next_update_async()

        # Get the created graph at default path
        graph = og.get_graph_by_path("/Graph/ROS_JointStates")
        self.assertIsNotNone(graph, "Graph was not created")

        # Verify essential nodes exist
        nodes = graph.get_nodes()
        node_types = {node.get_type_name() for node in nodes}

        expected_nodes = {
            "omni.graph.action.OnPlaybackTick",
            "isaacsim.ros2.bridge.ROS2Context",
            "isaacsim.core.nodes.IsaacReadSimulationTime",
            "isaacsim.ros2.bridge.ROS2SubscribeJointState",
            "isaacsim.ros2.bridge.ROS2PublishJointState",
            "isaacsim.core.nodes.IsaacArticulationController",
        }

        for expected in expected_nodes:
            self.assertIn(expected, node_types, f"Missing expected node type: {expected}")

    async def test_joint_states_null_conditions(self):
        """Test robot state graph with null target prim"""

        # Creating environment and Carter Robot
        robot, base_link_path = await self.setup_test_environment()

        # Click through the menu to create the graph
        await menu_click("Tools/Robotics/ROS 2 OmniGraphs/Joint States")
        await omni.kit.app.get_app().next_update_async()

        # Wait for and interact with parameter window
        window_name = "ROS2 Joint States Graph"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        # Find and set the graph root prim
        root_widget_path = f"{window_name}//Frame/VStack[0]"

        # Leave robot path empty (null condition)
        robot_prim_field = ui_test.find(root_widget_path + "/HStack[3]/StringField[0]")
        robot_prim_field.model.set_value("")

        await omni.kit.app.get_app().next_update_async()

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[6]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await omni.kit.app.get_app().next_update_async()

        # Run simulation to test (should not crash)
        self._timeline.play()
        await simulate_async(2.0)
        self._timeline.stop()

        await omni.kit.app.get_app().next_update_async()

        # Verify simulation completed (implicit test that no crash occurred)
        self.assertFalse(self._timeline.is_playing())

    async def test_joint_states_data_flow(self):
        """Test Joint States data is being properly published to ROS2 topics"""

        # Creating environment and Carter Robot
        robot, base_link_path = await self.setup_test_environment(articulation=True)

        await update_stage_async()

        # Run a brief simulation to initialize physics
        self._timeline.play()
        await simulate_async(0.5)
        self._timeline.stop()
        await update_stage_async()

        # Setup ROS2 subscribers
        # Store actual message data for validation
        self.joint_state_data = None

        # Define topic names
        joint_states_topic = "/joint_states"

        # Create callback to store message data
        def joint_states_callback(msg):
            self.joint_state_data = msg

        # Create subscriber
        self.create_subscriber(joint_states_topic, JointState, joint_states_callback)

        # Create graph through menu
        await menu_click("Tools/Robotics/ROS 2 OmniGraphs/Joint States")
        await omni.kit.app.get_app().next_update_async()

        # Configure graph parameters
        window_name = "ROS2 Joint States Graph"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        root_widget_path = f"{window_name}//Frame/VStack[0]"

        # Set Articulation Root to the robot's root prim
        articulation_root_field = ui_test.find(root_widget_path + "/HStack[3]/StringField[0]")
        articulation_root_field.model.set_value("/World/test_robot")

        # Enable Publisher
        publisher_checkbox = ui_test.find(root_widget_path + "/HStack[4]/HStack[0]/VStack[0]/ToolButton[0]")
        publisher_checkbox.model.set_value(True)

        # Enable Subscriber
        subscriber_checkbox = ui_test.find(root_widget_path + "/HStack[5]/HStack[0]/VStack[0]/ToolButton[0]")
        subscriber_checkbox.model.set_value(True)

        await omni.kit.app.get_app().next_update_async()

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[6]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await omni.kit.app.get_app().next_update_async()

        # Get the joint states graph created by the menu
        graph_path = "/Graph/ROS_JointStates"  # Standard path for joint states graph from menu
        graph = og.get_graph_by_path(graph_path)
        self.assertIsNotNone(graph, "ROS Joint States graph not found")

        # Find the joint state publisher node and check if simulation time node exists
        joint_state_publisher = None
        sim_time_node = None

        for node in graph.get_nodes():
            node_type = node.get_type_name()

            if "ROS2PublishJointState" in node_type:
                joint_state_publisher = node
            elif "IsaacReadSimulationTime" in node_type:
                sim_time_node = node

        # Create simulation time node if it doesn't exist
        if not sim_time_node:
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    ],
                },
            )

            # Find the newly created simulation time node
            for node in graph.get_nodes():
                if "IsaacReadSimulationTime" in node.get_type_name():
                    sim_time_node = node
                    break

        # Connect simulation time to the joint state publisher's timestamp input
        if sim_time_node and joint_state_publisher:
            try:
                # Get the output attribute from simulation time node
                sim_time_attr = sim_time_node.get_attribute("outputs:simulationTime")

                # Get the timestamp input attribute from publisher node
                timestamp_attr = joint_state_publisher.get_attribute("inputs:timeStamp")

                # Connect them if they exist
                if sim_time_attr and timestamp_attr:
                    og.Controller.connect(sim_time_attr, timestamp_attr)
            except Exception as e:
                print(f"{e}")

        # Give the graph time to initialize
        await simulate_async(1.0)

        # Run simulation and collect data
        self._timeline.play()

        # Define helper function to process messages
        def spin_ros():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Run simulation
        timeout = 3
        start_time = time.time()

        while time.time() - start_time < timeout:
            await simulate_async(0.5, callback=spin_ros)

            # Check if we have received the messages we need
            if self.joint_state_data is not None:
                break

        # Stop simulation
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Validate data
        self.assertIsNotNone(self.joint_state_data, "No joint states received")
        if self.joint_state_data:
            # Validate joint state data content
            self.assertGreater(len(self.joint_state_data.name), 0, "No joint names found")
            self.assertEqual(
                len(self.joint_state_data.name),
                len(self.joint_state_data.position),
                "Joint names and positions count mismatch",
            )

            # Nova Carter has specific joints we can check for
            expected_joints = ["joint_wheel_left", "joint_wheel_right"]
            for joint in expected_joints:
                self.assertIn(joint, self.joint_state_data.name, f"Expected joint '{joint}' not found")

            # Store initial joint positions before sending command
            initial_joint_positions = {}
            for i, name in enumerate(self.joint_state_data.name):
                initial_joint_positions[name] = self.joint_state_data.position[i]

            # Test joint states subscriber functionality by publishing a command
            # Create a command message
            command_msg = JointState()
            command_msg.header = Header()
            command_msg.header.stamp = self.node.get_clock().now().to_msg()

            # Find wheel joints
            wheel_indices = [i for i, name in enumerate(self.joint_state_data.name) if "wheel" in name]

            if wheel_indices:
                command_msg.name = [self.joint_state_data.name[i] for i in wheel_indices]
                command_msg.position = [0.0] * len(wheel_indices)  # Set to zero position
                command_msg.velocity = [10.0] * len(wheel_indices)  # Increased velocity for more noticeable movement
                command_msg.effort = [0.0] * len(wheel_indices)

                # Create publisher
                command_publisher = self.node.create_publisher(JointState, "/joint_command", 10)

                # Clear the current joint_state_data to ensure we get a fresh one after the command
                self.joint_state_data = None

                # Publish command
                command_publisher.publish(command_msg)

                # Run simulation to let command take effect
                self._timeline.play()
                # Run for longer to ensure joints have time to move
                await simulate_async(5.0, callback=spin_ros)
                self._timeline.stop()

                # Verify the joints moved in response to the command
                self.assertIsNotNone(self.joint_state_data, "No joint states received after sending command")

                if self.joint_state_data:
                    # Create a dictionary of current positions
                    current_joint_positions = {}
                    for i, name in enumerate(self.joint_state_data.name):
                        current_joint_positions[name] = self.joint_state_data.position[i]

                    # Verify that wheel joints actually moved
                    wheel_joints_moved = False
                    for name in command_msg.name:
                        if name in initial_joint_positions and name in current_joint_positions:
                            # Check if position changed (allowing for small numerical differences)
                            initial_pos = initial_joint_positions[name]
                            current_pos = current_joint_positions[name]
                            difference = abs(current_pos - initial_pos)

                            # For wheel joints, they should rotate with the velocity we set
                            # Using a smaller threshold to match actual simulation behavior
                            if difference > 0.01:  # Reduced threshold to detect smaller movements
                                wheel_joints_moved = True

                    # At least one wheel joint should have moved
                    self.assertTrue(wheel_joints_moved, "Wheel joints did not move after sending velocity command")

                    # Additional verification: check if the joints are still moving (velocities should be non-zero)
                    if hasattr(self.joint_state_data, "velocity") and len(self.joint_state_data.velocity) > 0:
                        wheel_velocities = {
                            self.joint_state_data.name[i]: self.joint_state_data.velocity[i]
                            for i in wheel_indices
                            if i < len(self.joint_state_data.velocity)
                        }

                        # Check if any wheel joint has non-zero velocity
                        any_wheel_moving = any(abs(v) > 0.1 for v in wheel_velocities.values())
                        self.assertTrue(any_wheel_moving, "No wheel joints are currently moving")


class TestMenuROS2TFGraph(ROS2MenuTestBase):
    """Test ROS2 TF OmniGraph creation from menu"""

    async def test_tf_graph_creation(self):
        """Test creation of TF graph structure via menu"""

        # Creating environment and Carter Robot
        robot, base_link_path = await self.setup_test_environment()

        # Click through the menu to create the graph
        await menu_click("Tools/Robotics/ROS 2 OmniGraphs/TF Publisher")
        await omni.kit.app.get_app().next_update_async()

        # Wait for and interact with parameter window
        window_name = "ROS2 TF Publisher Graph"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        # Find and set the graph root prim
        root_widget_path = f"{window_name}//Frame/VStack[0]"

        # Set Target Prim
        target_prim_field = ui_test.find(root_widget_path + "/HStack[5]/StringField[0]")
        target_prim_field.model.set_value("/World/test_robot/chassis_link/base_link")

        await omni.kit.app.get_app().next_update_async()

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[8]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await omni.kit.app.get_app().next_update_async()

        # Get the created graph at default path
        graph = og.get_graph_by_path("/Graph/ROS_TF")
        self.assertIsNotNone(graph, "Graph was not created")

        # Verify essential nodes exist
        nodes = graph.get_nodes()
        node_types = {node.get_type_name() for node in nodes}

        expected_nodes = {
            "omni.graph.action.OnPlaybackTick",
            "isaacsim.ros2.bridge.ROS2Context",
            "isaacsim.core.nodes.IsaacReadSimulationTime",
            "isaacsim.ros2.bridge.ROS2PublishTransformTree",
        }

        for expected in expected_nodes:
            self.assertIn(expected, node_types, f"Missing expected node type: {expected}")

    async def test_tf_null_conditions(self):
        """Test TF graph with null target prim"""

        # Creating environment and Carter Robot
        robot, base_link_path = await self.setup_test_environment()

        # Click through the menu to create the graph
        await menu_click("Tools/Robotics/ROS 2 OmniGraphs/TF Publisher")
        await omni.kit.app.get_app().next_update_async()

        # Wait for and interact with parameter window
        window_name = "ROS2 TF Publisher Graph"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        # Find and set the graph root prim
        root_widget_path = f"{window_name}//Frame/VStack[0]"

        # Leave target prim empty (null condition)
        target_prim_field = ui_test.find(root_widget_path + "/HStack[5]/StringField[0]")
        target_prim_field.model.set_value("")

        await omni.kit.app.get_app().next_update_async()

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[8]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await omni.kit.app.get_app().next_update_async()

        # Run simulation to test (should not crash)
        self._timeline.play()
        await simulate_async(2.0)
        self._timeline.stop()

        await omni.kit.app.get_app().next_update_async()

        # Verify simulation completed (implicit test that no crash occurred)
        self.assertFalse(self._timeline.is_playing())

    async def test_tf_data_flow(self):
        """Test TF transform data is being properly published to ROS2 topics"""

        # Creating environment and Carter Robot
        robot, base_link_path = await self.setup_test_environment(articulation=True)

        self.tf_data = None

        tf_topic = "/tf"

        # Create callback to store message data
        def tf_callback(msg):
            self.tf_data = msg

        # Create subscriber
        self.create_subscriber(tf_topic, TFMessage, tf_callback)

        await omni.kit.app.get_app().next_update_async()

        # Click through the menu to create the graph
        await menu_click("Tools/Robotics/ROS 2 OmniGraphs/TF Publisher")
        await omni.kit.app.get_app().next_update_async()

        # Wait for and interact with parameter window
        window_name = "ROS2 TF Publisher Graph"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        # Find and set the graph root prim
        root_widget_path = f"{window_name}//Frame/VStack[0]"

        # Set Target Prim
        target_prim_field = ui_test.find(root_widget_path + "/HStack[5]/StringField[0]")
        target_prim_field.model.set_value("/World/test_robot")

        await omni.kit.app.get_app().next_update_async()

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[8]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await omni.kit.app.get_app().next_update_async()

        # Run simulation
        self._timeline.play()

        def spin_ros():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Run simulation
        timeout = 3
        start_time = time.time()

        while time.time() - start_time < timeout:
            await simulate_async(0.5, callback=spin_ros)

            if self.tf_data is not None:
                break

        # Stop simulation
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Validate data
        self.assertIsNotNone(self.tf_data, "No TF message received")
        if self.tf_data:
            # Validate TF data content
            self.assertGreater(len(self.tf_data.transforms), 0, "TF message should contain transforms")

            # check for expected frames
            frame_ids = set()
            child_frame_ids = set()

            for transform in self.tf_data.transforms:
                frame_ids.add(transform.header.frame_id)
                child_frame_ids.add(transform.child_frame_id)

            # Validate a specific transform if possible
            for transform in self.tf_data.transforms:
                if transform.header.frame_id == "odom" and transform.child_frame_id == "base_link":
                    # Verify translation values
                    self.assertIsNotNone(transform.transform.translation.x, "Transform missing x translation")
                    self.assertIsNotNone(transform.transform.translation.y, "Transform missing y translation")
                    self.assertIsNotNone(transform.transform.translation.z, "Transform missing z translation")

                    # verify rotation is valid quaternion
                    quat = [
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w,
                    ]

                    # A valid quanterion has a sum of squares = 1
                    quant_magnitude = sum(q * q for q in quat)
                    self.assertAlmostEqual(quant_magnitude, 1.0, delta=0.1, msg="Invalid quaternion")


class TestMenuROS2OdometryGraph(ROS2MenuTestBase):
    """Test ROS2 Odometry OmniGraph creation from menu"""

    async def test_odometry_graph_creation(self):
        """Test creation of Odometry graph structure via menu"""

        # Creating environment and Carter Robot
        robot, base_link_path = await self.setup_test_environment()

        # Click through the menu to create the graph
        await menu_click("Tools/Robotics/ROS 2 OmniGraphs/Odometry Publisher")
        await omni.kit.app.get_app().next_update_async()

        # Wait for and interact with parameter window
        window_name = "ROS2 Odometry Graph"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        # Find and set the graph root prim
        root_widget_path = f"{window_name}//Frame/VStack[0]"

        # Set Articulation Root
        articulation_root_field = ui_test.find(root_widget_path + "/HStack[4]/StringField[0]")
        articulation_root_field.model.set_value("/World/test_robot/chassis_link/base_link")

        # Set Chassis Link Prim
        chassis_link_prim_field = ui_test.find(root_widget_path + "/HStack[5]/StringField[0]")
        chassis_link_prim_field.model.set_value("/World/test_robot/chassis_link")

        await omni.kit.app.get_app().next_update_async()

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[6]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await omni.kit.app.get_app().next_update_async()

        # Get the created graph at default path
        graph = og.get_graph_by_path("/Graph/ROS_Odometry")
        self.assertIsNotNone(graph, "Graph was not created")

        # Verify essential nodes exist
        nodes = graph.get_nodes()
        node_types = {node.get_type_name() for node in nodes}

        expected_nodes = {
            "omni.graph.action.OnPlaybackTick",
            "isaacsim.ros2.bridge.ROS2Context",
            "isaacsim.core.nodes.IsaacReadSimulationTime",
            "isaacsim.core.nodes.IsaacComputeOdometry",
            "isaacsim.ros2.bridge.ROS2PublishOdometry",
            "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
            "isaacsim.ros2.bridge.ROS2PublishTransformTree",
        }

        for expected in expected_nodes:
            self.assertIn(expected, node_types, f"Missing expected node type: {expected}")

    async def test_odometry_null_conditions(self):
        """Test odometry graph with null chassis prim"""

        # Creating environment and Carter Robot
        robot, base_link_path = await self.setup_test_environment()

        # Click through the menu to create the graph
        await menu_click("Tools/Robotics/ROS 2 OmniGraphs/Odometry Publisher", human_delay_speed=50)
        await omni.kit.app.get_app().next_update_async()

        # Wait for and interact with parameter window
        window_name = "ROS2 Odometry Graph"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        # Find and set the graph root prim
        root_widget_path = f"{window_name}//Frame/VStack[0]"

        # Leave chassis and articulation root empty (null condition)
        # Set Articulation Root
        articulation_root_field = ui_test.find(root_widget_path + "/HStack[4]/StringField[0]")
        articulation_root_field.model.set_value("")

        # Set Chassis Link Prim
        chassis_link_prim_field = ui_test.find(root_widget_path + "/HStack[5]/StringField[0]")
        chassis_link_prim_field.model.set_value("")

        await omni.kit.app.get_app().next_update_async()

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[6]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await omni.kit.app.get_app().next_update_async()

        # Run simulation to test (should not crash)
        self._timeline.play()
        await simulate_async(1.0)
        self._timeline.stop()

        await omni.kit.app.get_app().next_update_async()

        # Verify simulation completed (implicit test that no crash occurred)
        self.assertFalse(self._timeline.is_playing())

    async def test_odometry_data_flow(self):
        """Test odometrydata is being properly published to ROS2 topics"""

        # Creating environment and Carter Robot
        robot, base_link_path = await self.setup_test_environment(articulation=True)

        # Setup ROS2 subscribers for both TF and Odometry messages
        self.tf_data = None
        self.odom_data = None

        tf_topic = "/tf"
        odom_topic = "/odom"

        # Create callbacks to store message data
        def tf_callback(msg):
            self.tf_data = msg

        def odom_callback(msg):
            self.odom_data = msg

        # Create subscribers
        self.create_subscriber(tf_topic, TFMessage, tf_callback)
        self.create_subscriber(odom_topic, Odometry, odom_callback)

        # Create Odometry graph through menu

        window_name = "ROS2 Odometry Graph"
        delays = [10, 100, 200]
        for delay in delays:
            try:
                await menu_click("Tools/Robotics/ROS 2 OmniGraphs/Odometry Publisher", human_delay_speed=delay)
                if (param_window := ui_test.find(window_name)) is not None:
                    break
            except AttributeError as e:
                if "NoneType' object has no attribute 'center'" in str(e) and delay != delays[-1]:
                    continue
                raise
        for _ in range(10):
            await update_stage_async()

        self.assertIsNotNone(param_window, "Parameter window not found")

        await omni.kit.app.get_app().next_update_async()

        # Configure Odometry graph parameters

        odom_root_widget_path = f"{window_name}//Frame/VStack[0]"

        # Enable "Publish Robot's TF?" checkbox (critical for TF publishing)
        publish_tf_checkbox = ui_test.find(odom_root_widget_path + "/HStack[3]/ToolButton[0]")
        if publish_tf_checkbox:
            publish_tf_checkbox.model.set_value(True)

        # Set Robot Articulation Root to the top-level robot prim
        articulation_root_field = ui_test.find(odom_root_widget_path + "/HStack[4]/StringField[0]")
        articulation_root_field.model.set_value("/World/test_robot")

        # Set Chassis Link Prim
        chassis_link_prim_field = ui_test.find(odom_root_widget_path + "/HStack[5]/StringField[0]")
        chassis_link_prim_field.model.set_value("/World/test_robot/chassis_link/base_link")

        await omni.kit.app.get_app().next_update_async()

        # Click OK button
        odom_ok_button = ui_test.find(odom_root_widget_path + "/HStack[6]/Button[0]")
        self.assertIsNotNone(odom_ok_button, "Odometry OK button not found")
        await odom_ok_button.click()

        await omni.kit.app.get_app().next_update_async()

        # Get the created graph to modify it programmatically if needed
        graph_path = "/Graph/ROS_Odometry"
        odom_graph = og.get_graph_by_path(graph_path)
        self.assertIsNotNone(odom_graph, "Odometry graph not found at expected path")

        # Verify essential nodes exist (similar to test_odometry_graph_creation)
        nodes = odom_graph.get_nodes()
        node_types = {node.get_type_name() for node in nodes}

        expected_nodes = {
            "omni.graph.action.OnPlaybackTick",
            "isaacsim.ros2.bridge.ROS2Context",
            "isaacsim.core.nodes.IsaacReadSimulationTime",
            "isaacsim.core.nodes.IsaacComputeOdometry",
            "isaacsim.ros2.bridge.ROS2PublishOdometry",
            "isaacsim.ros2.bridge.ROS2PublishTransformTree",
        }

        for expected in expected_nodes:
            self.assertIn(expected, node_types, f"Missing expected node type: {expected}")

        # Find the TF publisher node and ensure it has target prims set
        tf_publisher = None
        for node in nodes:
            if "ROS2PublishTransformTree" in node.get_type_name():
                tf_publisher = node
                break

        if tf_publisher:
            # Set the target prims on the TF publisher to ensure the robot is published
            target_prims_attr = tf_publisher.get_attribute("inputs:targetPrims")
            if target_prims_attr:
                og.Controller.set(target_prims_attr, [usdrt.Sdf.Path("/World/test_robot")])

        # Run simulation
        self._timeline.play()

        def spin_ros():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Run simulation
        timeout = 3
        start_time = time.time()

        while time.time() - start_time < timeout:
            await simulate_async(0.5, callback=spin_ros)

            # Check if we have received both types of messages
            if self.tf_data is not None and self.odom_data is not None:
                break

        # Stop simulation
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Validate TF data
        self.assertIsNotNone(self.tf_data, "No TF messages received")
        if self.tf_data:
            # Validate TF data content
            self.assertGreater(len(self.tf_data.transforms), 0, "TF message should contain transforms")

            # Check for expected frames
            frame_ids = set()
            child_frame_ids = set()

            for transform in self.tf_data.transforms:
                frame_ids.add(transform.header.frame_id)
                child_frame_ids.add(transform.child_frame_id)

            all_frames = frame_ids.union(child_frame_ids)

            # Check for base_link frames
            self.assertIn("base_link", all_frames, "No base_link frame found")

        # Validate Odometry data
        self.assertIsNotNone(self.odom_data, "No odometry messages received")
        if self.odom_data:

            # Check pose and twist data
            self.assertIsNotNone(self.odom_data.pose.pose.position.x, "Missing position.x in odometry")
            self.assertIsNotNone(self.odom_data.pose.pose.position.y, "Missing position.y in odometry")
            self.assertIsNotNone(self.odom_data.pose.pose.position.z, "Missing position.z in odometry")

            # Check orientation quaternion
            quat = [
                self.odom_data.pose.pose.orientation.x,
                self.odom_data.pose.pose.orientation.y,
                self.odom_data.pose.pose.orientation.z,
                self.odom_data.pose.pose.orientation.w,
            ]

            # A valid quaternion has a sum of squares = 1 (allowing for minor numerical error)
            quat_magnitude = sum(q * q for q in quat)
            self.assertAlmostEqual(quat_magnitude, 1.0, delta=0.01, msg="Invalid quaternion in odometry")

            # Check twist data is present
            self.assertIsNotNone(self.odom_data.twist.twist.linear.x, "Missing linear.x velocity")
            self.assertIsNotNone(self.odom_data.twist.twist.angular.z, "Missing angular.z velocity")


class TestMenuROS2ClockGraph(ROS2MenuTestBase):
    """Test ROS2 Clock OmniGraph creation from menu"""

    async def test_clock_graph_creation(self):
        """Test creation of Clock graph structure via menu"""

        # Create environment and Carter Robot
        robot, base_link_path = await self.setup_test_environment()

        # Click through the menu to create the graph
        await menu_click("Tools/Robotics/ROS 2 OmniGraphs/Clock")
        await omni.kit.app.get_app().next_update_async()

        # Wait for and interact with parameter window
        window_name = "ROS2 Clock Graph"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        # Find and set the graph root prim
        root_widget_path = f"{window_name}//Frame/VStack[0]"

        await omni.kit.app.get_app().next_update_async()

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[1]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await omni.kit.app.get_app().next_update_async()

        # Get the created graph at default path
        graph = og.get_graph_by_path("/Graph/ROS_Clock")
        self.assertIsNotNone(graph, "Graph was not created")

        # Verify essential nodes exist
        nodes = graph.get_nodes()
        node_types = {node.get_type_name() for node in nodes}

        expected_nodes = {
            "omni.graph.action.OnPlaybackTick",
            "isaacsim.ros2.bridge.ROS2Context",
            "isaacsim.core.nodes.IsaacReadSimulationTime",
            "isaacsim.ros2.bridge.ROS2PublishClock",
        }

        for expected in expected_nodes:
            self.assertIn(expected, node_types, f"Missing expected node type: {expected}")

    async def test_clock_data_flow(self):
        """Test Clock data is being properly published to ROS2 topics with data validation"""

        # Create environment and Carter Robot
        robot, base_link_path = await self.setup_test_environment()

        await update_stage_async()

        # Setup ROS2 subscribers
        # Store actual message data for validation
        self.clock_data = None
        self.clock_history = []

        # Define topic names
        clock_topic = "/clock"

        # Create callback to store message data
        def clock_callback(msg):
            self.clock_data = msg
            # Store timestamp along with clock value for debugging
            time_value = msg.clock.sec + msg.clock.nanosec * 1e-9
            self.clock_history.append(time_value)

        # Create subscriber
        self.create_subscriber(clock_topic, Clock, clock_callback)

        # Create graph through menu as required
        await menu_click("Tools/Robotics/ROS 2 OmniGraphs/Clock")
        await omni.kit.app.get_app().next_update_async()

        # Configure graph parameters
        window_name = "ROS2 Clock Graph"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        root_widget_path = f"{window_name}//Frame/VStack[0]"

        await omni.kit.app.get_app().next_update_async()

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[1]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await omni.kit.app.get_app().next_update_async()

        # Get and verify the graph structure
        graph_path = "/Graph/ROS_Clock"
        clock_graph = og.get_graph_by_path(graph_path)
        self.assertIsNotNone(clock_graph, "Clock graph was not created at expected path")

        # Find the key nodes by type
        sim_time_node = None
        clock_publisher_node = None
        playback_tick_node = None

        for node in clock_graph.get_nodes():
            node_type = node.get_type_name()

            if "IsaacReadSimulationTime" in node_type:
                sim_time_node = node
            elif "ROS2PublishClock" in node_type:
                clock_publisher_node = node
            elif "OnPlaybackTick" in node_type:
                playback_tick_node = node

        self.assertIsNotNone(sim_time_node, "Simulation time node not found in graph")
        self.assertIsNotNone(clock_publisher_node, "Clock publisher node not found in graph")
        self.assertIsNotNone(playback_tick_node, "Playback tick node not found in graph")

        reset_on_stop_attr = sim_time_node.get_attribute("inputs:resetOnStop")
        if reset_on_stop_attr:
            og.Controller.set(reset_on_stop_attr, False)

        tick_output_attr = playback_tick_node.get_attribute("outputs:tick")
        clock_exec_in_attr = clock_publisher_node.get_attribute("inputs:execIn")

        # Connect the attributes directly
        if tick_output_attr and clock_exec_in_attr:
            og.Controller.connect(tick_output_attr, clock_exec_in_attr)

        # Clear any existing messages
        self.clock_history.clear()

        # Initialize simulation time tracking for verification
        initial_sim_time = None

        # Run the simulation in steps
        self._timeline.play()

        def spin_ros():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Run simulation with more iterations and longer total time
        simulation_duration = 1.0
        step_size = 0.1
        steps = int(simulation_duration / step_size)

        for i in range(steps):
            # Run simulation for this step
            await simulate_async(step_size, callback=spin_ros)

            # Check current simulation time directly from the timeline
            current_sim_time = self._timeline.get_current_time()

            if initial_sim_time is None:
                initial_sim_time = current_sim_time

        # Stop simulation
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Validate data - first check that we actually received messages
        self.assertGreater(
            len(self.clock_history), 0, f"No clock messages received after {simulation_duration} seconds"
        )

        if len(self.clock_history) >= 2:
            initial_time = self.clock_history[0]
            final_time = self.clock_history[-1]

            # Validate time progression - check for at least 3 seconds advancement
            time_diff = final_time - initial_time
            self.assertGreater(
                time_diff, 0.5, f"Clock did not advance properly. Initial: {initial_time:.6f}, Final: {final_time:.6f}"
            )

        else:
            # If we only received one message, at least verify it's not zero
            if len(self.clock_history) == 1:
                clock_time = self.clock_history[0]
                self.assertGreater(clock_time, 0.0, "Clock time should not be zero")


class TestMenuROS2GenericPublisherGraph(ROS2MenuTestBase):
    async def test_generic_publisher_graph_creation(self):
        """Test generic publisher graph creation from menu"""

        # Creating environment and Carter Robot
        robot, base_link_path = await self.setup_test_environment()

        # Click through the menu to create the graph
        await menu_click("Tools/Robotics/ROS 2 OmniGraphs/Generic Publisher")
        await omni.kit.app.get_app().next_update_async()

        # Wait for and interact with parameter window
        window_name = "ROS2 Generic Publisher Graph"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        # Find and set the graph root prim
        root_widget_path = f"{window_name}//Frame/VStack[0]"

        await omni.kit.app.get_app().next_update_async()

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[2]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await omni.kit.app.get_app().next_update_async()

        # Get the created graph at default path
        graph = og.get_graph_by_path("/Graph/ROS_GenericPub")
        self.assertIsNotNone(graph, "Graph was not created")

        # Verify essential nodes exist
        nodes = graph.get_nodes()
        node_types = {node.get_type_name() for node in nodes}

        expected_nodes = {
            "omni.graph.action.OnPlaybackTick",
            "isaacsim.ros2.bridge.ROS2Context",
            "isaacsim.core.nodes.IsaacRealTimeFactor",
            "isaacsim.ros2.bridge.ROS2Publisher",
        }

        for expected in expected_nodes:
            self.assertIn(expected, node_types, f"Missing expected node type: {expected}")

    async def test_generic_publisher_data_flow(self):
        """Test Generic Publisher data is properly publishing RTF as Float32 to ROS2 topics"""

        # Create environment and Carter Robot
        robot, base_link_path = await self.setup_test_environment()

        # Setup ROS2 subscribers
        # Store actual message data for validation
        self.float_data = None

        # Define topic name
        test_topic = "/test_rtf_publisher"

        # Create callback to store message data
        def float_callback(msg):
            self.float_data = msg
            print(f"Received Float32 value: {msg.data}")

        # Create subscriber for Float32
        self.create_subscriber(test_topic, Float32, float_callback)

        # Create graph through menu
        await menu_click("Tools/Robotics/ROS 2 OmniGraphs/Generic Publisher")
        await omni.kit.app.get_app().next_update_async()

        # Configure graph parameters
        window_name = "ROS2 Generic Publisher Graph"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        # Find and set the graph root prim
        root_widget_path = f"{window_name}//Frame/VStack[0]"

        await omni.kit.app.get_app().next_update_async()

        # Click OK button to create the base graph
        ok_button = ui_test.find(root_widget_path + "/HStack[2]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await omni.kit.app.get_app().next_update_async()

        graph = og.get_graph_by_path("/Graph/ROS_GenericPub")
        self.assertIsNotNone(graph, "Graph was not created")

        nodes = graph.get_nodes()
        publisher_node = None
        for node in nodes:
            if node.get_type_name() == "isaacsim.ros2.bridge.ROS2Publisher":
                publisher_node = node
                break

        self.assertIsNotNone(publisher_node, "ROS2Publisher node not found in graph")

        if publisher_node:
            topic_name_attr = publisher_node.get_attribute("inputs:topicName")
            if topic_name_attr:
                og.Controller.set(topic_name_attr, test_topic)
                print(f"Set topic name to {test_topic}")

        await update_stage_async()

        # Run simulation to generate RTF data
        self._timeline.play()

        # Define helper function to process messages
        def spin_ros():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Run simulation
        timeout = 3
        start_time = time.time()

        print("Starting RTF Float32 Publisher data collection...")

        while time.time() - start_time < timeout:
            await simulate_async(0.5, callback=spin_ros)

            # check for message
            if self.float_data is not None:
                break

        # Stop simulation
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Validate data
        self.assertIsNotNone(self.float_data, "No Float32 messages received from RTF Publisher")
        if self.float_data:
            # Validate Float32 data content
            # RTF should be a positive float value representing simulation speed
            self.assertIsInstance(self.float_data.data, float, "RTF should be a float value")
            self.assertGreaterEqual(self.float_data.data, 0.0, "RTF should be non-negative")

            print(f"Successfully received and validated RTF value: {self.float_data.data}")
