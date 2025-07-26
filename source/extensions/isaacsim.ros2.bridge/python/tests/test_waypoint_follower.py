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

import asyncio

import omni.appwindow
import omni.ext
import omni.graph.core as og
import omni.kit.commands
import omni.kit.test
import omni.kit.viewport.utility
import omni.usd
from isaacsim.core.utils.physics import simulate_async
from isaacsim.core.utils.prims import delete_prim, get_prim_at_path
from isaacsim.core.utils.stage import create_new_stage_async, get_next_free_path
from pxr import Gf, UsdGeom

from .common import ROS2TestCase

WAYPOINT_SCRIPT = """
import rclpy
import threading
import time
from ast import literal_eval

try:
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    from geometry_msgs.msg import PoseStamped
except:
    pass

import carb.events
import omni.kit.app
from isaacsim.core.nodes import BaseResetNode
from omni.kit.notification_manager import NotificationStatus, post_notification

class WaypointFollower(BaseResetNode):
    def __init__(self, db: og.Database):
        self._navigator = None
        self._db = db
        super().__init__(initialize=False)

    def initialize_ros2_node(self):
        try:
            rclpy.init()
            self._navigator = BasicNavigator()
        except:
            pass
        self.initialized = True

    def create_pose_stamped_msg(self, waypoint, frame_id):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self._navigator.get_clock().now().to_msg()
        pose.pose.position.x = waypoint[0]
        pose.pose.position.y = waypoint[1]
        pose.pose.position.z = waypoint[2]
        pose.pose.orientation.w = waypoint[3]
        pose.pose.orientation.x = waypoint[4]
        pose.pose.orientation.y = waypoint[5]
        pose.pose.orientation.z = waypoint[6]
        return pose

    def start_waypoint_follower(self):
        # We are not starting navigation stack
        # try:
        #     # Avoid setting bogus initial pose
        #     self._navigator.initial_pose_received = True
        #     # Wait until the full navigation system is up and running.
        #     self._navigator.waitUntilNav2Active()
        # except Exception as e:
        #     post_notification("WaypointFollower Stopped!", status=NotificationStatus.WARNING)
        #     return

        waypoint = None
        result = None

        data = self._db.inputs.waypoints
        # parse string of lists containing waypoints
        parsed_waypoints = [literal_eval(s) for s in data]
        try:
            for transform in parsed_waypoints:
                translate = transform[0]
                orient = transform[1]
                _waypoint = translate + orient

                waypoint = self.create_pose_stamped_msg(_waypoint, self._db.inputs.frame_id)
        except Exception as e:
            return

        post_notification("Moving towards waypoint", status=NotificationStatus.INFO)

        try:
            # Call NavigateToPose action server with a goal/waypoint
            self._navigator.goToPose(waypoint)
            time_diff = 10
            past_time = time.time()

            # Check if task is completed or not
            while not self._navigator.isTaskComplete():
                feedback = self._navigator.getFeedback()
                if time.time() - past_time > time_diff:
                    post_notification(f'{feedback}', status=NotificationStatus.INFO)
                    past_time = time.time()
            result = self._navigator.getResult()
        except Exception as e:
            post_notification("WaypointFollower Stopped!", status=NotificationStatus.WARNING)
            return
        try:
            # unlock for next goal
            self._db.per_instance_state.lock = False

            if result == TaskResult.SUCCEEDED:
                post_notification("Goal succeeded", status=NotificationStatus.INFO)
                print("Goal succeeded")
            elif result == TaskResult.CANCELED:
                post_notification("Goal canceled", status=NotificationStatus.WARNING)
                print("Goal canceled")
            elif result == TaskResult.FAILED:
                post_notification("Goal failed!", status=NotificationStatus.WARNING)
                print("Goal canceled")
        except:
            pass

    # Overriding a function from BaseResetNode.
    # This is automatically called when simulation is stopped.
    # This will also be called when the OmniGraph node is released.
    def custom_reset(self):
        try:
            if self._navigator:
                self._navigator.destroy_node()
                # Remove try-except block once below fix is reflected in debian file
                # nav2_simple_commander/nav2_simple_commander/robot_navigator.py
                # https://github.com/ros-navigation/navigation2/issues/4696
                try:
                    self._navigator.assisted_teleop_client.destroy()
                except Exception as ex:
                    print("Error while destroying Assisted Teleop",ex)
                self._navigator = None

                self.initialized = False

                rclpy.try_shutdown()

            global waypoint_follower
            waypoint_follower = None
        except Exception as e:
            post_notification("Node is reset!", status=NotificationStatus.WARNING)

def test_waypoint_script(db: og.Database):
    import rclpy
    from std_msgs.msg import String

    try:
        rclpy.init()
    except:
        pass

    node = rclpy.create_node('waypoint_publisher')

    # Create a publisher that publishes String messages on the 'topic' topic
    publisher = node.create_publisher(String, 'topic', 10)

    msg = String()
    data = db.inputs.waypoints

    msg.data = f'Waypoint Mode: {db.inputs.waypoints}'

    import time
    end_time = time.time() + 5
    while time.time() < end_time:
        try:
            publisher.publish(msg)
            print(f"published: {msg.data}")
            time.sleep(0.5)
        except:
            print("Context was destroyed")
            break

    node.destroy_node()
    try:
        rclpy.shutdown()
    except:
        pass

def setup(db: og.Database):
    # To lock the waypoint mode if it is already running
    db.per_instance_state.lock = False
    global waypoint_follower
    waypoint_follower = None

def cleanup(db: og.Database):
    db.per_instance_state.lock = None

def compute(db: og.Database):
    if db:
        test_waypoint_script(db)
        return True

    # global waypoint_follower

    # # Reset script node's states if simulation is stopped
    # if db.inputs.reset_state == 1:
    #     db.per_instance_state.lock = False
    #     waypoint_follower = None
    #     return True

    # if not db.per_instance_state.lock and db.inputs.reset_state == 0:
    #     db.per_instance_state.lock = True

    #     # WaypointFollower object must be created only once in execution
    #     if waypoint_follower == None:
    #         waypoint_follower = WaypointFollower(db)
    #         waypoint_follower.initialize_ros2_node()

    #     # send another goal only if previous task is executed
    #     if waypoint_follower._navigator != None and waypoint_follower._navigator.isTaskComplete():
    #         post_notification("Running Waypoint Mode", status=NotificationStatus.INFO)
    #         try:
    #             thread = threading.Thread(target=waypoint_follower.start_waypoint_follower)
    #             thread.start()
    #         except Exception as e:
    #             print(e)
    # else:
    #     post_notification("Previous waypoint is in progress!", status=NotificationStatus.WARNING)

    # return True

"""

PATROLLING_SCRIPT = """
import rclpy
import threading
import time
from ast import literal_eval

try:
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    from geometry_msgs.msg import PoseStamped
except:
    pass

import carb.events
import omni.kit.app
from isaacsim.core.nodes import BaseResetNode
from omni.kit.notification_manager import NotificationStatus, post_notification

class Patrolling(BaseResetNode):
    def __init__(self, db: og.Database):
        self._navigator = None
        self._counter = 1
        self._db = db
        super().__init__(initialize=False)

    def initialize_ros2_node(self):
        try:
            rclpy.init()
            self._navigator = BasicNavigator()
        except:
            pass
        self.initialized = True

    def create_pose_stamped_msg(self, waypoint, frame_id):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self._navigator.get_clock().now().to_msg()
        pose.pose.position.x = waypoint[0]
        pose.pose.position.y = waypoint[1]
        pose.pose.position.z = waypoint[2]
        pose.pose.orientation.x = waypoint[3]
        pose.pose.orientation.y = waypoint[4]
        pose.pose.orientation.z = waypoint[5]
        pose.pose.orientation.w = waypoint[6]
        return pose

    def start_patrolling(self):
        # We are not starting navigation stack
        # try:
        #     # Avoid setting bogus initial pose
        #     self._navigator.initial_pose_received = True
        #     # Wait until the full navigation system is up and running
        #     self._navigator.waitUntilNav2Active()
        # except Exception as e:
        #     post_notification("Patrolling Stopped!", status=NotificationStatus.WARNING)
        #     return

        waypoints = []
        result = None

        data = self._db.inputs.waypoints
        # parse string of lists containing waypoints
        parsed_waypoints = [literal_eval(s) for s in data]
        try:
            for transform in parsed_waypoints:
                translate = transform[0]
                orient = transform[1]
                _waypoint = translate + orient

                waypoint = self.create_pose_stamped_msg(_waypoint, self._db.inputs.frame_id)
                waypoints.append(waypoint)
        except Exception as e:
            return

        post_notification(f'Round {self._counter} in progress..', status=NotificationStatus.INFO)

        while rclpy.ok():
            try:
                # Call FollowWaypoints action server with list of waypoints
                self._navigator.followWaypoints(waypoints)
                time_diff = 10
                past_time = time.time()

                # Check if task is completed or not
                while not self._navigator.isTaskComplete():
                    feedback = self._navigator.getFeedback()
                    if time.time() - past_time > time_diff:
                        post_notification(f'{feedback}', status=NotificationStatus.INFO)
                        past_time = time.time()
                result = self._navigator.getResult()
            except Exception as e:
                post_notification("Patrolling Stopped!", status=NotificationStatus.WARNING)
                return
            try:
                if result == TaskResult.SUCCEEDED:
                    post_notification(f'Round {self._counter} is completed', status=NotificationStatus.INFO)
                    print(f'Round {self._counter} is completed')
                    self._counter = self._counter + 1
                else:
                    post_notification(f'Round {self._counter} is either Failed or Cancelled!', status=NotificationStatus.WARNING)
                    print(f'Round {self._counter} is completed')
                    break
            except:
                pass

    # Overriding a function from BaseResetNode.
    # This is automatically called when simulation is stopped.
    # This will also be called when the OmniGraph node is released.
    def custom_reset(self):
        try:
            if self._navigator:
                self._navigator.destroy_node()
                # Remove try-except block once below fix is reflected in debian file
                # nav2_simple_commander/nav2_simple_commander/robot_navigator.py
                # https://github.com/ros-navigation/navigation2/issues/4696
                try:
                    self._navigator.assisted_teleop_client.destroy()
                except Exception as ex:
                    print("Error while destroying Assisted Teleop",ex)
                self._navigator = None

                self.initialized = False

                rclpy.try_shutdown()
        except Exception as e:
            post_notification("Node is reset!", status=NotificationStatus.WARNING)

def test_patrolling_script(db: og.Database):
    import rclpy
    from std_msgs.msg import String

    try:
        rclpy.init()
    except:
        pass

    node = rclpy.create_node('patrolling_publisher')

    # Create a publisher that publishes String messages on the 'topic' topic
    publisher = node.create_publisher(String, 'topic', 10)

    msg = String()
    data = db.inputs.waypoints

    msg.data = f'Patrolling Mode: {db.inputs.waypoints}'

    import time
    end_time = time.time() + 5
    while time.time() < end_time:
        try:
            publisher.publish(msg)
            print(f"published: {msg.data}")
            time.sleep(0.5)
        except:
            print("Context was destroyed")
            break

    node.destroy_node()
    try:
        rclpy.shutdown()
    except:
        pass

def setup(db: og.Database):
    # To lock the patrolling node if it is already running
    db.per_instance_state.initialized = False

def cleanup(db: og.Database):
    db.per_instance_state.initialized = None

def compute(db: og.Database):
    if db:
        test_patrolling_script(db)
        return True

    # Reset script node's states if simulation is stopped
    # if db.inputs.reset_state == 1:
    #     db.per_instance_state.initialized = False
    #     return True

    # if not db.per_instance_state.initialized and db.inputs.reset_state == 0:
    #     db.per_instance_state.initialized = True
    #     patrolling = Patrolling(db)
    #     patrolling.initialize_ros2_node()

    #     post_notification("Starting Robot Patrolling", status=NotificationStatus.INFO)
    #     try:
    #         thread = threading.Thread(target=patrolling.start_patrolling)
    #         thread.start()
    #     except Exception as e:
    #         print(e)
    # else:
    #     post_notification("Robot is already Patrolling!", status=NotificationStatus.WARNING)

    # return True

"""

GATHER_WAYPOINTS_SCRIPT = """
def setup(db: og.Database):
    db.per_instance_state.waypoint_state = []
    db.per_instance_state.counter = 0

def cleanup(db: og.Database):
    db.per_instance_state.waypoint_state = None
    db.per_instance_state.counter = None

def compute(db: og.Database):
    # Reset script node's states for patrolling if simulation is stopped
    if db.per_instance_state.counter >= db.inputs.waypoint_count or db.inputs.reset_state == 1:
        db.per_instance_state.counter = 0
        waypoint_state = db.per_instance_state.waypoint_state
        waypoint_state.clear()
        db.per_instance_state.waypoint_state = waypoint_state
    else:
        db.per_instance_state.counter = db.per_instance_state.counter + 1
        waypoint_state = db.per_instance_state.waypoint_state
        translation_list = db.inputs.translation.tolist()
        orientation_list = db.inputs.orientation.tolist()

        pose_list = translation_list + orientation_list

        waypoint_state.append(f'{pose_list}')
        db.per_instance_state.waypoint_state = waypoint_state
        db.outputs.waypoints = waypoint_state

        # Reset script node's states for waypoint mode if simulation is stopped
        if db.inputs.waypoint_count == 1:
            db.per_instance_state.counter = 0
            waypoint_state = db.per_instance_state.waypoint_state
            waypoint_state.clear()
            db.per_instance_state.waypoint_state = waypoint_state

"""


class TestRos2Nav2WaypointFollower(ROS2TestCase):

    # Before running each test
    async def setUp(self):
        await super().setUp()

        self._og_path = "/Graph/ROS_Nav2_Waypoint_Follower"
        self._goal_parent_prim = "/World/Waypoints"
        self._frame_id = "map"
        self._enable_patrolling = False
        self._enable_multi_robot = False
        self._waypoints = [
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [1.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [2.0, 2.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        ]
        self._number_of_waypoints = len(self._waypoints)

        await create_new_stage_async()

    # After running each test
    async def tearDown(self):
        await super().tearDown()

    # ----------------------------------------------------------------------
    # TODO: Import from main script
    def _create_ros_action_graph(self):
        keys = og.Controller.Keys

        if self._enable_multi_robot:
            self._og_path = get_next_free_path(self._og_path, "")
        try:
            og.Controller.edit(
                {"graph_path": self._og_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                        ("OnStageEvent", "omni.graph.action.OnStageEvent"),
                        ("Timer", "omni.graph.nodes.Timer"),
                        ("WaypointScriptNode", "omni.graph.scriptnode.ScriptNode"),
                        ("PatrollingScriptNode", "omni.graph.scriptnode.ScriptNode"),
                        ("GatherWaypointsScriptNode", "omni.graph.scriptnode.ScriptNode"),
                        ("MakeArrayTranslation", "omni.graph.nodes.ConstructArray"),
                        ("MakeArrayOrientation", "omni.graph.nodes.ConstructArray"),
                        ("Branch", "omni.graph.action.Branch"),
                        ("ResetBranch", "omni.graph.action.Branch"),
                        ("TranslateStringConstant", "omni.graph.nodes.ConstantToken"),
                        ("OrientStringConstant", "omni.graph.nodes.ConstantToken"),
                        ("FrameIdConstant", "omni.graph.nodes.ConstantToken"),
                        ("WaypointPrimPath", "omni.graph.nodes.ConstantString"),
                        ("WaypointCountConstant", "omni.graph.nodes.ConstantInt"),
                        ("TranslateReadPrimAttribute", "omni.graph.nodes.ReadPrimAttribute"),
                        ("OrientReadPrimAttribute", "omni.graph.nodes.ReadPrimAttribute"),
                        ("IsaacReadSimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("ForEach", "omni.graph.action.ForEach"),
                        ("GetPrimPaths", "omni.graph.nodes.GetPrimPaths"),
                        ("GetPrims", "omni.replicator.core.OgnGetPrims"),
                    ],
                    keys.SET_VALUES: [
                        ("WaypointPrimPath.inputs:value", self._goal_parent_prim + "/"),
                        ("TranslateStringConstant.inputs:value", "xformOp:translate"),
                        ("OrientStringConstant.inputs:value", "xformOp:orient"),
                        ("FrameIdConstant.inputs:value", self._frame_id),
                        ("WaypointCountConstant.inputs:value", self._number_of_waypoints),
                        ("TranslateReadPrimAttribute.inputs:usePath", True),
                        ("OrientReadPrimAttribute.inputs:usePath", True),
                        ("GetPrims.inputs:cachePrims", False),
                        ("GetPrims.inputs:ignoreCase", False),
                        ("WaypointScriptNode.inputs:script", WAYPOINT_SCRIPT),
                        ("PatrollingScriptNode.inputs:script", PATROLLING_SCRIPT),
                        ("GatherWaypointsScriptNode.inputs:script", GATHER_WAYPOINTS_SCRIPT),
                        ("Branch.inputs:condition", self._enable_patrolling),
                        ("ResetBranch.inputs:condition", self._enable_patrolling),
                        ("OnStageEvent.inputs:eventName", "Simulation Stop Play"),
                        ("MakeArrayTranslation.inputs:arrayType", "auto"),
                        ("MakeArrayTranslation.inputs:arraySize", 1),
                        ("MakeArrayOrientation.inputs:arrayType", "auto"),
                        ("MakeArrayOrientation.inputs:arraySize", 1),
                    ],
                    keys.CONNECT: [
                        ("OnImpulseEvent.outputs:execOut", "Timer.inputs:play"),
                        ("OnImpulseEvent.outputs:execOut", "GetPrims.inputs:execIn"),
                        ("Timer.outputs:finished", "Branch.inputs:execIn"),
                        ("Branch.outputs:execFalse", "WaypointScriptNode.inputs:execIn"),
                        ("Branch.outputs:execTrue", "PatrollingScriptNode.inputs:execIn"),
                        ("ResetBranch.outputs:execFalse", "WaypointScriptNode.inputs:reset_state"),
                        ("ResetBranch.outputs:execTrue", "PatrollingScriptNode.inputs:reset_state"),
                        ("OnStageEvent.outputs:execOut", "GatherWaypointsScriptNode.inputs:reset_state"),
                        ("OnStageEvent.outputs:execOut", "ResetBranch.inputs:execIn"),
                        ("GetPrims.outputs:execOut", "ForEach.inputs:execIn"),
                        ("GetPrims.outputs:prims", "GetPrimPaths.inputs:prims"),
                        ("GetPrimPaths.outputs:primPaths", "ForEach.inputs:arrayIn"),
                        ("ForEach.outputs:loopBody", "GatherWaypointsScriptNode.inputs:execIn"),
                        ("ForEach.outputs:element", "TranslateReadPrimAttribute.inputs:primPath"),
                        ("ForEach.outputs:element", "OrientReadPrimAttribute.inputs:primPath"),
                        ("TranslateStringConstant.inputs:value", "TranslateReadPrimAttribute.inputs:name"),
                        ("OrientStringConstant.inputs:value", "OrientReadPrimAttribute.inputs:name"),
                        (
                            "IsaacReadSimulationTime.outputs:simulationTime",
                            "TranslateReadPrimAttribute.inputs:usdTimecode",
                        ),
                        (
                            "IsaacReadSimulationTime.outputs:simulationTime",
                            "OrientReadPrimAttribute.inputs:usdTimecode",
                        ),
                        ("TranslateReadPrimAttribute.outputs:value", "MakeArrayTranslation.inputs:input0"),
                        ("OrientReadPrimAttribute.outputs:value", "MakeArrayOrientation.inputs:input0"),
                        ("MakeArrayTranslation.outputs:array", "GatherWaypointsScriptNode.inputs:translation"),
                        ("MakeArrayOrientation.outputs:array", "GatherWaypointsScriptNode.inputs:orientation"),
                        ("WaypointCountConstant.inputs:value", "GatherWaypointsScriptNode.inputs:waypoint_count"),
                        ("GatherWaypointsScriptNode.outputs:waypoints", "WaypointScriptNode.inputs:waypoints"),
                        ("GatherWaypointsScriptNode.outputs:waypoints", "PatrollingScriptNode.inputs:waypoints"),
                        ("FrameIdConstant.inputs:value", "WaypointScriptNode.inputs:frame_id"),
                        ("FrameIdConstant.inputs:value", "PatrollingScriptNode.inputs:frame_id"),
                        ("WaypointPrimPath.inputs:value", "GetPrims.inputs:pathMatch"),
                    ],
                    keys.CREATE_ATTRIBUTES: [
                        (self._og_path + "/WaypointScriptNode.inputs:waypoints", "token[]"),
                        (self._og_path + "/WaypointScriptNode.inputs:frame_id", "token"),
                        (self._og_path + "/WaypointScriptNode.inputs:reset_state", "execution"),
                        (self._og_path + "/PatrollingScriptNode.inputs:waypoints", "token[]"),
                        (self._og_path + "/PatrollingScriptNode.inputs:frame_id", "token"),
                        (self._og_path + "/PatrollingScriptNode.state:initialized", "bool"),
                        (self._og_path + "/PatrollingScriptNode.inputs:reset_state", "execution"),
                        (self._og_path + "/GatherWaypointsScriptNode.inputs:translation", "any"),
                        (self._og_path + "/GatherWaypointsScriptNode.inputs:orientation", "any"),
                        (self._og_path + "/GatherWaypointsScriptNode.outputs:waypoints", "token[]"),
                        (self._og_path + "/GatherWaypointsScriptNode.state:waypoint_state", "token[]"),
                        (self._og_path + "/GatherWaypointsScriptNode.inputs:waypoint_count", "int"),
                        (self._og_path + "/GatherWaypointsScriptNode.state:counter", "int"),
                        (self._og_path + "/GatherWaypointsScriptNode.inputs:reset_state", "execution"),
                    ],
                },
            )
        except Exception as e:
            return False

        return True

    def _create_waypoints(self, waypoint, xform_path):
        try:
            stage = omni.usd.get_context().get_stage()

            xform_prim = UsdGeom.Xform.Define(stage, xform_path)

            translation = Gf.Vec3f(waypoint[0], waypoint[1], waypoint[2])
            quaternion = Gf.Quatf(waypoint[3], waypoint[4], waypoint[5], waypoint[6])
            scale = Gf.Vec3f(1.0, 1.0, 1.0)

            xform_prim.AddTranslateOp().Set(translation)
            xform_prim.AddOrientOp().Set(quaternion)
            xform_prim.AddScaleOp().Set(scale)
        except:
            return False

        return True

    # TODO: Import from main script
    def _check_params(self):
        try:
            if self._enable_patrolling:
                if self._number_of_waypoints < 2 or self._number_of_waypoints > 50:
                    return False
            else:
                self._number_of_waypoints = 1
        except:
            return False

        return True

    async def test_duplicate_graph(self):
        # Test to verify duplicate graphs present in the stage
        self.assertTrue(self._create_ros_action_graph(), "ActionGraph is not created.")

        self.assertFalse(self._create_ros_action_graph(), "Duplicate ActionGraph is generated")

        delete_prim(self._og_path)
        self.assertTrue(self._create_ros_action_graph(), "ActionGraph is not created after deleting old graph.")

        self._enable_multi_robot = True
        self.assertTrue(self._create_ros_action_graph(), "ActionGraph is not created for multi robot case.")

    async def test_waypoint_limit(self):
        # Test to verify waypoint limit should be between 2 to 50 inclusive
        self._enable_patrolling = True
        self._enable_multi_robot = False

        self._number_of_waypoints = 1
        self.assertFalse(self._check_params(), "Waypoint count must be between 2 to 50 for patrolling inclusive.")

        self._number_of_waypoints = 51
        self.assertFalse(self._check_params(), "Waypoint count must be between 2 to 50 for patrolling inclusive.")

        self._number_of_waypoints = 2
        self.assertTrue(self._check_params(), "Waypoint limit is incorrect.")

        self._number_of_waypoints = 50
        self.assertTrue(self._check_params(), "Waypoint limit is incorrect.")

        self._number_of_waypoints = 31
        self.assertTrue(self._check_params(), "Waypoint limit is incorrect.")

    async def test_waypoint_generation(self):
        # Test to verify waypoint is generated or not
        _dummy_waypoint = [1.0, 2.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        _prim_path = f"{self._goal_parent_prim}/waypoint_0"
        self.assertTrue(self._create_waypoints(_dummy_waypoint, _prim_path), "Waypoint is not created!")

        xform_prim = get_prim_at_path(_prim_path)
        self.assertIsNotNone(xform_prim)

        self.assertFalse(self._create_waypoints(_dummy_waypoint, _prim_path), "Waypoint with same name is created!")

    def spin_thread(self):
        import rclpy

        rclpy.spin_once(self.__node, timeout_sec=10)

    async def test_waypoint_mode_action_graph(self):
        import threading

        import rclpy
        from std_msgs.msg import String

        # Test to verify waypoint mode action graph
        self._enable_patrolling = False  # Switch waypoint mode
        self._enable_multi_robot = False
        self._waypoints = [[1.0, 3.0, 0.0, 1.0, 1.0, 1.0, 0.0]]
        self._number_of_waypoints = len(self._waypoints)

        self.__expected_result = "Waypoint Mode: ['[[1.0, 3.0, 0.0], [1.0, 1.0, 0.0, 1.0]]']"
        self.__result = False

        # Create a node named 'waypoint_subscriber'
        self.__node = rclpy.create_node("waypoint_subscriber")

        print(f"Expected Result: {self.__expected_result}")

        # Create a single waypoint in the scene
        self._create_waypoints(self._waypoints[0], f"{self._goal_parent_prim}/waypoint_{0}")
        self._create_ros_action_graph()

        # Define a callback function that will be called when a message is received
        def listener_callback(msg):
            print(f"Received: {msg.data}")
            self.__result = msg.data == self.__expected_result

        # Create a subscription to the 'topic' topic, listening for String messages
        self.__node.create_subscription(String, "topic", listener_callback, 10)

        # Start the spin function in a separate thread
        thread = threading.Thread(target=self.spin_thread, daemon=True)
        thread.start()

        self._timeline.play()
        await simulate_async(0.5)
        og.Controller.set(og.Controller.attribute(f"{self._og_path}/OnImpulseEvent.state:enableImpulse"), True)
        await asyncio.sleep(2.0)

        self.__node.destroy_node()

        self.assertTrue(self.__result, "Waypoint Mode Graph is not generated properly.")

    async def test_patrolling_mode_action_graph(self):
        import threading

        import rclpy
        from std_msgs.msg import String

        # Test to verify patrolling mode action graph
        self._enable_patrolling = True  # Switch patrolling mode
        self._enable_multi_robot = False
        self._waypoints = [[1.0, 3.0, 0.0, 1.0, 1.0, 1.0, 0.0], [15.0, 35.0, 0.0, 15.0, 15.0, 15.0, 0.0]]
        self._number_of_waypoints = len(self._waypoints)

        self.__expected_result = "Patrolling Mode: ['[[1.0, 3.0, 0.0], [1.0, 1.0, 0.0, 1.0]]', '[[15.0, 35.0, 0.0], [15.0, 15.0, 0.0, 15.0]]']"
        self.__result = False

        # Create a node named 'patrolling_subscriber'
        self.__node = rclpy.create_node("patrolling_subscriber")

        print(f"Expected Result: {self.__expected_result}")

        # Create multiple waypoints in the scene
        for _xform in range(self._number_of_waypoints):
            self._create_waypoints(self._waypoints[_xform], f"{self._goal_parent_prim}/waypoint_{_xform}")
        self._create_ros_action_graph()

        # Define a callback function that will be called when a message is received
        def listener_callback(msg):
            print(f"Received: {msg.data}")
            self.__result = msg.data == self.__expected_result

        # Create a subscription to the 'topic' topic, listening for String messages
        self.__node.create_subscription(String, "topic", listener_callback, 10)

        # Start the spin function in a separate thread
        thread = threading.Thread(target=self.spin_thread, daemon=True)
        thread.start()

        self._timeline.play()
        await simulate_async(0.5)
        og.Controller.set(og.Controller.attribute(f"{self._og_path}/OnImpulseEvent.state:enableImpulse"), True)
        await asyncio.sleep(2.0)

        self.__node.destroy_node()

        self.assertTrue(self.__result, "Patrolling Mode Graph is not generated properly.")
