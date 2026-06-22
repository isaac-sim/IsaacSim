# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Module for ROS2 waypoint following examples and UI integration."""

import gc
import os
import weakref

import carb
import omni.appwindow
import omni.ext
import omni.graph.core as og
import omni.kit.commands
import omni.kit.viewport.utility
import omni.ui as ui
import omni.usd
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.gui.components.ui_utils import setup_ui_headers
from isaacsim.gui.components.widgets import ParamWidget
from omni.kit.notification_manager import NotificationStatus, post_notification
from pxr import Gf, UsdGeom

MENU_NAME = "Add Waypoint Follower"
MENU_CATEGORY = "ROS2/Navigation"

WAYPOINT_SCRIPT = """
import rclpy
import threading
import time
from ast import literal_eval

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

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
        try:
            # Avoid setting bogus initial pose
            self._navigator.initial_pose_received = True
            # Wait until the full navigation system is up and running.
            self._navigator.waitUntilNav2Active()
        except Exception as e:
            post_notification("WaypointFollower Stopped!", status=NotificationStatus.WARNING)
            return

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
                self._navigator.get_logger().info("Goal succeeded")
            elif result == TaskResult.CANCELED:
                post_notification("Goal canceled", status=NotificationStatus.WARNING)
                self._navigator.get_logger().info("Goal canceled")
            elif result == TaskResult.FAILED:
                post_notification("Goal failed!", status=NotificationStatus.WARNING)
                self._navigator.get_logger().info("Goal canceled")
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

def setup(db: og.Database):
    # To lock the waypoint mode if it is already running
    db.per_instance_state.lock = False
    global waypoint_follower
    waypoint_follower = None

def cleanup(db: og.Database):
    db.per_instance_state.lock = None

def compute(db: og.Database):
    global waypoint_follower

    # Reset script node's states if simulation is stopped
    if db.inputs.reset_state == 1:
        db.per_instance_state.lock = False
        waypoint_follower = None
        return True

    if not db.per_instance_state.lock and db.inputs.reset_state == 0:
        db.per_instance_state.lock = True

        # WaypointFollower object must be created only once in execution
        if waypoint_follower == None:
            waypoint_follower = WaypointFollower(db)
            waypoint_follower.initialize_ros2_node()

        # send another goal only if previous task is executed
        if waypoint_follower._navigator != None and waypoint_follower._navigator.isTaskComplete():
            post_notification("Running Waypoint Mode", status=NotificationStatus.INFO)
            try:
                thread = threading.Thread(target=waypoint_follower.start_waypoint_follower)
                thread.start()
            except Exception as e:
                print(e)
    else:
        post_notification("Previous waypoint is in progress!", status=NotificationStatus.WARNING)

    return True

"""

PATROLLING_SCRIPT = """
import rclpy
import threading
import time
from ast import literal_eval

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

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
        try:
            # Avoid setting bogus initial pose
            self._navigator.initial_pose_received = True
            # Wait until the full navigation system is up and running
            self._navigator.waitUntilNav2Active()
        except Exception as e:
            post_notification("Patrolling Stopped!", status=NotificationStatus.WARNING)
            return

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
                    self._navigator.get_logger().info(f'Round {self._counter} is completed')
                    self._counter = self._counter + 1
                else:
                    post_notification(f'Round {self._counter} is either Failed or Cancelled!', status=NotificationStatus.WARNING)
                    self._navigator.get_logger().info(f'Round {self._counter} is completed')
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

def setup(db: og.Database):
    # To lock the patrolling node if it is already running
    db.per_instance_state.initialized = False

def cleanup(db: og.Database):
    db.per_instance_state.initialized = None

def compute(db: og.Database):
    # Reset script node's states if simulation is stopped
    if db.inputs.reset_state == 1:
        db.per_instance_state.initialized = False
        return True

    if not db.per_instance_state.initialized and db.inputs.reset_state == 0:
        db.per_instance_state.initialized = True
        patrolling = Patrolling(db)
        patrolling.initialize_ros2_node()

        post_notification("Starting Robot Patrolling", status=NotificationStatus.INFO)
        try:
            thread = threading.Thread(target=patrolling.start_patrolling)
            thread.start()
        except Exception as e:
            print(e)
    else:
        post_notification("Robot is already Patrolling!", status=NotificationStatus.WARNING)

    return True

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


class Extension(omni.ext.IExt):
    """Extension providing the ROS 2 waypoint follower example."""

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension.

        Args:
            ext_id: Extension identifier provided by Kit.
        """
        self._ext_id = ext_id
        """Initialize extension and UI elements"""
        self._timeline = omni.timeline.get_timeline_interface()
        """Initialize variables"""
        self._og_path = "/Graph/ROS_Nav2_Waypoint_Follower"
        self._goal_parent_prim = "/World/Waypoints"
        self._frame_id = "map"
        self._enable_patrolling = False
        self._number_of_waypoints = 1
        self._enable_multi_robot = False

        get_browser_instance().register_example(
            name=MENU_NAME, ui_hook=lambda a=weakref.proxy(self): a.build_ui(), category=MENU_CATEGORY
        )

    def build_ui(self) -> None:
        """Build the extension user interface."""
        # check if ros2 bridge is enabled before proceeding
        extension_enabled = omni.kit.app.get_app().get_extension_manager().is_extension_enabled("isaacsim.ros2.bridge")
        if not extension_enabled:
            msg = "ROS2 Bridge is not enabled. Please enable the extension to use this feature."
            carb.log_error(msg)
        else:
            overview = "This sample demonstrates how to use OmniGraph to interact with ROS2 Navigation. \n\nThe ActionGraph retrieves waypoint transforms and sends them to the Nav2 stack, enabling robot navigation."
            og_path_def = ParamWidget.FieldDef(
                name="og_path", label="Graph Path", type=ui.StringField, default=self._og_path
            )
            frame_id_def = ParamWidget.FieldDef(
                name="frame_id", label="Frame ID", type=ui.StringField, default=self._frame_id
            )
            self.radio_collection = ui.RadioCollection()

            setup_ui_headers(
                self._ext_id,
                file_path=os.path.abspath(__file__),
                title="Waypoint Follower Parameters",
                overview=overview,
                info_collapsed=False,
            )
            with ui.VStack(spacing=4):
                ui.Spacer(height=2)
                self.og_path_input = ParamWidget(field_def=og_path_def)
                self.frame_id_input = ParamWidget(field_def=frame_id_def)
                ui.Spacer(height=10)
                with ui.Frame():
                    with ui.VStack():
                        ui.Label("Select Navigation Mode:")
                        with ui.HStack():
                            ui.RadioButton(
                                text="Waypoint",
                                radio_collection=self.radio_collection,
                                clicked_fn=lambda: self._on_radio_selected("Waypoint"),
                                height=ui.Percent(30),
                            )
                            ui.RadioButton(
                                text="Patrolling",
                                radio_collection=self.radio_collection,
                                clicked_fn=lambda: self._on_radio_selected("Patrolling"),
                                height=ui.Percent(30),
                            )
                        ui.Spacer(height=2)
                        with ui.HStack():
                            self.waypoint_count_label = ui.Label(
                                "Waypoint Count:",
                                visible=False,
                                height=ui.Percent(40),
                                width=ui.Percent(29),
                                word_wrap=True,
                            )
                            ui.Spacer(width=ui.Percent(1))
                            self.waypoint_count_field = ui.IntField(
                                visible=False,
                                height=ui.Percent(30),
                                width=ui.Percent(70),
                            )
                            self.waypoint_count_field.model.set_value(2)
                ui.Spacer(height=10)
                with ui.Frame(height=30):
                    with ui.HStack():
                        ui.Spacer(width=ui.Percent(10))
                        ui.Button(
                            "Load Waypoint Follower ActionGraph",
                            width=ui.Percent(80),
                            clicked_fn=self._on_environment_setup,
                        )

    def _create_ros_action_graph(self) -> None:
        self._timeline.stop()

        keys = og.Controller.Keys

        if self._enable_multi_robot:
            self._og_path = stage_utils.generate_next_free_path(self._og_path, prepend_default_prim=False)
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
                        ("FrameIdConstant", "omni.graph.nodes.ConstantToken"),
                        ("WaypointPrimPath", "omni.graph.nodes.ConstantString"),
                        ("WaypointCountConstant", "omni.graph.nodes.ConstantInt"),
                        ("ReadPrimLocalTransform", "omni.graph.nodes.ReadPrimLocalTransform"),
                        ("TokenToTarget", "omni.graph.nodes.ToTarget"),
                        ("GetTranslation", "omni.graph.nodes.GetMatrix4Translation"),
                        ("GetRotationQuaternion", "omni.graph.nodes.GetMatrix4Quaternion"),
                        ("ForEach", "omni.graph.action.ForEach"),
                        ("GetPrimPaths", "omni.graph.nodes.GetPrimPaths"),
                        ("GetPrims", "omni.replicator.core.OgnGetPrims"),
                    ],
                    keys.SET_VALUES: [
                        ("WaypointPrimPath.inputs:value", self._goal_parent_prim + "/"),
                        ("FrameIdConstant.inputs:value", self._frame_id),
                        ("WaypointCountConstant.inputs:value", self._number_of_waypoints),
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
                        ("ForEach.outputs:element", "TokenToTarget.inputs:value"),
                        ("TokenToTarget.outputs:converted", "ReadPrimLocalTransform.inputs:prim"),
                        ("ReadPrimLocalTransform.outputs:value", "GetTranslation.inputs:matrix"),
                        ("ReadPrimLocalTransform.outputs:value", "GetRotationQuaternion.inputs:matrix"),
                        ("GetTranslation.outputs:translation", "MakeArrayTranslation.inputs:input0"),
                        ("GetRotationQuaternion.outputs:quaternion", "MakeArrayOrientation.inputs:input0"),
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
            print(e)

    def _create_waypoints(self, xform_path: str) -> None:
        stage = omni.usd.get_context().get_stage()

        xform_prim = UsdGeom.Xform.Define(stage, xform_path)

        translation = Gf.Vec3f(0.0, 0.0, 0.0)
        quaternion = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        quaternion = quaternion.GetNormalized()
        scale = Gf.Vec3f(1.0, 1.0, 1.0)

        xform_prim.AddTranslateOp().Set(translation)
        xform_prim.AddOrientOp().Set(quaternion)
        xform_prim.AddScaleOp().Set(scale)

    def _on_radio_selected(self, label: str) -> None:
        if label == "Patrolling":
            self._enable_patrolling = True
            self.waypoint_count_label.visible = True
            self.waypoint_count_field.visible = True
        else:
            self._enable_patrolling = False
            self.waypoint_count_label.visible = False
            self.waypoint_count_field.visible = False

    def _check_params(self) -> bool:
        try:
            if self._enable_patrolling:
                self._number_of_waypoints = self.waypoint_count_field.model.get_value_as_int()
                if self._number_of_waypoints < 2 or self._number_of_waypoints > 50:
                    post_notification(
                        "Waypoint count must be between 2 to 50 inclusive for patrolling",
                        status=NotificationStatus.WARNING,
                    )
                    return False
            else:
                self._number_of_waypoints = 1
        except Exception:
            post_notification("Waypoint count must be an integer", status=NotificationStatus.WARNING)
            return False

        return True

    def _on_environment_setup(self) -> None:
        self._og_path = self.og_path_input.get_value()
        self._frame_id = self.frame_id_input.get_value()

        param_check = self._check_params()
        if param_check:
            try:
                self._create_waypoints(self._goal_parent_prim)
                self._create_ros_action_graph()
                for _xform in range(self._number_of_waypoints):
                    self._create_waypoints(f"{self._goal_parent_prim}/waypoint_{_xform}")
            except Exception:
                post_notification(
                    f"Please delete {self._og_path} and /World/Waypoints from stage and try again",
                    status=NotificationStatus.WARNING,
                )

    def on_shutdown(self) -> None:
        """Cleanup objects on extension shutdown."""
        get_browser_instance().deregister_example(name=MENU_NAME, category=MENU_CATEGORY)
        self._timeline.stop()
        gc.collect()
