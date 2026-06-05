# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provides a ROS 2 MoveIt integration example extension for controlling a Franka robot in Isaac Sim."""

import asyncio
import gc
import os
import weakref

import carb
import omni.appwindow
import omni.ext
import omni.graph.core as og
import omni.kit.commands
import omni.ui as ui
import usdrt.Sdf
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import PhysicsScene
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.gui.components.ui_utils import setup_ui_headers
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf

MENU_NAME = "Franka MoveIt"
MENU_CATEGORY = "ROS2/MoveIt"
FRANKA_STAGE_PATH = "/Franka"


class Extension(omni.ext.IExt):
    """Extension providing the Franka MoveIt example."""

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension."""
        self._ext_id = ext_id
        """Initialize extension and UI elements"""
        self._timeline = omni.timeline.get_timeline_interface()
        self._usd_context = omni.usd.get_context()
        self._stage = self._usd_context.get_stage()

        get_browser_instance().register_example(
            name="Franka MoveIt", ui_hook=lambda a=weakref.proxy(self): a.build_ui(), category=MENU_CATEGORY
        )

    def build_ui(self) -> None:
        """Build the extension user interface."""
        # check if ros2 bridge is enabled before proceeding
        extension_enabled = omni.kit.app.get_app().get_extension_manager().is_extension_enabled("isaacsim.ros2.bridge")
        if not extension_enabled:
            msg = "ROS2 Bridge is not enabled. Please enable the extension to use this feature."
            carb.log_error(msg)
        else:
            overview = "This sample demonstrates how to use MoveIt with Isaac Sim. \n\n The Environment Loaded already contains the OmniGraphs needed to connect with MoveIt."
            self._main_stack = ui.VStack(spacing=5, height=0)
            with self._main_stack:
                setup_ui_headers(
                    self._ext_id,
                    file_path=os.path.abspath(__file__),
                    title="MoveIt Example Environment",
                    overview=overview,
                    info_collapsed=False,
                )
                ui.Button("Load Sample Scene", clicked_fn=self._on_environment_setup)

    def create_ros_action_graph(self, franka_stage_path: str) -> None:
        """Create the ROS 2 action graph for joint state communication.

        Args:
            franka_stage_path: USD prim path for the Franka robot.
        """
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
                        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                        ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                        ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                        ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                        ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ReadJointState.inputs:execIn"),
                        ("ReadJointState.outputs:execOut", "PublishJointState.inputs:execIn"),
                        ("ReadJointState.outputs:jointNames", "PublishJointState.inputs:jointNames"),
                        ("ReadJointState.outputs:jointPositions", "PublishJointState.inputs:jointPositions"),
                        ("ReadJointState.outputs:jointVelocities", "PublishJointState.inputs:jointVelocities"),
                        ("ReadJointState.outputs:jointEfforts", "PublishJointState.inputs:jointEfforts"),
                        ("ReadJointState.outputs:jointDofTypes", "PublishJointState.inputs:jointDofTypes"),
                        ("ReadJointState.outputs:stageMetersPerUnit", "PublishJointState.inputs:stageMetersPerUnit"),
                        ("ReadJointState.outputs:sensorTime", "PublishJointState.inputs:sensorTime"),
                        ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                        ("Context.outputs:context", "PublishJointState.inputs:context"),
                        ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                        ("Context.outputs:context", "PublishClock.inputs:context"),
                        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                        ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                        (
                            "SubscribeJointState.outputs:positionCommand",
                            "ArticulationController.inputs:positionCommand",
                        ),
                        (
                            "SubscribeJointState.outputs:velocityCommand",
                            "ArticulationController.inputs:velocityCommand",
                        ),
                        ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        # Setting the /Franka target prim to Articulation Controller node
                        ("ArticulationController.inputs:robotPath", franka_stage_path),
                        ("ReadJointState.inputs:prim", [usdrt.Sdf.Path(franka_stage_path)]),
                        ("PublishJointState.inputs:topicName", "isaac_joint_states"),
                        ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
                    ],
                },
            )
        except Exception as e:
            print(e)

    def create_franka(self, stage_path: str) -> None:
        """Create a Franka robot prim on the stage.

        Args:
            stage_path: USD prim path where the Franka robot will be added.
        """
        usd_path = "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        asset_path = self._assets_root_path + usd_path
        prim = self._stage.DefinePrim(stage_path, "Xform")
        prim.GetReferences().AddReference(asset_path)

        prim.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
        prim.GetVariantSet("Mesh").SetVariantSelection("Quality")

        rot_mat = Gf.Matrix3d(Gf.Rotation((0, 0, 1), 90))
        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=prim.GetPath(),
            old_transform_matrix=None,
            new_transform_matrix=Gf.Matrix4d().SetRotate(rot_mat).SetTranslateOnly(Gf.Vec3d(0, -0.64, 0)),
        )

    async def _create_moveit_sample(self) -> None:
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[1.20, 1.20, 0.80], target=[0, 0, 0.50])
        self._stage = self._usd_context.get_stage()

        self.create_franka(FRANKA_STAGE_PATH)
        await omni.kit.app.get_app().next_update_async()
        stage_utils.add_reference_to_stage(
            usd_path=self._assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd", path="/background"
        )
        await omni.kit.app.get_app().next_update_async()
        physics_scene = PhysicsScene("/physicsScene")
        physics_scene.set_dt(1.0 / 60.0)
        await omni.kit.app.get_app().next_update_async()
        self.create_ros_action_graph(FRANKA_STAGE_PATH)
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()

    def _on_environment_setup(self) -> None:
        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        asyncio.ensure_future(self._create_moveit_sample())

    def on_shutdown(self) -> None:
        """Cleanup objects on extension shutdown."""
        get_browser_instance().deregister_example(name=MENU_NAME, category=MENU_CATEGORY)
        self._timeline.stop()
        gc.collect()
