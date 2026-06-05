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

"""User interface for generating ROS2 camera and RTX lidar sensor OmniGraph action graphs."""

from pathlib import Path

import carb
import omni.graph.core as og
import omni.kit.viewport.utility
import omni.ui as ui
import OmniGraphSchema
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.gui.components.callbacks import on_docs_link_clicked, on_open_IDE_clicked
from isaacsim.gui.components.style import get_style
from isaacsim.gui.components.widgets import ParamWidget, SelectPrimWidget
from omni.kit.menu.utils import MenuHelperWindow
from omni.kit.notification_manager import NotificationStatus, post_notification
from omni.kit.window.extensions import SimpleCheckBox
from pxr import UsdGeom


class Ros2CameraGraph(MenuHelperWindow):
    """A UI window for generating ROS2 camera graphs in Isaac Sim.

    This class provides a graphical interface to create or extend OmniGraph action graphs that publish camera data to ROS2 topics. It supports multiple camera output types including RGB, depth, point clouds, semantic segmentation, instance segmentation, and 2D/3D bounding boxes.

    The generated graph can either be created as a new standalone graph or added to an existing graph. It automatically configures the necessary nodes including tick sources, render products, ROS2 contexts, and topic publishers based on user selections.

    Users can configure:

    - Graph path and camera prim selection
    - ROS2 frame ID and node namespace
    - Publication of RGB images
    - Publication of depth images
    - Publication of depth point clouds
    - Publication of instance segmentation data
    - Publication of semantic segmentation data
    - Publication of 2D tight bounding boxes
    - Publication of 2D loose bounding boxes
    - Publication of 3D bounding boxes
    - Custom topic names for each output type

    The window validates the selected camera prim to ensure it is a valid UsdGeom.Camera before generating the graph. If adding to an existing graph, it verifies the graph contains required nodes such as OnPlaybackTick and ROS2Context.
    """

    def __init__(self) -> None:
        """Initialize the ROS2 camera graph window."""
        super().__init__("ROS2 Camera Graph", width=500, height=600)
        # Initialize parameters
        self._og_path = "/Graph/ROS_Camera"
        self._camera_prim = "/OmniverseKit_Persp"  # default camera prim is the perspective camera
        self._add_to_existing_graph = False
        self._frame_id = "sim_camera"
        self._node_namespace = ""
        self._camera_info_topic = "camera_info"
        self._rgb_pub = True
        self._rgb_topic = "/rgb"
        self._depth_pub = True
        self._depth_topic = "/depth"
        self._depth_pcl_pub = False
        self._depth_pcl_topic = "/depth_pcl"
        self._instance_pub = False
        self._instance_topic = "/instance_segmentation"
        self._semantic_pub = False
        self._semantic_topic = "/semantic_segmentation"
        self._bbox2d_tight_pub = False
        self._bbox2d_tight_topic = "/bbox_2d_tight"
        self._bbox2d_loose_pub = False
        self._bbox2d_loose_topic = "/bbox_2d_loose"
        self._bbox3d_pub = False
        self._bbox3d_topic = "/bbox_3d"
        # Build the UI automatically from here
        self._build_ui()

    def make_graph(self) -> None:
        """Create a ROS2 camera graph in OmniGraph based on the configured parameters.

        If adding to an existing graph, validates and reuses existing nodes (tick, context, render product).
        Otherwise, creates a new graph with base nodes. Adds publisher nodes for each enabled camera output type
        (RGB, depth, depth point cloud, instance segmentation, semantic segmentation, 2D/3D bounding boxes) and
        connects them to the render product and ROS2 context.
        """
        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline.stop()

        keys = og.Controller.Keys
        # if starting from a new graph, start it with just a tick and context, render product and camera info (no sim time), the rest is the same for adding to exsiting graph
        if not self._add_to_existing_graph:
            self._og_path = stage_utils.generate_next_free_path(self._og_path, prepend_default_prim=False)
            graph_handle, nodes, _, _ = og.Controller.edit(
                {"graph_path": self._og_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                        ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                        ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ],
                    keys.SET_VALUES: [
                        ("RenderProduct.inputs:cameraPrim", self._camera_prim),
                        ("CameraInfoPublish.inputs:topicName", self._camera_info_topic),
                        ("CameraInfoPublish.inputs:frameId", self._frame_id),
                        ("CameraInfoPublish.inputs:nodeNamespace", self._node_namespace),
                        ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
                        ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
                        ("RenderProduct.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
                        ("RenderProduct.outputs:renderProductPath", "CameraInfoPublish.inputs:renderProductPath"),
                        ("Context.outputs:context", "CameraInfoPublish.inputs:context"),
                    ],
                },
            )
        else:
            graph_handle = og.get_graph_by_path(self._og_path)

        # to an existin graph
        # traverse through the graph
        all_nodes = graph_handle.get_nodes()
        tick_node = None
        context_node = None
        render_node = None
        for node in all_nodes:
            node_path = node.get_prim_path()
            node_type = node.get_type_name()
            if node_type == "omni.graph.action.OnPlaybackTick" or node_type == "omni.graph.action.OnTick":
                tick_node = node_path
            elif node_type == "isaacsim.ros2.bridge.ROS2Context":
                context_node = node_path
            elif node_type == "isaacsim.core.nodes.IsaacCreateRenderProduct":
                render_node_path = node_path
                render_node = node

        if not tick_node or not context_node:
            carb.log_warn(
                f"ActionGraph {self._og_path} missing node(s) necessary to build ROS2 graph. Skipping graph generation. Consider building new graph using tool."
            )
            return

        # if the existing graph doesn't already have a render node, or if the existing node does not use the same camera, then create a new render node and connect it to the new camera
        # TODO: so far only support if there's one existing render node. If there are multiple render nodes, it won't check if every node has unique camera prims.
        if render_node is None or render_node.get_attribute("inputs:cameraPrim").get()[0] != self._camera_prim:
            render_node = stage_utils.generate_next_free_path(
                self._og_path + "/RenderProduct", ""
            )  # this is actually a string path at this point, not a node prim despite the name. This is so that it's consistent with the others.
            render_node_name = Path(render_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (render_node_name, "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ],
                    keys.SET_VALUES: [
                        (render_node_name + ".inputs:cameraPrim", self._camera_prim),
                    ],
                    keys.CONNECT: [
                        (tick_node + ".outputs:tick", render_node_name + ".inputs:execIn"),
                    ],
                },
            )
        else:
            render_node = render_node_path  # once again set render_node to the actual path, as oppose to the node_prim, just for consistency

        if self._rgb_pub:
            rgb_node = stage_utils.generate_next_free_path(self._og_path + "/RGBPublish", prepend_default_prim=False)
            rgb_node_name = Path(rgb_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (rgb_node_name, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ],
                    keys.SET_VALUES: [
                        (rgb_node + ".inputs:topicName", self._rgb_topic),
                        (rgb_node + ".inputs:type", "rgb"),
                        (rgb_node + ".inputs:resetSimulationTimeOnStop", True),
                        (rgb_node + ".inputs:frameId", self._frame_id),
                        (rgb_node + ".inputs:nodeNamespace", self._node_namespace),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", rgb_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", rgb_node + ".inputs:renderProductPath"),
                    ],
                },
            )

            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(rgb_node + ".inputs:context"),
                )

        if self._depth_pub:
            depth_node = stage_utils.generate_next_free_path(
                self._og_path + "/DepthPublish", prepend_default_prim=False
            )
            depth_node_name = Path(depth_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (depth_node_name, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ],
                    keys.SET_VALUES: [
                        (depth_node + ".inputs:topicName", self._depth_topic),
                        (depth_node + ".inputs:type", "depth"),
                        (depth_node + ".inputs:resetSimulationTimeOnStop", True),
                        (depth_node + ".inputs:frameId", self._frame_id),
                        (depth_node + ".inputs:nodeNamespace", self._node_namespace),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", depth_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", depth_node + ".inputs:renderProductPath"),
                    ],
                },
            )
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(depth_node + ".inputs:context"),
                )

        if self._depth_pcl_pub:
            depth_pcl_node = stage_utils.generate_next_free_path(
                self._og_path + "/DepthPclPublish", prepend_default_prim=False
            )
            depth_pcl_node_name = Path(depth_pcl_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (depth_pcl_node_name, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ],
                    keys.SET_VALUES: [
                        (depth_pcl_node + ".inputs:topicName", self._depth_pcl_topic),
                        (depth_pcl_node + ".inputs:type", "depth_pcl"),
                        (depth_pcl_node + ".inputs:resetSimulationTimeOnStop", True),
                        (depth_pcl_node + ".inputs:frameId", self._frame_id),
                        (depth_pcl_node + ".inputs:nodeNamespace", self._node_namespace),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", depth_pcl_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", depth_pcl_node + ".inputs:renderProductPath"),
                        (context_node + ".outputs:context", depth_pcl_node + ".inputs:context"),
                    ],
                },
            )
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(depth_pcl_node + ".inputs:context"),
                )

        if self._instance_pub:
            instance_node = stage_utils.generate_next_free_path(
                self._og_path + "/InstancePublish", prepend_default_prim=False
            )
            instance_node_name = Path(instance_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (instance_node_name, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ],
                    keys.SET_VALUES: [
                        (instance_node + ".inputs:topicName", self._instance_topic),
                        (instance_node + ".inputs:type", "instance_segmentation"),
                        (instance_node + ".inputs:resetSimulationTimeOnStop", True),
                        (instance_node + ".inputs:frameId", self._frame_id),
                        (instance_node + ".inputs:nodeNamespace", self._node_namespace),
                        (instance_node + ".inputs:enableSemanticLabels", True),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", instance_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", instance_node + ".inputs:renderProductPath"),
                        (context_node + ".outputs:context", instance_node + ".inputs:context"),
                    ],
                },
            )
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(instance_node + ".inputs:context"),
                )

        if self._semantic_pub:
            semantic_node = stage_utils.generate_next_free_path(
                self._og_path + "/SemanticPublish", prepend_default_prim=False
            )
            semantic_node_name = Path(semantic_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (semantic_node_name, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ],
                    keys.SET_VALUES: [
                        (semantic_node + ".inputs:topicName", self._semantic_topic),
                        (semantic_node + ".inputs:type", "semantic_segmentation"),
                        (semantic_node + ".inputs:resetSimulationTimeOnStop", True),
                        (semantic_node + ".inputs:frameId", self._frame_id),
                        (semantic_node + ".inputs:nodeNamespace", self._node_namespace),
                        (semantic_node + ".inputs:enableSemanticLabels", True),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", semantic_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", semantic_node + ".inputs:renderProductPath"),
                        (context_node + ".outputs:context", semantic_node + ".inputs:context"),
                    ],
                },
            )
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(semantic_node + ".inputs:context"),
                )

        if self._bbox2d_tight_pub:
            bbox2d_tight_node = stage_utils.generate_next_free_path(
                self._og_path + "/Bbox2dTightPublish", prepend_default_prim=False
            )
            bbox2d_tight_node_name = Path(bbox2d_tight_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (bbox2d_tight_node_name, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ],
                    keys.SET_VALUES: [
                        (bbox2d_tight_node + ".inputs:topicName", self._bbox2d_tight_topic),
                        (bbox2d_tight_node + ".inputs:type", "bbox_2d_tight"),
                        (bbox2d_tight_node + ".inputs:resetSimulationTimeOnStop", True),
                        (bbox2d_tight_node + ".inputs:frameId", self._frame_id),
                        (bbox2d_tight_node + ".inputs:nodeNamespace", self._node_namespace),
                        (bbox2d_tight_node + ".inputs:enableSemanticLabels", True),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", bbox2d_tight_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", bbox2d_tight_node + ".inputs:renderProductPath"),
                        (context_node + ".outputs:context", bbox2d_tight_node + ".inputs:context"),
                    ],
                },
            )
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(bbox2d_tight_node + ".inputs:context"),
                )

        if self._bbox2d_loose_pub:
            bbox2d_loose_node = stage_utils.generate_next_free_path(
                self._og_path + "/Bbox2dLoosePublish", prepend_default_prim=False
            )
            bbox2d_loose_node_name = Path(bbox2d_loose_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (bbox2d_loose_node_name, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ],
                    keys.SET_VALUES: [
                        (bbox2d_loose_node + ".inputs:topicName", self._bbox2d_loose_topic),
                        (bbox2d_loose_node + ".inputs:type", "bbox_2d_loose"),
                        (bbox2d_loose_node + ".inputs:resetSimulationTimeOnStop", True),
                        (bbox2d_loose_node + ".inputs:frameId", self._frame_id),
                        (bbox2d_loose_node + ".inputs:nodeNamespace", self._node_namespace),
                        (bbox2d_loose_node + ".inputs:enableSemanticLabels", True),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", bbox2d_loose_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", bbox2d_loose_node + ".inputs:renderProductPath"),
                        (context_node + ".outputs:context", bbox2d_loose_node + ".inputs:context"),
                    ],
                },
            )
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(bbox2d_loose_node + ".inputs:context"),
                )

        if self._bbox3d_pub:
            bbox3d_node = stage_utils.generate_next_free_path(
                self._og_path + "/Bbox3dPublish", prepend_default_prim=False
            )
            bbox3d_node_name = Path(bbox3d_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (bbox3d_node_name, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ],
                    keys.SET_VALUES: [
                        (bbox3d_node + ".inputs:topicName", self._bbox3d_topic),
                        (bbox3d_node + ".inputs:type", "bbox_3d"),
                        (bbox3d_node + ".inputs:resetSimulationTimeOnStop", True),
                        (bbox3d_node + ".inputs:frameId", self._frame_id),
                        (bbox3d_node + ".inputs:nodeNamespace", self._node_namespace),
                        (bbox3d_node + ".inputs:enableSemanticLabels", True),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", bbox3d_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", bbox3d_node + ".inputs:renderProductPath"),
                        (context_node + ".outputs:context", bbox3d_node + ".inputs:context"),
                    ],
                },
            )
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(bbox3d_node + ".inputs:context"),
                )

    def _build_ui(self) -> None:
        """Construct the user interface for configuring the ROS2 camera graph.

        Creates UI elements for graph path, camera prim selection, frame ID, node namespace, and topic
        configuration. Includes checkboxes to enable/disable publishing for each camera output type (RGB, depth,
        depth point cloud, instance, semantic, bounding boxes) with corresponding topic name fields. Adds OK/Cancel
        buttons and documentation links.
        """
        og_path_def = ParamWidget.FieldDef(
            name="og_path", label="Graph Path", type=ui.StringField, default=self._og_path
        )
        frame_id_def = ParamWidget.FieldDef(
            name="frame_id", label="Frame ID", type=ui.StringField, default=self._frame_id
        )
        node_namespace_def = ParamWidget.FieldDef(
            name="node_namespace", label="Node Namespace", type=ui.StringField, default=self._node_namespace
        )
        rgb_topic_def = ParamWidget.FieldDef(
            name="rgb topic", label="RGB Topic", type=ui.StringField, default=self._rgb_topic
        )
        depth_topic_def = ParamWidget.FieldDef(
            name="depth topic", label="Depth Topic", type=ui.StringField, default=self._depth_topic
        )
        depth_pcl_topic_def = ParamWidget.FieldDef(
            name="depth_pcl topic", label="Depth PCL Topic", type=ui.StringField, default=self._depth_pcl_topic
        )
        instance_topic_def = ParamWidget.FieldDef(
            name="instance topic", label="Instance Topic", type=ui.StringField, default=self._instance_topic
        )
        semantic_topic_def = ParamWidget.FieldDef(
            name="semantic topic", label="Semantic Topic", type=ui.StringField, default=self._semantic_topic
        )
        bbox2d_tight_topic_def = ParamWidget.FieldDef(
            name="bbox2d_tight topic", label="Bbox2d Tight Topic", type=ui.StringField, default=self._bbox2d_tight_topic
        )
        bbox2d_loose_topic_def = ParamWidget.FieldDef(
            name="bbox2d_loose topic", label="Bbox2d Loose Topic", type=ui.StringField, default=self._bbox2d_loose_topic
        )
        bbox3d_topic_def = ParamWidget.FieldDef(
            name="bbox3d topic", label="Bbox3d Topic", type=ui.StringField, default=self._bbox3d_topic
        )

        with self.frame:
            with ui.VStack(spacing=4):
                with ui.HStack():
                    ui.Label("Add to an existing graph?", width=ui.Percent(30))
                    cb = ui.SimpleBoolModel(default_value=self._add_to_existing_graph)
                    SimpleCheckBox(self._add_to_existing_graph, self._on_use_existing_graph, model=cb)
                self.og_path_input = ParamWidget(field_def=og_path_def)
                self.camera_prim_input = SelectPrimWidget(label="Camera Prim", default=self._camera_prim)
                self.frame_id_input = ParamWidget(field_def=frame_id_def)
                self.node_namespace_input = ParamWidget(field_def=node_namespace_def)
                ui.Spacer(height=5)
                with ui.HStack():
                    ui.Label("RGB", width=ui.Percent(15))
                    cb = ui.SimpleBoolModel(default_value=self._rgb_pub)
                    SimpleCheckBox(self._rgb_pub, self._on_rgb_pub, model=cb)
                    ui.Spacer(width=ui.Percent(5))
                    self.rgb_topic_input = ParamWidget(field_def=rgb_topic_def)
                with ui.HStack():
                    ui.Label("Depth", width=ui.Percent(15))
                    cb = ui.SimpleBoolModel(default_value=self._depth_pub)
                    SimpleCheckBox(self._depth_pub, self._on_depth_pub, model=cb)
                    ui.Spacer(width=ui.Percent(5))
                    self.depth_topic_input = ParamWidget(field_def=depth_topic_def)
                with ui.HStack():
                    ui.Label("Depth Point Clouds", width=ui.Percent(15), word_wrap=True)
                    cb = ui.SimpleBoolModel(default_value=self._depth_pcl_pub)
                    SimpleCheckBox(self._depth_pcl_pub, self._on_depth_pcl_pub, model=cb)
                    ui.Spacer(width=ui.Percent(5))
                    self.depth_pcl_topic_input = ParamWidget(field_def=depth_pcl_topic_def)
                with ui.HStack():
                    ui.Label("Instance", width=ui.Percent(15))
                    cb = ui.SimpleBoolModel(default_value=self._instance_pub)
                    SimpleCheckBox(self._instance_pub, self._on_instance_pub, model=cb)
                    ui.Spacer(width=ui.Percent(5))
                    self.instance_topic_input = ParamWidget(field_def=instance_topic_def)
                with ui.HStack():
                    ui.Label("Semantic", width=ui.Percent(15), word_wrap=True)
                    cb = ui.SimpleBoolModel(default_value=self._semantic_pub)
                    SimpleCheckBox(self._semantic_pub, self._on_semantic_pub, model=cb)
                    ui.Spacer(width=ui.Percent(5))
                    self.semantic_topic_input = ParamWidget(field_def=semantic_topic_def)
                with ui.HStack():
                    ui.Label("BoundingBox2D Tight", width=ui.Percent(15), word_wrap=True)
                    cb = ui.SimpleBoolModel(default_value=self._bbox2d_tight_pub)
                    SimpleCheckBox(self._bbox2d_tight_pub, self._on_bbox2d_tight_pub, model=cb)
                    ui.Spacer(width=ui.Percent(5))
                    self.bbox2d_tight_topic_input = ParamWidget(field_def=bbox2d_tight_topic_def)
                with ui.HStack():
                    ui.Label("BoundingBox2D Loose", width=ui.Percent(15), word_wrap=True)
                    cb = ui.SimpleBoolModel(default_value=self._bbox2d_loose_pub)
                    SimpleCheckBox(self._bbox2d_loose_pub, self._on_bbox2d_loose_pub, model=cb)
                    ui.Spacer(width=ui.Percent(5))
                    self.bbox2d_loose_topic_input = ParamWidget(field_def=bbox2d_loose_topic_def)
                with ui.HStack():
                    ui.Label("BoundingBox3D", width=ui.Percent(15), word_wrap=True)
                    cb = ui.SimpleBoolModel(default_value=self._bbox3d_pub)
                    SimpleCheckBox(self._bbox3d_pub, self._on_bbox3d_pub, model=cb)
                    ui.Spacer(width=ui.Percent(5))
                    self.bbox3d_topic_input = ParamWidget(field_def=bbox3d_topic_def)

                with ui.HStack():
                    ui.Spacer(width=ui.Percent(10))
                    ui.Button("OK", height=40, width=ui.Percent(30), clicked_fn=self._on_ok)
                    ui.Spacer(width=ui.Percent(20))
                    ui.Button("Cancel", height=40, width=ui.Percent(30), clicked_fn=self._on_cancel)
                    ui.Spacer(width=ui.Percent(10))
                with ui.Frame(height=30):
                    with ui.VStack():
                        with ui.HStack():
                            ui.Label("Python Script for Graph Generation", width=ui.Percent(30))
                            ui.Button(
                                name="IconButton",
                                width=24,
                                height=24,
                                clicked_fn=lambda: on_open_IDE_clicked("", __file__),
                                style=get_style()["IconButton.Image::OpenConfig"],
                            )
                        with ui.HStack():
                            ui.Label("Documentations", width=0, word_wrap=True)
                            ui.Button(
                                name="IconButton",
                                width=24,
                                height=24,
                                clicked_fn=lambda: on_docs_link_clicked(
                                    "https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_camera.html#graph-shortcut"
                                ),
                                style=get_style()["IconButton.Image::OpenLink"],
                            )

        return

    def _on_ok(self) -> None:
        """Handle the OK button click event.

        Retrieves all parameter values from UI widgets, validates them using parameter check, and if validation
        passes, generates the ROS2 camera graph and closes the window. Otherwise, displays a warning notification.
        """
        self._og_path = self.og_path_input.get_value()
        self._camera_prim = self.camera_prim_input.get_value()
        self._frame_id = self.frame_id_input.get_value()
        self._node_namespace = self.node_namespace_input.get_value()
        self._rgb_topic = self.rgb_topic_input.get_value()
        self._depth_topic = self.depth_topic_input.get_value()
        self._depth_pcl_topic = self.depth_pcl_topic_input.get_value()
        self._instance_topic = self.instance_topic_input.get_value()
        self._semantic_topic = self.semantic_topic_input.get_value()
        self._bbox2d_tight_topic = self.bbox2d_tight_topic_input.get_value()
        self._bbox2d_loose_topic = self.bbox2d_loose_topic_input.get_value()
        self._bbox3d_topic = self.bbox3d_topic_input.get_value()

        param_check = self._check_params()
        if param_check:
            self.make_graph()
            self.visible = False
        else:
            post_notification("Parameter check failed", status=NotificationStatus.WARNING)

    def _check_params(self) -> bool:
        """Validate the configured parameters before graph creation.

        When adding to an existing graph, verifies the graph path points to a valid OmniGraph prim. Validates that
        the camera prim path points to a valid UsdGeom.Camera prim. Displays warning notifications for invalid
        parameters.

        Returns:
            True if all parameters are valid, False otherwise.

        """
        stage = omni.usd.get_context().get_stage()

        if self._add_to_existing_graph:
            # make sure the "existing" graph exist
            og_prim = stage.GetPrimAtPath(self._og_path)
            if og_prim.IsValid() and og_prim.IsA(OmniGraphSchema.OmniGraph):
                pass
            else:
                msg = self._og_path + "is not an existing graph, check the og path"
                post_notification(msg, status=NotificationStatus.WARNING)
                return False

        # check if the camera prim is valid
        camera_prim = stage.GetPrimAtPath(self._camera_prim)
        if camera_prim.IsValid() and camera_prim.IsA(UsdGeom.Camera):
            return True

        msg = self._camera_prim + " is not a valid camera prim, check the camera prim"
        post_notification(msg, status=NotificationStatus.WARNING)
        return False

    def _on_cancel(self) -> None:
        """Handle the Cancel button click event by closing the window."""
        self.visible = False

    def _on_use_existing_graph(self, check_state: bool) -> None:
        """Handle the checkbox state change for adding to an existing graph.

        Args:
            check_state: Whether the checkbox is checked.

        """
        self._add_to_existing_graph = check_state

    def _on_rgb_pub(self, check_state: bool) -> None:
        """Handle the checkbox state change for RGB publishing.

        Args:
            check_state: Whether the checkbox is checked.

        """
        self._rgb_pub = check_state

    def _on_depth_pub(self, check_state: bool) -> None:
        """Handle the checkbox state change for depth publishing.

        Args:
            check_state: Whether the checkbox is checked.

        """
        self._depth_pub = check_state

    def _on_depth_pcl_pub(self, check_state: bool) -> None:
        """Handle the checkbox state change for depth point cloud publishing.

        Args:
            check_state: Whether the checkbox is checked.

        """
        self._depth_pcl_pub = check_state

    def _on_instance_pub(self, check_state: bool) -> None:
        """Handle the checkbox state change for instance segmentation publishing.

        Args:
            check_state: Whether the checkbox is checked.

        """
        self._instance_pub = check_state

    def _on_semantic_pub(self, check_state: bool) -> None:
        """Handle the semantic segmentation publishing checkbox state change.

        Args:
            check_state: The new state of the semantic segmentation checkbox.

        """
        self._semantic_pub = check_state

    def _on_bbox2d_tight_pub(self, check_state: bool) -> None:
        """Handle the 2D tight bounding box publishing checkbox state change.

        Args:
            check_state: The new state of the 2D tight bounding box checkbox.

        """
        self._bbox2d_tight_pub = check_state

    def _on_bbox2d_loose_pub(self, check_state: bool) -> None:
        """Handle the 2D loose bounding box publishing checkbox state change.

        Args:
            check_state: The new state of the 2D loose bounding box checkbox.

        """
        self._bbox2d_loose_pub = check_state

    def _on_bbox3d_pub(self, check_state: bool) -> None:
        """Handle the 3D bounding box publishing checkbox state change.

        Args:
            check_state: The new state of the 3D bounding box checkbox.

        """
        self._bbox3d_pub = check_state


class Ros2RtxLidarGraph(MenuHelperWindow):
    """A UI helper window for generating ROS2 action graphs for RTX lidar sensors.

    This window provides an interface to configure and generate OmniGraph action graphs that publish RTX lidar data to ROS2 topics. It supports both creating new graphs and adding nodes to existing graphs. The generated graph can publish laser scan messages and point cloud messages with configurable metadata fields. Users can select which point cloud metadata to include such as intensity, timestamp, emitter ID, channel ID, material ID, tick ID, hit normal, velocity, object ID, echo ID, and tick state.
    """

    # Point cloud metadata options: (display_name, attribute_name)
    # attribute_name corresponds to the input on ROS2RtxLidarPointCloudConfig node
    METADATA_OPTIONS = [
        ("Intensity", "Intensity"),
        ("Timestamp", "Timestamp"),
        ("Emitter ID", "EmitterId"),
        ("Channel ID", "ChannelId"),
        ("Material ID", "MaterialId"),
        ("Tick ID", "TickId"),
        ("Hit Normal", "HitNormal"),
        ("Velocity", "Velocity"),
        ("Object ID", "ObjectId"),
        ("Echo ID", "EchoId"),
        ("Tick State", "TickState"),
    ]
    """Point cloud metadata options available for selection.
    
    Each tuple contains (display_name, attribute_name) where display_name is shown in the UI and attribute_name
    corresponds to the input on the ROS2RtxLidarPointCloudConfig node.
    """

    def __init__(self) -> None:
        """Initialize the ROS2 RTX lidar graph window."""
        super().__init__("ROS2 RTX Lidar Graph", width=400, height=650)
        self._og_path = "/Graph/ROS_LidarRTX"
        self._frame_id = "sim_lidar"
        self._node_namespace = ""
        self._add_to_existing_graph = False
        self._lidar_prim = ""
        self._laser_scan_pub = True
        self._laser_scan_topic = "/laser_scan"
        self._point_cloud_pub = False
        self._point_cloud_topic = "/point_cloud"

        # Point cloud metadata options - dictionary keyed by attribute name
        self._metadata_selected = {attr_name: False for _, attr_name in self.METADATA_OPTIONS}

        # build UI
        self._build_ui()

    def make_graph(self) -> None:
        """Create or modifies an action graph for ROS2 RTX Lidar publishing.

        Generates a new graph or extends an existing one to publish RTX Lidar data. The graph includes nodes for laser scan and/or point cloud publishing based on the configured settings. When point cloud metadata options are selected, a configuration node is created to control which metadata fields are included in the published point cloud messages.
        """
        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline.stop()

        keys = og.Controller.Keys
        # if starting from a new graph, start it with just a tick, context, and render product, (no sim time), the rest is the same for adding to exsiting graph
        if not self._add_to_existing_graph:
            self._og_path = stage_utils.generate_next_free_path(self._og_path, prepend_default_prim=False)
            graph_handle, nodes, _, _ = og.Controller.edit(
                {"graph_path": self._og_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                        ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ],
                    keys.SET_VALUES: [("RenderProduct.inputs:cameraPrim", self._lidar_prim)],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
                        ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
                    ],
                },
            )
        else:
            graph_handle = og.get_graph_by_path(self._og_path)

        # to an existin graph
        # traverse through the graph
        all_nodes = graph_handle.get_nodes()
        tick_node = None
        context_node = None
        render_node = None
        for node in all_nodes:
            node_path = node.get_prim_path()
            node_type = node.get_type_name()
            if node_type == "omni.graph.action.OnPlaybackTick" or node_type == "omni.graph.action.OnTick":
                tick_node = node_path
            elif node_type == "isaacsim.ros2.bridge.ROS2Context":
                context_node = node_path
            elif node_type == "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame":
                run_once_node = node_path
            elif node_type == "isaacsim.core.nodes.IsaacCreateRenderProduct":
                render_node_path = node_path
                render_node = node

        if not tick_node or not context_node or not run_once_node:
            carb.log_warn(
                f"ActionGraph {self._og_path} missing node(s) necessary to build ROS2 graph. Skipping graph generation. Consider building new graph using tool."
            )
            return

        # if the existing graph doesn't already have a render node, or if the existing node does not use the same camera, then create a new render node and connect it to the new camera
        # TODO: so far only support if there's one existing render node. If there are multiple render nodes, it won't check if every node has unique camera prims.
        if render_node is None or render_node.get_attribute("inputs:cameraPrim").get()[0] != self._lidar_prim:
            render_node = stage_utils.generate_next_free_path(
                self._og_path + "/RenderProduct", ""
            )  # this is actually a string path at this point, not a node prim despite the name. This is so that it's consistent with the others.
            render_node_name = Path(render_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (render_node_name, "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ],
                    keys.SET_VALUES: [
                        (render_node_name + ".inputs:cameraPrim", self._lidar_prim),
                    ],
                    keys.CONNECT: [
                        (run_once_node + ".outputs:step", render_node_name + ".inputs:execIn"),
                    ],
                },
            )
        else:
            render_node = render_node_path  # once again set render_node to the actual path, as oppose to the node_prim, just for consistency

        if self._laser_scan_pub:
            laser_scan_node = stage_utils.generate_next_free_path(
                self._og_path + "/LaserScanPublish", prepend_default_prim=False
            )
            laser_scan_node_name = Path(laser_scan_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (laser_scan_node_name, "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                    ],
                    keys.SET_VALUES: [
                        (laser_scan_node + ".inputs:topicName", self._laser_scan_topic),
                        (laser_scan_node + ".inputs:type", "laser_scan"),
                        (laser_scan_node + ".inputs:frameId", self._frame_id),
                        (laser_scan_node + ".inputs:nodeNamespace", self._node_namespace),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", laser_scan_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", laser_scan_node + ".inputs:renderProductPath"),
                    ],
                },
            )
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(laser_scan_node + ".inputs:context"),
                )

        if self._point_cloud_pub:
            point_cloud_node = stage_utils.generate_next_free_path(
                self._og_path + "/PointCloudPublish", prepend_default_prim=False
            )
            point_cloud_node_name = Path(point_cloud_node).name

            # Check if any metadata is selected, create config node and connect it
            selected_metadata = [attr for attr, selected in self._metadata_selected.items() if selected]

            if selected_metadata:
                pcl_config_node = stage_utils.generate_next_free_path(
                    self._og_path + "/PointCloudConfig", prepend_default_prim=False
                )
                pcl_config_node_name = Path(pcl_config_node).name

                # Create the config node with the selected metadata
                config_set_values = [(f"{pcl_config_node}.inputs:output{attr}", True) for attr in selected_metadata]

                # Point cloud helper values
                helper_set_values = [
                    (point_cloud_node + ".inputs:topicName", self._point_cloud_topic),
                    (point_cloud_node + ".inputs:type", "point_cloud"),
                    (point_cloud_node + ".inputs:frameId", self._frame_id),
                    (point_cloud_node + ".inputs:nodeNamespace", self._node_namespace),
                ]

                # Enable Object ID map publishing if ObjectId metadata is selected
                if "ObjectId" in selected_metadata:
                    helper_set_values.append((point_cloud_node + ".inputs:enableObjectIdMap", True))

                og.Controller.edit(
                    graph_handle,
                    {
                        keys.CREATE_NODES: [
                            (pcl_config_node_name, "isaacsim.ros2.bridge.ROS2RtxLidarPointCloudConfig"),
                            (point_cloud_node_name, "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                        ],
                        keys.SET_VALUES: config_set_values + helper_set_values,
                        keys.CONNECT: [
                            (render_node + ".outputs:execOut", point_cloud_node + ".inputs:execIn"),
                            (
                                render_node + ".outputs:renderProductPath",
                                point_cloud_node + ".inputs:renderProductPath",
                            ),
                            (
                                pcl_config_node + ".outputs:selectedMetadata",
                                point_cloud_node + ".inputs:selectedMetadata",
                            ),
                        ],
                    },
                )
            else:
                og.Controller.edit(
                    graph_handle,
                    {
                        keys.CREATE_NODES: [
                            (point_cloud_node_name, "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                        ],
                        keys.SET_VALUES: [
                            (point_cloud_node + ".inputs:topicName", self._point_cloud_topic),
                            (point_cloud_node + ".inputs:type", "point_cloud"),
                            (point_cloud_node + ".inputs:frameId", self._frame_id),
                            (point_cloud_node + ".inputs:nodeNamespace", self._node_namespace),
                        ],
                        keys.CONNECT: [
                            (render_node + ".outputs:execOut", point_cloud_node + ".inputs:execIn"),
                            (
                                render_node + ".outputs:renderProductPath",
                                point_cloud_node + ".inputs:renderProductPath",
                            ),
                        ],
                    },
                )
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(point_cloud_node + ".inputs:context"),
                )
        elif any(self._metadata_selected.values()):
            # Warn if metadata is selected but point cloud is not enabled
            post_notification(
                "Point cloud metadata options are selected but Point Cloud publishing is disabled. Metadata options will be ignored.",
                status=NotificationStatus.WARNING,
            )

    def _build_ui(self) -> None:
        """Build the user interface for the RTX Lidar graph configuration window.

        Creates input fields for graph path, lidar prim selection, frame ID, node namespace, and topic names. Includes checkboxes for enabling laser scan and point cloud publishing, along with a two-column layout for selecting point cloud metadata options.
        """
        og_path_def = ParamWidget.FieldDef(
            name="og_path", label="Graph Path", type=ui.StringField, default=self._og_path
        )
        frame_id_def = ParamWidget.FieldDef(
            name="frame_id", label="Frame ID", type=ui.StringField, default=self._frame_id
        )
        node_namespace_def = ParamWidget.FieldDef(
            name="node_namespace", label="Node Namespace", type=ui.StringField, default=self._node_namespace
        )
        laser_scan_topic_def = ParamWidget.FieldDef(
            name="laser_scan_topic", label="LaserScan Topic", type=ui.StringField, default=self._laser_scan_topic
        )
        point_cloud_topic_def = ParamWidget.FieldDef(
            name="point_cloud_topic", label="Point Cloud Topic", type=ui.StringField, default=self._point_cloud_topic
        )

        with self.frame:
            with ui.VStack(spacing=4):
                with ui.HStack():
                    ui.Label("Add to an existing graph?", width=ui.Percent(30))
                    cb = ui.SimpleBoolModel(default_value=self._add_to_existing_graph)
                    SimpleCheckBox(self._add_to_existing_graph, self._on_use_existing_graph, model=cb)
                self.og_path_input = ParamWidget(field_def=og_path_def)
                self.lidar_prim_input = SelectPrimWidget(label="Lidar Prim", default=self._lidar_prim)
                self.frame_id_input = ParamWidget(field_def=frame_id_def)
                self.node_namespace_input = ParamWidget(field_def=node_namespace_def)
                ui.Spacer(height=5)
                with ui.HStack():
                    ui.Label("Laser Scan", width=ui.Percent(15))
                    cb = ui.SimpleBoolModel(default_value=self._laser_scan_pub)
                    SimpleCheckBox(self._laser_scan_pub, self._on_laser_scan_pub, model=cb)
                    ui.Spacer(width=ui.Percent(5))
                    self.laser_scan_topic_input = ParamWidget(field_def=laser_scan_topic_def)
                with ui.HStack():
                    ui.Label("Point Cloud", width=ui.Percent(15))
                    cb = ui.SimpleBoolModel(default_value=self._point_cloud_pub)
                    SimpleCheckBox(self._point_cloud_pub, self._on_point_cloud_pub, model=cb)
                    ui.Spacer(width=ui.Percent(5))
                    self.point_cloud_topic_input = ParamWidget(field_def=point_cloud_topic_def)

                # Point Cloud Metadata Section
                ui.Spacer(height=5)
                ui.Label("Point Cloud Metadata", word_wrap=True)
                with ui.HStack():
                    # Split metadata options into two columns
                    mid_point = (len(self.METADATA_OPTIONS) + 1) // 2
                    left_options = self.METADATA_OPTIONS[:mid_point]
                    right_options = self.METADATA_OPTIONS[mid_point:]

                    with ui.VStack(spacing=2):
                        for display_name, attr_name in left_options:
                            with ui.HStack():
                                ui.Label(display_name, width=ui.Percent(30))
                                cb = ui.SimpleBoolModel(default_value=self._metadata_selected[attr_name])
                                SimpleCheckBox(
                                    self._metadata_selected[attr_name],
                                    lambda checked, attr=attr_name: self._on_metadata_changed(attr, checked),
                                    model=cb,
                                )
                    with ui.VStack(spacing=2):
                        for display_name, attr_name in right_options:
                            with ui.HStack():
                                ui.Label(display_name, width=ui.Percent(30))
                                cb = ui.SimpleBoolModel(default_value=self._metadata_selected[attr_name])
                                SimpleCheckBox(
                                    self._metadata_selected[attr_name],
                                    lambda checked, attr=attr_name: self._on_metadata_changed(attr, checked),
                                    model=cb,
                                )

                with ui.HStack():
                    ui.Spacer(width=ui.Percent(10))
                    ui.Button("OK", height=40, width=ui.Percent(30), clicked_fn=self._on_ok)
                    ui.Spacer(width=ui.Percent(20))
                    ui.Button("Cancel", height=40, width=ui.Percent(30), clicked_fn=self._on_cancel)
                    ui.Spacer(width=ui.Percent(10))
                with ui.Frame(height=30):
                    with ui.VStack():
                        with ui.HStack():
                            ui.Label("Python Script for Graph Generation", width=ui.Percent(30))
                            ui.Button(
                                name="IconButton",
                                width=24,
                                height=24,
                                clicked_fn=lambda: on_open_IDE_clicked("", __file__),
                                style=get_style()["IconButton.Image::OpenConfig"],
                            )
                        with ui.HStack():
                            ui.Label("Documentations", width=0, word_wrap=True)
                            ui.Button(
                                name="IconButton",
                                width=24,
                                height=24,
                                clicked_fn=lambda: on_docs_link_clicked(
                                    "https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_rtx_lidar.html#graph-shortcut"
                                ),
                                style=get_style()["IconButton.Image::OpenLink"],
                            )

        return

    def _on_ok(self) -> None:
        """Handle the OK button click event.

        Collects values from all UI input fields, validates the parameters, and generates the graph if validation passes. Closes the window upon successful graph generation.
        """
        self._og_path = self.og_path_input.get_value()
        self._lidar_prim = self.lidar_prim_input.get_value()
        self._frame_id = self.frame_id_input.get_value()
        self._node_namespace = self.node_namespace_input.get_value()
        self._laser_scan_topic = self.laser_scan_topic_input.get_value()
        self._point_cloud_topic = self.point_cloud_topic_input.get_value()

        param_check = self._check_params()
        if param_check:
            self.make_graph()
            self.visible = False
        else:
            post_notification("Parameter check failed", status=NotificationStatus.WARNING)

    def _on_cancel(self) -> None:
        """Handle the Cancel button click event.

        Closes the window without generating or modifying the graph.
        """
        self.visible = False

    def _check_params(self) -> bool:
        """Validate the graph and lidar prim parameters.

        Verifies that the specified graph path exists if adding to an existing graph, and confirms that the lidar prim is a valid RTX lidar (either a Camera with IsaacRtxLidarSensorAPI or an OmniLidar with OmniSensorGenericLidarCoreAPI). Displays warning notifications for any validation failures.

        Returns:
            True if all parameters are valid, False otherwise.

        """
        stage = omni.usd.get_context().get_stage()

        if self._add_to_existing_graph:
            # make sure the "existing" graph exist
            og_prim = stage.GetPrimAtPath(self._og_path)
            if og_prim.IsValid() and og_prim.IsA(OmniGraphSchema.OmniGraph):
                pass
            else:
                msg = self._og_path + "is not an existing graph, check the og path"
                post_notification(msg, status=NotificationStatus.WARNING)
                return False

        # check if the lidar prim is valid
        lidar_prim = stage.GetPrimAtPath(self._lidar_prim)
        if lidar_prim.IsValid():
            if lidar_prim.GetTypeName() == "OmniLidar" and lidar_prim.HasAPI("OmniSensorGenericLidarCoreAPI"):
                return True

        msg = self._lidar_prim + " is not a valid RTX lidar prim, check the lidar prim"
        post_notification(msg, status=NotificationStatus.WARNING)
        return False

    def _on_use_existing_graph(self, check_state: bool) -> None:
        """Handle the checkbox state change for using an existing graph.

        Args:
            check_state: Whether to add nodes to an existing graph instead of creating a new one.

        """
        self._add_to_existing_graph = check_state

    def _on_laser_scan_pub(self, check_state: bool) -> None:
        """Handle the checkbox state change for laser scan publishing.

        Args:
            check_state: Whether to enable laser scan publishing.

        """
        self._laser_scan_pub = check_state

    def _on_point_cloud_pub(self, check_state: bool) -> None:
        """Handle the checkbox state change for point cloud publishing.

        Args:
            check_state: Whether to enable point cloud publishing.

        """
        self._point_cloud_pub = check_state

    def _on_metadata_changed(self, attr_name: str, check_state: bool) -> None:
        """Handle metadata checkbox state change.

        Args:
            attr_name: Name of the metadata attribute being toggled.
            check_state: Whether the metadata option is enabled.

        """
        self._metadata_selected[attr_name] = check_state
