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

"""Provides OmniGraph nodes for ROS 2 communication including publishers, subscribers, and services."""

import os

import carb
import omni.ext
import omni.replicator.core as rep
import omni.syntheticdata
import omni.syntheticdata._syntheticdata as sd
from isaacsim.core.nodes.scripts.utils import register_node_writer_with_telemetry
from isaacsim.ros2.nodes.bindings._ros2_nodes import acquire_interface, release_interface

from .ros2_common import BRIDGE_NAME, BRIDGE_PREFIX


class ROS2NodesExtension(omni.ext.IExt):
    """ROS 2 Nodes Extension - OmniGraph nodes for ROS 2 integration.

    This extension provides all the OmniGraph nodes for ROS 2 communication,
    including publishers, subscribers, and services. It relies on the ros2.core
    extension for foundational ROS 2 functionality.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize the ROS 2 nodes extension.

        Args:
            ext_id: The extension ID.

        """
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self._extension_path = ext_manager.get_extension_path(ext_id)

        # Load the nodes plugin that contains the C++ OmniGraph nodes
        carb.get_framework().load_plugins(
            loaded_file_wildcards=["isaacsim.ros2.nodes.plugin"],
            search_paths=[os.path.abspath(os.path.join(self._extension_path, "bin"))],
        )

        self.__interface = acquire_interface()

        # Register all the synthetic data node writers
        self.register_nodes()

        carb.log_info("ROS2 Nodes: OmniGraph nodes extension loaded successfully")

    def on_shutdown(self) -> None:
        """Shutdown the nodes extension.

        Clean up registered nodes and release the core interface.
        """
        self.unregister_nodes()
        self.__interface = release_interface(self.__interface)
        carb.log_info("ROS2 Nodes: Extension shutdown complete")

    def register_nodes(self) -> None:
        """Register ROS 2 OmniGraph node writers."""
        # For Simulation and System time. Removed first S char in keys to account for both upper and lower cases.
        TIME_TYPES = [("imulationTime", ""), ("ystemTime", "SystemTime")]

        for time_type in TIME_TYPES:
            ##### Publish RGB
            rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
            register_node_writer_with_telemetry(
                name=f"{rv}{BRIDGE_PREFIX}{time_type[1]}PublishImage",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishImage",
                annotators=[
                    f"{rv}IsaacConvertRGBAToRGB",
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                category=BRIDGE_NAME,
            )

            ##### Publish Depth
            rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                sd.SensorType.DistanceToImagePlane.name
            )
            register_node_writer_with_telemetry(
                name=f"{rv}{BRIDGE_PREFIX}{time_type[1]}PublishImage",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishImage",
                annotators=[
                    f"{rv}IsaacPassthroughImagePtr",
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                encoding="32FC1",
                category=BRIDGE_NAME,
            )

            # publish depth pcl
            register_node_writer_with_telemetry(
                name=f"{rv}{BRIDGE_PREFIX}{time_type[1]}PublishPointCloud",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishPointCloud",
                annotators=[
                    f"{rv}IsaacConvertDepthToPointCloud",
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                category=BRIDGE_NAME,
            )

            # instance
            register_node_writer_with_telemetry(
                name=f"{BRIDGE_PREFIX}{time_type[1]}PublishInstanceSegmentation",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishImage",
                annotators=[
                    "instance_segmentation",
                    f'{omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar("InstanceSegmentation")}IsaacSimulationGate',
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                encoding="32SC1",
                category=BRIDGE_NAME,
            )

            # Semantic
            register_node_writer_with_telemetry(
                name=f"{BRIDGE_PREFIX}{time_type[1]}PublishSemanticSegmentation",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishImage",
                annotators=[
                    "semantic_segmentation",
                    f'{omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar("SemanticSegmentation")}IsaacSimulationGate',
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                encoding="32SC1",
                category=BRIDGE_NAME,
            )

            # Bbox2d tight
            register_node_writer_with_telemetry(
                name=f"{BRIDGE_PREFIX}{time_type[1]}PublishBoundingBox2DTight",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishBbox2D",
                annotators=[
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        "bounding_box_2d_tight", attributes_mapping={"input:semanticTypes": ["class"]}
                    ),
                    f'{omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar("BoundingBox2DTight")}IsaacSimulationGate',
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                category=BRIDGE_NAME,
            )

            # bbox2d Loose
            register_node_writer_with_telemetry(
                name=f"{BRIDGE_PREFIX}{time_type[1]}PublishBoundingBox2DLoose",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishBbox2D",
                annotators=[
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        "bounding_box_2d_loose",
                        attributes_mapping={"input:semanticTypes": ["class"], "outputs:data": "inputs:data"},
                    ),
                    f'{omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar("BoundingBox2DLoose")}IsaacSimulationGate',
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                category=BRIDGE_NAME,
            )

            # bbox3d Loose
            register_node_writer_with_telemetry(
                name=f"{BRIDGE_PREFIX}{time_type[1]}PublishBoundingBox3D",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishBbox3D",
                annotators=[
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        "bounding_box_3d",
                        attributes_mapping={"input:semanticTypes": ["class"], "outputs:data": "inputs:data"},
                    ),
                    f'{omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar("BoundingBox3D")}IsaacSimulationGate',
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                category=BRIDGE_NAME,
            )

            # camera info
            register_node_writer_with_telemetry(
                name=f"{BRIDGE_PREFIX}{time_type[1]}PublishCameraInfo",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishCameraInfo",
                annotators=[
                    "PostProcessDispatchIsaacSimulationGate",
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                category=BRIDGE_NAME,
            )

            # outputs that we can publish labels for
            label_names = {
                "instance_segmentation": omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                    "InstanceSegmentation"
                ),
                "semantic_segmentation": omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                    "SemanticSegmentation"
                ),
                "bounding_box_2d_tight": omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                    "BoundingBox2DTight"
                ),
                "bounding_box_2d_loose": omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                    "BoundingBox2DLoose"
                ),
                "bounding_box_3d": omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar("BoundingBox3D"),
            }
            for annotator, annotator_name in label_names.items():
                register_node_writer_with_telemetry(
                    name=f"{annotator_name}{BRIDGE_PREFIX}{time_type[1]}PublishSemanticLabels",
                    node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishSemanticLabels",
                    annotators=[
                        annotator,
                        f"{annotator_name}IsaacSimulationGate",
                        omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                            f"IsaacReadS{time_type[0]}",
                            attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"},
                        ),
                    ],
                    category=BRIDGE_NAME,
                )

            # RTX lidar/radar PCL publisher (direct from GMO)
            gmo_pcl_annotators = [
                "IsaacExtractRTXSensorPointCloud",
                "PostProcessDispatchIsaacSimulationGate",
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                ),
            ]
            for name_prefix in ("RtxLidar", "RtxRadar"):
                register_node_writer_with_telemetry(
                    name=f"{name_prefix}{BRIDGE_PREFIX}{time_type[1]}PublishPointCloud",
                    node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishPointCloud",
                    annotators=gmo_pcl_annotators,
                    category=BRIDGE_NAME,
                )

            register_node_writer_with_telemetry(
                name=f"RtxLidar{BRIDGE_PREFIX}{time_type[1]}PublishPointCloudBuffer",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishPointCloud",
                annotators=gmo_pcl_annotators,
                category=BRIDGE_NAME,
            )

            # RTX lidar LaserScan publisher (direct from GMO)
            register_node_writer_with_telemetry(
                name=f"RtxLidar{BRIDGE_PREFIX}{time_type[1]}PublishLaserScan",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishLaserScan",
                annotators=[
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        "GenericModelOutputPtr",
                        attributes_mapping={
                            "outputs:dataPtr": "inputs:dataPtr",
                            "outputs:bufferSize": "inputs:bufferSize",
                        },
                    ),
                    "PostProcessDispatchIsaacSimulationGate",
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}",
                        attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"},
                    ),
                ],
                category=BRIDGE_NAME,
            )

            # Object ID Map publisher
            register_node_writer_with_telemetry(
                name=f"{BRIDGE_PREFIX}{time_type[1]}PublishObjectIdMap",
                node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishObjectIdMap",
                annotators=[
                    "StableIdMap",
                    "PostProcessDispatchIsaacSimulationGate",
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                category=BRIDGE_NAME,
            )

    def unregister_nodes(self) -> None:
        """Unregister ROS 2 OmniGraph node writers."""
        for writer in rep.WriterRegistry.get_writers(category=BRIDGE_NAME):
            rep.writers.unregister_writer(writer)
