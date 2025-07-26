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

import ctypes
import gc
from typing import List, Tuple, Union

import carb
import carb.settings
import isaacsim.sensors.rtx.generic_model_output as gmo_utils
import numpy as np
import omni.ext
import omni.graph.core as og
import omni.kit.commands
import omni.replicator.core as rep
from isaacsim.core.nodes.scripts.utils import (
    register_annotator_from_node_with_telemetry,
    register_node_writer_with_telemetry,
)
from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.sensors.rtx.bindings._isaacsim_sensors_rtx import acquire_interface as _acquire
from isaacsim.sensors.rtx.bindings._isaacsim_sensors_rtx import release_interface as _release
from omni.replicator.core import AnnotatorRegistry
from omni.syntheticdata import sensors

EXTENSION_NAME = "Isaac Sensor"


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self.__interface = _acquire()
        self._lidar_point_accumulator_count = 0
        self.registered_templates = []
        self.registered_annotators = []
        try:
            self.register_nodes()
        except Exception as e:
            carb.log_warn(f"Could not register node templates {e}")

    def on_shutdown(self):

        _release(self.__interface)
        self.__interface = None

        try:
            self.unregister_nodes()
        except Exception as e:
            carb.log_warn(f"Could not unregister node templates {e}")
        gc.collect()

    def _update_upstream_node_attributes(self, upstream_node_type_name: str, attribute: str, value, node: og.Node):
        # Test if the originating node is of the appropriate type
        if node.get_type_name == upstream_node_type_name:
            carb.log_warn(
                f"Provided node {node} is of type {upstream_node_type_name}. Setting attribute {attribute} to {value}."
            )
            if not node.get_attribute(attribute).set(value=value):
                carb.log_error(
                    f"Error setting {upstream_node_type_name}:{attribute} to {value}. Attribute will not be set."
                )
            return
        # Else, traverse the upstream graph
        for upstream_node in og.traverse_upstream_graph([get_prim_at_path(node.get_prim_path())]):
            if upstream_node.get_type_name() == upstream_node_type_name:
                if not upstream_node.get_attribute(attribute).set(value=value):
                    carb.log_error(
                        f"Error setting {upstream_node_type_name}:{attribute} to {value}. Attribute will not be set."
                    )
                return
        # Warn if we couldn't find the appropriate type
        carb.log_warn(
            f"Could not find node of type {upstream_node_type_name} upstream of {node}. Attribute will not be set."
        )
        return

    def _on_attach_callback_base(
        self, annotator_name: str, connections: List[Tuple[str, str, str, str]], node: og.Node
    ):
        """
        Callback function for annotator attachment. Will connect ancestral upstream node(s) to each other and annotator node, if user
        desires connections beyond immediate parent nodes.

        Args:
            annotator_name (str): Name of annotator being attached.
            connections (List[Tuple[str, str, str, str]]): List of connections to create between nodes, specified as (source_node, source_attr, target_node, target_attr). source_node and target_node are node types; if desired target is annotator node, provide node prim path.
            node (og.Node): Annotator node being attached.
        """

        # Define map of parent node types to their prim paths
        parent_nodes = {}
        for source_node, _, target_node, _ in connections:
            parent_nodes[source_node] = None
            if target_node != node.get_prim_path() and target_node not in parent_nodes:
                parent_nodes[target_node] = None

        # Traverse upstream nodes to find desired parent nodes
        for upstream_node in og.traverse_upstream_graph([get_prim_at_path(node.get_prim_path())]):
            if upstream_node.get_type_name() in parent_nodes and parent_nodes[upstream_node.get_type_name()] is None:
                parent_nodes[upstream_node.get_type_name()] = upstream_node.get_prim_path()

        # Check if we found all parent nodes
        for parent_node_type in parent_nodes:
            if parent_nodes[parent_node_type] is None:
                carb.log_warn(
                    f"Missing parent node type {parent_node_type} when connecting {annotator_name}. Annotator will not be attached correctly."
                )
                return

        # Generate properly formatted connection list, using node prim paths collected earlier
        controller_connections = []
        for source_node, source_attr, target_node, target_attr in connections:
            if target_node in parent_nodes:
                target_node = parent_nodes[target_node]
            controller_connections.append(
                (f"{parent_nodes[source_node]}.outputs:{source_attr}", f"{target_node}.inputs:{target_attr}")
            )

        # Now connect the nodes
        try:
            controller = og.Controller()
            keys = og.Controller.Keys
            controller.edit(node.get_graph().get_path_to_graph(), {keys.CONNECT: controller_connections})

        except Exception as e:
            carb.log_error(f"Error connecting {annotator_name}: {e}. Annotator will not be attached correctly.")

        return

    def register_nodes(self):

        # Connects:
        # (GenericModelOutput and RtxSensorMetadata) -> LidarPointAccumulator
        annotator_name = "GenericModelOutput" + "LidarPointAccumulator"

        def _on_attach_gmo_lidar_point_accumulator(node: og.Node):
            # Repeat annotator name definition in callback to avoid scope issues
            annotator_name = "GenericModelOutput" + "LidarPointAccumulator"
            # Check /app/sensors/nv/lidar/outputBufferOnGPU and set node input appropriately
            output_lidar_buffer_on_gpu = carb.settings.get_settings().get("/app/sensors/nv/lidar/outputBufferOnGPU")
            self._update_upstream_node_attributes(
                upstream_node_type_name="omni.sensors.nv.lidar.LidarPointAccumulator",
                attribute="inputs:outputOnGPU",
                value=output_lidar_buffer_on_gpu,
                node=node,
            )
            self._update_upstream_node_attributes(
                upstream_node_type_name="omni.sensors.nv.lidar.LidarPointAccumulator",
                attribute="inputs:sendDataId",
                value=self._lidar_point_accumulator_count,
                node=node,
            )
            self._lidar_point_accumulator_count += 1
            # Draw upstream connections
            return self._on_attach_callback_base(
                annotator_name=annotator_name,
                connections=[
                    ("omni.syntheticdata.SdOnNewRenderProductFrame", "cudaStream", node.get_prim_path(), "cudaStream"),
                ],
                node=node,
            )

        register_annotator_from_node_with_telemetry(
            name=annotator_name,
            input_rendervars=[
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "GenericModelOutput" + "Ptr", attributes_mapping={"outputs:dataPtr": "inputs:src"}
                ),
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "RtxSensorMetadata" + "Ptr", attributes_mapping={"outputs:dataPtr": "inputs:srcMeta"}
                ),
            ],
            node_type_id="omni.sensors.nv.lidar.LidarPointAccumulator",
            init_params={
                "exportBytePoints": True,
                "publishVizPoints": False,
                "sendDataId": 2,
                "colorCode": 1,
                "sensorMount6DPose": [0, 0, 0, 0, 0, 0],
                "targetPID": 0,
                "desiredCoordsType": "SPHERICAL",
            },
            on_attach_callback=_on_attach_gmo_lidar_point_accumulator,
        )
        self.registered_annotators.append(annotator_name)

        # Connects:
        # (GenericModelOutput and RtxSensorMetadata) -> LidarPointAccumulator -> IsaacComputeRTXLidarFlatScan
        annotator_name = "IsaacComputeRTXLidarFlatScan"

        def _on_attach_gmo_flatscan(node: og.Node):
            # Repeat annotator name definition in callback to avoid scope issues
            annotator_name = "IsaacComputeRTXLidarFlatScan"

            # Enforce LidarPointAccumulator output on CPU
            self._update_upstream_node_attributes(
                upstream_node_type_name="omni.sensors.nv.lidar.LidarPointAccumulator",
                attribute="inputs:outputOnGPU",
                value=False,
                node=node,
            )
            self._update_upstream_node_attributes(
                upstream_node_type_name="omni.sensors.nv.lidar.LidarPointAccumulator",
                attribute="inputs:sendDataId",
                value=self._lidar_point_accumulator_count,
                node=node,
            )
            self._lidar_point_accumulator_count += 1
            return self._on_attach_callback_base(
                annotator_name=annotator_name,
                connections=[
                    (
                        "omni.syntheticdata.SdOnNewRenderProductFrame",
                        "cudaStream",
                        "omni.sensors.nv.lidar.LidarPointAccumulator",
                        "cudaStream",
                    ),
                    (
                        "omni.syntheticdata.SdRenderVarPtr",
                        "exec",
                        node.get_prim_path(),
                        "exec",
                    ),
                    (
                        "omni.syntheticdata.SdOnNewRenderProductFrame",
                        "renderProductPath",
                        node.get_prim_path(),
                        "renderProductPath",
                    ),
                ],
                node=node,
            )

        register_annotator_from_node_with_telemetry(
            name=annotator_name,
            input_rendervars=[
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "GenericModelOutput" + "LidarPointAccumulator",
                    attributes_mapping={
                        "outputs:dest": "inputs:dataPtr",
                        "outputs:cudaStream": "inputs:cudaStream",
                        "outputs:newData": "inputs:newData",
                    },
                ),
            ],
            node_type_id="isaacsim.sensors.rtx.IsaacComputeRTXLidarFlatScan",
            output_data_type=np.float32,
            output_channels=3,
            on_attach_callback=_on_attach_gmo_flatscan,
        )
        self.registered_annotators.append(annotator_name)

        # Connects:
        # GenericModelOutput -> IsaacExtractRTXSensorPointCloud
        annotator_name = "IsaacExtractRTXSensorPointCloud" + "NoAccumulator"

        def _on_attach_isaac_extract_rtx_sensor_point_cloud_no_accumulator(node: og.Node):
            # Repeat annotator name definition in callback to avoid scope issues
            annotator_name = "IsaacExtractRTXSensorPointCloudNoAccumulator"
            # Draw upstream connections
            return self._on_attach_callback_base(
                annotator_name=annotator_name,
                connections=[
                    ("omni.syntheticdata.SdOnNewRenderProductFrame", "cudaStream", node.get_prim_path(), "cudaStream"),
                ],
                node=node,
            )

        register_annotator_from_node_with_telemetry(
            name=annotator_name,
            input_rendervars=["GenericModelOutput" + "Ptr"],
            node_type_id="isaacsim.sensors.rtx.IsaacExtractRTXSensorPointCloud",
            output_data_type=np.float32,
            output_channels=3,
            on_attach_callback=_on_attach_isaac_extract_rtx_sensor_point_cloud_no_accumulator,
        )
        self.registered_annotators.append(annotator_name)

        # Connects:
        # (GenericModelOutput and RtxSensorMetadata) -> LidarPointAccumulator -> IsaacExtractRTXSensorPointCloud
        annotator_name = "IsaacExtractRTXSensorPointCloud"

        def _on_attach_gmo_extract_rtx_sensor_point_cloud(node: og.Node):
            # Repeat annotator name definition in callback to avoid scope issues
            annotator_name = "IsaacExtractRTXSensorPointCloud"

            # Check /app/sensors/nv/lidar/outputBufferOnGPU and set node input appropriately
            output_lidar_buffer_on_gpu = carb.settings.get_settings().get("/app/sensors/nv/lidar/outputBufferOnGPU")
            self._update_upstream_node_attributes(
                upstream_node_type_name="omni.sensors.nv.lidar.LidarPointAccumulator",
                attribute="inputs:outputOnGPU",
                value=output_lidar_buffer_on_gpu,
                node=node,
            )
            self._update_upstream_node_attributes(
                upstream_node_type_name="omni.sensors.nv.lidar.LidarPointAccumulator",
                attribute="inputs:sendDataId",
                value=self._lidar_point_accumulator_count,
                node=node,
            )
            self._lidar_point_accumulator_count += 1

            return self._on_attach_callback_base(
                annotator_name=annotator_name,
                connections=[
                    (
                        "omni.syntheticdata.SdOnNewRenderProductFrame",
                        "cudaStream",
                        "omni.sensors.nv.lidar.LidarPointAccumulator",
                        "cudaStream",
                    ),
                    (
                        "omni.syntheticdata.SdRenderVarPtr",
                        "exec",
                        node.get_prim_path(),
                        "exec",
                    ),
                ],
                node=node,
            )

        register_annotator_from_node_with_telemetry(
            name=annotator_name,
            input_rendervars=[
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "GenericModelOutput" + "LidarPointAccumulator",
                    attributes_mapping={
                        "outputs:dest": "inputs:dataPtr",
                        "outputs:cudaStream": "inputs:cudaStream",
                        "outputs:newData": "inputs:newData",
                    },
                ),
            ],
            node_type_id="isaacsim.sensors.rtx.IsaacExtractRTXSensorPointCloud",
            init_params={"accumulateLidarScan": True},
            output_data_type=np.float32,
            output_channels=3,
            on_attach_callback=_on_attach_gmo_extract_rtx_sensor_point_cloud,
        )
        self.registered_annotators.append(annotator_name)

        # RTX lidar Debug Draw Writer
        register_node_writer_with_telemetry(
            name="RtxLidar" + "DebugDrawPointCloud",
            node_type_id="isaacsim.util.debug_draw.DebugDrawPointCloud",
            annotators=["IsaacExtractRTXSensorPointCloudNoAccumulator"],
            doTransform=True,
            category="isaacsim.sensors.rtx",
        )

        # RTX lidar Debug Draw Writer
        register_node_writer_with_telemetry(
            name="RtxLidar" + "DebugDrawPointCloud" + "Buffer",
            node_type_id="isaacsim.util.debug_draw.DebugDrawPointCloud",
            annotators=["IsaacExtractRTXSensorPointCloud"],
            doTransform=True,
            category="isaacsim.sensors.rtx",
        )

        # RTX radar Debug Draw Writer
        register_node_writer_with_telemetry(
            name="RtxRadar" + "DebugDrawPointCloud",
            node_type_id=f"isaacsim.util.debug_draw.DebugDrawPointCloud",
            annotators=["IsaacExtractRTXSensorPointCloudNoAccumulator"],
            # hard to see radar points... so make them more visible.
            size=0.2,
            color=[1, 0.2, 0.3, 1],
            doTransform=True,
            category="isaacsim.sensors.rtx",
        )
        register_node_writer_with_telemetry(
            name="RtxRadar" + "DebugDrawPointCloud" + "Buffer",
            node_type_id=f"isaacsim.util.debug_draw.DebugDrawPointCloud",
            annotators=["IsaacExtractRTXSensorPointCloud"],
            # hard to see radar points... so make them more visible.
            size=0.2,
            color=[1, 0.2, 0.3, 1],
            doTransform=True,
            category="isaacsim.sensors.rtx",
        )

    def unregister_nodes(self):
        for writer in rep.WriterRegistry.get_writers(category="isaacsim.sensors.rtx"):
            rep.writers.unregister_writer(writer)
        for annotator in self.registered_annotators:
            AnnotatorRegistry.unregister_annotator(annotator)
        for template in self.registered_templates:
            sensors.get_synthetic_data().unregister_node_template(template)


def get_gmo_data(dataPtr: Union[int, np.ndarray]) -> gmo_utils.GenericModelOutput:
    """Retrieves GMO buffer from pointer to GMO buffer.

    Args:
        dataPtr (int): Expected uint64 pointer to GMO buffer.

    Returns:
        gmo_utils.GenericModelOutput: GMO buffer at dataPtr. Empty struct if dataPtr is 0 or None.
    """
    if dataPtr is None:
        carb.log_warn(
            "isaacsim.sensors.rtx.get_gmo_data: received null pointer (None), returning empty GenericModelOutput buffer."
        )
        return gmo_utils.GenericModelOutput
    elif isinstance(dataPtr, np.ndarray):
        return gmo_utils.getModelOutputFromBuffer(dataPtr)
    elif dataPtr == 0:
        carb.log_warn(
            "isaacsim.sensors.rtx.get_gmo_data: received null pointer (0), returning empty GenericModelOutput buffer."
        )
        return gmo_utils.GenericModelOutput
    # Reach 28 bytes into the GMO data buffer using the pointer address
    size_buffer = (ctypes.c_char * 28).from_address(dataPtr)
    # Resolve bytes 16-23 as a uint64, corresponding to GMO size_in_bytes field
    gmo_size = int(np.frombuffer((size_buffer[16:24]), np.uint64)[0])
    # Use size_in_bytes field to get full GMO buffer
    buffer = (ctypes.c_char * gmo_size).from_address(dataPtr)
    # Retrieve GMO data from buffer as struct with well-defined fields
    gmo_data = gmo_utils.getModelOutputFromBuffer(buffer)
    return gmo_data
