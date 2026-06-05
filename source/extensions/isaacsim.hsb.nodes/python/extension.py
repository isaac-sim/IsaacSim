# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
"""HSB Nodes Extension — registers OmniGraph nodes and Replicator writers/annotators."""

import carb
import numpy as np
import omni.ext
import omni.replicator.core as rep
import omni.syntheticdata
import omni.syntheticdata._syntheticdata as sd
from isaacsim.core.nodes import register_annotator_from_node_with_telemetry, register_node_writer_with_telemetry

# Node type IDs live in isaacsim.hsb.nodes; category kept as isaacsim.hsb.bridge for UI compat
NODES_EXT = "isaacsim.hsb.nodes"
BRIDGE_CATEGORY = "isaacsim.hsb.bridge"
BRIDGE_PREFIX = "HSB"


class HsbNodesExtension(omni.ext.IExt):
    """HSB Nodes Extension class."""

    def __init__(self) -> None:
        super().__init__()
        self.registered_annotators = []

    def on_startup(self, ext_id: str) -> None:
        """Called when the extension is loaded."""
        carb.log_info("HSB Nodes Extension starting up")

        from .bindings._hsb_nodes import acquire_interface

        self._interface = acquire_interface()

        self.register_nodes()
        carb.log_info("HSB Nodes Extension started")

    def on_shutdown(self) -> None:
        """Called when the extension is unloaded."""
        carb.log_info("HSB Nodes Extension shutting down")

        self.unregister_nodes()

        from .bindings._hsb_nodes import release_interface

        if hasattr(self, "_interface") and self._interface is not None:
            release_interface(self._interface)
            self._interface = None

        carb.log_info("HSB Nodes Extension shut down")

    def register_nodes(self) -> None:
        """Register OmniGraph nodes with Replicator."""
        # Get the RGB rendervar name
        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)

        # Register the RGBToVB1940 annotator for VB1940 CSI Linux (RoCEv2) frame output
        register_annotator_from_node_with_telemetry(
            name=f"{rv}IsaacConvertRGBToVB1940CSILinux",
            input_rendervars=[
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    f"{rv}IsaacConvertRGBAToRGB",
                    attributes_mapping={
                        "outputs:dataPtr": "inputs:dataPtr",
                        "outputs:width": "inputs:width",
                        "outputs:height": "inputs:height",
                        "outputs:encoding": "inputs:encoding",
                        "outputs:cudaDeviceIndex": "inputs:cudaDeviceIndex",
                        "outputs:bufferSize": "inputs:bufferSize",
                    },
                ),
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(f"{rv}IsaacSimulationGate"),
            ],
            node_type_id=f"{NODES_EXT}.RGBToVB1940",
            init_params={"outputMode": "vb1940_csi_linux"},
            output_data_type=np.uint8,
            output_channels=1,
        )
        self.registered_annotators.append(f"{rv}IsaacConvertRGBToVB1940CSILinux")

        # Register the RGBToVB1940 annotator for VB1940 CSI COE frame output (3p4b for AGX Thor)
        register_annotator_from_node_with_telemetry(
            name=f"{rv}IsaacConvertRGBToVB1940CSICOE",
            input_rendervars=[
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    f"{rv}IsaacConvertRGBAToRGB",
                    attributes_mapping={
                        "outputs:dataPtr": "inputs:dataPtr",
                        "outputs:width": "inputs:width",
                        "outputs:height": "inputs:height",
                        "outputs:encoding": "inputs:encoding",
                        "outputs:cudaDeviceIndex": "inputs:cudaDeviceIndex",
                        "outputs:bufferSize": "inputs:bufferSize",
                    },
                ),
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(f"{rv}IsaacSimulationGate"),
            ],
            node_type_id=f"{NODES_EXT}.RGBToVB1940",
            init_params={"outputMode": "vb1940_csi_coe"},
            output_data_type=np.uint8,
            output_channels=1,
        )
        self.registered_annotators.append(f"{rv}IsaacConvertRGBToVB1940CSICOE")

        # For Simulation and System time. Removed first S char in keys to account for both upper and lower cases.
        TIME_TYPES = [("imulationTime", ""), ("ystemTime", "SystemTime")]

        for time_type in TIME_TYPES:
            # Send VB1940 CSI Linux Frame (RGB converted to VB1940 CSI Linux via RGBToVB1940 annotator)
            register_node_writer_with_telemetry(
                name=f"{rv}{BRIDGE_PREFIX}{time_type[1]}SendVB1940CSILinux",
                node_type_id=f"{NODES_EXT}.{BRIDGE_PREFIX}Send",
                annotators=[
                    f"{rv}IsaacConvertRGBToVB1940CSILinux",
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                category=BRIDGE_CATEGORY,
            )

            # Send VB1940 CSI COE Frame (3p4b for AGX Thor COE ingress)
            register_node_writer_with_telemetry(
                name=f"{rv}{BRIDGE_PREFIX}{time_type[1]}SendVB1940CSICOE",
                node_type_id=f"{NODES_EXT}.{BRIDGE_PREFIX}Send",
                annotators=[
                    f"{rv}IsaacConvertRGBToVB1940CSICOE",
                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                        f"IsaacReadS{time_type[0]}", attributes_mapping={f"outputs:s{time_type[0]}": "inputs:timeStamp"}
                    ),
                ],
                category=BRIDGE_CATEGORY,
            )

    def unregister_nodes(self) -> None:
        """Unregister Replicator writers and annotators."""
        # Unregister writers
        for writer in rep.WriterRegistry.get_writers(category=BRIDGE_CATEGORY):
            rep.writers.unregister_writer(writer)

        # Unregister annotators
        for annotator in self.registered_annotators:
            try:
                rep.AnnotatorRegistry.unregister_annotator(annotator)
            except Exception:
                pass
        self.registered_annotators = []
