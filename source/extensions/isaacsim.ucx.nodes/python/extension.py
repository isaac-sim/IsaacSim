# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""UCX Nodes Extension - manages lifecycle of native plugin."""

import os

import carb
import omni
import omni.ext
import omni.replicator.core as rep
import omni.syntheticdata
import omni.syntheticdata._syntheticdata as sd
from isaacsim.core.nodes.scripts.utils import register_node_writer_with_telemetry

# Bridge constants
BRIDGE_NAME = "isaacsim.ucx.nodes"
BRIDGE_PREFIX = "UCX"


class UCXBridgeExtension(omni.ext.IExt):
    """UCX Bridge Extension class."""

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension when it is loaded.

        Args:
            ext_id: The extension identifier.
        """
        carb.log_info("UCX Bridge Extension starting up")

        # Must be set before UCX context creation.
        os.environ["UCX_CM_USE_ALL_DEVICES"] = "n"

        # Acquire the native plugin interface - this triggers plugin load
        from .bindings._ucx_nodes import acquire_interface

        self._interface = acquire_interface()

        self.register_nodes()

        carb.log_info("UCX Bridge Extension started")

    def on_shutdown(self) -> None:
        """Clean up resources when the extension is unloaded."""
        carb.log_info("UCX Bridge Extension shutting down")

        # Release the native plugin interface
        from .bindings._ucx_nodes import release_interface

        if hasattr(self, "_interface") and self._interface is not None:
            release_interface(self._interface)
            self._interface = None

        self.unregister_nodes()

        carb.log_info("UCX Bridge Extension shut down")

    def register_nodes(self) -> None:
        """Register the nodes for the UCX Bridge Extension."""
        # For Simulation and System time. Removed first S char in keys to account for both upper and lower cases.
        TIME_TYPES = [("imulationTime", ""), ("ystemTime", "SystemTime")]

        for time_type in TIME_TYPES:
            # Publish Image
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

    def unregister_nodes(self) -> None:
        """Unregister the nodes for the UCX Bridge Extension."""
        for writer in rep.WriterRegistry.get_writers(category=BRIDGE_NAME):
            rep.writers.unregister_writer(writer)
