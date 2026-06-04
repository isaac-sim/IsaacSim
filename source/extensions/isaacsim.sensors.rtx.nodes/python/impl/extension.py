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

"""RTX Sensor nodes extension — registers point cloud annotator and debug draw writer."""

import carb
import numpy as np
import omni.ext
from isaacsim.core.nodes.scripts.utils import (
    register_annotator_from_node_with_telemetry,
    register_node_writer_with_telemetry,
)
from isaacsim.sensors.experimental.rtx import (
    register_writer_spec,
    unregister_writer_spec,
)
from omni.replicator.core import AnnotatorRegistry, WriterRegistry

from ..bindings._isaacsim_sensors_rtx_nodes import acquire_interface, release_interface

NODE_TYPE_ID = "isaacsim.sensors.rtx.nodes.OgnIsaacExtractRTXSensorPointCloud"
ANNOTATOR_NAME = "IsaacExtractRTXSensorPointCloud"
WRITER_NAME = "RtxSensorDebugDrawPointCloud"
SENSOR_RUNTIME_WRITER = "draw-point-cloud"
CATEGORY = "isaacsim.sensors.experimental.rtx"


class Extension(omni.ext.IExt):
    """Registers the RTX sensor point cloud annotator and debug draw writer.

    The annotator extracts a Cartesian point cloud from the GenericModelOutput
    buffer produced by OmniLidar or OmniRadar prims.  The writer feeds that
    point cloud to the ``DebugDrawPointCloud`` node for viewport visualisation.
    """

    def on_startup(self, ext_id: str = "") -> None:
        """Register annotator and writer on extension load."""
        self._interface = acquire_interface()
        self._registered_annotators: list[str] = []
        try:
            self._register_nodes()
        except Exception as e:
            carb.log_error(f"isaacsim.sensors.rtx.nodes: failed to register nodes: {e}")

    def on_shutdown(self) -> None:
        """Unregister annotator and writer on extension unload."""
        try:
            self._unregister_nodes()
        except Exception as e:
            carb.log_warn(f"isaacsim.sensors.rtx.nodes: failed to unregister nodes: {e}")
        release_interface(self._interface)
        self._interface = None

    # ------------------------------------------------------------------

    def _register_nodes(self) -> None:
        # Register the point cloud extraction annotator
        register_annotator_from_node_with_telemetry(
            name=ANNOTATOR_NAME,
            input_rendervars=["GenericModelOutputPtr"],
            node_type_id=NODE_TYPE_ID,
            output_data_type=np.float32,
            output_channels=3,
            hidden=True,
        )
        self._registered_annotators.append(ANNOTATOR_NAME)

        # Register a single debug draw writer for all RTX sensor modalities
        # (lidar + radar).  Users customise appearance via writer.initialize().
        register_node_writer_with_telemetry(
            name=WRITER_NAME,
            node_type_id="isaacsim.util.debug_draw.DebugDrawPointCloud",
            annotators=[ANNOTATOR_NAME],
            doTransform=True,
            category=CATEGORY,
        )

        # Make "debug-draw" available as a writer name on sensor runtime classes
        register_writer_spec(SENSOR_RUNTIME_WRITER, {"name": WRITER_NAME, "defaults": {}})

    def _unregister_nodes(self) -> None:
        unregister_writer_spec(SENSOR_RUNTIME_WRITER)
        for writer in WriterRegistry.get_writers(category=CATEGORY):
            WriterRegistry.unregister_writer(writer)
        for annotator in self._registered_annotators:
            AnnotatorRegistry.unregister_annotator(annotator)
