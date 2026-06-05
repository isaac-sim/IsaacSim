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

"""OmniGraph node for ROS 2 RTX radar point cloud publishing."""

from __future__ import annotations

import traceback

import carb
import omni
import omni.replicator.core as rep
from isaacsim.core.nodes import BaseWriterNode
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.ros2.core import collect_namespace
from pxr import Usd


class OgnROS2RtxRadarHelperInternalState(BaseWriterNode):
    """Internal state for the ROS2RtxRadarHelper OmniGraph node."""

    def __init__(self) -> None:
        """Initialize the ROS2 RTX radar helper internal state."""
        self.viewport = None
        self.viewport_name = ""
        self.resetSimulationTimeOnStop = False
        self.publishStepSize = 1
        super().__init__(initialize=False)


class OgnROS2RtxRadarHelper:
    """OmniGraph node that automates RTX Radar point cloud publishing to ROS 2.

    Publishes radar detections as sensor_msgs/PointCloud2 messages with optional
    per-point radial velocity metadata. Requires an OmniRadar prim with the
    OmniSensorGenericRadarWpmDmatAPI schema applied.
    """

    @staticmethod
    def internal_state() -> OgnROS2RtxRadarHelperInternalState:
        """Return the internal state object for this node."""
        return OgnROS2RtxRadarHelperInternalState()

    @staticmethod
    def compute(db) -> bool:
        """Compute the node."""
        state = db.per_instance_state

        if state.initialized:
            return True

        if not db.inputs.enabled:
            return True

        stage = omni.usd.get_context().get_stage()
        render_product_path = db.inputs.renderProductPath
        if not render_product_path:
            carb.log_warn(f"Render product '{render_product_path}' not valid")
            return False
        if stage.GetPrimAtPath(render_product_path) is None:
            carb.log_warn(f"Render product '{render_product_path}' not created yet, retrying on next call")
            return False

        prim = ViewportManager.get_camera(render_product_path).GetPrim()
        if prim.GetTypeName() != "OmniRadar":
            carb.log_warn(
                f"Render product '{render_product_path}' not attached to RTX Radar (OmniRadar prim required)."
            )
            return False

        state.render_product_path = render_product_path
        state.resetSimulationTimeOnStop = db.inputs.resetSimulationTimeOnStop
        state.publishStepSize = 1

        time_type = ""
        if db.inputs.useSystemTime:
            time_type = "SystemTime"
            if db.inputs.resetSimulationTimeOnStop:
                carb.log_warn("System timestamp is being used. Ignoring resetSimulationTimeOnStop input")

        init_params = dict(
            frameId=db.inputs.frameId,
            nodeNamespace=collect_namespace(db.inputs.nodeNamespace, render_product_path),
            queueSize=db.inputs.queueSize,
            topicName=db.inputs.topicName,
            context=db.inputs.context,
            qosProfile=db.inputs.qosProfile,
        )

        # Collect enabled metadata field names.
        metadata = []
        if db.inputs.outputRadialVelocityMS:
            metadata.append("radialVelocityMS")
        if db.inputs.outputIntensity:
            metadata.append("intensity")
        if db.inputs.outputTimestamp:
            metadata.append("timestamp")

        try:
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                for item in metadata:
                    init_params[f"output{item[0].upper()}{item[1:]}"] = True
                writer = rep.writers.get(f"RtxRadarROS2{time_type}PublishPointCloud")
                writer.initialize(**init_params)
                state.append_writer(writer)

                if db.inputs.showDebugView:
                    doTransform = prim.GetAttribute("omni:sensor:WpmDmat:outputFrameOfReference").Get() != "WORLD"
                    debug_writer = rep.writers.get("RtxSensorDebugDrawPointCloud")
                    # size/color match the legacy RtxRadarDebugDrawPointCloud writer defaults
                    # so radar points stay visible against the background.
                    debug_writer.initialize(doTransform=doTransform, size=0.2, color=[1.0, 0.2, 0.3, 1.0])
                    state.append_writer(debug_writer)

                state.attach_writers(render_product_path)
        except Exception:
            carb.log_error(f"Failed to initialize radar helper writer: {traceback.format_exc()}")
            return False

        state.initialized = True
        return True

    @staticmethod
    def release_instance(node, graph_instance_id) -> None:
        """Release resources for a graph instance."""
        try:
            state = OgnROS2RtxRadarHelperInternalState.per_instance_internal_state(node)
        except Exception:
            state = None

        if state is not None:
            state.reset()
