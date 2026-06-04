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

"""OmniGraph node for publishing camera images over HSB."""

from __future__ import annotations

import traceback

import carb
import omni
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata
import omni.syntheticdata._syntheticdata as sd
from isaacsim.core.nodes import BaseWriterNode
from pxr import Usd


class OgnHSBCameraHelperInternalState(BaseWriterNode):
    """Internal state for the HSB camera helper OmniGraph node."""

    def __init__(self) -> None:
        self.rv = ""
        self.resetSimulationTimeOnStop = True
        self.publishStepSize = 1

        super().__init__(initialize=False)

    def post_attach(self, writer: rep.Writer, render_product: str | list[str]) -> None:
        """Configure node attributes after attaching a writer to a render product."""
        try:
            if self.rv != "":
                omni.syntheticdata.SyntheticData.Get().set_node_attributes(
                    self.rv + "IsaacSimulationGate", {"inputs:step": self.publishStepSize}, render_product
                )

            omni.syntheticdata.SyntheticData.Get().set_node_attributes(
                "IsaacReadSimulationTime", {"inputs:resetOnStop": self.resetSimulationTimeOnStop}, render_product
            )

        except Exception:
            pass


class OgnHSBCameraHelper:
    """OmniGraph node that publishes camera images over HSB."""

    @staticmethod
    def internal_state() -> OgnHSBCameraHelperInternalState:
        """Return a new internal state instance."""
        return OgnHSBCameraHelperInternalState()

    @staticmethod
    def compute(db: og.Database) -> bool:
        """Compute the node output by initializing and attaching HSB writers."""
        if db.per_instance_state.initialized is False:
            db.per_instance_state.initialized = True
            stage = omni.usd.get_context().get_stage()
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                render_product_path = db.inputs.renderProductPath
                if not render_product_path:
                    carb.log_warn("Render product not valid")
                    db.per_instance_state.initialized = False
                    return False
                if not stage.GetPrimAtPath(render_product_path).IsValid():
                    carb.log_warn("Render product not created yet, retrying on next call")
                    db.per_instance_state.initialized = False
                    return False
                db.per_instance_state.resetSimulationTimeOnStop = db.inputs.resetSimulationTimeOnStop
                db.per_instance_state.publishStepSize = 1

                writer = None

                time_type = ""
                if db.inputs.useSystemTime:
                    time_type = "SystemTime"
                    if db.inputs.resetSimulationTimeOnStop:
                        carb.log_warn("System timestamp is being used. Ignoring resetSimulationTimeOnStop input")

                db.per_instance_state.rv = ""
                sensor_type = db.inputs.type

                try:
                    db.per_instance_state.rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                        sd.SensorType.Rgb.name
                    )

                    writer_name = None
                    if sensor_type == "vb1940_csi_linux":
                        writer_name = db.per_instance_state.rv + f"HSB{time_type}SendVB1940CSILinux"
                    elif sensor_type == "vb1940_csi_coe":
                        writer_name = db.per_instance_state.rv + f"HSB{time_type}SendVB1940CSICOE"
                    else:
                        carb.log_error(f"Sensor type '{sensor_type}' is not supported")
                        db.per_instance_state.initialized = False
                        return False

                    writer = rep.writers.get(writer_name)

                    if writer is None:
                        carb.log_error(f"Writer '{writer_name}' not found")
                        db.per_instance_state.initialized = False
                        return False

                    init_kwargs = {
                        "ipAddress": db.inputs.ipAddress,
                        "dataPlaneType": db.inputs.dataPlaneType,
                        "dataPlaneId": db.inputs.dataPlaneId,
                        "sensorId": db.inputs.sensorId,
                    }
                    writer.initialize(**init_kwargs)

                    db.per_instance_state.append_writer(writer)
                    db.per_instance_state.attach_writers(render_product_path)
                except Exception as e:
                    carb.log_error(f"HSBCameraHelper: Failed to setup writer: {e}")
                    print(traceback.format_exc())
                    return False

        db.outputs.execOut = og.ExecutionAttributeState.ENABLED
        return True

    @staticmethod
    def release_instance(node: object, graph_instance_id: int) -> None:
        """Release resources when a node instance is destroyed."""
        try:
            state = OgnHSBCameraHelperInternalState.per_instance_internal_state(node)
        except Exception:
            state = None

        if state is not None:
            state.reset()
