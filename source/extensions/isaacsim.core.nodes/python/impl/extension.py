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

import carb
import numpy as np
import omni.ext
import omni.kit.commands
import omni.syntheticdata
import omni.syntheticdata._syntheticdata as sd
from isaacsim.core.nodes.scripts.utils import register_annotator_from_node_with_telemetry
from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.core.utils.stage import get_current_stage
from omni.replicator.core import AnnotatorRegistry
from omni.syntheticdata import sensors
from pxr import Sdf, Usd

from ..bindings._isaacsim_core_nodes import acquire_interface, release_interface

# Any class derived from `omni.ext.IExt` in a top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when the extension is enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() will be called.


class Extension(omni.ext.IExt):
    def on_startup(self):
        self.__interface = acquire_interface()
        self.registered_templates = []
        self.registered_annotators = []
        try:
            self.register_nodes()
        except Exception as e:
            carb.log_error(f"Could not register node templates {e}")

        self._stage_event_sub = (
            omni.usd.get_context()
            .get_stage_event_stream()
            .create_subscription_to_pop_by_type(int(omni.usd.StageEventType.OPENED), self._on_stage_open_event)
        )
        pass

    def on_shutdown(self):
        release_interface(self.__interface)
        self.__interface = None
        try:
            self.unregister_nodes()
        except Exception as e:
            carb.log_warn(f"Could not unregister node templates {e}")
        self._stage_event_sub = None
        pass

    def _on_stage_open_event(self, event):
        # Workaround for issue where an opened stage can contain a dirty /Render path
        stage = get_current_stage()
        path = "/Render"
        # delete any deltas on the root layer
        try:
            from omni.kit.widget.layers.layer_commands import RemovePrimSpecCommand

            RemovePrimSpecCommand(layer_identifier=stage.GetRootLayer().realPath, prim_spec_path=[Sdf.Path(path)]).do()
        except:
            pass
        # Make sure /Render is hidden
        if get_prim_at_path(path):
            get_prim_at_path(path).SetMetadata("hide_in_stage_window", True)

    def register_nodes(self):
        ## Annotators
        ### Add template to no_op
        annotator_name = "IsaacNoop"
        register_annotator_from_node_with_telemetry(
            name=annotator_name,
            input_rendervars=["PostProcessDispatch"],
            node_type_id="omni.syntheticdata.SdNoOp",
        )
        self.registered_annotators.append(annotator_name)

        annotator_name = "IsaacReadCameraInfo"
        register_annotator_from_node_with_telemetry(
            name=annotator_name,
            input_rendervars=["PostProcessDispatch"],
            node_type_id="isaacsim.core.nodes.IsaacReadCameraInfo",
        )
        self.registered_annotators.append(annotator_name)

        ##### Time

        annotator_name = "IsaacReadSimulationTime"
        register_annotator_from_node_with_telemetry(
            name=annotator_name,
            input_rendervars=[
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    f"DispatchSync",
                    attributes_mapping={
                        "outputs:referenceTimeNumerator": "inputs:referenceTimeNumerator",
                        "outputs:referenceTimeDenominator": "inputs:referenceTimeDenominator",
                    },
                ),
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    f"PostProcessDispatch",
                ),
            ],
            node_type_id="isaacsim.core.nodes.IsaacReadSimulationTimeAnnotator",
        )
        self.registered_annotators.append(annotator_name)

        annotator_name = "IsaacReadSystemTime"
        register_annotator_from_node_with_telemetry(
            name=annotator_name,
            input_rendervars=[
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    f"ReferenceTime", attributes_mapping={f"outputs:exec": "inputs:execIn"}
                )
            ],
            node_type_id="isaacsim.core.nodes.IsaacReadSystemTimeAnnotator",
        )
        self.registered_annotators.append(annotator_name)

        annotator_name = "IsaacReadWorldPose"
        register_annotator_from_node_with_telemetry(
            name=annotator_name,
            input_rendervars=["PostProcessDispatch"],
            node_type_id="isaacsim.core.nodes.IsaacReadWorldPose",
        )
        self.registered_annotators.append(annotator_name)

        ##### Simulation Gates
        for rv in sensors.get_synthetic_data()._ogn_rendervars:
            if sensors.get_synthetic_data().is_node_template_registered(rv + "Ptr"):
                template_name = rv + "IsaacSimulationGate"
                if template_name not in sensors.get_synthetic_data()._ogn_templates_registry:
                    template = sensors.get_synthetic_data().register_node_template(
                        omni.syntheticdata.SyntheticData.NodeTemplate(
                            omni.syntheticdata.SyntheticDataStage.ON_DEMAND,
                            "isaacsim.core.nodes.IsaacSimulationGate",
                            [
                                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                                    rv + "Ptr", attributes_mapping={"outputs:exec": "inputs:execIn"}
                                )
                            ],
                        ),
                        template_name=template_name,
                    )
                    self.registered_templates.append(template)
        # These gates connect to annotators
        # instance_segmentation = anotator name?
        # InstanceSegmentation = sensor type
        # InstanceSegmentationSD = rendervar
        annotator_names = [
            "instance_segmentation_fast",
            "semantic_segmentation",
            "bounding_box_2d_tight_fast",
            "bounding_box_2d_loose_fast",
            "bounding_box_3d_fast",
            "PostProcessDispatch",  # this is so we have a simulation gate on the base dispatch node if needed
        ]
        for name in annotator_names:
            template_name = name + "IsaacSimulationGate"
            if template_name not in sensors.get_synthetic_data()._ogn_templates_registry:
                template = sensors.get_synthetic_data().register_node_template(
                    omni.syntheticdata.SyntheticData.NodeTemplate(
                        omni.syntheticdata.SyntheticDataStage.ON_DEMAND,
                        "isaacsim.core.nodes.IsaacSimulationGate",
                        [
                            omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                                name, attributes_mapping={"outputs:exec": "inputs:execIn"}
                            )
                        ],
                    ),
                    template_name=template_name,
                )
                self.registered_templates.append(template)

        # ##### RGBA to RGB
        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
        annotator_name = rv + "IsaacConvertRGBAToRGB"
        register_annotator_from_node_with_telemetry(
            name=annotator_name,
            input_rendervars=[
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(rv + "Ptr"),
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(rv + "IsaacSimulationGate"),
            ],
            node_type_id="isaacsim.core.nodes.IsaacConvertRGBAToRGB",
            init_params={"encoding": "rgba8"},
            output_data_type=np.uint8,
            output_channels=3,
        )
        self.registered_annotators.append(annotator_name)

        # Depth ptr passthrough
        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.DistanceToImagePlane.name)
        annotator_name = rv + "IsaacPassthroughImagePtr"
        register_annotator_from_node_with_telemetry(
            name=annotator_name,
            input_rendervars=[
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(rv + "Ptr"),
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(rv + "IsaacSimulationGate"),
            ],
            node_type_id="isaacsim.core.nodes.IsaacPassthroughImagePtr",
        )
        self.registered_annotators.append(annotator_name)

        # # convert depth to pcl
        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.DistanceToImagePlane.name)
        annotator_name = rv + "IsaacConvertDepthToPointCloud"
        register_annotator_from_node_with_telemetry(
            name=annotator_name,
            input_rendervars=[
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    rv + "IsaacSimulationGate", attributes_mapping={"outputs:execOut": "inputs:execIn"}
                ),
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    rv + "Ptr",
                ),
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "IsaacReadCameraInfo",
                    attributes_mapping={
                        "outputs:focalLength": "inputs:focalLength",
                        "outputs:horizontalAperture": "inputs:horizontalAperture",
                        "outputs:verticalAperture": "inputs:verticalAperture",
                    },
                ),
            ],
            node_type_id="isaacsim.core.nodes.IsaacConvertDepthToPointCloud",
            output_data_type=np.float32,
            output_channels=3,
        )
        self.registered_annotators.append(annotator_name)

    def unregister_nodes(self):
        for template in self.registered_templates:
            sensors.get_synthetic_data().unregister_node_template(template)
        for annotator in self.registered_annotators:
            AnnotatorRegistry.unregister_annotator(annotator)
