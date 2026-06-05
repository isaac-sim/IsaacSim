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

"""Isaac Sim core nodes extension providing computational nodes for data processing and annotation in robotics simulations."""

import carb
import numpy as np
import omni.ext
import omni.kit.commands
import omni.syntheticdata
import omni.syntheticdata._syntheticdata as sd
from isaacsim.core.nodes.scripts.utils import register_annotator_from_node_with_telemetry
from omni.replicator.core import AnnotatorRegistry
from omni.syntheticdata import sensors

from ..bindings._isaacsim_core_nodes import acquire_interface, release_interface

# Any class derived from `omni.ext.IExt` in a top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when the extension is enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() will be called.


class Extension(omni.ext.IExt):
    """The isaacsim.core.nodes extension provides computational nodes for Isaac Sim data processing and annotation.

    This extension registers various annotators and node templates for synthetic data processing, including camera
    information reading, time utilities, pose data extraction, and image format conversion. The annotators enable
    data extraction from rendered scenes and provide utilities for robotics simulations, such as converting RGBA
    images to RGB format, generating point clouds from depth data, and managing simulation gates for controlled
    data flow.

    The extension automatically registers the following key components:

    - Time-based annotators for simulation and system time tracking
    - Camera information readers for extracting camera parameters
    - World pose readers for spatial data
    - Image conversion utilities (RGBA to RGB)
    - Depth-to-point-cloud converters
    - Simulation gates for controlling data flow
    - Passthrough nodes for image pointer data
    """

    def on_startup(self) -> None:
        """Called when the extension is enabled to initialize the interface and register nodes."""
        self.__interface = acquire_interface()
        self.registered_templates = []
        self.registered_annotators = []
        try:
            self.register_nodes()
        except Exception as e:
            carb.log_error(f"Could not register node templates {e}")

    def on_shutdown(self) -> None:
        """Called when the extension is disabled to release the interface and unregister nodes."""
        release_interface(self.__interface)
        self.__interface = None
        try:
            self.unregister_nodes()
        except Exception as e:
            carb.log_warn(f"Could not unregister node templates {e}")

    def register_nodes(self) -> None:
        """Register Isaac Sim annotator node templates and simulation gates for synthetic data generation.

        Registers annotators for camera info, time reading, world pose, image conversion, and point cloud
        generation. Also creates simulation gates for various render variables and annotator types.
        """
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
                    f"rpFabricTime",
                    attributes_mapping={
                        "outputs:exec": "inputs:execIn",
                        "outputs:fabricFrameTimeNumerator": "inputs:referenceTimeNumerator",
                        "outputs:fabricFrameTimeDenominator": "inputs:referenceTimeDenominator",
                    },
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
                    f"rpFabricTime",
                    attributes_mapping={
                        "outputs:exec": "inputs:execIn",
                        "outputs:fabricFrameTimeNumerator": "inputs:referenceTimeNumerator",
                        "outputs:fabricFrameTimeDenominator": "inputs:referenceTimeDenominator",
                    },
                ),
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
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(rv + "IsaacSimulationGate"),
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

    def unregister_nodes(self) -> None:
        """Unregisters all previously registered node templates and annotators from the synthetic data system."""
        for template in self.registered_templates:
            sensors.get_synthetic_data().unregister_node_template(template)
        for annotator in self.registered_annotators:
            AnnotatorRegistry.unregister_annotator(annotator)
