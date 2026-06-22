# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""OmniGraph node for ROS 2 camera helper publishers."""

from __future__ import annotations

import traceback
from typing import Any

import carb
import omni
import omni.replicator.core as rep
import omni.syntheticdata
from isaacsim.core.nodes import BaseWriterNode
from isaacsim.ros2.core import collect_namespace
from isaacsim.ros2.nodes.impl.ros2_common import (
    USE_SRTX_SETTING,
    CompressedImageManager,
    _start_or_extend_continuous_capture,
    cleanup_srtx_state,
    ensure_render_var_on_product,
    prepare_srtx_sensor_set,
    validate_srtx_platform,
)
from pxr import Usd, UsdRender

# Per-sensor-type configuration.
#   aov:          AOV (render var)
#   writer_suffix: appended after "ROS2" / "ROS2SystemTime" to form the writer name.
#   srtx:         True if SRTX publishing is supported for this sensor type.
SENSOR_CONFIGS = {
    "rgb": {
        "aov": "LdrColorSD",
        "writer_suffix": "PublishImage",
        "srtx": True,
        "is_image": True,
        "aov_in_writer": True,
    },
    "depth": {
        "aov": "DistanceToImagePlaneSD",
        "writer_suffix": "PublishImage",
        "srtx": True,
        "is_image": False,
        "aov_in_writer": True,
    },
    "distance_to_image_plane": {
        "aov": "DistanceToImagePlaneSD",
        "writer_suffix": "PublishImage",
        "srtx": True,
        "is_image": False,
        "aov_in_writer": True,
    },
    "distance_to_camera": {
        "aov": "DistanceToCameraSD",
        "writer_suffix": "PublishImage",
        "srtx": True,
        "is_image": False,
        "aov_in_writer": True,
    },
    "depth_pcl": {
        "aov": "DistanceToImagePlaneSD",
        "writer_suffix": "PublishPointCloud",
        "srtx": True,
        "is_image": False,
        "aov_in_writer": True,
    },
    "instance_segmentation": {
        "aov": "InstanceSegmentationSD",
        "writer_suffix": "PublishInstanceSegmentation",
        "srtx": True,
        "is_image": False,
        "aov_in_writer": False,
    },
    "semantic_segmentation": {
        "aov": "SemanticSegmentationSD",
        "writer_suffix": "PublishSemanticSegmentation",
        "srtx": True,
        "is_image": False,
        "aov_in_writer": False,
    },
    "bbox_2d_tight": {
        "aov": "BoundingBox2DTightSD",
        "writer_suffix": "PublishBoundingBox2DTight",
        "srtx": False,
        "is_image": False,
        "aov_in_writer": False,
    },
    "bbox_2d_loose": {
        "aov": "BoundingBox2DLooseSD",
        "writer_suffix": "PublishBoundingBox2DLoose",
        "srtx": False,
        "is_image": False,
        "aov_in_writer": False,
    },
    "bbox_3d": {
        "aov": "BoundingBox3DSD",
        "writer_suffix": "PublishBoundingBox3D",
        "srtx": False,
        "is_image": False,
        "aov_in_writer": False,
    },
}


class OgnROS2CameraHelperInternalState(BaseWriterNode):
    """Internal state for the ROS2CameraHelper OmniGraph node."""

    def __init__(self) -> None:
        self.rv = ""
        self.resetSimulationTimeOnStop = False
        self.publishStepSize = 1
        self._h264_render_product = None
        self._srtx_callback_handle = None
        self._srtx_capsule = None
        self._srtx_sensor_set = None
        self._srtx_output_path = None
        super().__init__(initialize=False)

    def custom_reset(self) -> None:
        """Reset the internal state."""
        if self._h264_render_product is not None:
            CompressedImageManager.detach(self._h264_render_product)
            self._h264_render_product = None
        cleanup_srtx_state(self)
        super().custom_reset()

    def post_attach(self, writer: Any, render_product: Any) -> None:
        """Configure writer attributes after attaching to a render product.

        Args:
            writer: Writer attached to the render product.
            render_product: Render product path or prim.
        """
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


class OgnROS2CameraHelper:
    """OmniGraph node that sets up ROS 2 camera publishers."""

    @staticmethod
    def internal_state() -> OgnROS2CameraHelperInternalState:
        """Return the internal state object for this node.

        Returns:
            Internal state object for this node.
        """
        return OgnROS2CameraHelperInternalState()

    @staticmethod
    def _setup_srtx(
        config: Any, init_params: Any, render_product_path: Any, sensor_type: Any, state: Any, compression_type: Any
    ) -> bool:
        if not config["srtx"]:
            carb.log_error(
                f"Sensor type '{sensor_type}' is not supported with SRTX. "
                f"Disable SRTX or use a supported type (rgb, depth, distance_to_image_plane, distance_to_camera)."
            )
            return False

        from isaacsim.ros2.nodes.bindings._ros2_nodes import create_image_publisher_capsule
        from omni.replicator.srtx import SrtxCore

        stage = omni.usd.get_context().get_stage()
        usd_scene = str(stage.GetRootLayer().identifier) if stage else ""
        srtx_instance = SrtxCore.get_instance(usd_scene)
        if srtx_instance is None:
            carb.log_error(f"No SRTX instance for stage '{usd_scene}'")
            return False

        sensor_set_name = prepare_srtx_sensor_set(srtx_instance, render_product_path)
        if sensor_set_name is None:
            carb.log_error(f"Failed to prepare SRTX sensor set for {render_product_path}")
            return False
        sensor_name = render_product_path.rsplit("/", 1)[-1]
        rp_prim = stage.GetPrimAtPath(render_product_path)
        if rp_prim and rp_prim.IsValid():
            camera_rel = UsdRender.Product(rp_prim).GetCameraRel()
            if camera_rel:
                targets = camera_rel.GetTargets()
                if targets:
                    sensor_name = str(targets[0]).rsplit("/", 1)[-1]
        srtx_instance.add_sensor(sensor_set_name, sensor_name, render_product_path)

        capsule = create_image_publisher_capsule(
            topic_name=init_params["topicName"],
            frame_id=init_params["frameId"],
            node_namespace=init_params["nodeNamespace"],
            queue_size=init_params["queueSize"],
            qos_profile=init_params["qosProfile"],
        )

        success, rendervar_path = ensure_render_var_on_product(
            stage, render_product_path, config["aov"], compression_type, config["is_image"]
        )
        if not success:
            state.initialized = False
            return False

        handle = srtx_instance.register_frame_callback(sensor_set_name, rendervar_path, capsule)
        if handle == 0:
            carb.log_error(f"Failed to register SRTX frame callback for {rendervar_path}")
            state.initialized = False
            return False

        _start_or_extend_continuous_capture(srtx_instance, sensor_set_name, rendervar_path)

        state._srtx_callback_handle = handle
        state._srtx_capsule = capsule
        state._srtx_sensor_set = sensor_set_name
        state._srtx_output_path = rendervar_path

        return True

    @staticmethod
    def compute(db: Any) -> bool:
        """Configure ROS 2 camera publishing for the requested render product and sensor type.

        Args:
            db: OmniGraph database for the node.

        Returns:
            True if the node was configured or can retry later, otherwise False.
        """
        state = db.per_instance_state
        if not db.inputs.enabled:
            if state.initialized:
                # Camera helper was disabled, reset the state
                state.custom_reset()
            return True

        if state.initialized:
            # Camera helper was already initialized, nothing to do
            return True

        # Initialize!
        sensor_type = db.inputs.type
        stage = omni.usd.get_context().get_stage()
        render_product_path = db.inputs.renderProductPath
        if not render_product_path:
            carb.log_warn(f"Render product '{render_product_path}' not valid")
            return False
        if not stage.GetPrimAtPath(render_product_path).IsValid():
            carb.log_warn(f"Render product '{render_product_path}' not created yet, retrying on next call")
            return False

        state.resetSimulationTimeOnStop = db.inputs.resetSimulationTimeOnStop
        if db.inputs.frameSkipCount > 0:
            carb.log_warn(
                "The frameSkipCount input is deprecated. "
                "Control publish rate by setting omni:sensor:tickRate on the sensor prim instead, and setting frameSkipCount to 0."
            )
        state.publishStepSize = db.inputs.frameSkipCount + 1

        time_type = ""
        if db.inputs.useSystemTime:
            time_type = "SystemTime"
            if db.inputs.resetSimulationTimeOnStop:
                carb.log_warn("System timestamp is being used. Ignoring resetSimulationTimeOnStop input")

        state.rv = ""

        init_params = {
            "frameId": db.inputs.frameId,
            "nodeNamespace": collect_namespace(db.inputs.nodeNamespace, render_product_path),
            "queueSize": db.inputs.queueSize,
            "topicName": db.inputs.topicName,
            "context": db.inputs.context,
            "qosProfile": db.inputs.qosProfile,
        }

        if sensor_type == "rgb_h264":
            rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                omni.syntheticdata._syntheticdata.SensorType.Rgb.name
            )
            state.rv = rv
            CompressedImageManager.attach(render_product_path)
            state._h264_render_product = render_product_path
            writer = CompressedImageManager.get_writer(render_product_path, use_system_time=db.inputs.useSystemTime)
            writer.initialize(**init_params)
            db.per_instance_state.append_writer(writer)
            db.per_instance_state.attach_writers(render_product_path)
            state.initialized = True
            return True

        config = SENSOR_CONFIGS.get(sensor_type)
        if config is None:
            carb.log_error(f"Sensor type '{sensor_type}' is not supported")
            return False

        use_srtx = carb.settings.get_settings().get_as_bool(USE_SRTX_SETTING)
        if use_srtx:
            if not validate_srtx_platform():
                return False
            if not OgnROS2CameraHelper._setup_srtx(
                config, init_params, render_product_path, sensor_type, state, db.inputs.srtxCompressionType
            ):
                db.log_error("Failed to setup SRTX for camera helper")
                return False
            state.initialized = True
            return True

        try:
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                aov = config["aov"]
                if aov:
                    db.per_instance_state.rv = aov
                writer_prefix = aov if config.get("aov_in_writer") else ""
                writer = rep.writers.get(f"{writer_prefix}ROS2{time_type}{config['writer_suffix']}")

                writer.initialize(**init_params)
                db.per_instance_state.append_writer(writer)

                if aov:
                    db.per_instance_state.rv = aov
                    if db.inputs.enableSemanticLabels:
                        semantic_writer = rep.writers.get(f"{aov}ROS2{time_type}PublishSemanticLabels")
                        semantic_writer.initialize(
                            nodeNamespace=init_params["nodeNamespace"],
                            queueSize=init_params["queueSize"],
                            topicName=db.inputs.semanticLabelsTopicName,
                            context=init_params["context"],
                            qosProfile=init_params["qosProfile"],
                        )
                        db.per_instance_state.append_writer(semantic_writer)

                db.per_instance_state.attach_writers(render_product_path)
        except Exception:
            carb.log_error(f"Failed to initialize camera helper writer: {traceback.format_exc()}")
            return False

        state.initialized = True
        return True

    @staticmethod
    def release_instance(node: Any, graph_instance_id: Any) -> None:
        """Release resources for a graph instance.

        Args:
            node: OmniGraph node being released.
            graph_instance_id: Graph instance identifier.
        """
        try:
            state = OgnROS2CameraHelperInternalState.per_instance_internal_state(node)
        except Exception:
            state = None

        if state is not None:
            cleanup_srtx_state(state)
            state.reset()
