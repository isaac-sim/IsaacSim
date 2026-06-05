# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""OmniGraph node for ROS 2 camera info publishers."""

from __future__ import annotations

import contextlib

import carb
import cv2 as cv
import numpy as np
import omni
import omni.replicator.core as rep
import omni.syntheticdata
from isaacsim.core.nodes import BaseWriterNode
from isaacsim.ros2.core import collect_namespace, compute_relative_pose, read_camera_info
from isaacsim.ros2.nodes.impl.ros2_common import (
    USE_SRTX_SETTING,
    prepare_srtx_sensor_set,
    validate_srtx_platform,
)
from pxr import Usd

SRTX_CAMERA_INFO_LEGACY_TRIGGER_NAME = "CameraInfoTrigger"
SRTX_CAMERA_INFO_PREFERRED_SOURCES = ("LdrColorSD", "LdrColor", "DistanceToImagePlaneSD", "DistanceToCameraSD")


class OgnROS2CameraInfoHelperInternalState(BaseWriterNode):
    """Internal state for the ROS2CameraInfoHelper OmniGraph node."""

    def __init__(self) -> None:
        """Initialize the ROS2 camera info helper internal state."""
        self.viewport = None
        self.viewport_name = ""
        self.rv = ""
        self.rvRight = ""
        self.resetSimulationTimeOnStop = False
        self.publishStepSize = 1
        self._srtx_callback_handles = []
        self._srtx_callback_sensor_sets = []
        self._srtx_capsules = []
        self._srtx_sensor_set = None

        super().__init__(initialize=False)

    def custom_reset(self) -> None:
        """Reset the internal state."""
        cleanup_srtx_camera_info_state(self)
        super().custom_reset()

    def post_attach(self, writer, render_product) -> None:
        """Configure writer attributes after attaching to a render product."""
        try:
            if self.rv != "":
                omni.syntheticdata.SyntheticData.Get().set_node_attributes(
                    self.rv + "IsaacSimulationGate", {"inputs:step": self.publishStepSize}, render_product
                )
            if self.rvRight != "":
                omni.syntheticdata.SyntheticData.Get().set_node_attributes(
                    self.rvRight + "IsaacSimulationGate", {"inputs:step": self.publishStepSize}, render_product
                )

            omni.syntheticdata.SyntheticData.Get().set_node_attributes(
                "IsaacReadSimulationTime", {"inputs:resetOnStop": self.resetSimulationTimeOnStop}, render_product
            )
        except Exception:
            pass


def cleanup_srtx_camera_info_state(state: OgnROS2CameraInfoHelperInternalState) -> None:
    """Unregister SRTX camera info callbacks owned by a CameraInfo helper state."""
    handles = getattr(state, "_srtx_callback_handles", [])
    sensor_sets = getattr(state, "_srtx_callback_sensor_sets", [])
    fallback_sensor_set = getattr(state, "_srtx_sensor_set", None)
    if handles:
        try:
            from omni.replicator.srtx import SrtxCore

            stage = omni.usd.get_context().get_stage()
            if stage:
                usd_scene = str(stage.GetRootLayer().identifier)
                srtx_instance = SrtxCore.get_instance(usd_scene)
                if srtx_instance is not None:
                    for index, handle in enumerate(handles):
                        sensor_set = sensor_sets[index] if index < len(sensor_sets) else fallback_sensor_set
                        if sensor_set is None:
                            continue
                        srtx_instance.unregister_frame_callback(sensor_set, handle)
        except Exception as e:
            carb.log_warn(f"Error during SRTX camera info cleanup: {e}")

    state._srtx_callback_handles = []
    state._srtx_callback_sensor_sets = []
    state._srtx_capsules = []
    state._srtx_sensor_set = None


def get_srtx_camera_info_callback_output(stage, render_product_path: str) -> str | None:
    """Return an existing RenderVar path to use as the CameraInfo timing source."""
    rp_prim = stage.GetPrimAtPath(render_product_path)
    if not rp_prim or not rp_prim.IsValid():
        return None

    ordered_vars_rel = rp_prim.GetRelationship("orderedVars")
    ordered_var_paths = ordered_vars_rel.GetTargets() if ordered_vars_rel else []
    candidates = []
    for ordered_var_path in ordered_var_paths:
        render_var_path = str(ordered_var_path)
        if render_var_path.rsplit("/", 1)[-1] == SRTX_CAMERA_INFO_LEGACY_TRIGGER_NAME:
            continue
        if not render_var_path.startswith(render_product_path + "/"):
            continue

        render_var_prim = stage.GetPrimAtPath(render_var_path)
        if not render_var_prim or not render_var_prim.IsValid():
            continue

        source_name_attr = render_var_prim.GetAttribute("sourceName")
        if not source_name_attr:
            continue

        source_name = source_name_attr.Get()
        if source_name is None:
            continue

        candidates.append((render_var_path, str(source_name)))

    for preferred_source in SRTX_CAMERA_INFO_PREFERRED_SOURCES:
        for render_var_path, source_name in candidates:
            if source_name == preferred_source:
                return render_var_path

    if candidates:
        return candidates[0][0]

    carb.log_warn(
        f"SRTX CameraInfo helper found no existing RenderVar outputs on '{render_product_path}'. "
        "CameraInfo will retry until a camera image/depth helper has registered the sensor output."
    )
    return None


class OgnROS2CameraInfoHelper:
    """OmniGraph node that sets up ROS 2 camera info publishers."""

    @staticmethod
    def internal_state() -> OgnROS2CameraInfoHelperInternalState:
        """Return the internal state object for this node."""
        return OgnROS2CameraInfoHelperInternalState()

    @staticmethod
    def add_camera_info_writer(
        db, frameId, topicName, camera_info, render_product_path: str, time_type: str = ""
    ) -> None:
        """Add a camera info writer for the given render product."""
        writer = rep.writers.get(f"ROS2{time_type}PublishCameraInfo")
        writer.initialize(
            frameId=frameId,
            nodeNamespace=collect_namespace(db.inputs.nodeNamespace, render_product_path),
            queueSize=db.inputs.queueSize,
            topicName=topicName,
            context=db.inputs.context,
            qosProfile=db.inputs.qosProfile,
            width=camera_info.width,
            height=camera_info.height,
            projectionType=camera_info.distortion_model,
            k=camera_info.k,
            r=camera_info.r,
            p=camera_info.p,
            physicalDistortionModel=camera_info.distortion_model,
            physicalDistortionCoefficients=camera_info.d,
        )
        db.per_instance_state.attach_writer(writer, render_product_path)

    @staticmethod
    def add_srtx_camera_info_publisher(
        state,
        stage,
        srtx_instance,
        sensor_set_name,
        frameId,
        topicName,
        nodeNamespace,
        queueSize,
        qosProfile,
        camera_info,
        render_product_path: str,
    ) -> bool:
        """Add an SRTX callback-backed camera info publisher for the given render product."""
        from isaacsim.ros2.nodes.bindings._ros2_nodes import create_camera_info_publisher_capsule

        render_var_path = get_srtx_camera_info_callback_output(stage, render_product_path)
        if render_var_path is None:
            carb.log_error("No render_var_path from get_srtx_camera_info_callback_output was found")
            return False

        capsule = create_camera_info_publisher_capsule(
            topic_name=topicName,
            frame_id=frameId,
            node_namespace=nodeNamespace,
            queue_size=queueSize,
            qos_profile=qosProfile,
            width=camera_info.width,
            height=camera_info.height,
            distortion_model=camera_info.distortion_model,
            k=camera_info.k,
            r=camera_info.r,
            p=camera_info.p,
            d=camera_info.d,
        )

        handle = srtx_instance.register_frame_callback(sensor_set_name, render_var_path, capsule)
        if handle == 0:
            carb.log_error(f"Failed to register SRTX camera info callback for {render_var_path}")
            return False

        state._srtx_callback_handles.append(handle)
        state._srtx_callback_sensor_sets.append(sensor_set_name)
        state._srtx_capsules.append(capsule)
        state._srtx_sensor_set = sensor_set_name
        return True

    @staticmethod
    def compute(db) -> bool:
        """Compute the node outputs."""
        state = db.per_instance_state
        if not db.inputs.enabled:
            if state.initialized:
                # Camera helper was disabled, reset the state
                state.custom_reset()
            return True

        if state.initialized:
            # Camera Info helper was already initialized, nothing to do
            return True

        render_product_path = db.inputs.renderProductPath
        render_product_path_right = db.inputs.renderProductPathRight

        stage = omni.usd.get_context().get_stage()
        use_srtx = carb.settings.get_settings().get_as_bool(USE_SRTX_SETTING)
        if use_srtx and not validate_srtx_platform():
            return False
        ctx = contextlib.nullcontext() if use_srtx else Usd.EditContext(stage, stage.GetSessionLayer())
        with ctx:
            is_stereo = False
            if not render_product_path:
                carb.log_warn(f"Render product '{render_product_path}' not valid")
                return False
            if stage.GetPrimAtPath(render_product_path) is None:
                carb.log_warn(f"Render product '{render_product_path}' not created yet, retrying on next call")
                return False
            if render_product_path_right:
                is_stereo = True
                if stage.GetPrimAtPath(render_product_path_right) is None:
                    carb.log_warn(
                        f"Render product '{render_product_path_right}' not created yet, retrying on next call"
                    )
                    return False

            db.per_instance_state.resetSimulationTimeOnStop = db.inputs.resetSimulationTimeOnStop
            if db.inputs.frameSkipCount > 0:
                carb.log_warn(
                    "The frameSkipCount input is deprecated. "
                    "Control publish rate by setting omni:sensor:tickRate on the sensor prim instead, and setting frameSkipCount to 0."
                )
            db.per_instance_state.publishStepSize = db.inputs.frameSkipCount + 1

            time_type_input = ""
            if db.inputs.useSystemTime:
                time_type_input = "SystemTime"
                if db.inputs.resetSimulationTimeOnStop:
                    carb.log_warn("System timestamp is being used. Ignoring resetSimulationTimeOnStop input")

            camera_info_left, camera_left = read_camera_info(render_product_path=render_product_path)

            if is_stereo:
                camera_info_right, camera_right = read_camera_info(render_product_path=render_product_path_right)

                width_left = camera_info_left.width
                height_left = camera_info_left.height
                width_right = camera_info_right.width
                height_right = camera_info_right.height
                if width_left != width_right or height_left != height_right:
                    carb.log_warn(
                        f"Mismatched stereo camera resolutions: left = [{width_left}, {height_left}], right = [{width_right}, {height_right}]"
                    )
                    return False

                distortion_model_left = camera_info_left.distortion_model
                distortion_model_right = camera_info_right.distortion_model
                if distortion_model_left != distortion_model_right:
                    carb.log_warn(
                        f"Mismatched stereo camera distortion models: left = {distortion_model_left}, right = {distortion_model_right}."
                    )
                    return False

                translation, orientation = compute_relative_pose(
                    left_camera_prim=camera_left, right_camera_prim=camera_right
                )

                # Compute stereo rectification parameters
                if distortion_model_left == "equidistant":
                    (
                        R1,
                        R2,
                        P1,
                        P2,
                        _,
                    ) = cv.fisheye.stereoRectify(
                        K1=np.reshape(camera_info_left.k, [3, 3]),
                        D1=np.array(camera_info_left.d),
                        K2=np.reshape(camera_info_right.k, [3, 3]),
                        D2=np.array(camera_info_right.d),
                        imageSize=(width_left, height_left),
                        R=orientation,
                        tvec=translation,
                        flags=cv.CALIB_ZERO_DISPARITY,
                    )
                else:
                    R1, R2, P1, P2, _, _, _ = cv.stereoRectify(
                        cameraMatrix1=np.reshape(camera_info_left.k, [3, 3]),
                        distCoeffs1=np.array(camera_info_left.d),
                        cameraMatrix2=np.reshape(camera_info_right.k, [3, 3]),
                        distCoeffs2=np.array(camera_info_right.d),
                        imageSize=(width_left, height_left),
                        R=orientation,
                        T=translation,
                        flags=cv.CALIB_ZERO_DISPARITY,
                    )

                camera_info_left.r = R1.ravel().tolist()
                camera_info_right.r = R2.ravel().tolist()
                camera_info_left.p = P1.ravel().tolist()
                camera_info_right.p = P2.ravel().tolist()

                if use_srtx:
                    from omni.replicator.srtx import SrtxCore

                    usd_scene = str(stage.GetRootLayer().identifier) if stage else ""
                    srtx_instance = SrtxCore.get_instance(usd_scene)
                    if srtx_instance is None:
                        carb.log_error(f"No SRTX instance for stage '{usd_scene}'")
                        return False

                    sensor_set_name_left = prepare_srtx_sensor_set(srtx_instance, render_product_path)
                    if sensor_set_name_left is None:
                        carb.log_error(f"Failed to prepare SRTX sensor set for {render_product_path}")
                        return False
                    sensor_set_name_right = prepare_srtx_sensor_set(srtx_instance, render_product_path_right)
                    if sensor_set_name_right is None:
                        carb.log_error(f"Failed to prepare SRTX sensor set for {render_product_path_right}")
                        return False
                    node_namespace_left = collect_namespace(db.inputs.nodeNamespace, render_product_path)
                    node_namespace_right = collect_namespace(db.inputs.nodeNamespace, render_product_path_right)
                    if not OgnROS2CameraInfoHelper.add_srtx_camera_info_publisher(
                        state,
                        stage,
                        srtx_instance,
                        sensor_set_name_right,
                        db.inputs.frameIdRight,
                        db.inputs.topicNameRight,
                        node_namespace_right,
                        db.inputs.queueSize,
                        db.inputs.qosProfile,
                        camera_info_right,
                        render_product_path_right,
                    ):
                        cleanup_srtx_camera_info_state(state)
                        return False
                    if not OgnROS2CameraInfoHelper.add_srtx_camera_info_publisher(
                        state,
                        stage,
                        srtx_instance,
                        sensor_set_name_left,
                        db.inputs.frameId,
                        db.inputs.topicName,
                        node_namespace_left,
                        db.inputs.queueSize,
                        db.inputs.qosProfile,
                        camera_info_left,
                        render_product_path,
                    ):
                        cleanup_srtx_camera_info_state(state)
                        return False

                    state.initialized = True
                    return True

                # Create right-side writer
                db.per_instance_state.rvRight = "PostProcessDispatchRight"
                OgnROS2CameraInfoHelper.add_camera_info_writer(
                    db,
                    topicName=db.inputs.topicNameRight,
                    frameId=db.inputs.frameIdRight,
                    camera_info=camera_info_right,
                    render_product_path=render_product_path_right,
                    time_type=time_type_input,
                )

            elif use_srtx:
                from omni.replicator.srtx import SrtxCore

                usd_scene = str(stage.GetRootLayer().identifier) if stage else ""
                srtx_instance = SrtxCore.get_instance(usd_scene)
                if srtx_instance is None:
                    carb.log_error(f"No SRTX instance for stage '{usd_scene}'")
                    return False

                sensor_set_name = prepare_srtx_sensor_set(srtx_instance, render_product_path)
                if sensor_set_name is None:
                    carb.log_error(f"Failed to prepare SRTX sensor set for {render_product_path}")
                    return False
                node_namespace = collect_namespace(db.inputs.nodeNamespace, render_product_path)
                if not OgnROS2CameraInfoHelper.add_srtx_camera_info_publisher(
                    state,
                    stage,
                    srtx_instance,
                    sensor_set_name,
                    db.inputs.frameId,
                    db.inputs.topicName,
                    node_namespace,
                    db.inputs.queueSize,
                    db.inputs.qosProfile,
                    camera_info_left,
                    render_product_path,
                ):
                    return False

                state.initialized = True
                return True

            # Create left-side writer
            db.per_instance_state.rv = "PostProcessDispatch"
            OgnROS2CameraInfoHelper.add_camera_info_writer(
                db,
                topicName=db.inputs.topicName,
                frameId=db.inputs.frameId,
                camera_info=camera_info_left,
                render_product_path=render_product_path,
                time_type=time_type_input,
            )

        state.initialized = True
        return True

    @staticmethod
    def release_instance(node, graph_instance_id) -> None:
        """Release resources for a graph instance."""
        try:
            state = OgnROS2CameraInfoHelperInternalState.per_instance_internal_state(node)
        except Exception:
            state = None

        if state is not None:
            cleanup_srtx_camera_info_state(state)
            state.reset()
