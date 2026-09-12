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
import isaacsim.core.experimental.utils.prim as prim_utils
import omni
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata
import omni.syntheticdata._syntheticdata as sd
from isaacsim.core.experimental.objects import Camera
from isaacsim.core.nodes import BaseWriterNode
from pxr import Usd

# Real Eagle hardware ships a stereo rig with the two VB1940 imagers separated by ~88 mm
# along the rig +x axis. The value we program into the EEPROM's T field is NEGATED because
# staging's hololink::sensors::NativeVb1940Sensor reconstructs base_T_cam as
# `Pose3d{q, calib.T}.inverse()` — so to land at +0.088 m on the right imager's base_T_cam
# (matching the previous staging-side hardcoded intrinsics fallback), we need calib.T = -0.088 m.
_EAGLE_STEREO_BASELINE_X_M: float = -0.088


def _read_camera_intrinsics(render_product_path: str) -> tuple[list[float], list[float]] | None:
    """Compute (calibrationIntrinsics, calibrationTranslation) from the camera prim referenced by the render product.

    Returns the packed-8-doubles intrinsics list (matching the layout consumed by OgnHSBSend)
    and a 3-element translation list, or None if the USDA scene is missing values needed to
    compute intrinsics. The emulated rig EEPROM stays zero-initialized in that case and the
    downstream cuVSLAM receiver will reject — that's a scene-hookup error, not a runtime issue.

    Both halves (L_*, R_*) of the returned intrinsics are populated with the same values
    under the assumption that left and right cameras have identical intrinsics, which holds
    for both real Eagle hardware and the cuvslam_vb1940 sim scene.
    """
    rp_prim = prim_utils.get_prim_at_path(render_product_path)
    if not rp_prim.IsValid():
        return None
    camera_targets = rp_prim.GetRelationship("camera").GetTargets()
    if not camera_targets:
        return None
    resolution = prim_utils.get_prim_attribute_value(rp_prim, "resolution")
    camera = Camera(str(camera_targets[0]), reset_xform_op_properties=False)
    focal_length = camera.get_focal_lengths().numpy().item()
    h_aperture, v_aperture = (v.numpy().item() for v in camera.get_apertures())
    # Aperture offsets shift the lens center relative to the film plane; they translate to a
    # shift of the principal point (cx/cy) away from the image center. Zero for VB1940
    # physical hardware, where the imager is aligned with the lens optical axis.
    h_aperture_offset, v_aperture_offset = (v.numpy().item() for v in camera.get_aperture_offsets())
    if focal_length is None or resolution is None:
        return None
    width, height = float(resolution[0]), float(resolution[1])
    if h_aperture <= 0.0 or v_aperture <= 0.0 or width <= 0.0 or height <= 0.0:
        return None
    fx = focal_length * width / h_aperture
    fy = focal_length * height / v_aperture
    # cx/cy: image center plus the aperture-offset ratio (a positive USD aperture offset
    # shifts the lens center in +x/+y in aperture units, which moves the principal point
    # by the same fraction of the image in pixels).
    cx = width * (0.5 + h_aperture_offset / h_aperture)
    cy = height * (0.5 + v_aperture_offset / v_aperture)
    intrinsics = [fx, fy, cx, cy, fx, fy, cx, cy]
    # Baseline is hardcoded to the Eagle hardware value; the USD inter-camera transform
    # is not consulted. Deriving it from actual camera prims is a follow-up.
    carb.log_info(
        f"HSBCameraHelper: applying Eagle default stereo baseline "
        f"T=({_EAGLE_STEREO_BASELINE_X_M}, 0.0, 0.0) m. Non-Eagle rigs will get an "
        f"incorrect baseline in the EEPROM."
    )
    translation = [_EAGLE_STEREO_BASELINE_X_M, 0.0, 0.0]
    return intrinsics, translation


class OgnHSBCameraHelperInternalState(BaseWriterNode):
    """Internal state for the HSB camera helper OmniGraph node."""

    def __init__(self) -> None:
        self.rv = ""
        self.resetSimulationTimeOnStop = True
        self.publishStepSize = 1

        super().__init__(initialize=False)

    def post_attach(self, writer: rep.Writer, render_product: str | list[str]) -> None:
        """Configure node attributes after attaching a writer to a render product.

        Args:
            writer: Replicator writer attached to the render product.
            render_product: Render product path or paths being written.
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


class OgnHSBCameraHelper:
    """OmniGraph node that publishes camera images over HSB."""

    @staticmethod
    def internal_state() -> OgnHSBCameraHelperInternalState:
        """Return a new internal state instance.

        Returns:
            Internal state for one OmniGraph node instance.
        """
        return OgnHSBCameraHelperInternalState()

    @staticmethod
    def compute(db: og.Database) -> bool:
        """Compute the node output by initializing and attaching HSB writers.

        Args:
            db: OmniGraph database for this node evaluation.

        Returns:
            True if the node computed successfully.
        """
        if db.per_instance_state.initialized is False:
            stage = omni.usd.get_context().get_stage()
            if stage is None:
                carb.log_warn("USD stage is not available yet, retrying on next call")
                return False
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
                    # Program the emulated rig EEPROM with intrinsics + extrinsics computed
                    # from the USDA Camera prim. The downstream OgnHSBSend node serializes
                    # these into the 30-double EEPROM layout that hololink's NativeVb1940Sensor
                    # reads via I2C. Wrapped in try/except so a scene-side hookup error doesn't
                    # take down the whole writer-init path; instead we log and let cuVSLAM
                    # surface the zero-intrinsics failure with a clearer error.
                    try:
                        calibration = _read_camera_intrinsics(render_product_path)
                        if calibration is not None:
                            init_kwargs["calibrationIntrinsics"] = calibration[0]
                            init_kwargs["calibrationTranslation"] = calibration[1]
                            carb.log_info(
                                f"HSBCameraHelper: calibration from {render_product_path} -> "
                                f"intrinsics={calibration[0]}, translation={calibration[1]}"
                            )
                        else:
                            carb.log_warn(
                                f"HSBCameraHelper: could not derive calibration from {render_product_path}; "
                                "rig EEPROM will stay zero-initialized"
                            )
                    except Exception as cal_exc:
                        carb.log_error(f"HSBCameraHelper: _read_camera_intrinsics raised: {cal_exc}")
                        print(traceback.format_exc())
                    writer.initialize(**init_kwargs)

                    db.per_instance_state.append_writer(writer)
                    db.per_instance_state.attach_writers(render_product_path)
                except Exception as e:
                    carb.log_error(f"HSBCameraHelper: Failed to setup writer: {e}")
                    print(traceback.format_exc())
                    return False

                db.per_instance_state.initialized = True

        db.outputs.execOut = og.ExecutionAttributeState.ENABLED
        return True

    @staticmethod
    def release_instance(node: object, graph_instance_id: int) -> None:
        """Release resources when a node instance is destroyed.

        Args:
            node: OmniGraph node whose resources are being released.
            graph_instance_id: Graph instance identifier for the node.
        """
        try:
            state = OgnHSBCameraHelperInternalState.per_instance_internal_state(node)
        except Exception:
            state = None

        if state is not None:
            state.reset()
