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

"""MobilityGenCamera class for managing camera rendering and state capture."""

import carb
import numpy as np
import omni.replicator.core as rep
from isaacsim.core.experimental.utils.prim import get_prim_at_path
from pxr import Gf, Usd, UsdGeom

from .common import Buffer, Module


class MobilityGenCamera(Module):
    """A camera module for capturing images and state in Isaac Sim.

    Args:
        prim_path: The USD prim path for the camera.
        resolution: The (width, height) resolution in pixels.
    """

    def __init__(self, prim_path: str, resolution: tuple[int, int]) -> None:

        self._prim_path = prim_path
        self._resolution = resolution
        self._render_product = None
        self._rgb_annotator = None
        self._segmentation_annotator = None
        self._instance_id_segmentation_annotator = None
        self._normals_annotator = None
        self._depth_annotator = None
        self._prim = get_prim_at_path(self._prim_path)

        self.rgb_image = Buffer(tags=["rgb"])
        self.segmentation_image = Buffer(tags=["segmentation"])
        self.segmentation_info = Buffer()
        self.depth_image = Buffer(tags=["depth"])
        # TODO: tag should be ["instance_id_segmentation"], not ["segmentation"]. Using the same tag causes
        # state_dict_segmentation() to return both buffers, mixing instance ID data into state/segmentation/.
        # Also, the annotator used is the hidden legacy "instance_id_segmentation" node; should be
        # "instance_id_segmentation_fast" to match the supported API.
        self.instance_id_segmentation_image = Buffer(tags=["segmentation"])
        self.instance_id_segmentation_info = Buffer()
        self.normals_image = Buffer(tags=["normals"])
        self.position = Buffer()
        self.orientation = Buffer()

    def enable_rendering(self) -> None:
        """Enable rendering by creating a render product for this camera."""
        self._render_product = rep.create.render_product(self._prim_path, self._resolution, force_new=False)
        # Disable hydra texture updates while annotators are being attached so that
        # OmniGraph does not try to evaluate partially-constructed nodes
        # (CreateAttrCommand with node=? crash).  Call finalize_rendering() once all
        # annotators are attached to re-enable updates.
        self._render_product.hydra_texture.set_updates_enabled(False)

    def finalize_rendering(self) -> None:
        """Re-enable hydra texture updates after all annotators have been attached."""
        if self._render_product is not None:
            self._render_product.hydra_texture.set_updates_enabled(True)

    def disable_rendering(self) -> None:
        """Disable rendering and release all annotators and the render product."""
        if self._render_product is None:
            return

        if self._rgb_annotator is not None:
            self._rgb_annotator.detach()
            self._rgb_annotator = None

        if self._segmentation_annotator is not None:
            self._segmentation_annotator.detach()
            self._segmentation_annotator = None

        if self._depth_annotator is not None:
            self._depth_annotator.detach()
            self._depth_annotator = None

        if self._instance_id_segmentation_annotator is not None:
            self._instance_id_segmentation_annotator.detach()
            self._instance_id_segmentation_annotator = None

        if self._normals_annotator is not None:
            self._normals_annotator.detach()
            self._normals_annotator = None

        self._render_product.destroy()
        self._render_product = None

    def enable_rgb_rendering(self) -> None:
        """Enable RGB image capture for this camera."""
        if self._render_product is None:
            self.enable_rendering()
        if self._rgb_annotator is not None:
            return
        self._rgb_annotator = rep.AnnotatorRegistry.get_annotator("LdrColor")
        self._rgb_annotator.attach(self._render_product)

    def enable_segmentation_rendering(self) -> None:
        """Enable semantic segmentation capture for this camera."""
        if self._render_product is None:
            self.enable_rendering()
        if self._segmentation_annotator is not None:
            return
        self._segmentation_annotator = rep.AnnotatorRegistry.get_annotator(
            "semantic_segmentation", init_params={"colorize": False}
        )
        self._segmentation_annotator.attach(self._render_product)

    def enable_instance_id_segmentation_rendering(self) -> None:
        """Enable instance ID segmentation capture for this camera."""
        if self._render_product is None:
            self.enable_rendering()
        if self._instance_id_segmentation_annotator is not None:
            return
        self._instance_id_segmentation_annotator = rep.AnnotatorRegistry.get_annotator(
            "instance_id_segmentation", init_params={"colorize": False}
        )
        self._instance_id_segmentation_annotator.attach(self._render_product)

    def enable_depth_rendering(self) -> None:
        """Enable depth image capture for this camera."""
        if self._render_product is None:
            self.enable_rendering()
        if self._depth_annotator is not None:
            return
        self._depth_annotator = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
        self._depth_annotator.attach(self._render_product)

    def enable_normals_rendering(self) -> None:
        """Enable surface normals capture for this camera."""
        if self._render_product is None:
            self.enable_rendering()
        if self._normals_annotator is not None:
            return
        self._normals_annotator = rep.AnnotatorRegistry.get_annotator("normals")
        self._normals_annotator.attach(self._render_product)

    def update_state(self) -> None:
        """Update all camera state buffers by reading from annotators and USD."""
        if not self._prim or not self._prim.IsValid():
            carb.log_warn(f"MobilityGenCamera: prim at '{self._prim_path}' is invalid; skipping update")
            super().update_state()
            return

        # do_array_copy=True is required: annotators return views into shared internal buffers that are
        # overwritten each render frame. Without copying, Buffer.value would be silently corrupted.
        if self._rgb_annotator is not None:
            data = self._rgb_annotator.get_data(do_array_copy=True)
            if data.ndim == 3:
                self.rgb_image.set_value(data[:, :, :3])
            elif data.ndim == 1 and data.size > 0:
                # Replicator returns a flat (H*W*4,) buffer on some builds; reshape to (H, W, 4).
                w, h = self._resolution
                self.rgb_image.set_value(data.reshape(h, w, -1)[:, :, :3])
        # An annotator can briefly return an empty (0,) buffer before the renderer has
        # produced a frame; writing one to a PNG raises "tile cannot extend outside
        # image" and aborts the batch (RGB above already guards this). Keep the prior
        # value rather than clear it — MobilityGenReader expects one image per step.
        if self._segmentation_annotator is not None:
            data = self._segmentation_annotator.get_data(do_array_copy=True)
            seg_image = data["data"]
            if seg_image is not None and seg_image.size > 0:
                self.segmentation_image.set_value(seg_image)
                self.segmentation_info.set_value(data["info"])
            else:
                carb.log_warn(f"MobilityGenCamera: empty segmentation buffer at '{self._prim_path}'; skipping frame")

        if self._depth_annotator is not None:
            depth_data = self._depth_annotator.get_data(do_array_copy=True)
            if depth_data is not None and depth_data.size > 0:
                self.depth_image.set_value(depth_data)
            else:
                carb.log_warn(f"MobilityGenCamera: empty depth buffer at '{self._prim_path}'; skipping frame")

        if self._instance_id_segmentation_annotator is not None:
            data = self._instance_id_segmentation_annotator.get_data(do_array_copy=True)
            id_seg_image = data["data"]
            if id_seg_image is not None and id_seg_image.size > 0:
                self.instance_id_segmentation_image.set_value(id_seg_image)
                self.instance_id_segmentation_info.set_value(data["info"])
            else:
                carb.log_warn(f"MobilityGenCamera: empty instance-id buffer at '{self._prim_path}'; skipping frame")

        if self._normals_annotator is not None:
            normals_data = self._normals_annotator.get_data(do_array_copy=True)
            if normals_data is not None and normals_data.size > 0:
                self.normals_image.set_value(normals_data)
            else:
                carb.log_warn(f"MobilityGenCamera: empty normals buffer at '{self._prim_path}'; skipping frame")

        # Use USD directly (no warp tensors) to avoid a CPU-GPU sync every step.
        xform = UsdGeom.Xformable(self._prim)
        world_mat: Gf.Matrix4d = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t: Gf.Vec3d = world_mat.ExtractTranslation()
        r = world_mat.ExtractRotation().GetQuaternion()
        imag = r.GetImaginary()
        self.position.set_value(np.array(t))
        self.orientation.set_value(np.array([r.GetReal(), imag[0], imag[1], imag[2]]))

        super().update_state()
