# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
from typing import Any, List, Optional, Tuple, Union

import numpy as np
import omni.replicator.core as rep
import torch
import warp as wp
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.carb import get_carb_setting
from pxr import Usd, Vt

# from ROS camera convention to USD camera convention
U_R_TRANSFORM = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

# from USD camera convention to ROS camera convention
R_U_TRANSFORM = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

# from USD camera convention to World camera convention
W_U_TRANSFORM = np.array([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])

# from World camera convention to USD camera convention
U_W_TRANSFORM = np.array([[0, -1, 0, 0], [0, 0, 1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])

ANNOTATOR_SPEC = {
    "rgb": {"name": "rgba", "channels": 4, "dtype": wp.uint8},
    "rgba": {"name": "rgba", "channels": 4, "dtype": wp.uint8},
    "depth": {"name": "distance_to_image_plane", "channels": 1, "dtype": wp.float32},
    "distance_to_image_plane": {"name": "distance_to_image_plane", "channels": 1, "dtype": wp.float32},
    "distance_to_camera": {"name": "distance_to_camera", "channels": 1, "dtype": wp.float32},
    "normals": {"name": "normals", "channels": 4, "dtype": wp.float32},
    "motion_vectors": {"name": "motion_vectors", "channels": 4, "dtype": wp.float32},
    "semantic_segmentation": {"name": "semantic_segmentation", "channels": 1, "dtype": wp.uint32},
    "instance_segmentation_fast": {"name": "instance_segmentation_fast", "channels": 1, "dtype": wp.uint32},
    "instance_id_segmentation_fast": {"name": "instance_id_segmentation_fast", "channels": 1, "dtype": wp.uint32},
}


@wp.kernel
def reshape_tiled_image(
    tiled_image_buffer: Any,
    batched_image: Any,
    image_height: int,
    image_width: int,
    num_channels: int,
    num_output_channels: int,
    num_tiles_x: int,
    offset: int,
):
    """Reshape a tiled image (height*width*num_channels*num_cameras,) to a batch of images (num_cameras, height, width, num_channels).

    Args:
        tiled_image_buffer: The input image buffer. Shape is ((height*width*num_channels*num_cameras,).
        batched_image: The output image. Shape is (num_cameras, height, width, num_channels).
        image_width: The width of the image.
        image_height: The height of the image.
        num_channels: The number of channels in the image.
        num_tiles_x: The number of tiles in x direction.
        offset: The offset in the image buffer. This is used when multiple image types are concatenated in the buffer.
    """
    # get the thread id
    camera_id, height_id, width_id = wp.tid()
    # resolve the tile indices
    tile_x_id = camera_id % num_tiles_x
    tile_y_id = camera_id // num_tiles_x
    # compute the start index of the pixel in the tiled image buffer
    pixel_start = (
        offset
        + num_channels * num_tiles_x * image_width * (image_height * tile_y_id + height_id)
        + num_channels * tile_x_id * image_width
        + num_channels * width_id
    )
    # copy the pixel values into the batched image
    for i in range(num_output_channels):
        batched_image[camera_id, height_id, width_id, i] = batched_image.dtype(tiled_image_buffer[pixel_start + i])


wp.overload(
    reshape_tiled_image,
    {"tiled_image_buffer": wp.array(dtype=wp.uint8), "batched_image": wp.array(dtype=wp.uint8, ndim=4)},
)

wp.overload(
    reshape_tiled_image,
    {"tiled_image_buffer": wp.array(dtype=wp.float32), "batched_image": wp.array(dtype=wp.float32, ndim=4)},
)


class CameraView(XFormPrim):
    """Provides high level functions to deal tiled/batched data from cameras

    .. list-table::
        :header-rows: 1

        * - Annotator type
            - Channels
            - Dtype
        * - ``"rgb"``
            - 3
            - ``uint8``
        * - ``"rgba"``
            - 4
            - ``uint8``
        * - ``"depth"`` / ``"distance_to_image_plane"``
            - 1
            - ``float32``
        * - ``"distance_to_camera"``
            - 1
            - ``float32``
        * - ``"normals"``
            - 4
            - ``float32``
        * - ``"motion_vectors"``
            - 4
            - ``float32``
        * - ``"semantic_segmentation"``
            - 1
            - ``uint32``
        * - ``"instance_segmentation_fast"``
            - 1
            - ``int32``
        * - ``"instance_id_segmentation_fast"``
            - 1
            - ``int32``

    Args:
        prim_paths_expr: Prim paths regex to encapsulate all prims that match it. E.g.: "/World/Env[1-5]/Camera" will match
                         /World/Env1/Camera, /World/Env2/Camera..etc. Additionally a list of regex can be provided.
        camera_resolution: Resolution of each sensor (width, height).
        output_annotators: Annotator/sensor types to configure.
        name (str, optional): Shortname to be used as a key by Scene class.
                              Note: needs to be unique if the object is added to the Scene.
        positions Default positions in the world frame of the prim. Shape is (N, 3).
                  Defaults to None, which means left unchanged.
        translations: Default translations in the local frame of the prims (with respect to its parent prims). shape is (N, 3).
                      Defaults to None, which means left unchanged.
        orientations: Default quaternion orientations in the world/ local frame of the prim (depends if translation
                      or position is specified). Quaternion is scalar-first (w, x, y, z). Shape is (N, 4).
                      Defaults to None, which means left unchanged.
        scales: Local scales to be applied to the prim's dimensions. Shape is (N, 3).
                Defaults to None, which means left unchanged.
        visibilities: Set to False for an invisible prim in the stage while rendering.
                      Shape is (N,). Defaults to None.
        reset_xform_properties: True if the prims don't have the right set of xform properties (i.e: translate,
                                orient and scale) ONLY and in that order. Set this parameter to False if the object
                                were cloned using using the cloner api in isaacsim.core.cloner. Defaults to True.

    Raises:
        Exception: if translations and positions defined at the same time.
        Exception: No prim was matched using the prim_paths_expr provided.
    """

    def __init__(
        self,
        prim_paths_expr: str = None,
        name: str = "camera_prim_view",
        camera_resolution: Tuple[int, int] = (256, 256),
        output_annotators: Optional[List[str]] = ["rgb", "depth"],
        positions: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        translations: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        orientations: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        scales: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        visibilities: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        reset_xform_properties: bool = True,
    ):
        XFormPrim.__init__(
            self,
            prim_paths_expr=prim_paths_expr,
            name=name,
            positions=positions,
            translations=translations,
            orientations=orientations,
            scales=scales,
            visibilities=visibilities,
            reset_xform_properties=reset_xform_properties,
        )
        self._output_annotators = output_annotators
        self._annotators = dict()
        self.camera_resolution = camera_resolution
        self._tiled_render_product = None
        self._setup_tiled_sensor()

    def __del__(self):
        self.destroy()

    def destroy(self) -> None:
        self._clean_up_tiled_sensor()
        super().destroy()

    def _clean_up_tiled_sensor(self):
        """Clean up the sensor by detaching annotators and destroying render products, and removing related prims."""
        if self._tiled_render_product is not None:
            for annotator in self._annotators.values():
                annotator.detach([self._tiled_render_product.path])
            self._tiled_render_product.destroy()
            self._tiled_render_product = None

    def _get_tiled_resolution(self, num_cameras, resolution) -> Tuple[int, int]:
        """Calculate the resolution for the tiled sensor based on the number of cameras and individual camera resolution.

        Args:
            num_cameras (int): Total number of cameras.
            resolution (Tuple[int, int]): Resolution of each individual camera.

        Returns:
            Tuple[int, int]: The total resolution for the tiled sensor layout.
        """
        num_rows = round(num_cameras**0.5)
        num_columns = (num_cameras + num_rows - 1) // num_rows

        return (num_columns * resolution[0], num_rows * resolution[1])

    def _setup_tiled_sensor(self):
        """Set up the tiled sensor, compute resolutions, attach annotators, and initiate the render process."""
        self._clean_up_tiled_sensor()

        self.tiled_resolution = self._get_tiled_resolution(len(self.prims), self.camera_resolution)
        self._tiled_render_product = rep.create.render_product_tiled(
            cameras=self.prim_paths,
            tile_resolution=self.camera_resolution,
            name=f"{self.name}_tiled_sensor",
        )
        # define the annotators based on defined types
        self._render_product_path = self._tiled_render_product.path
        for annotator_type in self._output_annotators:
            # check for supported annotator
            if annotator_type not in ANNOTATOR_SPEC:
                raise ValueError(
                    f"Unsupported annotator type: {annotator_type}. Supported types are {list(ANNOTATOR_SPEC.keys())}"
                )
            # get annotator
            if annotator_type == "rgba" or annotator_type == "rgb":
                self._annotators["rgba"] = rep.AnnotatorRegistry.get_annotator(
                    "rgb", device="cuda", do_array_copy=False
                )
            elif annotator_type == "depth" or annotator_type == "distance_to_image_plane":
                self._annotators["distance_to_image_plane"] = rep.AnnotatorRegistry.get_annotator(
                    "distance_to_image_plane", device="cuda", do_array_copy=False
                )
            else:
                self._annotators[annotator_type] = rep.AnnotatorRegistry.get_annotator(
                    annotator_type, device="cuda", do_array_copy=False
                )
        # attach the annotator to the render product
        for annotator in self._annotators.values():
            annotator.attach(self._render_product_path)

    def get_render_product_path(self) -> str:
        """Retrieve the file path of the render product associated with the tiled sensor.

        Returns:
            str: The path to the render product, or None if not available.
        """
        return self._render_product_path

    # TODO: add functionality to pause and resume tiled sensor, similar to camera.py
    def set_resolutions(self, resolution: Tuple[int, int]) -> None:
        """Set the resolutions for all cameras, updating the tiled sensor configuration if changed.

        Args:
            resolution (Tuple[int, int]): The new resolution to apply to all cameras.
        """
        if not resolution == self.camera_resolution:
            self.camera_resolution = resolution
            # update tiled sensor after changing resolution
            self._setup_tiled_sensor()

    def get_resolutions(self) -> Tuple[int, int]:
        """Retrieve the current resolution setting for all cameras.

        Returns:
            Tuple[int, int]: The resolution of the cameras.
        """
        return self.camera_resolution

    def get_aspect_ratios(self) -> float:
        """Calculate the aspect ratio of the cameras based on current resolution settings.

        Returns:
            float: The aspect ratio, defined as width divided by height.
        """
        width, height = self.get_resolutions()
        return float(width) / float(height)

    def _convert_camera_axes(self, orientations, transform_matrix):
        """Convert orientations using the specified transformation matrix.

        Args:
            orientations (Union[np.ndarray, torch.Tensor, wp.array]): Input orientations.
            transform_matrix (np.ndarray): The transformation matrix to apply.

        Returns:
            The converted orientations.
        """
        if isinstance(orientations, np.ndarray):
            orientation_matrices = self._backend_utils.quats_to_rot_matrices(orientations)
            converted_matrices = np.einsum("ij,kjl->kil", transform_matrix[:3, :3], orientation_matrices)
            return self._backend_utils.rot_matrices_to_quats(converted_matrices)
        elif isinstance(orientations, torch.Tensor):
            orientation_matrices = self._backend_utils.quats_to_rot_matrices(orientations)
            converted_matrices = torch.matmul(
                torch.tensor(transform_matrix[:3, :3], dtype=torch.float32, device=orientation_matrices.device),
                orientation_matrices,
            )
            return self._backend_utils.rot_matrices_to_quats(converted_matrices)
        elif isinstance(orientations, wp.array):
            # Assuming similar operations are possible with wp.array
            orientation_matrices = self._backend_utils.quats_to_rot_matrices(orientations)
            converted_matrices = wp.matmul(
                wp.array(transform_matrix[:3, :3], device=orientation_matrices.device), orientation_matrices
            )
            return self._backend_utils.rot_matrices_to_quats(converted_matrices)
        else:
            raise TypeError("Unsupported type for orientations")

    def get_world_poses(
        self,
        indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None,
        camera_axes: str = "world",
        usd: bool = True,
    ) -> Union[
        Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor], Tuple[wp.indexedarray, wp.indexedarray]
    ]:
        """Get the poses of the prims in the view with respect to the world's frame

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            usd (bool, optional): True to query from usd. Otherwise False to query from Fabric data. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor], Tuple[wp.indexedarray, wp.indexedarray]]: first index is positions in the world frame of the prims. shape is (M, 3).
                                                                                     second index is quaternion orientations in the world frame of the prims.
                                                                                     quaternion is scalar-first (w, x, y, z). shape is (M, 4).

        Example:

        """
        if camera_axes not in ["world", "ros", "usd"]:
            raise Exception(
                "camera axes passed {} is not supported: accepted values are ["
                "world"
                ", "
                "ros"
                ", "
                "usd"
                "] only".format(camera_axes)
            )

        translations, orientations = XFormPrim.get_world_poses(self, indices, usd=usd)

        if orientations is not None:
            if camera_axes == "world":
                orientations = self._convert_camera_axes(orientations, U_W_TRANSFORM)
            elif camera_axes == "ros":
                orientations = self._convert_camera_axes(orientations, U_R_TRANSFORM)

        return translations, orientations

    def set_world_poses(
        self,
        positions: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        orientations: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None,
        camera_axes: str = "world",
        usd: bool = True,
    ) -> None:
        """Set the world poses for all cameras, adjusting their positions and orientations based on specified indices and coordinate system.

        Args:
            positions (Optional[Union[np.ndarray, torch.Tensor, wp.array]]): New positions for the cameras.
            orientations (Optional[Union[np.ndarray, torch.Tensor, wp.array]]): New orientations for the cameras.
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]]): Specific cameras to update.
            camera_axes (str): The coordinate system to use ('world', 'ros', 'usd').
            usd (bool, optional): True to query from usd. Otherwise False to query from Fabric data. Defaults to True.

        Raises:
            Exception: If the provided camera_axes is not supported.
        """
        if camera_axes not in ["world", "ros", "usd"]:
            raise Exception(
                "camera axes passed {} is not supported: accepted values are ["
                "world"
                ", "
                "ros"
                ", "
                "usd"
                "] only".format(camera_axes)
            )

        if orientations is not None:
            if camera_axes == "world":
                orientations = self._convert_camera_axes(orientations, W_U_TRANSFORM)
            elif camera_axes == "ros":
                orientations = self._convert_camera_axes(orientations, R_U_TRANSFORM)

        return XFormPrim.set_world_poses(self, positions, orientations, indices, usd=usd)

    def get_local_poses(
        self,
        indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None,
        camera_axes: str = "world",
    ) -> Union[
        Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor], Tuple[wp.indexedarray, wp.indexedarray]
    ]:
        """Get prim poses in the view with respect to the local frame (the prim's parent frame)

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor], Tuple[wp.indexedarray, wp.indexedarray]]: first index is positions in the local frame of the prims. shape is (M, 3).
                                                                                     second index is quaternion orientations in the local frame of the prims.
                                                                                     quaternion is scalar-first (w, x, y, z). shape is (M, 4).

        Example:

        """
        if camera_axes not in ["world", "ros", "usd"]:
            raise Exception(
                "camera axes passed {} is not supported: accepted values are ["
                "world"
                ", "
                "ros"
                ", "
                "usd"
                "] only".format(camera_axes)
            )

        translations, orientations = XFormPrim.get_local_poses(self, indices)

        if orientations is not None:
            if camera_axes == "world":
                orientations = self._convert_camera_axes(orientations, U_W_TRANSFORM)
            elif camera_axes == "ros":
                orientations = self._convert_camera_axes(orientations, U_R_TRANSFORM)

        return translations, orientations

    def set_local_poses(
        self,
        positions: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        orientations: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None,
        camera_axes: str = "world",
    ) -> None:
        """Set the local poses for all cameras, adjusting their positions and orientations based on specified indices and coordinate system.

        Args:
            positions (Optional[Union[np.ndarray, torch.Tensor, wp.array]]): New positions for the cameras.
            orientations (Optional[Union[np.ndarray, torch.Tensor, wp.array]]): New orientations for the cameras.
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]]): Specific cameras to update.
            camera_axes (str): The coordinate system to use ('world', 'ros', 'usd').

        Raises:
            Exception: If the provided camera_axes is not supported.
        """

        if camera_axes not in ["world", "ros", "usd"]:
            raise Exception(
                "camera axes passed {} is not supported: accepted values are ["
                "world"
                ", "
                "ros"
                ", "
                "usd"
                "] only".format(camera_axes)
            )

        if orientations is not None:
            if camera_axes == "world":
                orientations = self._convert_camera_axes(orientations, W_U_TRANSFORM)
            elif camera_axes == "ros":
                orientations = self._convert_camera_axes(orientations, R_U_TRANSFORM)

        return XFormPrim.set_local_poses(self, positions, orientations, indices)

    def get_data(
        self, annotator_type: str, *, tiled: bool = False, out: Optional[wp.array] = None
    ) -> Tuple[wp.array, dict[str, Any]]:
        """Fetch the specified annotator/sensor data for all cameras as a batch of images or as a single tiled image.

        Args:
            annotator_type: Annotator/sensor type from which fetch the data.
            tiled: Whether to get annotator/sensor data as a single tiled image.
            out: Pre-allocated array to fill with the fetched data.

        Returns:
            2-items tuple. The first item is an array containing the fetched data (if ``out`` is defined,
            its instance will be returned). The second item is a dictionary containing additional information according
            to the requested annotator/sensor type.

        Raises:
            ValueError: If the specified annotator type is not supported.
            ValueError: If the specified annotator type is not configured when instantiating the object.
        """
        # get and check annotator specification
        spec = ANNOTATOR_SPEC.get(annotator_type)
        if spec is None:
            raise ValueError(
                f"Unsupported annotator type: {annotator_type}. Supported types are {list(ANNOTATOR_SPEC.keys())}"
            )
        if spec["name"] not in self._annotators:
            raise ValueError(
                f"The specified annotator type ({annotator_type}) was not configured. Enable it when instantiating the object"
            )
        # request data on the same device as output if specified
        output_device = str(out.device) if out is not None else "cuda"
        # get the linear sensor data from the tiled annotator and (if needed) slice it to get only the RGB data
        data = self._annotators[spec["name"]].get_data(device=output_device)
        # If there is no data available yet, return empty array
        if data is None:
            return wp.empty(0, device=output_device), {}
        # check whether returned data is a dict (used for segmentation)
        if isinstance(data, dict):
            tiled_data: wp.array = data["data"]
            info = data["info"]
        else:
            tiled_data: wp.array = data
            info = {}
        # If there is no data available yet, return empty array
        if tiled_data.shape[0] == 0:
            return wp.empty(0, device=output_device), {}
        # tiled image
        if tiled:
            shape = (*self.tiled_resolution, spec["channels"])
            if out is None:
                out = wp.clone(tiled_data[:, :, :3]) if annotator_type == "rgb" else tiled_data.reshape(shape)
            else:
                wp.copy(out, tiled_data[:, :, :3] if annotator_type == "rgb" else tiled_data.reshape(shape))
        # batched images
        else:
            # define internal variables
            channels = spec["channels"]
            output_channels = 3 if annotator_type == "rgb" else channels
            height, width = self.camera_resolution
            # check if the data should be copied to the pre-allocated memory
            if out is None:
                shape = (len(self.prims), height, width, output_channels)
                out = wp.zeros(shape, dtype=spec["dtype"], device=tiled_data.device)
            else:
                # copy the data to the correct device if the devices do not match
                if out.device != tiled_data.device:
                    tiled_data = wp.clone(tiled_data, out.device)
            # use a warp kernel to convert the linear sensor data to a batch of images
            num_tiles_x = self.tiled_resolution[0] // width
            wp.launch(
                kernel=reshape_tiled_image,
                dim=(len(self.prims), height, width),
                inputs=[
                    tiled_data.flatten(),
                    out,
                    height,
                    width,
                    channels,
                    output_channels,
                    num_tiles_x,
                    0,  # offset is always 0 since we sliced the data
                ],
                device=tiled_data.device,
            )
        return out, info

    def get_rgb_tiled(self, out=None, device="cpu") -> np.ndarray | torch.Tensor:
        """Fetch the RGB data for all cameras as a single tiled image.

        Args:
            device (str, optional): The device to return the data on ("cpu" or "cuda"). Defaults to "cpu".
            out (np.ndarray | torch.Tensor, optional): Pre-allocated array or tensor to fill with the RGBA data.

        Returns:
            np.ndarray | torch.Tensor: containing the RGBA data for each camera. Depth channel is excluded if present.
        """
        if out is None:
            if device == "cuda":
                out = wp.to_torch(self.get_data("rgb", tiled=True)[0])
            else:
                out = self.get_data("rgb", tiled=True)[0].numpy()
        else:
            if isinstance(out, np.ndarray):
                out[:] = self.get_data("rgb", tiled=True)[0].numpy()
            elif isinstance(out, torch.Tensor):
                self.get_data("rgb", tiled=True, out=wp.from_torch(out))
        return out

    def get_depth_tiled(self, out=None, device="cpu") -> np.ndarray | torch.Tensor:
        """Fetch the depth data for all cameras as a single tiled image.

        Args:
            device (str, optional): The device to return the data on ("cpu" or "cuda"). Defaults to "cpu".
            out (np.ndarray | torch.Tensor, optional): Pre-allocated array or tensor to fill with the depth data.

        Returns:
            np.ndarray | torch.Tensor: containing the depth data for each camera.
        """
        if out is None:
            if device == "cuda":
                out = wp.to_torch(self.get_data("distance_to_image_plane", tiled=True)[0])
            else:
                out = self.get_data("distance_to_image_plane", tiled=True)[0].numpy()
        else:
            if isinstance(out, np.ndarray):
                out[:] = self.get_data("distance_to_image_plane", tiled=True)[0].numpy()
            elif isinstance(out, torch.Tensor):
                self.get_data("distance_to_image_plane", tiled=True, out=wp.from_torch(out))
        return out

    def get_rgb(self, out=None) -> torch.Tensor:
        """Get the RGB data for all cameras as a batch of images (num_cameras, height, width, 3).

        Returns:
            torch.Tensor: containing the RGB data for each camera.
            Shape is (num_cameras, height, width, 3) with type torch.float32.
        """
        if out is None:
            out = wp.to_torch(self.get_data("rgb")[0])
        else:
            self.get_data("rgb", out=wp.from_torch(out))
        return out

    def get_depth(self, out=None) -> torch.Tensor:
        """Get the depth data for all cameras as a batch of images (num_cameras, height, width, 1).

        Returns:
            torch.Tensor: containing the depth data for each camera.
            Shape is (num_cameras, height, width, 1) with type torch.float32.
        """
        if out is None:
            out = wp.to_torch(self.get_data("distance_to_image_plane")[0])
        else:
            self.get_data("distance_to_image_plane", out=wp.from_torch(out))
        return out

    def get_focal_lengths(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None
    ) -> List[float]:
        """Get the focal length for all cameras

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).

        Returns:
            list[float]: list containing the focal lengths of the cameras.
        """
        focal_lengths = []
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)

        for i in indices:
            focal_lengths.append(self.prims[i].GetAttribute("focalLength").Get() / 10.0)

        return focal_lengths

    def set_focal_lengths(
        self,
        values: List[float],
        indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the focal length for cameras specific in indices. If indices is None, set for all cameras.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).
            values (List[float]): list containing the focal lengths to set for the cameras.
                Length of values must match length of indices.
        """

        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)

        # Ensure length of values matches indices
        if len(indices) != len(values):
            raise ValueError("Length of values must match length of indices")

        for i in indices:
            self.prims[i].GetAttribute("focalLength").Set(values[i] * 10.0)

        return

    def get_focus_distances(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None
    ) -> List[float]:
        """Get the focus distances for cameras specific in indices. If indices is None, get for all cameras.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).

        Returns:
            list[float]: list containing the focal distances of the cameras.
        """
        focus_distances = []
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)

        for i in indices:
            focus_distances.append(self.prims[i].GetAttribute("focusDistance").Get())

        return focus_distances

    def set_focus_distances(
        self,
        values: List[float],
        indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the focus distance for cameras specific in indices. If indices is None, set for all cameras.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                    to set. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).
            values (List[float]): list containing the focus distances to set for the cameras.
                Length of values must match length of indices.
        """

        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)

        # Ensure length of values matches indices
        if len(indices) != len(values):
            raise ValueError("Length of values must match length of indices")

        for i in indices:
            self.prims[i].GetAttribute("focusDistance").Set(values[i])

        return

    def get_lens_apertures(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None
    ) -> List[float]:
        """Get the lens apertures for cameras specific in indices. If indices is None, get for all cameras.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).

        Returns:
            list[float]: list containing the focal distances of the cameras.
        """
        lens_apertures = []
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)

        for i in indices:
            lens_apertures.append(self.prims[i].GetAttribute("fStop").Get())

        return lens_apertures

    def set_lens_apertures(
        self,
        values: List[float],
        indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the lens apertures for cameras specific in indices. If indices is None, set for all cameras.

        Controls Distance Blurring. Lower Numbers decrease focus range, larger numbers increase it.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                    to set. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).
            values (List[float]): list containing the lens apertures to set for the cameras.
                Length of values must match length of indices.
        """

        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)

        # Ensure length of values matches indices
        if len(indices) != len(values):
            raise ValueError("Length of values must match length of indices")

        for i in indices:
            self.prims[i].GetAttribute("fStop").Set(values[i])

        return

    def get_horizontal_apertures(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None
    ) -> List[float]:
        """Get the horizontal apertures for cameras specific in indices. If indices is None, get for all cameras.

        Emulates sensor/film width on a camera.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).

        Returns:
            list[float]: list containing the focal distances of the cameras.
        """
        horizontal_apertures = []
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)

        for i in indices:
            horizontal_apertures.append(self.prims[i].GetAttribute("horizontalAperture").Get())

        return horizontal_apertures

    def set_horizontal_apertures(
        self,
        values: List[float],
        indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the horizontal apertures for cameras specific in indices. If indices is None, set for all cameras.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                    to set. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).
            values (List[float]): list containing the horizontal apertures to set for the cameras.
                Length of values must match length of indices.
        """

        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)

        # Ensure length of values matches indices
        if len(indices) != len(values):
            raise ValueError("Length of values must match length of indices")

        for i in indices:
            self.prims[i].GetAttribute("horizontalAperture").Set(values[i])

        return

    def get_vertical_apertures(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None
    ) -> List[float]:
        """Get the vertical apertures for cameras specific in indices. If indices is None, get for all cameras.

        Emulates sensor/film height on a camera.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).

        Returns:
            list[float]: list containing the focal distances of the cameras.
        """
        vertical_apertures = []
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)

        for i in indices:
            vertical_apertures.append(self.prims[i].GetAttribute("verticalAperture").Get())

        return vertical_apertures

    def set_vertical_apertures(
        self,
        values: List[float],
        indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the vertical apertures for cameras specific in indices. If indices is None, set for all cameras.

        Emulates sensor/film height on a camera.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                    to set. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).
            values (List[float]): list containing the vertical apertures to set for the cameras.
                Length of values must match length of indices.
        """

        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)

        # Ensure length of values matches indices
        if len(indices) != len(values):
            raise ValueError("Length of values must match length of indices")

        for i in indices:
            self.prims[i].GetAttribute("verticalAperture").Set(values[i])

        return

    def get_projection_types(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None
    ) -> List[str]:
        """Get the projection types for cameras specific in indices. If indices is None, get for all cameras.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).

        Returns:
            list[str]: list of projection types (pinhole, fisheyeOrthographic, fisheyeEquidistant, fisheyeEquisolid, fisheyePolynomial or fisheyeSpherical)
        """
        projection_types = []
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)

        for i in indices:
            projection_type = self.prims[i].GetAttribute("cameraProjectionType").Get()
            if projection_type is None:
                projection_type = "pinhole"
            projection_types.append(projection_type)

        return projection_types

    def set_projection_types(
        self,
        values: List[str],
        indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the projection types for cameras specific in indices. If indices is None, set for all cameras.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                    to set. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).
            values (List[str]): list of projection types (pinhole, fisheyeOrthographic, fisheyeEquidistant, fisheyeEquisolid, fisheyePolynomial or fisheyeSpherical)
                Length of values must match length of indices.
        """

        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)

        # Ensure length of values matches indices
        if len(indices) != len(values):
            raise ValueError("Length of values must match length of indices")

        for i in indices:
            self.prims[i].GetAttribute("cameraProjectionType").Set(Vt.Token(values[i]))

        return

    def get_projection_modes(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None
    ) -> List[str]:
        """Get the projection modes for cameras specific in indices. If indices is None, get for all cameras.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).

        Returns:
            list[str]: list of projection modes (perspective, orthographic)
        """
        projection_modes = []
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)

        for i in indices:
            projection_modes.append(self.prims[i].GetAttribute("projection").Get())

        return projection_modes

    def set_projection_modes(
        self,
        values: List[str],
        indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the projection modes for cameras specific in indices. If indices is None, set for all cameras.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                    to set. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).
            values (List[str]): list of projection modes (perspective, orthographic)
                Length of values must match length of indices.
        """

        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)

        # Ensure length of values matches indices
        if len(indices) != len(values):
            raise ValueError("Length of values must match length of indices")

        for i in indices:
            self.prims[i].GetAttribute("projection").Set(values[i])

        return

    def get_stereo_roles(self, indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None) -> List[str]:
        """Get the stereo roles for cameras specific in indices. If indices is None, get for all cameras.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).

        Returns:
            list[str]: list of stereo roles (mono, left, right)
        """
        stereo_roles = []
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)

        for i in indices:
            stereo_roles.append(self.prims[i].GetAttribute("stereoRole").Get())

        return stereo_roles

    def set_stereo_roles(
        self,
        values: List[str],
        indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the stereo roles for cameras specific in indices. If indices is None, set for all cameras.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                    to set. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).
            values (List[str]): list of stereo roles (mono, left, right)
                Length of values must match length of indices.
        """

        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)

        # Ensure length of values matches indices
        if len(indices) != len(values):
            raise ValueError("Length of values must match length of indices")

        for i in indices:
            self.prims[i].GetAttribute("stereoRole").Set(values[i])

        return

    def get_shutter_properties(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None
    ) -> List[Tuple[float, float]]:
        """Get the (delay_open, delay_close) of shutter for cameras specific in indices. If indices is None, get for all cameras.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).

        Returns:
            list[Tuple[float, float]]: list of tuple (delay_open, delay_close)
        """
        shutter_properties = []
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)

        for i in indices:
            shutter_properties.append(
                (self.prims[i].GetAttribute("shutter:open").Get(), self.prims[i].GetAttribute("shutter:close").Get())
            )

        return shutter_properties

    def set_shutter_properties(
        self,
        values: List[Tuple[float, float]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the (delay_open, delay_close) of shutter for cameras specific in indices. If indices is None, set for all cameras.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                    to set. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view).
            values (List[Tuple[float, float]]): list of tuple (delay_open, delay_close)
                Length of values must match length of indices.
        """

        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)

        # Ensure length of values matches indices
        if len(indices) != len(values):
            raise ValueError("Length of values must match length of indices")

        for i in indices:
            delay_open, delay_close = values[i]

            if delay_open:
                self.prims[i].GetAttribute("shutter:open").Set(delay_open)
            if delay_close:
                self.prims[i].GetAttribute("shutter:close").Set(delay_close)

        return
