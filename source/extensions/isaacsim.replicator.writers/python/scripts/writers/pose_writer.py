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

"""A Replicator writer that outputs pose data including 3D bounding box annotations, camera parameters, and optional debug images for object pose estimation tasks."""

from functools import partial
from typing import Any

import numpy as np
from isaacsim.replicator.writers.scripts.utils import (
    calculate_truncation_ratio_simple,
    project_point_to_screen,
)
from omni.replicator.core import AnnotatorRegistry, Writer
from omni.replicator.core import functional as F
from omni.replicator.core.scripts.backends import BackendDispatch, BackendGroup, BaseBackend
from PIL import Image, ImageDraw
from pxr import Gf

__version__ = "0.2.0"


class PoseWriter(Writer):
    """Pose Writer.

    Args:
        output_dir:
            Output directory string that indicates the directory to save the results when no backend is supplied.
        use_subfolders:
            If True, the writer will create subfolders for each render product, otherwise all data is saved in the same folder.
        visibility_threshold:
            Objects with visibility below this threshold will be skipped.
        skip_empty_frames:
            If True, the writer will skip frames that do not have visible objects.
        write_debug_images:
            If True, the writer will include rgb images overlaid with the projected 3d bounding boxes.
        frame_padding:
            Pad the frame number with leading zeroes.
        format:
            Specifies which format the data will be outputted as.
        use_s3:
            If True, uses S3 backend for data storage.
        s3_bucket:
            S3 bucket name when using S3 backend.
        s3_endpoint_url:
            S3 endpoint URL when using S3 backend.
        s3_region:
            S3 region when using S3 backend.
        backend:
            Backend instance used to handle I/O scheduling. Prefer using this argument, for example with
            ``rep.backends.get("DiskBackend")``.
        image_output_format:
            Image file format for RGB and debug outputs (``jpeg``, ``jpg``, ``png`` or ``exr``).
    """

    RGB_ANNOT_NAME = "rgb"
    """Name of the RGB annotator used for retrieving color image data."""
    BB3D_ANNOT_NAME = "bounding_box_3d_fast"
    """Name of the 3D bounding box annotator used for retrieving object pose and geometry data."""
    CAM_PARAMS_ANNOT_NAME = "camera_params"
    """Name of the camera parameters annotator used for retrieving camera intrinsic and extrinsic data."""
    SUPPORTED_FORMATS = {"dope", "centerpose"}
    """Set of supported output formats for pose data."""
    CUBOID_KEYPOINTS_ORDER_DEFAULT = ["Center", "LDB", "LDF", "LUB", "LUF", "RDB", "RDF", "RUB", "RUF"]
    """Default ordering of cuboid keypoints for 3D bounding box representation."""
    CUBOID_KEYPOINT_ORDER_DOPE = ["LUF", "RUF", "RDF", "LDF", "LUB", "RUB", "RDB", "LDB", "Center"]
    """DOPE format ordering of cuboid keypoints for 3D bounding box representation."""
    CUBOID_KEYPOINT_COLORS = ["white", "red", "green", "blue", "yellow", "cyan", "magenta", "orange", "purple"]
    """Colors used for drawing cuboid keypoints in debug visualization images."""
    CUBOID_EDGE_COLORS = {"front": "red", "back": "blue", "connecting": "green"}
    """Colors used for drawing different types of cuboid edges in debug visualization images."""

    def __init__(
        self,
        output_dir: str = None,
        use_subfolders: bool = False,
        visibility_threshold: float = 0.0,
        skip_empty_frames: bool = True,
        write_debug_images: bool = False,
        frame_padding: int = 6,
        format: str = None,
        use_s3: bool = False,
        s3_bucket: str = None,
        s3_endpoint_url: str = None,
        s3_region: str = None,
        backend: BaseBackend = None,
        image_output_format: str = "png",
    ) -> None:
        self.version = __version__
        self.data_structure = "renderProduct"

        if backend is not None and not isinstance(backend, BaseBackend):
            raise TypeError("`backend` must inherit from omni.replicator.core.scripts.backends.BaseBackend.")

        if not image_output_format:
            raise ValueError("`image_output_format` must be a non-empty string.")

        self._image_output_format = image_output_format.lower()
        if self._image_output_format not in {"jpeg", "jpg", "png", "exr"}:
            raise ValueError("Unsupported `image_output_format`. Valid options are {'jpeg', 'jpg', 'png', 'exr'}.")

        dispatch_backend = None
        if use_s3:
            dispatch_backend = BackendDispatch(
                key_prefix=output_dir, bucket=s3_bucket, endpoint_url=s3_endpoint_url, region=s3_region
            )
        elif output_dir:
            dispatch_backend = BackendDispatch(output_dir=output_dir)

        if backend and dispatch_backend:
            if isinstance(backend, BackendGroup):
                existing_backends = list(backend._backends)
            else:
                existing_backends = [backend]
            self.backend = BackendGroup([*existing_backends, *dispatch_backend._backends])
        elif backend:
            self.backend = backend
        elif dispatch_backend:
            self.backend = dispatch_backend
        else:
            raise ValueError(
                "No backend configured. Provide `backend`, `output_dir`, or enable S3 parameters via `use_s3`."
            )

        self._use_subfolders = use_subfolders
        self._visibility_threshold = visibility_threshold
        self._skip_empty_frames = skip_empty_frames
        self._write_debug_images = write_debug_images
        self._frame_padding = frame_padding
        self._frame_id = 0
        if format is not None and format.lower() not in self.SUPPORTED_FORMATS:
            raise ValueError(f"Unsupported format: {format}. Supported formats: {self.SUPPORTED_FORMATS}")
        else:
            self._format = format

        # Store processed data to be written every frame in the selected format
        self._frame_data = {}

        # Store debug related data to write overlay images (projected cuboid, local frame axes, etc.)
        self._debug_frame_data = {}

        if self._format == "dope":
            self._cuboid_keypoints_order = self.CUBOID_KEYPOINT_ORDER_DOPE
        else:
            self._cuboid_keypoints_order = self.CUBOID_KEYPOINTS_ORDER_DEFAULT

        # For more details: https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/annotators_details.html
        self.annotators = []
        self.annotators.append(AnnotatorRegistry.get_annotator(self.RGB_ANNOT_NAME))
        self.annotators.append(AnnotatorRegistry.get_annotator(self.BB3D_ANNOT_NAME))
        self.annotators.append(AnnotatorRegistry.get_annotator(self.CAM_PARAMS_ANNOT_NAME))

    # Abstract method from Writer to access the annotator data and write to disk
    def write(self, data: dict) -> None:
        """Write pose data for all render products in the frame.

        Args:
            data: Dictionary containing render product data with annotator information.
        """
        # Iterate over the render products
        for rp_name, annotators_data in data["renderProducts"].items():

            # Process the frame data of the current render product
            bounding_box_3d_data = annotators_data[self.BB3D_ANNOT_NAME]
            camera_params_data = annotators_data[self.CAM_PARAMS_ANNOT_NAME]
            num_objs = self._process_frame_data(bounding_box_3d_data, camera_params_data)

            # Early exist if empty frames should not be written
            if self._skip_empty_frames and num_objs == 0:
                continue

            # Create render product name subfolder if data should be separated for each render product
            rp_subfolder = f"{rp_name}/" if self._use_subfolders else ""

            # Write frame data to disk
            rgb_data = annotators_data[self.RGB_ANNOT_NAME]["data"]
            self._write_frame_data(rgb_data, rp_subfolder)
            if self._write_debug_images:
                self._write_debug_data(rgb_data, rp_subfolder)

            # If render products are NOT separated into subfolders increment the frame id after processing each render product
            if not self._use_subfolders:
                self._frame_id += 1

        # If render products are separated into subfolders increment the frame id after processing all render products
        if self._use_subfolders:
            self._frame_id += 1

    # Note that the frame id can be incremented for each render product write (if use_subfolders is False) or for each step (if use_subfolders is True)
    def get_current_frame_id(self) -> Any:
        """Current frame ID counter.

        Returns:
            The current frame ID counter value.
        """
        return self._frame_id

    # Process the render product data and store it in the selected format, return the number of objects in the frame
    def _process_frame_data(self, bounding_box_3d_data: dict, camera_params_data: dict) -> int:
        """Process frame data and stores it in the selected format.

        Args:
            bounding_box_3d_data: 3D bounding box annotator data containing object information.
            camera_params_data: Camera parameters annotator data containing camera configuration.

        Returns:
            Number of visible objects in the frame.
        """
        # Store the frame data for writing to disk
        self._frame_data = {}

        # Get and process the bounding box 3d annotator data in the selected format
        objs_data = self._process_bounding_boxes(bounding_box_3d_data, camera_params_data)

        # Early exist if empty frames should be skipped and there are no visible objects in the frame
        if self._skip_empty_frames and len(objs_data) == 0:
            return 0

        # Store the camera information in the
        self._frame_data["camera_data"] = self._process_camera_parameters(camera_params_data)

        # Store the predefined order of the cuboid keypoints
        self._frame_data["keypoint_order"] = self._cuboid_keypoints_order

        # Add the objects data to the frame entries
        self._frame_data["objects"] = objs_data

        return len(objs_data)

    # Process the bounding box annotator data (extract objects label, location, rotation, visibility, etc.)
    def _process_bounding_boxes(self, bounding_box_3d_data: dict, camera_params: dict) -> list:
        """Process 3D bounding box data to extract object pose information.

        Args:
            bounding_box_3d_data: 3D bounding box annotator data containing object information.
            camera_params: Camera parameters for projection calculations.

        Returns:
            List of processed object data with pose, keypoints, and visibility information.
        """
        # Map the ids to class names from the bbox annotator "idToLabels" data
        # ('idToLabels': {0: {'class': 'cube'}, 1: {'class': 'sphere'}} -> {0: 'cube', 1: 'sphere'})
        id_to_labels = {k: v["class"] for k, v in bounding_box_3d_data["idToLabels"].items()}

        if self._write_debug_images:
            self._debug_frame_data["world_frame_transforms"] = []
            self._debug_frame_data["projected_keypoints"] = []
            self._debug_frame_data["size_local"] = []
            self._debug_frame_data["center_local"] = []
        # Iterate the bounding box data and extract the object informations
        objs = []
        for i, bbox in enumerate(bounding_box_3d_data["data"]):
            obj = {}
            # `occlusionRatio` represents (visible pixels / total pixels) where `0.0` is fully visible and `1.0` is fully occluded
            # NOTE: `obj_visibility` is inverted to match the format where `0.0` is fully occluded and `1.0`` is fully visible
            obj_visibility = 1.0 - abs(float(bbox["occlusionRatio"]))

            # Early exit if visibility is below the given threshold
            if obj_visibility <= self._visibility_threshold:
                continue

            if self._format == "dope":
                obj["class"] = id_to_labels[bbox["semanticId"]]
            else:
                obj["label"] = id_to_labels[bbox["semanticId"]]
            obj["prim_path"] = bounding_box_3d_data["primPaths"][i]
            obj["visibility"] = round(obj_visibility, 3)

            # Local space to to world transform (row-major)
            local_to_world_tf = bbox["transform"]

            if self._format is None:
                obj["local_to_world_transform"] = local_to_world_tf.tolist()
                # Extract world frame location (last row) and rotation matrix (3x3) from the row-major transform matrix
                location_world_frame = local_to_world_tf[3, :3]
                obj["location_world_frame"] = location_world_frame.tolist()
                rotation_matrix_world_frame = local_to_world_tf[:3, :3]
                obj["rotation_matrix_world_frame"] = rotation_matrix_world_frame.tolist()

                # Get the world frame quaternion using Gf.Transform (row-major)
                local_to_world_tf_gf = Gf.Transform()
                local_to_world_tf_gf.SetMatrix(Gf.Matrix4d(local_to_world_tf.tolist()))
                quat_world_frame_gf = local_to_world_tf_gf.GetRotation().GetQuat()
                obj["quat_wxyz_world_frame"] = [quat_world_frame_gf.GetReal()] + list(
                    quat_world_frame_gf.GetImaginary()
                )
            if self._write_debug_images:
                self._debug_frame_data["world_frame_transforms"].append(local_to_world_tf)

            # World to camera transform (row-major) (transform a point from world coordinate to camera coordinate)
            world_to_camera_tf = camera_params["cameraViewTransform"].reshape(4, 4)
            # Object world space to camera frame transform (row-major matrix multiplication)
            obj_to_camera_tf = world_to_camera_tf @ local_to_world_tf
            # Extract camera frame location (last row) and rotation matrix (3x3) from the row-major transform matrix
            location_camera_frame = obj_to_camera_tf[3, :3]
            if self._format is None:
                obj["location_camera_frame"] = location_camera_frame.tolist()
            elif self._format == "centerpose" or self._format == "dope":
                obj["location"] = location_camera_frame.tolist()
            if self._format is None:
                rotation_matrix_camera_frame = obj_to_camera_tf[:3, :3]
                obj["rotation_matrix_camera_frame"] = rotation_matrix_camera_frame.tolist()
            # Get the camera frame quaternion using Gf.Transform (row-major)
            obj_to_camera_tf_gf = Gf.Transform()
            obj_to_camera_tf_gf.SetMatrix(Gf.Matrix4d(obj_to_camera_tf.tolist()))
            quat_camera_frame_gf = obj_to_camera_tf_gf.GetRotation().GetQuat()
            if self._format is None:
                obj["quat_wxyz_camera_frame"] = [quat_camera_frame_gf.GetReal()] + list(
                    quat_camera_frame_gf.GetImaginary()
                )
            elif self._format == "centerpose" or self._format == "dope":
                obj["quaternion_xyzw"] = list(quat_camera_frame_gf.GetImaginary()) + [quat_camera_frame_gf.GetReal()]

            # Size of the object before scale (NOTE: scale is not applied yet to objects in local frame)
            min_local = np.array([bbox["x_min"], bbox["y_min"], bbox["z_min"], 1])
            max_local = np.array([bbox["x_max"], bbox["y_max"], bbox["z_max"], 1])
            size_local = np.abs(max_local - min_local)[:3]
            center_local = min_local + (max_local - min_local) / 2
            if self._write_debug_images:
                self._debug_frame_data["size_local"].append(size_local.tolist())
                self._debug_frame_data["center_local"].append(center_local[:3].tolist())

            # Cuboid keypoints in local frame
            keypoints_local = {
                "Center": center_local,
                "LDB": np.array([bbox["x_min"], bbox["y_min"], bbox["z_min"], 1]),  # Left-Down-Back
                "LDF": np.array([bbox["x_min"], bbox["y_min"], bbox["z_max"], 1]),  # Left-Down-Front
                "LUB": np.array([bbox["x_min"], bbox["y_max"], bbox["z_min"], 1]),  # Left-Up-Back
                "LUF": np.array([bbox["x_min"], bbox["y_max"], bbox["z_max"], 1]),  # Left-Up-Front
                "RDB": np.array([bbox["x_max"], bbox["y_min"], bbox["z_min"], 1]),  # Right-Down-Back
                "RDF": np.array([bbox["x_max"], bbox["y_min"], bbox["z_max"], 1]),  # Right-Down-Front
                "RUB": np.array([bbox["x_max"], bbox["y_max"], bbox["z_min"], 1]),  # Right-Up-Back
                "RUF": np.array([bbox["x_max"], bbox["y_max"], bbox["z_max"], 1]),  # Right-Up-Front
            }

            # Calculate the oriented bounding box (OBB) size by combining local size with world scale
            local_to_world_tf_gf = Gf.Transform()
            local_to_world_tf_gf.SetMatrix(Gf.Matrix4d(local_to_world_tf.tolist()))
            world_scale = local_to_world_tf_gf.GetScale()
            size_obb = (size_local * world_scale).tolist()
            if self._format is None:
                obj["size"] = size_obb
            elif self._format == "centerpose":
                obj["scale"] = size_obb

            # Transform the cuboid keypoints from local to world frame in the given order
            keypoints_world_ordered = [keypoints_local[k] @ local_to_world_tf for k in self._cuboid_keypoints_order]
            if self._format is None:
                obj["cuboid_keypoints_world_frame"] = [point[:3].tolist() for point in keypoints_world_ordered]
            # Transform the cuboid keypoints from world to camera frame
            keypoints_camera_ordered = [point @ world_to_camera_tf for point in keypoints_world_ordered]
            if self._format is None:
                obj["cuboid_keypoints_camera_frame"] = [point[:3].tolist() for point in keypoints_camera_ordered]
            elif self._format == "centerpose":
                obj["keypoints_3d"] = [point[:3].tolist() for point in keypoints_camera_ordered]
            # Project the cuboid keypoints to screen space using the appropriate camera model
            screen_size = camera_params["renderProductResolution"]
            keypoints_projected_ordered = [
                project_point_to_screen(point, camera_params) for point in keypoints_camera_ordered
            ]
            if self._format is None:
                obj["cuboid_keypoints_projected"] = keypoints_projected_ordered
            elif self._format == "centerpose" or self._format == "dope":
                obj["projected_cuboid"] = keypoints_projected_ordered
            if self._write_debug_images:
                self._debug_frame_data["projected_keypoints"].append(keypoints_projected_ordered)

            obj["truncation_ratio"] = calculate_truncation_ratio_simple(
                keypoints_projected_ordered, screen_size[0], screen_size[1]
            )

            objs.append(obj)

        return objs

    # Get the camera parameters from the annotator data
    def _process_camera_parameters(self, camera_params: Any) -> dict:
        """Process camera parameters from annotator data.

        Args:
            camera_params: Raw camera parameters from the annotator.

        Returns:
            Processed camera data including intrinsics, view matrix, and projection matrix.
        """
        camera_data = {}
        if self._format is None:
            camera_data["aperture"] = camera_params["cameraAperture"].tolist()
            camera_data["aperture_offset"] = camera_params["cameraApertureOffset"].tolist()
            camera_data["focal_length"] = float(camera_params["cameraFocalLength"])
            camera_data["resolution"] = camera_params["renderProductResolution"].tolist()
            camera_data["meters_per_scene_unit"] = float(camera_params["metersPerSceneUnit"])

        # OV only supports square pixels, so the pixel size is the same in both x and y directions
        # https://docs.omniverse.nvidia.com/materials-and-rendering/latest/cameras.html#cameras
        pixel_size = camera_params["cameraAperture"][0] / camera_params["renderProductResolution"][0]
        camera_data["intrinsics"] = {
            "fx": camera_params["cameraFocalLength"] / pixel_size,
            "fy": camera_params["cameraFocalLength"] / pixel_size,
            "cx": camera_params["renderProductResolution"][0] / 2.0 + camera_params["cameraApertureOffset"][0],
            "cy": camera_params["renderProductResolution"][1] / 2.0 + camera_params["cameraApertureOffset"][1],
        }
        camera_data["camera_view_matrix"] = np.round(camera_params["cameraViewTransform"], 5).reshape(4, 4).tolist()
        camera_data["camera_projection_matrix"] = np.round(camera_params["cameraProjection"], 5).reshape(4, 4).tolist()

        if self._format == "centerpose":
            camera_data["width"] = camera_params["renderProductResolution"].tolist()[0]
            camera_data["height"] = camera_params["renderProductResolution"].tolist()[1]

        # Debug data needed for the overlay projections
        if self._write_debug_images:
            self._debug_frame_data["camera_params"] = camera_params

        return camera_data

    # Write the processed data to disk
    def _write_frame_data(self, rgb_data: dict, render_product_subfolder: str = "") -> None:
        """Write frame data and RGB image to disk.

        Args:
            rgb_data: RGB image data to be written.
            render_product_subfolder: Subfolder path for the render product.
        """
        # Write frame data to as a JSON file
        file_path_json = f"{render_product_subfolder}{self._frame_id:0{self._frame_padding}}.json"
        self.backend.schedule(F.write_json, path=file_path_json, data=self._frame_data, indent=2)

        # Write image to disk
        rgb_file_path = f"{render_product_subfolder}{self._frame_id:0{self._frame_padding}}.{self._image_output_format}"
        self.backend.schedule(F.write_image, path=rgb_file_path, data=rgb_data)

    # Write overlay debug data to disk
    def _write_debug_data(self, rgb_data: dict, render_product_subfolder: str = "") -> None:
        """Write debug overlay image with projected keypoints and coordinate axes.

        Args:
            rgb_data: RGB image data to overlay debug information on.
            render_product_subfolder: Subfolder path for the render product.
        """
        # Create overlay image from the RGB data
        rgb_img = Image.fromarray(rgb_data)
        draw = ImageDraw.Draw(rgb_img)

        # Draw the projected cuboid and its edges
        for keypoints in self._debug_frame_data["projected_keypoints"]:
            self._draw_projected_keypoints(draw, keypoints)

        # Get the stored camera parameters for debug purposes
        camera_params = self._debug_frame_data["camera_params"]

        # Draw objects local frame axes
        for i, tf in enumerate(self._debug_frame_data["world_frame_transforms"]):
            size = self._debug_frame_data["size_local"][i]
            center = self._debug_frame_data["center_local"][i]
            self._draw_local_frame_axes(
                draw,
                tf,
                camera_params,
                size_local=size,
                origin_local=center,
            )

        # Overlay the world frame axes on the bottom left part of the RGB image
        self._draw_world_frame_axes_bottom_left(draw, camera_params)

        file_path = (
            f"{render_product_subfolder}{self._frame_id:0{self._frame_padding}}_overlay.{self._image_output_format}"
        )
        overlay_data = np.asarray(rgb_img) if self._image_output_format == "exr" else rgb_img
        self.backend.schedule(F.write_image, path=file_path, data=overlay_data)

    # Transform a 3D point from world coordinates to camera coordinates
    def _world_point_to_camera_point(self, world_point: Any, view_matrix: Any) -> Any:
        """Transform a 3D point from world coordinates to camera coordinates.

        Args:
            world_point: 3D point in world coordinates.
            view_matrix: Camera view transformation matrix.

        Returns:
            Point transformed to camera coordinate space.
        """
        # Convert the 3D point to homogeneous coordinates (if not already in that form)
        point_homogeneous = np.array(world_point) if len(world_point) == 4 else np.array([*world_point, 1.0])

        # Transform to camera frame (row-major representation where the translation vector is on the left side of the multiplication)
        point_camera = point_homogeneous @ view_matrix

        return point_camera

    def _project_world_point_to_screen(self, world_point: Any, camera_params: Any) -> Any:
        """Project a 3D world point to 2D screen coordinates.

        Args:
            world_point: 3D point in world coordinates to project.
            camera_params: Camera parameters for projection calculations.

        Returns:
            2D screen coordinates of the projected point.
        """
        view_matrix = camera_params["cameraViewTransform"].reshape(4, 4)
        point_camera = self._world_point_to_camera_point(world_point, view_matrix)
        return project_point_to_screen(point_camera, camera_params)

    # Projects the local frame axes of the object to the screen
    def _draw_local_frame_axes(
        self,
        draw: Any,
        local_to_world_transform: Any,
        camera_params: Any,
        size_local: Any | None = None,
        origin_local: Any | None = None,
        axes_length_perc: float = 0.25,
    ) -> None:
        """Draw local coordinate frame axes of an object projected onto the screen.

        Args:
            draw: ImageDraw object for rendering the axes.
            local_to_world_transform: Transformation matrix from local to world coordinates.
            camera_params: Camera parameters for projection calculations.
            size_local: Local size of the object for scaling axes length.
            origin_local: Local origin point of the coordinate frame.
            axes_length_perc: Percentage of mean object size to use for axes length.
        """
        if size_local is None:
            size_local = [1, 1, 1]
        if origin_local is None:
            origin_local = [0, 0, 0]

        # The length of the local axes is a percentage of the mean size of the object in local frame (before any scaling)
        local_axes_length = np.mean(size_local) * axes_length_perc

        # Define the end points of the local coordinate system axes include the local center of the object bounds
        origin_local = np.array([origin_local[0], origin_local[1], origin_local[2], 1])
        x_axis_end_point_local = np.array([local_axes_length + origin_local[0], origin_local[1], origin_local[2], 1])
        y_axis_end_point_local = np.array([origin_local[0], local_axes_length + origin_local[1], origin_local[2], 1])
        z_axis_end_point_local = np.array([origin_local[0], origin_local[1], local_axes_length + origin_local[2], 1])

        # Transform local end points to world frame using row-major matrix multiplication (translation on the left side)
        origin_world = origin_local @ local_to_world_transform
        x_axis_end_point_world = x_axis_end_point_local @ local_to_world_transform
        y_axis_end_point_world = y_axis_end_point_local @ local_to_world_transform
        z_axis_end_point_world = z_axis_end_point_local @ local_to_world_transform

        # Define a partial helper function to project 3D world points to 2D screen points
        project_to_screen = partial(self._project_world_point_to_screen, camera_params=camera_params)

        # Project the origin and axes end points from 3D world coordinates to 2D screen coordinates
        origin_2d = project_to_screen(origin_world)
        x_axis_end_2d = project_to_screen(x_axis_end_point_world)
        y_axis_end_2d = project_to_screen(y_axis_end_point_world)
        z_axis_end_2d = project_to_screen(z_axis_end_point_world)

        # Draw the 3D axes on the 2D screen using lines with appropriate colors for each axis
        draw.line([origin_2d, x_axis_end_2d], fill="red", width=2)  # X-axis in red
        draw.line([origin_2d, y_axis_end_2d], fill="green", width=2)  # Y-axis in green
        draw.line([origin_2d, z_axis_end_2d], fill="blue", width=2)  # Z-axis in blue

    # Draws the world frame axes at the bottom left corner of the image.
    def _draw_world_frame_axes_bottom_left(
        self, draw: Any, camera_params: Any, axes_scale: float = 0.03, margin_percentage: float = 0.03
    ) -> None:
        """Draw the world coordinate system axes at the bottom-left corner of the image.

        Args:
            draw: ImageDraw instance for drawing on the image.
            camera_params: Camera parameters containing view transform and resolution data.
            axes_scale: Scale factor for the axes length in world units.
            margin_percentage: Margin as a percentage of screen size to offset axes from the edge.
        """
        camera_view_matrix = camera_params["cameraViewTransform"].reshape(4, 4)
        screen_size = camera_params["renderProductResolution"]

        # Set a world location for the axes origin (1 unit in front of the camera) where -Z is the camera's forward direction
        camera_to_world_matrix = np.linalg.inv(camera_view_matrix)
        point_in_camera_space = np.array([0, 0, -1, 1])

        # Create the axes in world (1 unit in front of the camera) with the given axes size
        origin_world = point_in_camera_space @ camera_to_world_matrix
        x_axis_end_point_world = np.array([axes_scale + origin_world[0], origin_world[1], origin_world[2], 1])
        y_axis_end_point_world = np.array([origin_world[0], axes_scale + origin_world[1], origin_world[2], 1])
        z_axis_end_point_world = np.array([origin_world[0], origin_world[1], axes_scale + origin_world[2], 1])

        # Create a partial function with fixed camera parameters
        project_to_screen = partial(self._project_world_point_to_screen, camera_params=camera_params)

        # Project the origin and axes end points into 2D screen coordinates
        origin_2d = project_to_screen(origin_world)
        x_axis_end_2d = project_to_screen(x_axis_end_point_world)
        y_axis_end_2d = project_to_screen(y_axis_end_point_world)
        z_axis_end_2d = project_to_screen(z_axis_end_point_world)

        # Calculate offset margin (a percentage of the screen size) to ensure axes are not on the edge of the screen
        margin = int(margin_percentage * min(screen_size))
        offset_x = margin - min(origin_2d[0], x_axis_end_2d[0], y_axis_end_2d[0], z_axis_end_2d[0])
        offset_y = screen_size[1] - margin - max(origin_2d[1], x_axis_end_2d[1], y_axis_end_2d[1], z_axis_end_2d[1])

        # Apply the offset to the projected points
        origin_2d = (origin_2d[0] + offset_x, origin_2d[1] + offset_y)
        x_axis_end_2d = (x_axis_end_2d[0] + offset_x, x_axis_end_2d[1] + offset_y)
        y_axis_end_2d = (y_axis_end_2d[0] + offset_x, y_axis_end_2d[1] + offset_y)
        z_axis_end_2d = (z_axis_end_2d[0] + offset_x, z_axis_end_2d[1] + offset_y)

        # Draw the axes with the specified colors
        draw.line([origin_2d, x_axis_end_2d], fill="red", width=2)  # X-axis in red
        draw.line([origin_2d, y_axis_end_2d], fill="green", width=2)  # Y-axis in green
        draw.line([origin_2d, z_axis_end_2d], fill="blue", width=2)  # Z-axis in blue

    # Draw the projected cuboid and its edges
    def _draw_projected_keypoints(self, draw: Any, keypoints: Any, point_size: int = 4, edge_size: int = 2) -> None:
        """Draw the projected cuboid keypoints and edges on the image.

        Args:
            draw: ImageDraw instance for drawing on the image.
            keypoints: List of projected keypoint coordinates.
            point_size: Radius of the keypoint circles in pixels.
            edge_size: Width of the edge lines in pixels.
        """
        # Draw the projected cuboid keypoint vertices in the specified colors
        for i, point in enumerate(keypoints):
            draw.ellipse(
                (point[0] - point_size, point[1] - point_size, point[0] + point_size, point[1] + point_size),
                fill=self.CUBOID_KEYPOINT_COLORS[i],
            )

        # Draw the edges of the projected cuboid with specified colors for each set
        edges = {
            "front": [(1, 2), (2, 4), (4, 3), (3, 1)],  # Front face
            "back": [(5, 6), (6, 8), (8, 7), (7, 5)],  # Back face
            "connecting": [(1, 5), (2, 6), (3, 7), (4, 8)],  # Connecting edges
        }
        for edge_type, edge_list in edges.items():
            for start, end in edge_list:
                draw.line(keypoints[start] + keypoints[end], fill=self.CUBOID_EDGE_COLORS[edge_type], width=edge_size)

    # Override to clear the writer state
    def detach(self) -> None:
        """Clear the writer state by resetting the frame counter to zero."""
        super().detach()
        self._frame_id = 0
