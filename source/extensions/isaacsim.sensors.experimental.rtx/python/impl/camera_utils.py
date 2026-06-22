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

"""Utility functions for drawing annotator data to images for visualization and debugging purposes."""

from __future__ import annotations

import io

import matplotlib.pyplot as plt
import numpy as np
import warp as wp


def draw_annotator_data_to_image(
    *, annotator: str, data: wp.array | np.ndarray, info: dict, frame: wp.array | np.ndarray = None
) -> np.ndarray:
    """Draw annotator data (and info) to an image suitable for OpenCV visualization, testing and debugging, for example.

    Args:
        annotator: Annotator type.
        data: Data to draw.
        info: Additional information according to the annotator.
        frame: Frame used as a background for drawing. If not provided, a black frame is used.

    Returns:
        Drawn image as a BGR NumPy array suitable for OpenCV operations.

    Raises:
        ValueError: If the annotator is not supported.

    Example:

    .. code-block:: python

        >>> import cv2
        >>> import isaacsim.core.experimental.utils.app as app_utils
        >>> from isaacsim.sensors.experimental.rtx import CameraSensor, draw_annotator_data_to_image
        >>>
        >>> camera_sensor = CameraSensor(
        ...     "/World/camera",
        ...     resolution=(512, 512),
        ...     annotators=["rgb", "motion_vectors"],
        ... )
        >>>
        >>> # play the simulation so the sensor can fetch data
        >>> app_utils.play(commit=True)
        >>>
        >>> # draw motion vectors on top of a RGB image captured from the camera
        >>> frame, _ = camera_sensor.get_data("rgb")  # capture the current camera view as background
        >>> data, info = camera_sensor.get_data("motion_vectors")
        >>> image = draw_annotator_data_to_image(
        ...     annotator="motion_vectors",
        ...     data=data,
        ...     info=info,
        ...     frame=frame,
        ... )  # doctest: +SKIP
        ...
        ... # save the image to a file
        >>> cv2.imwrite("motion_vectors.png", image)  # doctest: +SKIP
    """
    import cv2

    data = data.numpy() if isinstance(data, wp.array) else data
    if frame is not None:
        frame = frame.numpy() if isinstance(frame, wp.array) else frame
        frame = cv2.cvtColor(frame.astype(np.uint8), cv2.COLOR_RGB2BGR)
    # rgb
    if annotator == "rgb":
        return cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
    # rgba
    elif annotator == "rgba":
        return cv2.cvtColor(data, cv2.COLOR_RGBA2BGRA)
    # depth
    elif annotator in ("distance_to_image_plane", "distance_to_camera", "depth_sensor_distance"):
        data = data.squeeze()
        finite_mask = np.isfinite(data)
        infinite_mask = np.isinf(data)
        if finite_mask.any():
            min_val = data[finite_mask].min()
            max_val = data[finite_mask].max()
            normalized = np.where(
                finite_mask, (data - min_val) / (max_val - min_val) if max_val > min_val else 0.0, 1.0
            )
            data = (1 - normalized) * 255
        if infinite_mask.any():
            data[infinite_mask] = 0
        return data.astype(np.uint8)
    # normals
    elif annotator == "normals":
        normalized = ((data + 1.0) * 0.5 * 255).clip(0, 255).astype(np.uint8)
        return cv2.cvtColor(normalized, cv2.COLOR_RGB2BGR)
    # motion vectors
    elif annotator == "motion_vectors":
        step = 10
        height, width = data.shape[:2]
        y, x = np.mgrid[step // 2 : height : step, step // 2 : width : step].reshape(2, -1).astype(int)
        fx, fy = data[y, x].T
        lines = np.vstack([x, y, x + fx, y + fy]).T.reshape(-1, 2, 2)
        lines = np.int32(lines + 0.5)
        image = frame.copy() if frame is not None else np.zeros((height, width, 3), dtype=np.uint8)
        for (x1, y1), (x2, y2) in lines:
            cv2.arrowedLine(image, (x1, y1), (x2, y2), (0, 255, 0), 1, tipLength=0.3)
        return image
    # semantic/instance segmentation
    elif annotator in ("semantic_segmentation", "instance_segmentation", "instance_id_segmentation"):
        ids = data.squeeze()
        image = frame.copy() if frame is not None else np.zeros((*ids.shape, 3), dtype=np.uint8)
        for uid in np.unique(ids):
            if uid == 0:
                continue
            color = cv2.applyColorMap(np.array([[(uid * 30) % 256]], dtype=np.uint8), cv2.COLORMAP_TURBO)[0, 0]
            image[ids == uid] = color
        return image
    # bounding box 2D
    elif annotator in ("bounding_box_2d_tight", "bounding_box_2d_loose"):
        image = frame.copy() if frame is not None else np.zeros((*info["resolution"], 3), dtype=np.uint8)
        for bbox in data:
            color = cv2.applyColorMap(np.array([[(bbox[0] * 30) % 256]], dtype=np.uint8), cv2.COLORMAP_TURBO)[0, 0]
            cv2.rectangle(
                image,
                (int(bbox["x_min"]), int(bbox["y_min"])),
                (int(bbox["x_max"]), int(bbox["y_max"])),
                color.tolist(),
                2,
            )
        return image
    # bounding box 3D
    elif annotator == "bounding_box_3d":
        min_points = np.array([[bbox["x_min"], bbox["y_min"], bbox["z_min"]] for bbox in data])
        max_points = np.array([[bbox["x_max"], bbox["y_max"], bbox["z_max"]] for bbox in data])
        for index, (min_point, max_point) in enumerate(zip(min_points, max_points)):
            transform = np.array(data[index]["transform"]).reshape(4, 4).T
            min_points[index] = (transform @ np.append(min_point, 1.0))[:3]
            max_points[index] = (transform @ np.append(max_point, 1.0))[:3]
        # generate 3D plot
        fig = plt.figure(figsize=(5, 5), dpi=300)
        ax = fig.add_subplot(111, projection="3d")
        ax.view_init(elev=30, azim=45, roll=0)
        # - compute axes limits
        minimum = np.min(min_points, axis=0)
        maximum = np.max(max_points, axis=0)
        center = 0.5 * (maximum + minimum)
        diff = np.array([0.55 * np.max(maximum - minimum).item()] * 3)
        # - scale view
        ax.set_xlim((center[0] - diff[0], center[0] + diff[0]))
        ax.set_ylim((center[1] - diff[1], center[1] + diff[1]))
        ax.set_zlim((center[2] - diff[2], center[2] + diff[2]))
        ax.set_box_aspect(aspect=diff / diff[0])
        # - plot 3D bounding boxes
        for index, (min_point, max_point) in enumerate(zip(min_points, max_points)):
            color = cv2.applyColorMap(
                np.array([[(data[index]["semanticId"] * 30) % 256]], dtype=np.uint8), cv2.COLORMAP_TURBO
            )[0, 0]
            xmin, ymin, zmin = min_point
            xmax, ymax, zmax = max_point
            vertices = np.array(
                [
                    [xmin, ymin, zmin],
                    [xmax, ymin, zmin],
                    [xmax, ymax, zmin],
                    [xmin, ymax, zmin],
                    [xmin, ymin, zmax],
                    [xmax, ymin, zmax],
                    [xmax, ymax, zmax],
                    [xmin, ymax, zmax],
                ]
            )
            edges = [  # edge indices: (start index, end index)
                (0, 1),
                (1, 2),
                (2, 3),
                (3, 0),
                (4, 5),
                (5, 6),
                (6, 7),
                (7, 4),
                (0, 4),
                (1, 5),
                (2, 6),
                (3, 7),
            ]
            for i, j in edges:
                ax.plot3D(*zip(vertices[i], vertices[j]), color=(color / 255.0).tolist())
        # save the figure to a buffer
        buffer = io.BytesIO()
        fig.savefig(buffer, format="raw", dpi=300)
        buffer.seek(0)
        image = np.reshape(
            np.frombuffer(buffer.getvalue(), dtype=np.uint8),
            newshape=(int(fig.bbox.bounds[3]), int(fig.bbox.bounds[2]), -1),
        )
        buffer.close()
        plt.close(fig)
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    # pointcloud
    elif annotator in ("pointcloud", "depth_sensor_point_cloud_position"):
        if annotator == "depth_sensor_point_cloud_position":
            data = data.reshape(-1, 3)
        # generate 3D plot
        fig = plt.figure(figsize=(5, 5), dpi=300)
        ax = fig.add_subplot(111, projection="3d")
        ax.view_init(elev=30, azim=45, roll=0)
        # - compute axes limits
        minimum = np.min(data, axis=0)
        maximum = np.max(data, axis=0)
        center = 0.5 * (maximum + minimum)
        diff = np.array([0.55 * np.max(maximum - minimum).item()] * 3)
        # - scale view
        ax.set_xlim((center[0] - diff[0], center[0] + diff[0]))
        ax.set_ylim((center[1] - diff[1], center[1] + diff[1]))
        ax.set_zlim((center[2] - diff[2], center[2] + diff[2]))
        ax.set_box_aspect(aspect=diff / diff[0])
        # - draw points
        ax.scatter(data[:, 0], data[:, 1], data[:, 2], s=1)
        # save the figure to a buffer
        buffer = io.BytesIO()
        fig.savefig(buffer, format="raw", dpi=300)
        buffer.seek(0)
        image = np.reshape(
            np.frombuffer(buffer.getvalue(), dtype=np.uint8),
            newshape=(int(fig.bbox.bounds[3]), int(fig.bbox.bounds[2]), -1),
        )
        buffer.close()
        plt.close(fig)
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    # depth-sensor image
    elif annotator == "depth_sensor_imager":
        data = data.squeeze().astype(np.uint8)
        minimum = np.min(data)
        maximum = np.max(data)
        normalized = (data - minimum) / (maximum - minimum + 1e-6)
        return (normalized * 255).astype(np.uint8)
    # depth-sensor point cloud color
    elif annotator == "depth_sensor_point_cloud_color":
        # TODO: implement this
        return np.zeros((1, 1, 1), dtype=np.uint8)

    raise ValueError(f"Unsupported annotator '{annotator}' for rendering")
