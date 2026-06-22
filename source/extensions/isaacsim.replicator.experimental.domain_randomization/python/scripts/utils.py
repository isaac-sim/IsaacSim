# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Utility functions and classes for domain randomization operations in Isaac Sim Replicator."""

import json
from typing import Any

import numpy as np
from omni.replicator.core.utils import ReplicatorItem


def set_distribution_params(distribution: ReplicatorItem, parameters: dict) -> None:
    """Set parameters on a replicator distribution object.

    Args:
        distribution: The replicator distribution object to be modified.
        parameters: A dictionary where the keys are the names of the replicator
            distribution parameters and the values are the parameter values
            to be set.

    Raises:
        ValueError: If the distribution does not have the specified parameter.
    """
    node = distribution.node

    for parameter, value in parameters.items():
        attribute_name = "inputs:" + parameter
        if not node.get_attribute_exists(attribute_name):
            raise ValueError(f"This distribution does not have a parameter: `{parameter}`")
        node.get_attribute(attribute_name).set(value)


def get_distribution_params(distribution: ReplicatorItem, parameters: list[str]) -> list:
    """Get parameters from a replicator distribution object.

    Args:
        distribution: A replicator distribution object.
        parameters: A list of the names of the replicator distribution parameters.

    Returns:
        A list of the distribution parameters of the given replicator distribution object.

    Raises:
        ValueError: If the distribution does not have the specified parameter.
    """
    node = distribution.node
    params = []

    for parameter in parameters:
        attribute_name = "inputs:" + parameter
        if not node.get_attribute_exists(attribute_name):
            raise ValueError(f"This distribution does not have a parameter: `{parameter}`")
        value = node.get_attribute(attribute_name).get_array(
            on_gpu=False, get_for_write=False, reserved_element_count=0
        )
        params.append(value)
    return params


def get_image_space_points(points: Any, view_proj_matrix: Any) -> Any:
    """Project world space points into image space.

    Args:
        points: numpy array of N points (N, 3) in the world space.
        view_proj_matrix: Desired view projection matrix.

    Returns:
        numpy array of shape (N, 3) of points projected into the image space.
    """
    homo = np.pad(points, ((0, 0), (0, 1)), constant_values=1.0)
    tf_points = np.dot(homo, view_proj_matrix)
    tf_points = tf_points / (tf_points[..., -1:])
    tf_points[..., :2] = 0.5 * (tf_points[..., :2] + 1)
    image_space_points = tf_points[..., :3]

    return image_space_points


def calculate_truncation_ratio_simple(corners: Any, img_width: Any, img_height: Any) -> Any:
    """Calculate the truncation ratio of a cuboid using a simplified bounding box method.

    Args:
        corners: (9, 2) numpy array containing the projected corners of the cuboid.
        img_width: width of image.
        img_height: height of image.

    Returns:
        The truncation ratio of the cuboid (1 = fully truncated, 0 = fully visible).
    """
    x_min, y_min = np.min(corners, axis=0)
    x_max, y_max = np.max(corners, axis=0)

    original_area = (x_max - x_min) * (y_max - y_min)

    clipped_x_min = min(max(x_min, 0), img_width)
    clipped_y_min = min(max(y_min, 0), img_height)
    clipped_x_max = max(min(x_max, img_width), 0)
    clipped_y_max = max(min(y_max, img_height), 0)

    clipped_area = (clipped_x_max - clipped_x_min) * (clipped_y_max - clipped_y_min)

    truncation_ratio = 1 - clipped_area / original_area if original_area > 0 else 1

    return truncation_ratio


class NumpyEncoder(json.JSONEncoder):
    """JSON encoder that handles numpy arrays."""

    def default(self, obj: Any) -> Any:
        """Convert numpy arrays to JSON-serializable format.

        Args:
            obj: Object to be serialized. If it's a numpy array, converts it to a list.

        Returns:
            JSON-serializable representation of the object.
        """
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)
