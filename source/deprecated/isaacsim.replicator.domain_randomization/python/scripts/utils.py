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

"""Provides utility functions for domain randomization in Isaac Sim replicator, including distribution parameter management, semantic data processing, 3D to 2D projection, truncation calculations, and NumPy JSON serialization."""

import json
from typing import Any

import numpy as np
from omni.replicator.core.utils import ReplicatorItem


def set_distribution_params(distribution: ReplicatorItem, parameters: dict) -> None:
    """Updates parameter values of a replicator distribution object.

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
    """Retrieves parameter values from a replicator distribution object.

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


def get_semantics(
    num_semantics: Any,
    num_semantic_tokens: Any,
    instance_semantic_map: Any,
    min_semantic_idx: Any,
    max_semantic_hierarchy_depth: Any,
    semantic_token_map: Any,
    required_semantic_types: Any,
) -> tuple:
    """Extracts and processes semantic information from instance semantic data.

    Processes semantic tokens to build hierarchical relationships between semantic entities and their parents,
    filtering by required semantic types to generate semantic labels and unique identifiers.

    Args:
        num_semantics: Number of semantic entities to process.
        num_semantic_tokens: Number of tokens per semantic entity.
        instance_semantic_map: Array mapping instances to semantic indices.
        min_semantic_idx: Minimum semantic index value for offset calculation.
        max_semantic_hierarchy_depth: Maximum depth of semantic hierarchy.
        semantic_token_map: Array containing semantic token strings.
        required_semantic_types: List of semantic types to filter and include.

    Returns:
        A tuple containing (serialized_labels, semantic_ids, valid_count, prim_paths) where
        serialized_labels is JSON string of semantic mappings, semantic_ids is list of unique identifiers,
        valid_count is number of valid semantic entities, and prim_paths is list of primitive paths.
    """
    instance_to_semantic = instance_semantic_map - min_semantic_idx

    id_to_parents = {}
    # Mapping from a semantic itself to its parents.
    for i in range(0, len(instance_to_semantic), max_semantic_hierarchy_depth):
        curr_semantic_id = instance_to_semantic[i]
        id_to_parents[curr_semantic_id] = []
        for j in range(1, max_semantic_hierarchy_depth):
            parent_semantic_id = instance_to_semantic[i + j]
            if parent_semantic_id != 65535:  # Avoid invalid data
                id_to_parents[curr_semantic_id].append(parent_semantic_id)

    # Mapping from index to semantic labels of each prim
    index_to_labels = {}

    # Iterate through all semantic tokens, and choose those who required by the semantic types.
    valid_semantic_entity_count = 0
    prim_paths = []

    for i in range(num_semantics):
        # TODO Is there a validity check that needs to be performed here?
        is_valid = True
        if is_valid:
            index_to_labels[valid_semantic_entity_count] = {}

            # Find labels of itself and parent labels
            self_labels = semantic_token_map[i * num_semantic_tokens : (i + 1) * num_semantic_tokens]
            parent_labels = []

            if i in id_to_parents:
                for parent_semantic_id in id_to_parents[i]:
                    parent_labels.extend(
                        semantic_token_map[
                            parent_semantic_id * num_semantic_tokens : (parent_semantic_id + 1) * num_semantic_tokens
                        ]
                    )

            all_labels = self_labels + parent_labels

            prim_paths.append(all_labels[0])
            for label_string in all_labels:
                for label in label_string.split(" "):
                    if ":" not in label:
                        continue
                    semantic_type, semantic_data = label.split(":")
                    if semantic_type in required_semantic_types:
                        index_to_labels[valid_semantic_entity_count].setdefault(semantic_type, set()).add(semantic_data)
            # TODO I don't remember why I put this todo here :(   Maybe the bouding box node this code is taken from would provide a clue? Maybe it has to do with semantic filtering?
            valid_semantic_entity_count += 1

    semantic_ids = []
    labels_to_id = {}
    id_to_labels = {}
    id_count = 0

    for index, labels in index_to_labels.items():
        labels_str = str(labels)
        if labels_str not in labels_to_id:
            labels_to_id[labels_str] = id_count
            id_to_labels[id_count] = {}

            for label in labels:
                id_to_labels[id_count] = {k: ",".join(sorted(v)) for k, v in labels.items()}

            semantic_ids.append(id_count)
            id_count += 1
        else:
            semantic_ids.append(labels_to_id[labels_str])

    serialized_index_to_labels = json.dumps(id_to_labels)

    return serialized_index_to_labels, semantic_ids, valid_semantic_entity_count, prim_paths


def get_image_space_points(points: Any, view_proj_matrix: Any) -> np.ndarray:
    """Projects 3D world space points into 2D image space coordinates.

    Args:
        points: Numpy array of N points (N, 3) in the world space. Points will be projected into the image space.
        view_proj_matrix: Desired view projection matrix, transforming points from world frame to image space of desired camera.

    Returns:
        Numpy array of shape (N, 3) of points projected into the image space.
    """
    homo = np.pad(points, ((0, 0), (0, 1)), constant_values=1.0)
    tf_points = np.dot(homo, view_proj_matrix)
    tf_points = tf_points / (tf_points[..., -1:])
    tf_points[..., :2] = 0.5 * (tf_points[..., :2] + 1)
    image_space_points = tf_points[..., :3]

    return image_space_points


def calculate_truncation_ratio_simple(corners: Any, img_width: int, img_height: int) -> float:
    """Calculate the truncation ratio of a cuboid using a simplified bounding box method.

    Args:
        corners: (9, 2) numpy array containing the projected corners of the cuboid.
        img_width: Width of image.
        img_height: Height of image.

    Returns:
        The truncation ratio of the cuboid.
        1 means object is fully truncated and 0 means object is fully within screen.
    """
    # Calculate the bounding box of the cuboid
    x_min, y_min = np.min(corners, axis=0)
    x_max, y_max = np.max(corners, axis=0)

    # Original bounding box area
    original_area = (x_max - x_min) * (y_max - y_min)

    # Clip the bounding box to the screen
    clipped_x_min = min(max(x_min, 0), img_width)
    clipped_y_min = min(max(y_min, 0), img_height)
    clipped_x_max = max(min(x_max, img_width), 0)
    clipped_y_max = max(min(y_max, img_height), 0)

    # Clipped bounding box area
    clipped_area = (clipped_x_max - clipped_x_min) * (clipped_y_max - clipped_y_min)

    # Compute the truncation ratio
    truncation_ratio = 1 - clipped_area / original_area if original_area > 0 else 1

    return truncation_ratio


class NumpyEncoder(json.JSONEncoder):
    """Custom JSON encoder that handles NumPy arrays.

    This encoder extends the standard JSON encoder to properly serialize NumPy arrays by converting them to Python lists,
    making them JSON-serializable. When encountering a NumPy array during JSON serialization, it automatically converts
    the array to a list format that can be included in the JSON output.
    """

    def default(self, obj: Any) -> Any:
        """Converts NumPy arrays to Python lists for JSON serialization.

        Args:
            obj: Object to serialize. If it's a NumPy array, converts to list.

        Returns:
            Python list if obj is a NumPy array, otherwise the result of the parent class default method.
        """
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)
