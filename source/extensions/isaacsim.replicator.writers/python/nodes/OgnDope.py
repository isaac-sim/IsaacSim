# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""This is the implementation of the OGN node defined in OgnDope.ogn."""

import json
from typing import Any

import isaacsim.core.experimental.utils.transform as transform_utils
import numpy as np
import omni.graph.core as og
from isaacsim.replicator.writers.scripts.utils import get_image_space_points, pose_from_tf_matrix, tf_matrix_from_pose
from omni.syntheticdata.scripts.helpers import get_bbox_3d_corners


def _get_semantics(
    num_semantics: Any,
    num_semantic_tokens: Any,
    instance_semantic_map: Any,
    min_semantic_idx: Any,
    max_semantic_hierarchy_depth: Any,
    semantic_token_map: Any,
    required_semantic_types: Any,
) -> Any:
    """Process semantic data and return labels mapping.

    Args:
        num_semantics: Number of semantic entities.
        num_semantic_tokens: Number of tokens per semantic entity.
        instance_semantic_map: Mapping from instance to semantic index.
        min_semantic_idx: Minimum semantic index offset.
        max_semantic_hierarchy_depth: Maximum depth of semantic hierarchy.
        semantic_token_map: List of semantic token strings.
        required_semantic_types: List of semantic types to extract.

    Returns:
        Tuple of (serialized_index_to_labels, semantic_ids, valid_count, prim_paths).
    """
    instance_to_semantic = instance_semantic_map - min_semantic_idx

    id_to_parents = {}
    for i in range(0, len(instance_to_semantic), max_semantic_hierarchy_depth):
        curr_semantic_id = instance_to_semantic[i]
        id_to_parents[curr_semantic_id] = []
        for j in range(1, max_semantic_hierarchy_depth):
            parent_semantic_id = instance_to_semantic[i + j]
            if parent_semantic_id != 65535:
                id_to_parents[curr_semantic_id].append(parent_semantic_id)

    index_to_labels = {}
    valid_semantic_entity_count = 0
    prim_paths = []

    for i in range(num_semantics):
        is_valid = True
        if is_valid:
            index_to_labels[valid_semantic_entity_count] = {}

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


class OgnDope:
    """Get pose information of assets with semantic labels. Information is used to train a DOPE model."""

    @staticmethod
    def compute(db: Any) -> bool:
        """Compute the outputs from the current input.

        Args:
            db: Database object containing node inputs and outputs.

        Returns:
            True when the node computation completes.
        """
        db.log_warn("Deprecation warning: OgnDope has been deprecated and will be removed in the next major release.")

        return_data_dtype = np.dtype(
            [
                ("semanticId", "<u4"),
                ("visibility", "<f4"),
                ("location", "<f4", (3,)),
                ("rotation", "<f4", (4,)),  # Quaternion
                ("projected_cuboid", "<f4", (9, 2)),  # Includes Center
            ]
        )

        bbox_3d_dtype = np.dtype(
            [
                ("semanticId", "<u4"),
                ("x_min", "<f4"),
                ("y_min", "<f4"),
                ("z_min", "<f4"),
                ("x_max", "<f4"),
                ("y_max", "<f4"),
                ("z_max", "<f4"),
                ("transform", "<f4", (4, 4)),  # Local to World Transform Matrix
                ("occlusionRatio", "<f4"),
            ]
        )

        bboxes_3d = np.frombuffer(db.inputs.boundingBox3d.tobytes(), dtype=bbox_3d_dtype)

        # Semantics
        num_semantics = db.inputs.sdIMNumSemantics
        num_semantic_tokens = db.inputs.sdIMNumSemanticTokens

        if num_semantics == 0:
            return True

        instance_semantic_map = db.inputs.sdIMInstanceSemanticMap.view(np.uint16)
        min_semantic_idx = db.inputs.sdIMMinSemanticIndex
        max_semantic_hierarchy_depth = db.inputs.sdIMMaxSemanticHierarchyDepth
        semantic_token_map = db.inputs.sdIMSemanticTokenMap

        required_semantic_types = db.inputs.semanticTypes

        serialized_index_to_labels, _, _, _ = _get_semantics(
            num_semantics,
            num_semantic_tokens,
            instance_semantic_map,
            min_semantic_idx,
            max_semantic_hierarchy_depth,
            semantic_token_map,
            required_semantic_types,
        )

        # Get Camera Parameters
        cameraViewTransform = db.inputs.cameraViewTransform.reshape((4, 4))
        cameraProjection = db.inputs.cameraProjection

        # Desired view projection matrix, transforming points from world frame to image space of desired camera
        default_camera_to_desired_camera = tf_matrix_from_pose(
            translation=(0.0, 0.0, 0.0),
            orientation=transform_utils.euler_angles_to_quaternion(
                np.array(db.inputs.cameraRotation), degrees=True
            ).numpy(),
        )

        world_to_default_camera_row_major = np.asarray(cameraViewTransform).reshape((4, 4))
        default_camera_to_image_row_major = np.asarray(cameraProjection).reshape((4, 4))

        world_to_default_camera_row_major = np.asarray(cameraViewTransform).reshape((4, 4))
        world_to_default_camera = np.transpose(world_to_default_camera_row_major)

        all_cuboid_points = get_bbox_3d_corners(bboxes_3d)

        semantic_ids = []
        visibilities = []
        locations = []
        rotations = []
        projected_cuboids = []

        for idx, bbox in enumerate(bboxes_3d):
            semantic_ids.append(bbox["semanticId"])
            prim_to_world = np.copy(bbox["transform"])  # Row Major

            center = prim_to_world[-1][:3]

            # Get Center and Rotation of object in camera frame
            prim_to_world[:-1, :-1] = prim_to_world[:-1, :-1] * 100.0
            prim_to_world = np.transpose(prim_to_world)  # Convert to Column Major

            prim_to_default_camera = world_to_default_camera @ prim_to_world

            prim_to_desired_camera = default_camera_to_desired_camera @ prim_to_default_camera

            # location, rotation in 3D camera space
            location, rotation = pose_from_tf_matrix(prim_to_desired_camera)

            locations.append(location * 100)  # Convert to cm
            rotations.append(rotation)

            # Get Cuboid Points of object in image frame
            cuboid_points = np.concatenate((all_cuboid_points[idx], center.reshape(1, 3)))

            # Default view projection matrix, transforming points from world frame to image space of default camera
            world_to_default_image = world_to_default_camera_row_major @ default_camera_to_image_row_major

            default_camera_to_desired_camera_row_major = np.transpose(default_camera_to_desired_camera)

            view_proj_matrix = world_to_default_image @ default_camera_to_desired_camera_row_major

            # Convert Points from World Frame (3D) -> Image Frame (2D)
            image_space_points = get_image_space_points(cuboid_points, view_proj_matrix)

            resolution = np.array([[db.inputs.width, db.inputs.height, 1.0]])
            image_space_points *= resolution

            projected_cuboid_points = [
                [pixel_coordinate[0], pixel_coordinate[1]] for pixel_coordinate in image_space_points
            ]

            # points returned as: [RUB, LUB, RDB, LDB, RUF, LUF, RDF, LDF]
            # but DOPE expects  : [LUF, RUF, RDF, LDF, LUB, RUB, RDB, LDB]
            projected_cuboid = [
                projected_cuboid_points[5],
                projected_cuboid_points[4],
                projected_cuboid_points[6],
                projected_cuboid_points[7],
                projected_cuboid_points[1],
                projected_cuboid_points[0],
                projected_cuboid_points[2],
                projected_cuboid_points[3],
                projected_cuboid_points[8],  # center
            ]

            projected_cuboids.append(projected_cuboid)

            visibilities.append(1.0 - bbox["occlusionRatio"])

        data = np.zeros(len(bboxes_3d), dtype=return_data_dtype)

        data["semanticId"] = np.array(semantic_ids, dtype=np.dtype([("semanticId", "<u4")]))
        data["visibility"] = np.array(visibilities, dtype=np.dtype([("visibility", "<f4")]))

        if len(projected_cuboids) > 0:
            data["projected_cuboid"] = np.array(
                projected_cuboids, dtype=np.dtype([("projected_cuboid", "<f4", (9, 2))])
            )
        if len(locations) > 0:
            data["location"] = np.array(locations, dtype=np.dtype([("location", "<f4", (3,))]))
        if len(rotations) > 0:
            data["rotation"] = np.array(rotations, dtype=np.dtype([("rotation", "<f4", (4,))]))

        # TO-DO:
        # Pass camera location and camera rotation (in world frame) to writer

        db.outputs.data = np.frombuffer(data.tobytes(), dtype=np.uint8)
        db.outputs.idToLabels = serialized_index_to_labels

        db.outputs.exec = og.ExecutionAttributeState.ENABLED
        db.outputs.bufferSize = 0
        db.outputs.height = db.inputs.height
        db.outputs.width = db.inputs.width

        return True
