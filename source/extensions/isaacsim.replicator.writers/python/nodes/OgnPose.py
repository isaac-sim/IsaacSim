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

"""OmniGraph node implementation for OgnPose."""

import json
from typing import Any

import isaacsim.core.experimental.utils.transform as transform_utils
import numpy as np
import omni.graph.core as og
from isaacsim.replicator.writers.scripts.utils import (
    get_image_space_points,
    get_transform_with_normalized_rotation,
    tf_matrix_from_pose,
)


class OgnPose:
    """OmniGraph node for a Pose annotator, allowing the poses of assets with semantic labels to be retrieved."""

    @staticmethod
    def compute(db: Any) -> bool:
        """Compute semantic pose data for the deprecated Pose OmniGraph annotator.

        Build the semantic ID mapping, filter occluded prims when requested,
        transform prim poses into the desired camera frame, and optionally
        output image-space centers.

        Args:
            db: OmniGraph database object supplying semantic, camera, extent,
                and output attributes.

        Returns:
            True when pose data or an empty output buffer is written, False when
            no semantic types are requested.
        """
        db.log_warn("Deprecation warning: OgnPose has been deprecated and will be removed in the next major release.")

        include_occluded_prims = db.inputs.includeOccludedPrims
        get_centers = db.inputs.getCenters

        return_data_list = [("semanticId", "<u4"), ("prims_to_desired_camera", "<f4", (4, 4))]

        if get_centers:
            center_type = ("center_coords_image_space", "<f4", (2,))
            return_data_list.append(center_type)

        return_data_dtype = np.dtype(return_data_list)

        required_semantic_types = db.inputs.semanticTypes

        if db.inputs.bufferSize == 0:
            db.outputs.data = np.frombuffer(np.empty(0, dtype=return_data_dtype).tobytes(), dtype=np.uint8)
            db.outputs.exec = og.ExecutionAttributeState.ENABLED
            db.outputs.bufferSize = 0
            db.outputs.height = 0
            db.outputs.width = 0
            db.outputs.idToLabels = "{}"
            return True

        num_semantics = db.inputs.sdIMNumSemantics

        num_semantic_tokens = db.inputs.sdIMNumSemanticTokens

        instance_semantic_map = db.inputs.sdIMInstanceSemanticMap.view(np.uint16)
        min_semantic_idx = db.inputs.sdIMMinSemanticIndex
        max_semantic_hierarchy_depth = db.inputs.sdIMMaxSemanticHierarchyDepth
        semantic_token_map = db.inputs.sdIMSemanticTokenMap

        prims_to_world_row_major = db.inputs.sdIMSemanticWorldTransform.view(np.float32).reshape((num_semantics, 4, 4))
        for i in range(num_semantics):
            prims_to_world_row_major[i] = get_transform_with_normalized_rotation(prims_to_world_row_major[i])

        coord_types = [("x_min", "<i4"), ("y_min", "<i4"), ("x_max", "<i4"), ("y_max", "<i4")]
        desired_data_dtype = np.dtype(coord_types)

        data_attribute = db.node.get_attribute("inputs:data")
        if not hasattr(data_attribute, "get_array"):
            data_helper = og.AttributeValueHelper(data_attribute)
            extent_data = data_helper.get_array(False, False, 0).view(desired_data_dtype)
        else:
            extent_data = data_attribute.get_array(False, False, 0).view(desired_data_dtype)

        valid_pose_mask = extent_data["x_min"] != 2147483647

        # If list of semantics is empty, return False
        if len(required_semantic_types) == 0:
            return False

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
        pose_ids = []
        for i, is_valid in enumerate(valid_pose_mask.tolist()):
            if is_valid:
                pose_ids.append(i)
                index_to_labels[valid_semantic_entity_count] = {}

                # Find labels of itself and parent labels
                self_labels = semantic_token_map[i * num_semantic_tokens : (i + 1) * num_semantic_tokens]
                parent_labels = []

                if i in id_to_parents:
                    for parent_semantic_id in id_to_parents[i]:
                        parent_labels.extend(
                            semantic_token_map[
                                parent_semantic_id
                                * num_semantic_tokens : (parent_semantic_id + 1)
                                * num_semantic_tokens
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
                            index_to_labels[valid_semantic_entity_count].setdefault(semantic_type, set()).add(
                                semantic_data
                            )

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

        # Poses
        cameraRotation = db.inputs.cameraRotation
        width = db.inputs.imageWidth
        height = db.inputs.imageHeight
        cameraViewTransform = db.inputs.cameraViewTransform
        cameraProjection = db.inputs.cameraProjection

        default_camera_to_desired_camera = tf_matrix_from_pose(
            translation=(0.0, 0.0, 0.0),
            orientation=transform_utils.euler_angles_to_quaternion(cameraRotation, degrees=True).numpy(),
        )

        world_to_default_camera_row_major = np.asarray(cameraViewTransform).reshape((4, 4))
        world_to_default_camera = np.transpose(world_to_default_camera_row_major)

        if include_occluded_prims:
            prims_to_world_row_major_filtered = prims_to_world_row_major
        else:
            # Filter out transforms corresponding to fully occluded/out-of-frame prims
            prims_to_world_row_major_filtered = np.take(prims_to_world_row_major, pose_ids, axis=0)

        prims_to_world = np.transpose(prims_to_world_row_major_filtered, axes=(0, 2, 1))

        prims_to_desired_camera = default_camera_to_desired_camera @ world_to_default_camera @ prims_to_world

        # Centers
        if get_centers:
            prim_translations = prims_to_world[:, :-1, -1]

            default_camera_to_image_row_major = np.asarray(cameraProjection).reshape((4, 4))

            # Default view projection matrix, transforming points from world frame to image space of default camera
            world_to_default_image = world_to_default_camera_row_major @ default_camera_to_image_row_major
            default_camera_to_desired_camera_row_major = np.transpose(default_camera_to_desired_camera)

            # Desired view projection matrix, transforming points from world frame to image space of desired camera
            view_proj_matrix = world_to_default_image @ default_camera_to_desired_camera_row_major

            image_space_points = get_image_space_points(prim_translations, view_proj_matrix)

            resolution_homogenous = np.array([[width, height, 1.0]])
            pixel_coordinates = image_space_points * resolution_homogenous
            centers = pixel_coordinates[:, :-1]

        data = np.zeros(valid_semantic_entity_count, dtype=return_data_dtype)

        data["semanticId"] = np.array(semantic_ids)
        data["prims_to_desired_camera"] = prims_to_desired_camera

        if get_centers:
            data["center_coords_image_space"] = centers

        db.outputs.data = np.frombuffer(data.tobytes(), dtype=np.uint8)
        db.outputs.idToLabels = serialized_index_to_labels
        db.outputs.primPaths = prim_paths
        db.outputs.exec = og.ExecutionAttributeState.ENABLED
        db.outputs.bufferSize = 0
        db.outputs.height = 0
        db.outputs.width = 0

        return True
