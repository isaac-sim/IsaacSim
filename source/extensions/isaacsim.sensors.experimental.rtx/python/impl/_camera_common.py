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

"""Common utilities and specifications for camera sensor implementations."""

import warp as wp

CAMERA_ANNOTATOR_SPEC = {
    # standard annotators
    "bounding_box_2d_loose": {"name": "bounding_box_2d_loose_fast"},
    "bounding_box_2d_tight": {"name": "bounding_box_2d_tight_fast"},
    "bounding_box_3d": {"name": "bounding_box_3d_fast"},
    "distance_to_camera": {"name": "distance_to_camera", "channels": 1, "dtype": wp.float32},
    "distance_to_image_plane": {"name": "distance_to_image_plane", "channels": 1, "dtype": wp.float32},
    "instance_id_segmentation": {"name": "instance_id_segmentation_fast", "channels": 1, "dtype": wp.uint32},
    "instance_segmentation": {"name": "instance_segmentation_fast", "channels": 1, "dtype": wp.uint32},
    "motion_vectors": {"name": "motion_vectors", "channels": 4, "output_channels": 2, "dtype": wp.float32},
    "normals": {"name": "normals", "channels": 4, "output_channels": 3, "dtype": wp.float32},
    "pointcloud": {"name": "pointcloud"},
    "rgb": {"name": "rgb", "channels": 4, "output_channels": 3, "dtype": wp.uint8},
    "rgba": {"name": "rgb", "channels": 4, "dtype": wp.uint8},
    "semantic_segmentation": {"name": "semantic_segmentation", "channels": 1, "dtype": wp.uint32},
    # single view depth sensor annotators
    "depth_sensor_distance": {"name": "DepthSensorDistance", "channels": 1, "dtype": wp.float32},
    "depth_sensor_imager": {"name": "DepthSensorImager", "channels": 1, "dtype": wp.float32},
    "depth_sensor_point_cloud_color": {"name": "DepthSensorPointCloudColor", "channels": 4, "dtype": wp.uint8},
    "depth_sensor_point_cloud_position": {
        "name": "DepthSensorPointCloudPosition",
        "channels": 4,
        "output_channels": 3,
        "dtype": wp.float32,
    },
}
