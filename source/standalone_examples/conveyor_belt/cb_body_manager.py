# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Manage rigid body buffers for the conveyor belt example."""

import warp as wp

# not needed for the purpose of this sample
wp.config.enable_backward = False


class BodyManager:
    """Rigid body registry and buffer manager.

    Holds various rigid body related data buffers that are used for the computation of the
    forces that conveyor belts apply to the rigid bodies.
    """

    def __init__(
        self,
    ) -> None:

        self.body_path_list = []

        self.material_index_list = []

        # Array to hold the index of the material for each body
        self.material_index_buffer = None

        # Array to hold the bodyToWorld transform (center-of-mass to world) for each body
        self.world_transform_buffer = None

        # Array to hold the world space inverse inertia tensor matrix for each body
        self.inverse_inertia_buffer = None

        # Array that points to the index of the first contact patch for each body
        self.body_to_patch_buffer = None

        # Arrays to store the forces and torques to apply at each body (at the center of mass).
        self.force_buffer = None
        self.torque_buffer = None

    def add_body(
        self,
        path: str,
        material_index: int,
    ) -> None:
        """Register a rigid body prim path and its associated material index.

        Args:
            path: USD prim path of the rigid body.
            material_index: Index into the material pair friction table for this body.
        """
        self.body_path_list.append(path)

        self.material_index_list.append(material_index)

    def create_buffers(
        self,
        device: str | None = None,
    ) -> None:
        """Allocate all per-body Warp arrays (material indices, transforms, inertias, forces, etc.).

        Args:
            device: Warp device string. Uses the default device when ``None``.
        """
        body_count = len(self.body_path_list)

        self.material_index_buffer = wp.array(
            self.material_index_list,
            shape=body_count,
            dtype=wp.uint32,
            device=device,
        )

        self.world_transform_buffer = wp.empty(shape=body_count, dtype=wp.transform, device=device)

        self.inverse_inertia_buffer = wp.empty(shape=body_count, dtype=wp.mat33, device=device)

        self.body_to_patch_buffer = wp.empty(shape=body_count, dtype=wp.uint32, device=device)

        self.force_buffer = wp.zeros(shape=(body_count, 3), dtype=wp.float32, device=device)

        self.torque_buffer = wp.zeros(shape=(body_count, 3), dtype=wp.float32, device=device)
