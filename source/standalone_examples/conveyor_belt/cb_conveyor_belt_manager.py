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

"""Manage conveyor belt metadata and Warp buffers."""

import warp as wp

# not needed for the purpose of this sample
wp.config.enable_backward = False


class ConveyorBeltManager:
    """Conveyor belt registry and buffer manager.

    Holds various conveyor belt related data buffers that are used for the computation of the
    forces that conveyor belts apply to interacting rigid bodies.
    """

    def __init__(self) -> None:

        self.conveyor_belt_path_list = []

        self.conveyor_belt_to_indices_list = []

        self.surface_normal_list = []

        self.contact_processing_threshold_list = []

        self.conveyor_belt_to_indices_map = None

        self.surface_normal_buffer = None

        self.contact_processing_threshold_buffer = None

    def add_conveyor_belt(
        self,
        path: str,
        velocity_field_type: int,
        velocity_field_id: int,
        surface_normal: wp.vec3,
        contact_processing_threshold: float,
        material_index: int,
    ) -> None:
        """Register a conveyor belt object and its associated velocity field, material, etc.

        Args:
            path: USD prim path of the conveyor belt collision geometry.
            velocity_field_type: Integer constant identifying the velocity field type
                (e.g. ``VELOCITY_FIELD_TYPE_CONSTANT_VELOCITY`` or ``VELOCITY_FIELD_TYPE_PIVOT``).
            velocity_field_id: Index of the velocity field instance of the given type
                (see VelocityFieldActuator.add_...velocity_field()).
            surface_normal: Normal vector of the conveyor belt surface; contacts whose
                normal diverges beyond a certain threshold from this value will be ignored
                (see ``contact_processing_threshold``).
            contact_processing_threshold: Minimum dot-product between a contact normal and
                ``surface_normal`` required for a contact to be processed.
            material_index: Index into the material pair friction table for this belt
                (see MaterialPairManager).
        """
        self.conveyor_belt_path_list.append(path)

        self.conveyor_belt_to_indices_list.append(velocity_field_type)
        self.conveyor_belt_to_indices_list.append(velocity_field_id)
        self.conveyor_belt_to_indices_list.append(material_index)

        self.surface_normal_list.append(surface_normal)
        self.contact_processing_threshold_list.append(contact_processing_threshold)

    def create_buffers(
        self,
        device: str | None = None,
    ) -> None:
        """Allocate Warp arrays from the registered conveyor belt data.

        Args:
            device: Warp device string. Uses the default device when ``None``.
        """
        # (N, 3) array that holds for each conveyor belt the following indices:
        # - type of the velocity field the conveyor belt should use
        #   (see add_conveyor_belt())
        # - ID of the velocity field instance the conveyor belt should use
        #   (see add_conveyor_belt())
        # - index of the material assigned to the conveyor belt
        #   (see add_conveyor_belt())
        self.conveyor_belt_to_indices_map = wp.array(
            self.conveyor_belt_to_indices_list,
            shape=(len(self.conveyor_belt_path_list), 3),
            dtype=wp.uint32,
            device=device,
        )

        self.surface_normal_buffer = wp.array(
            self.surface_normal_list,
            dtype=wp.vec3,
            device=device,
        )

        self.contact_processing_threshold_buffer = wp.array(
            self.contact_processing_threshold_list,
            dtype=wp.float32,
            device=device,
        )
