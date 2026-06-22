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

"""Manage conveyor belt material-pair friction tables."""

import warp as wp

# not needed for the purpose of this sample
wp.config.enable_backward = False


class MaterialPairManager:
    """Class to get material indices and to set up friction coefficients per material pair.

    Holds a friction table that defines the friction coefficients for pairs of materials.
    """

    def __init__(
        self,
    ) -> None:

        self.next_transported_body_material_index = 0

        self.next_conveyor_belt_material_index = 0

        self.friction_map = {}

        self.friction_table = None

    def add_transported_body_material_index(
        self,
    ) -> int:
        """Allocate a transported-body material index.

        The index is used for rigid bodies that are transported by conveyor belts.
        """
        index = self.next_transported_body_material_index

        self.next_transported_body_material_index += 1

        return index

    def add_conveyor_belt_material_index(
        self,
    ) -> int:
        """Allocate a conveyor-belt material index.

        The index is used for conveyor belt objects.
        """
        index = self.next_conveyor_belt_material_index

        self.next_conveyor_belt_material_index += 1

        return index

    def set_material_pair_friction(
        self,
        transported_body_material_index: int,
        conveyor_belt_material_index: int,
        friction_coefficient: float,
    ) -> None:
        """Store the friction coefficient for a transported-body / conveyor-belt material pair.

        If this method is not called for a material pair, then a default value of zero is used.

        Args:
            transported_body_material_index: Index previously returned by
                ``add_transported_body_material_index``.
            conveyor_belt_material_index: Index previously returned by
                ``add_conveyor_belt_material_index``.
            friction_coefficient: Friction coefficient to use for contacts between this pair.
        """
        self.friction_map[(transported_body_material_index, conveyor_belt_material_index)] = friction_coefficient

    def create_buffers(
        self,
        device: str | None = None,
    ) -> None:
        """Build the 2-D Warp friction lookup table from the registered material pair data.

        The resulting ``friction_table`` is shaped
        ``(transported_body_material_count, conveyor_belt_material_count)``.

        Args:
            device: Warp device string. Uses the default device when ``None``.
        """
        dim0 = max(self.next_transported_body_material_index, 1)
        dim1 = max(self.next_conveyor_belt_material_index, 1)

        tmp_friction_table_data = [0.0] * (dim0 * dim1)

        for (index0, index1), friction_coefficient in self.friction_map.items():
            i = (index0 * dim1) + index1

            tmp_friction_table_data[i] = friction_coefficient

        self.friction_table = wp.array(
            tmp_friction_table_data,
            shape=(dim0, dim1),
            dtype=wp.float32,
            device=device,
        )
