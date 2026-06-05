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

"""Newton/Mujoco physics UI extension: schema registration and property widgets."""

import omni.ext
import omni.kit.property.physics as property
from omni.physics.isaacsimready import get_capability_manager, get_variant_switcher

from .mujoco_schemas import MujocoUiDefinitions, get_mujoco_schema_names
from .newton_schemas import NewtonUiDefinitions, get_newton_schema_names


class PhysicsNewtonUIExtension(omni.ext.IExt):
    """Extension that registers Newton and Mujoco schema names and property UI."""

    def __init__(self):
        super().__init__()

    def on_startup(self, _ext_id: str):
        """Register Newton/Mujoco schema names, property widgets, and variant switcher.

        Args:
            _ext_id: Extension identifier from the extension manager.
        """
        capability_manager = get_capability_manager()

        # Register all Mujoco schema names
        prim_types, api_schemas = get_mujoco_schema_names()
        capability_manager.register_schema_type_names(prim_types)
        capability_manager.register_api_schema_names(api_schemas)

        # Register all Newton schema names
        prim_types, api_schemas = get_newton_schema_names()
        capability_manager.register_schema_type_names(prim_types)
        capability_manager.register_api_schema_names(api_schemas)

        # Register newton widgets first, this will put them on top of the mujoco widgets
        property.register_parent_schema(
            "newton",
            "Newton",
            NewtonUiDefinitions.widgets,
            NewtonUiDefinitions.property_builders,
            NewtonUiDefinitions.property_order,
            NewtonUiDefinitions.extensions,
            NewtonUiDefinitions.extras,
            NewtonUiDefinitions.ignore,
        )

        # Register mujoco widgets
        property.register_parent_schema(
            "mjcPhysics",
            "Mujoco",
            MujocoUiDefinitions.widgets,
            MujocoUiDefinitions.property_builders,
            MujocoUiDefinitions.property_order,
            MujocoUiDefinitions.extensions,
            MujocoUiDefinitions.extras,
            MujocoUiDefinitions.ignore,
        )

        # Register simulator-to-variant mappings and the parent schema group
        get_variant_switcher().register_simulator_variant("PhysX", "physx")
        get_variant_switcher().register_simulator_variant("Newton", "mujoco")

        # Register group to fit with the simulator name
        property.register_parent_schema_group(
            "Newton",
            ["mjcPhysics", "newton"],
        )

    def on_shutdown(self):
        """Unregister Newton and Mujoco property schema groups and widgets."""
        # Unregister PhysX property widgets
        property.unregister_parent_schema_group("Newton")
        property.unregister_parent_schema("newton")
        property.unregister_parent_schema("mjcPhysics")
