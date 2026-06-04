# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Rule for converting MJCF actuator and joint attributes to PhysX schemas."""

from __future__ import annotations

from isaacsim.asset.importer.utils.impl import mjc_to_physx_conversion_utils
from isaacsim.asset.transformer import RuleConfigurationParam, RuleInterface


class MjcToPhysxConversionRule(RuleInterface):
    """Convert MJCF actuator and joint attributes to PhysX drive and joint schemas.

    Walks all ``MjcActuator`` prims and revolute/prismatic joints in the
    stage, converting MJCF gain/bias parameters to PhysX drive stiffness
    and damping, and MJCF joint attributes (friction, armature, ref) to
    their PhysX equivalents.  Mimic joints are left as ``NewtonMimicAPI``
    on the joint prim and consumed directly by the runtime; no equivalent
    ``PhysxMimicJointAPI`` is authored.
    """

    def get_configuration_parameters(self) -> list[RuleConfigurationParam]:
        """Return the configuration parameters for this rule.

        Returns:
            List of configuration parameters.

        Example:

        .. code-block:: python

            params = rule.get_configuration_parameters()

        """
        return []

    def process_rule(self) -> str | None:
        """Convert all MJCF actuators and joints to PhysX equivalents.

        Returns:
            None (this rule does not change the working stage).

        Example:

        .. code-block:: python

            rule.process_rule()

        """
        self.log_operation("MjcToPhysxConversionRule start")

        mjc_to_physx_conversion_utils.convert_mjc_to_physx(self.source_stage)

        self.log_operation("MjcToPhysxConversionRule completed")
        return None
