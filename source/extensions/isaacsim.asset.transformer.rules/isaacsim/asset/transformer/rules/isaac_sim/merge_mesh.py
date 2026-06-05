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

"""Rule for merging mesh prims grouped under rigid bodies."""

from __future__ import annotations

from isaacsim.asset.importer.utils.impl import merge_mesh_utils
from isaacsim.asset.transformer import RuleConfigurationParam, RuleInterface


class MergeMeshRule(RuleInterface):
    """Merge visual mesh prims that share a common rigid-body parent.

    Walks all rigid bodies in the stage and merges their child visual
    geometry prims using the Scene Optimizer merge operation.
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
        """Merge mesh prims grouped under rigid bodies.

        Returns:
            None (this rule does not change the working stage).

        Example:

        .. code-block:: python

            rule.process_rule()

        """
        self.log_operation("MergeMeshRule start")

        merged = merge_mesh_utils.merge_meshes_operation(self.source_stage)

        self.log_operation(f"MergeMeshRule completed: merged {merged} mesh group(s)")
        return None
