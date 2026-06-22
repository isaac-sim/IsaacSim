# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Resolve a viewport window name to its current render product path."""

from typing import Any

import carb
import omni
from isaacsim.core.nodes.ogn.OgnIsaacGetViewportRenderProductDatabase import OgnIsaacGetViewportRenderProductDatabase
from omni.kit.viewport.utility import get_viewport_from_window_name


class OgnIsaacGetViewportRenderProductInternalState:
    """Per-instance cache for the viewport API resolved from the input window name."""

    def __init__(self) -> None:
        viewport = None


class OgnIsaacGetViewportRenderProduct:
    """Isaac Sim Create Hydra Texture."""

    @staticmethod
    def internal_state() -> OgnIsaacGetViewportRenderProductInternalState:
        """Create the per-instance viewport cache.

        Returns:
            Per-instance viewport cache.
        """
        return OgnIsaacGetViewportRenderProductInternalState()

    @staticmethod
    def compute(db: Any) -> bool:
        """Publish the render product path for the requested viewport.

        The node reuses the cached viewport when available, returns False after warning when the
        viewport cannot be found, and enables `execOut` when the render product path is written.

        Args:
            db: OmniGraph database for this node.

        Returns:
            True when the render product path is written, False otherwise.
        """
        state = db.per_instance_state
        viewport_api = get_viewport_from_window_name(db.inputs.viewport)
        if viewport_api:
            db.per_instance_state.viewport = viewport_api
        if db.per_instance_state.viewport is None:
            carb.log_warn("viewport name {db.inputs.viewport} not found")
            db.per_instance_state.initialized = False
            return False

        viewport = db.per_instance_state.viewport
        db.outputs.renderProductPath = viewport.get_render_product_path()
        db.outputs.execOut = omni.graph.core.ExecutionAttributeState.ENABLED
        return True

    @staticmethod
    def release_instance(node: Any, graph_instance_id: Any) -> None:
        """Clear the cached viewport reference when the node instance is released.

        Args:
            node: OmniGraph node instance.
            graph_instance_id: Graph instance identifier.
        """
        try:
            state = OgnIsaacGetViewportRenderProductDatabase.per_instance_internal_state(node)
        except Exception:
            state = None

        if state is not None:
            state.viewport = None
