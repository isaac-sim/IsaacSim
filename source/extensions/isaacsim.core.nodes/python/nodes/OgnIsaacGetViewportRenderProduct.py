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

import carb
import omni
from isaacsim.core.nodes.ogn.OgnIsaacGetViewportRenderProductDatabase import OgnIsaacGetViewportRenderProductDatabase
from omni.kit.viewport.utility import get_viewport_from_window_name


class OgnIsaacGetViewportRenderProductInternalState:
    def __init__(self):
        viewport = None


class OgnIsaacGetViewportRenderProduct:
    """Isaac Sim Create Hydra Texture."""

    @staticmethod
    def internal_state():
        return OgnIsaacGetViewportRenderProductInternalState()

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state
        viewport_api = get_viewport_from_window_name(db.inputs.viewport)
        if viewport_api:
            db.per_instance_state.viewport = viewport_api
        if db.per_instance_state.viewport == None:
            carb.log_warn("viewport name {db.inputs.viewport} not found")
            db.per_instance_state.initialized = False
            return False

        viewport = db.per_instance_state.viewport
        db.outputs.renderProductPath = viewport.get_render_product_path()
        db.outputs.execOut = omni.graph.core.ExecutionAttributeState.ENABLED
        return True

    @staticmethod
    def release_instance(node, graph_instance_id):
        try:
            state = OgnIsaacGetViewportRenderProductDatabase.per_instance_internal_state(node)
        except Exception:
            state = None
            pass

        if state is not None:
            state.viewport = None
