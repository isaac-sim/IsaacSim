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

import omni
from omni.kit.viewport.utility import create_viewport_window, get_active_viewport_window


class OgnIsaacCreateViewportInternalState:
    def __init__(self):
        self.window = None


class OgnIsaacCreateViewport:
    """Isaac Sim Create Viewport."""

    @staticmethod
    def internal_state():
        return OgnIsaacCreateViewportInternalState()

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state
        if state.window is None:
            if len(db.inputs.name) > 0:
                state.window = create_viewport_window(db.inputs.name)
            else:
                if db.inputs.viewportId == 0:
                    state.window = get_active_viewport_window()
                else:
                    state.window = create_viewport_window(str(db.inputs.viewportId))
        db.outputs.viewport = state.window.title
        db.outputs.execOut = omni.graph.core.ExecutionAttributeState.ENABLED
        return True
