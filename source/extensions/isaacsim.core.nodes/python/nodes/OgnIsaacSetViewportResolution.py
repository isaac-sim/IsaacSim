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
from omni.kit.viewport.utility import get_active_viewport, get_viewport_from_window_name


class OgnIsaacSetViewportResolution:
    """Isaac Sim Set Viewport Resolution."""

    @staticmethod
    def compute(db) -> bool:
        viewport_name = db.inputs.viewport
        if viewport_name:
            viewport_api = get_viewport_from_window_name(viewport_name)
        else:
            viewport_api = get_active_viewport()

        if viewport_api:
            viewport_api.set_texture_resolution((db.inputs.width, db.inputs.height))
            settings = carb.settings.get_settings()
            settings.set_int("/app/hydra/aperture/conform", 3)
            settings.set_int("/app/hydra/aperture/conform", 4)

        db.outputs.execOut = omni.graph.core.ExecutionAttributeState.ENABLED
        return True
