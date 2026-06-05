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
from isaacsim.core.rendering_manager import ViewportManager


class OgnIsaacSetCameraOnRenderProduct:
    """Isaac Sim Set Camera On Render Product."""

    @staticmethod
    def compute(db) -> bool:
        if len(db.inputs.cameraPrim) == 0:
            db.log_error(f"Camera prim must be specified")
            return False
        ViewportManager.set_camera(
            db.inputs.cameraPrim[0].GetString(), render_product_or_viewport=db.inputs.renderProductPath
        )
        db.outputs.execOut = omni.graph.core.ExecutionAttributeState.ENABLED
        return True
