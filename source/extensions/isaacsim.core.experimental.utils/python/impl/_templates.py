# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from __future__ import annotations

from pxr import UsdLux


def gridroom() -> None:
    """Populate the stage with a grid-like room scene."""
    # defer imports to avoid circular dependencies
    import isaacsim.core.experimental.utils.prim as prim_utils
    import isaacsim.core.experimental.utils.stage as stage_utils
    from isaacsim.core.experimental.objects import GroundPlane, SphereLight

    # define stage structure
    stage_utils.define_prim("/World", "Xform")
    stage_utils.define_prim("/World/Environment", "Xform")
    # add prims
    # - ground plane
    GroundPlane("/World/GroundPlane", templates="wireframe-blue")
    # - light
    sphere_light = SphereLight("/World/Environment/SphereLight", radii=0.25, positions=(0.0, 0.0, 2.5))
    sphere_light.set_colors("white")
    sphere_light.set_intensities(100000)
    sphere_light.set_color_temperatures(6500)
    shaping_api = prim_utils.ensure_api(sphere_light.prims[0], UsdLux.ShapingAPI)
    shaping_api.GetShapingConeAngleAttr().Set(180)
