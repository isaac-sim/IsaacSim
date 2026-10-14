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

"""Display-ready RGB RenderVar rule for Camera products."""

from __future__ import annotations

from typing import Any

import usd_validation_nvidia
from pxr import Usd, UsdRender

from .camera_rule_utils import _DEPTH_API, _ISP_OUTPUTS, _cameras, _source_names
from .sensor_rule_utils import RuleRegistrar, fail, render_products_targeting

_RULES = RuleRegistrar("IsaacSim.Sensor.Camera")


@_RULES.rule("CAMERA-OUTPUT-001")
def _rgb_output(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    camera_paths = {camera.GetPath() for camera in _cameras(stage)}
    for render_product in render_products_targeting(stage, sensor_paths=camera_paths):
        names = _source_names(render_product)
        # Explicit Depth and ISP products can target the same Camera without
        # becoming Camera.RGB products themselves.
        if render_product.HasAPI(_DEPTH_API) or set(names) & (_ISP_OUTPUTS | {"HdrColor"}):
            continue
        count = names.count("LdrColor")
        if count != 1:
            fail(
                checker,
                requirement,
                f"Camera RenderProduct {render_product.GetPath()} orders {count} LdrColor RenderVars; expected one",
                UsdRender.Product(render_product).GetOrderedVarsRel(),
            )
