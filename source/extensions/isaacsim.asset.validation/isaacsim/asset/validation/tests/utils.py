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

"""Shared helpers for atomic sensor-rule tests."""

import usd_validation_nvidia
from isaacsim.asset.validation.sensor_capabilities import get_requirement
from pxr import Usd, UsdRender


def add_render_var(
    stage: Usd.Stage,
    product: UsdRender.Product,
    source_name: str,
    *,
    channels: list[str] | None = None,
) -> Usd.Prim:
    """Add an ordered native RenderVar to a sensor test product."""
    path = product.GetPath().AppendChild(f"Var{len(product.GetOrderedVarsRel().GetTargets())}")
    render_var = UsdRender.Var.Define(stage, path)
    render_var.CreateSourceNameAttr(source_name)
    product.CreateOrderedVarsRel().AddTarget(path)
    if channels is not None:
        prim = render_var.GetPrim()
        prim.ApplyAPI("RenderVarChannelsAPI")
        prim.GetAttribute("channels").Set(channels)
    return render_var.GetPrim()


def validate(stage: Usd.Stage, *requirement_ids: str) -> list[usd_validation_nvidia.Issue]:
    engine = usd_validation_nvidia.ValidationEngine(init_rules=False, variants=True)
    for requirement_id in requirement_ids:
        engine.enable_requirement(get_requirement(requirement_id))
    return list(engine.validate(stage).issues)


def failures(stage: Usd.Stage, *requirement_ids: str) -> list[usd_validation_nvidia.Issue]:
    return [
        issue
        for issue in validate(stage, *requirement_ids)
        if issue.severity in (usd_validation_nvidia.IssueSeverity.FAILURE, usd_validation_nvidia.IssueSeverity.ERROR)
    ]


def warnings(stage: Usd.Stage, *requirement_ids: str) -> list[usd_validation_nvidia.Issue]:
    return [
        issue
        for issue in validate(stage, *requirement_ids)
        if issue.severity == usd_validation_nvidia.IssueSeverity.WARNING
    ]
