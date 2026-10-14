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

"""Common Camera-prim and RenderVar selectors for camera capabilities."""

from __future__ import annotations

from pxr import Usd

from .sensor_rule_utils import ordered_render_vars, prims, render_var_source_name

_DEPTH_API = "OmniSensorDepthSensorSingleViewAPI"
_ISP_OUTPUTS = frozenset(
    {
        "IspModelGenericA",
        "IspModelGenericB",
        "CameraIspGeneric",
        "OmniCameraSensorInputSource",
        "OmniCameraSensorPostColorCorrection",
        "OmniCameraSensorPostCfa",
        "OmniCameraSensorPostNoise",
        "OmniCameraSensorPreIsp",
        "OmniCameraSensorPostYuvIsp",
        "OmniCameraSensorPostIsp",
    }
)


def _cameras(stage: Usd.Stage) -> list[Usd.Prim]:
    return prims(stage, {"Camera"})


def _source_names(render_product: Usd.Prim) -> list[str]:
    return [render_var_source_name(prim) for _, prim in ordered_render_vars(render_product)]
