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

"""Base Camera schema and presence rules."""

from __future__ import annotations

from typing import Any

import usd_validation_nvidia
from pxr import Usd

from .camera_rule_utils import _cameras
from .sensor_rule_utils import RuleRegistrar, fail

_RULES = RuleRegistrar("IsaacSim.Sensor.Camera")


@_RULES.rule("CAMERA-STRUCT-001")
def _camera_presence(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    if not _cameras(stage):
        fail(checker, requirement, "The validated asset subtree must contain a Camera prim", stage)


@_RULES.rule("CAMERA-SCHEMA-001")
def _camera_schema(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for camera in _cameras(stage):
        if not camera.HasAPI("OmniSensorAPI"):
            fail(checker, requirement, f"Camera {camera.GetPath()} does not conform to OmniSensorAPI", camera)
