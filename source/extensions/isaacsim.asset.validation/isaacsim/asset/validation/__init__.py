# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Isaac Sim asset validation.

The concrete validation rules that used to live in this extension (physics,
joints, drives, robot schema, and materials) have been migrated to the SimReady
foundation validation tiers (``simready-foundation-tier-core`` and
``simready-foundation-tier-isaac``) and are executed via the ``simready-validate``
framework. Those packages are declared as pip dependencies in this extension's
``config/extension.toml`` so they are importable inside Isaac Sim, where the
tier validators discover themselves through ``usd_validation_nvidia`` entry
points.

This extension also provides requirement-addressable authored sensor asset
capabilities and validators.
"""

from .extension import IsaacSimAssetValidationExtension  # noqa: F401
from .sensor_capabilities import CAPABILITIES as SENSOR_CAPABILITIES
from .sensor_capabilities import FEATURES as SENSOR_FEATURES
from .sensor_capabilities import REQUIREMENTS as SENSOR_REQUIREMENTS
from .sensor_rules import RULES_BY_REQUIREMENT_ID as SENSOR_RULES_BY_REQUIREMENT_ID

__all__ = [
    "IsaacSimAssetValidationExtension",
    "SENSOR_CAPABILITIES",
    "SENSOR_FEATURES",
    "SENSOR_REQUIREMENTS",
    "SENSOR_RULES_BY_REQUIREMENT_ID",
]
