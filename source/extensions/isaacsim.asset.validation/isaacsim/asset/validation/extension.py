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

"""Isaac Sim Asset Validation Extension.

Provisions the SimReady foundation validation tiers (via ``config/extension.toml``)
and registers their rules with the in-process Asset Validator registry. It also
registers validators that cannot live in the standalone tier wheels because they
depend on the live Isaac Sim / Kit PhysX runtime.
"""

import importlib

import carb
import omni.ext

_VALIDATION_MODULES = (
    "simready.foundation.tier_core.capabilities",
    "simready.foundation.tier_isaac.capabilities",
    "isaacsim.asset.validation.collision_validation",
    "isaacsim.asset.validation.sensor_capabilities",
    "isaacsim.asset.validation.sensor_rules",
)


class IsaacSimAssetValidationExtension(omni.ext.IExt):
    """Isaac Sim Asset Validation Extension.

    Load SimReady foundation tier rules into the shared Asset Validator registry
    and register runtime-dependent validators that require Isaac Sim.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension on startup.

        Import the tier capability hubs after Kit adds the extension prebundle to
        ``sys.path``. Their decorators register the rules with the shared
        ``usd_validation_nvidia`` registry used by ``omni.asset_validator.core``.

        Args:
            ext_id: The extension identifier.
        """
        for module_name in _VALIDATION_MODULES:
            try:
                importlib.import_module(module_name)
            except Exception as exc:  # pragma: no cover - depends on tier availability
                carb.log_warn(f"[isaacsim.asset.validation] Could not register validators from {module_name}: {exc}")

    def on_shutdown(self) -> None:
        """Clean up resources when the extension is shut down."""
