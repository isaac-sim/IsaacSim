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

"""Tests for the RTX sensor capability catalog."""

import inspect

import omni.kit.test
import usd_validation_nvidia
from isaacsim.asset.validation import radar_sensor_rules
from isaacsim.asset.validation.sensor_capabilities import (
    CAPABILITIES,
    FEATURE_SENSOR_PRIM_TYPES,
    FEATURES,
    REQUIREMENTS,
    get_requirement,
)
from isaacsim.asset.validation.sensor_rules import RULES_BY_REQUIREMENT_ID


class TestSensorCapabilities(omni.kit.test.AsyncTestCase):
    """Verify capability metadata and registry integration."""

    async def test_catalog_contains_every_reviewed_requirement(self) -> None:
        """The checked-in catalog contains every in-scope reviewed row once."""
        self.assertEqual(len(REQUIREMENTS), 372)
        self.assertEqual(len({requirement.code for requirement in REQUIREMENTS.values()}), 372)

    async def test_catalog_defines_all_capabilities_and_features(self) -> None:
        """Capabilities and feature catalogs preserve the reviewed grouping."""
        self.assertEqual(len(CAPABILITIES), 11)
        self.assertEqual(
            set(FEATURES),
            {
                "Camera.RGB",
                "Camera.Depth.SingleView",
                "Lidar.Generic",
                "Camera.ISP.Generic",
                "Radar.Generic.WpmDmat",
            },
        )
        self.assertEqual(FEATURE_SENSOR_PRIM_TYPES["Lidar.Generic"], "OmniLidar")
        self.assertEqual(FEATURE_SENSOR_PRIM_TYPES["Radar.Generic.WpmDmat"], "OmniRadar")
        lidar_requirements = {requirement.code for requirement in CAPABILITIES["RTX Lidar Core"].requirements}
        self.assertEqual(len(lidar_requirements), 75)
        self.assertTrue(
            lidar_requirements <= {requirement.code for requirement in FEATURES["Lidar.Generic"].requirements}
        )

    async def test_every_requirement_is_registered_as_metadata(self) -> None:
        """The SimReady registry can resolve each requirement by code and version."""
        registry = usd_validation_nvidia.RequirementsRegistry()
        for requirement in REQUIREMENTS.values():
            self.assertEqual(
                registry.find_requirement(requirement.code, requirement.version),
                requirement,
            )

    async def test_every_reviewed_requirement_has_an_atomic_checker(self) -> None:
        """Every in-scope catalog row is bound to one independently selectable rule."""
        self.assertEqual(set(REQUIREMENTS), set(RULES_BY_REQUIREMENT_ID))
        self.assertEqual(len(set(RULES_BY_REQUIREMENT_ID.values())), len(REQUIREMENTS))

    async def test_each_capability_has_a_dedicated_rule_module(self) -> None:
        """One implementation module owns each capability's substantive checks."""
        modules_by_capability = {}
        for name, capability in CAPABILITIES.items():
            modules = {
                RULES_BY_REQUIREMENT_ID[requirement.code.rsplit(".", 1)[-1]].__module__
                for requirement in capability.requirements
            }
            modules.discard("isaacsim.asset.validation.sensor_rule_utils")  # Non-constraining policy checks.
            self.assertEqual(len(modules), 1, name)
            modules_by_capability[name] = modules.pop()

        self.assertEqual(len(set(modules_by_capability.values())), len(CAPABILITIES))

    async def test_rule_decorator_preserves_the_named_check(self) -> None:
        """Named checks remain callable after their requirement is registered."""
        self.assertTrue(inspect.isfunction(radar_sensor_rules._prim_type))
        self.assertEqual(
            RULES_BY_REQUIREMENT_ID["RADAR-SCHEMA-001"].REQUIREMENT,
            get_requirement("RADAR-SCHEMA-001"),
        )

    async def test_capabilities_and_features_are_registered(self) -> None:
        """The capability and feature registries expose the sensor catalog."""
        capability_registry = usd_validation_nvidia.CapabilityRegistry()
        feature_registry = usd_validation_nvidia.FeatureRegistry()
        for capability in CAPABILITIES.values():
            self.assertEqual(capability_registry.find(capability.id, capability.version), capability)
        for feature in FEATURES.values():
            self.assertEqual(feature_registry.find(feature.id, feature.version), feature)

    async def test_requirement_lookup_uses_short_spreadsheet_id(self) -> None:
        """Validators can bind to a stable short requirement ID."""
        requirement = get_requirement("RADAR-SCHEMA-001")

        self.assertEqual(requirement.code, "com.nvidia.isaac.sensor.RADAR-SCHEMA-001")
        self.assertEqual(requirement.display_name, "Radar prim type")

    async def test_features_reference_capability_requirement_objects(self) -> None:
        """Feature expansion preserves requirements and supplies sensor-type scope."""
        feature = FEATURES["Camera.ISP.Generic"]
        expected_ids = {
            requirement.code
            for capability_name in (
                "Shippable Asset Structure",
                "Render Graph",
                "Core Sensor Contract",
                "Camera Core",
                "Generic Camera Core ISP",
                "Camera Lens Calibration",
                "SPG Post-Processing",
            )
            for requirement in CAPABILITIES[capability_name].requirements
        }

        self.assertEqual({requirement.code for requirement in feature.requirements}, expected_ids)
        self.assertNotIn("LUT Calibration Resources", CAPABILITIES)
        self.assertIn(get_requirement("CAMERA-LUT-001"), CAPABILITIES["Camera Lens Calibration"].requirements)
        scoped = next(
            requirement for requirement in feature.requirements if requirement.code.endswith("ASSET-STRUCT-003")
        )
        self.assertEqual(scoped.parameters[0].display_name, "SENSOR_PRIM_TYPE")
        self.assertEqual(scoped.parameters[0].assigned_value, "Camera")
