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

"""Tests for validating all supported lidar configurations and variants in the isaacsim.sensors.experimental.rtx extension."""

from pathlib import Path

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
from isaacsim.sensors.experimental.rtx import SUPPORTED_LIDAR_CONFIGS, Lidar
from isaacsim.sensors.experimental.rtx.sensor_checker import ModelInfo, SensorCheckerUtil

from .common import create_sarcophagus


class TestSupportedLidarConfigs(omni.kit.test.AsyncTestCase):
    """Test class for validating all supported lidar configurations and variants in the isaacsim.sensors.experimental.rtx extension.

    This test class dynamically generates test methods for each supported lidar configuration and variant combination
    from SUPPORTED_LIDAR_CONFIGS. Each test method creates a lidar sensor prim using the IsaacSensorCreateRtxLidar
    command and validates its parameters using the SensorCheckerUtil with the LidarCore model.

    The class sets up a test environment with a new stage and a sarcophagus object for sensor validation. It uses
    the SensorCheckerUtil to verify that each lidar configuration produces valid sensor parameters that conform to
    the expected schema and model requirements.

    Test methods are automatically generated at module load time using the _create_lidar_parameters_test function,
    which creates individual test cases for each config-variant pair. Each generated test validates sensor creation,
    parameter validation, and ensures the sensor prim has the correct type and valid parameters.
    """

    async def setUp(self):
        """Sets up the test environment for lidar configuration testing.

        Creates a new USD stage, initializes a sarcophagus object with non-visual materials,
        sets up the timeline interface, and configures a sensor checker utility with a generic
        LidarCore model for parameter validation.
        """
        await stage_utils.create_new_stage_async()
        await app_utils.update_app_async()

        create_sarcophagus()

        # Initialize with LidarCore model
        model_info = ModelInfo()
        model_info.modelName = "lidar.core"
        model_info.modelVersion = "1.0"
        model_info.schemaVersion = "1.0"
        model_info.modelVendor = "nv"
        model_info.marketName = "GenericLidar"

        self._checker = SensorCheckerUtil()
        self._checker.init(model_info)

    async def tearDown(self):
        """Cleans up the test environment after lidar configuration testing.

        Removes the sensor checker, waits for any pending asset loading to complete,
        and updates the stage to ensure a clean state for subsequent tests.
        """
        del self._checker
        await app_utils.update_app_async()
        stage_utils.close_stage()
        await app_utils.update_app_async()


# Iterate over all supported lidar configs and variants, creating a test for each as sensor prims
def _create_lidar_parameters_test(config_path, config_name, variant):
    """Creates a test function for validating lidar sensor parameters with specified configuration and variant.

    Generates an async test function that creates an RTX lidar sensor prim with the given configuration
    and variant, then validates the sensor parameters using the sensor checker utility.

    Args:
        config_path: USD asset path of the lidar configuration to test.
        config_name: Name of the lidar configuration to test.
        variant: Variant of the lidar configuration to test.

    Returns:
        An async test function that validates the lidar sensor parameters.
    """

    async def test_function(self):
        # instantiate lidar prim
        lidar = Lidar.create(
            path="/asset",
            attributes={"omni:sensor:Core:outputFrameOfReference": "WORLD"},
            config=config_name,
            variant=variant,
        )

        # test cases
        # - prim path
        if config_path.endswith(".usda"):
            self.assertEqual(lidar.paths[0], "/asset")
        else:
            self.assertNotEqual(lidar.paths[0], "/asset")
        # - type
        prim_type = lidar.prims[0].GetTypeName()
        self.assertEqual(prim_type, "OmniLidar", f"Expected OmniLidar prim, got {prim_type}. Was sensor prim created?")
        # - validation
        error_string = self._checker.validateParams(lidar.prims[0])
        self.assertIsNone(
            error_string,
            f"Sensor parameter validation failed for config {config_name} variant {variant}: {error_string}",
        )
        validated_params = self._checker.getValidatedParams()
        self.assertIsNotNone(validated_params, "Failed to retrieve validated parameters")
        self.assertGreater(
            validated_params.numParams,
            0,
            f"Expected validated parameters, got {validated_params.numParams} parameters",
        )

        del lidar

    return test_function


def _variant_label(v):
    if v is None:
        return "default"
    if isinstance(v, str):
        return v
    return "__".join(f"{k}_{val}" for k, val in v.items())


for config_path in SUPPORTED_LIDAR_CONFIGS:
    config_name = Path(config_path).stem
    for variant in SUPPORTED_LIDAR_CONFIGS[config_path] or [None]:
        test_name = f"test_lidar_parameters__{config_name}__{_variant_label(variant)}"
        test_func = _create_lidar_parameters_test(config_path, config_name, variant)
        test_func.__name__ = test_name
        test_func.__doc__ = (
            f"Test supported lidar configs and variants, with config {config_name} and variant {variant}."
        )
        setattr(TestSupportedLidarConfigs, test_name, test_func)
