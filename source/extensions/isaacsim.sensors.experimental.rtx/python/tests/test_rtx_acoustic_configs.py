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

"""Verifies every supported RTX acoustic configuration and variant by creating an Acoustic prim and validating its parameters with SensorCheckerUtil."""

from pathlib import Path
from typing import Any

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
from isaacsim.sensors.experimental.rtx import SUPPORTED_ACOUSTIC_CONFIGS, Acoustic
from isaacsim.sensors.experimental.rtx.sensor_checker import ModelInfo, SensorCheckerUtil

from .common import create_sarcophagus


class TestSupportedAcousticConfigs(omni.kit.test.AsyncTestCase):
    """Test class for validating all supported acoustic configurations and variants in the isaacsim.sensors.experimental.rtx extension.

    This test class dynamically generates test methods for each supported acoustic configuration and variant
    combination from SUPPORTED_ACOUSTIC_CONFIGS. Each test method creates an acoustic sensor prim using
    Acoustic.create() with the asset USD path and validates its parameters using the SensorCheckerUtil with
    the acoustic model.

    The class sets up a test environment with a new stage and a sarcophagus object for sensor validation. It
    uses the SensorCheckerUtil to verify that each acoustic configuration produces valid sensor parameters
    that conform to the expected schema and model requirements.

    Test methods are automatically generated at module load time using the _create_acoustic_parameters_test
    function, which creates individual test cases for each config-variant pair. SUPPORTED_ACOUSTIC_CONFIGS is
    empty by default; OEM acoustic assets get added there and validation tests appear automatically.
    """

    async def setUp(self) -> None:
        """Sets up the test environment for acoustic configuration testing.

        Creates a new USD stage, initializes a sarcophagus object with non-visual materials,
        sets up the timeline interface, and configures a sensor checker utility with a generic
        acoustic model for parameter validation.
        """
        await stage_utils.create_new_stage_async()
        await app_utils.update_app_async()

        create_sarcophagus()

        model_info = ModelInfo()
        model_info.modelName = "acoustic"
        model_info.modelVersion = "1.0"
        model_info.schemaVersion = "1.0"
        model_info.modelVendor = "nv"
        model_info.marketName = "Generic"

        self._checker = SensorCheckerUtil()
        self._checker.init(model_info)

    async def tearDown(self) -> None:
        """Cleans up the test environment after acoustic configuration testing.

        Removes the sensor checker, waits for any pending asset loading to complete,
        and updates the stage to ensure a clean state for subsequent tests.
        """
        del self._checker
        await app_utils.update_app_async()
        stage_utils.close_stage()
        await app_utils.update_app_async()


def _create_acoustic_parameters_test(config_path: Any, variant: Any) -> Any:
    """Creates a test function for validating acoustic sensor parameters with specified configuration and variant.

    Generates an async test function that creates an RTX acoustic sensor prim using Acoustic.create() with
    the given config name and variant, then validates the sensor parameters using the sensor checker utility.

    Args:
        config_path: USD asset path of the acoustic configuration to test.
        variant: Variant of the acoustic configuration to test, or None for the default.

    Returns:
        An async test function that validates the acoustic sensor parameters.
    """

    async def test_function(self: Any) -> None:
        config_name = Path(config_path).stem
        acoustic = Acoustic.create(
            path="/asset",
            config=config_name,
            variant=variant,
        )

        # - prim path: binary .usd files have a sub-prim discovered within the reference
        self.assertNotEqual(acoustic.paths[0], "/asset")
        # - type
        prim_type = acoustic.prims[0].GetTypeName()
        self.assertEqual(
            prim_type, "OmniAcoustic", f"Expected OmniAcoustic prim, got {prim_type}. Was sensor prim created?"
        )
        # - validation
        error_string = self._checker.validateParams(acoustic.prims[0])
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

        del acoustic

    return test_function


def _variant_label(v: Any) -> Any:
    if v is None:
        return "default"
    if isinstance(v, str):
        return v
    return "__".join(f"{k}_{val}" for k, val in v.items())


for config_path in SUPPORTED_ACOUSTIC_CONFIGS:
    config_name = Path(config_path).stem
    for variant in SUPPORTED_ACOUSTIC_CONFIGS[config_path] or [None]:
        test_name = f"test_acoustic_parameters__{config_name}__{_variant_label(variant)}"
        test_func = _create_acoustic_parameters_test(config_path, variant)
        test_func.__name__ = test_name
        test_func.__doc__ = (
            f"Test supported acoustic configs and variants, with config {config_name} and variant {variant}."
        )
        setattr(TestSupportedAcousticConfigs, test_name, test_func)
