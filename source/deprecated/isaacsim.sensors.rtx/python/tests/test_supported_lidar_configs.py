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

"""Tests for validating all supported lidar configurations and variants in the isaacsim.sensors.rtx extension."""

from pathlib import Path

import omni.kit.test
from isaacsim.core.utils.stage import create_new_stage_async, update_stage_async
from isaacsim.sensors.rtx import SUPPORTED_LIDAR_CONFIGS
from isaacsim.sensors.rtx.sensor_checker import ModelInfo, SensorCheckerUtil
from pxr import Gf

from .common import create_sarcophagus


class TestSupportedLidarConfigs(omni.kit.test.AsyncTestCase):
    """Validates all supported lidar configurations and variants.

    A single test method iterates over every (config, variant) pair in SUPPORTED_LIDAR_CONFIGS,
    creating a sensor prim for each and validating its parameters with SensorCheckerUtil.
    The stage and scene geometry are created once; the checker is reused across all configs.
    """

    async def setUp(self) -> None:
        """Set up the test environment with a new stage and sarcophagus scene."""
        await create_new_stage_async()
        await update_stage_async()
        create_sarcophagus(enable_nonvisual_material=True)

    async def tearDown(self) -> None:
        """Tear down the test environment."""
        await omni.kit.app.get_app().next_update_async()

    async def test_supported_lidar_configs(self) -> None:
        """Test all supported lidar configurations and variants."""
        checker = SensorCheckerUtil()

        model_info = ModelInfo()
        model_info.modelName = "lidar.core"
        model_info.modelVersion = "1.0"
        model_info.schemaVersion = "1.0"
        model_info.modelVendor = "nv"
        model_info.marketName = "GenericLidar"

        error = checker.init(model_info)
        self.assertIsNone(error, f"SensorCheckerUtil.init() failed: {error}")

        stage = omni.usd.get_context().get_stage()
        failures = []

        for config_path in SUPPORTED_LIDAR_CONFIGS:
            config_name = Path(config_path).stem
            for variant in SUPPORTED_LIDAR_CONFIGS[config_path] or [None]:
                label = f"{config_name}/{variant}"

                _, sensor = omni.kit.commands.execute(
                    "IsaacSensorCreateRtxLidar",
                    path="lidar",
                    parent=None,
                    translation=Gf.Vec3d(0.0, 0.0, 0.0),
                    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
                    config=config_name,
                    variant=variant,
                    **{"omni:sensor:Core:outputFrameOfReference": "WORLD"},
                )

                if sensor is None:
                    failures.append(f"{label}: IsaacSensorCreateRtxLidar returned None")
                    continue

                sensor_type = sensor.GetTypeName()
                if sensor_type != "OmniLidar":
                    failures.append(f"{label}: expected OmniLidar, got {sensor_type}")
                else:
                    error_string = checker.validateParams(sensor)
                    if error_string is not None:
                        failures.append(f"{label}: {error_string}")
                    else:
                        validated_params = checker.getValidatedParams()
                        if validated_params is None or validated_params.numParams == 0:
                            failures.append(f"{label}: got 0 validated parameters")

                prim_path = str(sensor.GetPath())
                stage.RemovePrim(prim_path)

        if failures:
            msg = f"{len(failures)} config(s) failed:\n" + "\n".join(f"  - {f}" for f in failures)
            self.fail(msg)
