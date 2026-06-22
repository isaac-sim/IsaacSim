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

"""Verifies RTX sensor creation commands for lidar, radar, IDS, and ultrasonic sensors. Covers config handling, USD path variants, parented prim creation, undo behavior, and radar motion BVH settings."""

from pathlib import Path
from typing import Any

import carb
import omni.kit.commands
import omni.kit.test
import omni.kit.undo
import omni.usd
from isaacsim.core.utils.prims import delete_prim, get_prim_at_path
from isaacsim.core.utils.stage import traverse_stage
from isaacsim.sensors.rtx import SUPPORTED_LIDAR_CONFIGS, SUPPORTED_LIDAR_VARIANT_SET_NAME
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, Sdf, UsdGeom


def _variant_label(v: Any) -> Any:
    if v is None:
        return "default"
    if isinstance(v, str):
        return v
    return "__".join(f"{k}_{val}" for k, val in v.items())


class TestRtxSensorCommands(omni.kit.test.AsyncTestCase):
    """Test cases for RTX sensor creation commands."""

    async def setUp(self) -> None:
        """Create a new stage for each test."""
        await omni.usd.get_context().new_stage_async()
        self.stage = omni.usd.get_context().get_stage()

    async def tearDown(self) -> None:
        """Clean up the stage after each test."""
        await omni.usd.get_context().close_stage_async()

    async def test_create_rtx_lidar_configs(self) -> None:
        """Test creating RTX Lidar sensors with different configurations."""
        translation = Gf.Vec3d(0.0, 0.0, 0.0)
        orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)

        for config_path in SUPPORTED_LIDAR_CONFIGS:
            config = Path(config_path).stem
            vendor_name = Path(config_path).parts[3]
            # Try all possible supported config names:
            # 1. exact match to config path stem
            # 2. exact match to config path stem, with spaces instead of underscores
            # 3. exact match to config path stem, with vendor name stripped
            # 4. exact match to config path stem, with spaces instead of underscores, and vendor name stripped
            allowed_configs = [
                config,
                config.replace("_", " "),
            ]
            if config.startswith(vendor_name):
                allowed_configs.append(config[len(vendor_name) + 1 :])
                allowed_configs.append(config[len(vendor_name) + 1 :].replace("_", " "))
            # Create new stage for each config to avoid conflicts
            await omni.usd.get_context().new_stage_async()
            self.stage = omni.usd.get_context().get_stage()
            for i, allowed_config in enumerate(allowed_configs):

                for variant in SUPPORTED_LIDAR_CONFIGS[config_path] or [None]:
                    path = f"/RtxLidar_{config}_{i}_{_variant_label(variant)}"
                    _, prim = omni.kit.commands.execute(
                        "IsaacSensorCreateRtxLidar",
                        path=path,
                        config=allowed_config,
                        variant=variant,
                        translation=translation,
                        orientation=orientation,
                    )

                    self.assertIsNotNone(
                        prim, f"Failed to create prim for config {allowed_config} and variant {variant}"
                    )
                    self.assertEqual(prim.GetTypeName(), "OmniLidar")

                    if variant is not None:
                        prim = get_prim_at_path(path)
                        pairs = (
                            list(variant.items())
                            if isinstance(variant, dict)
                            else [(SUPPORTED_LIDAR_VARIANT_SET_NAME, variant)]
                        )
                        for set_name, expected in pairs:
                            variant_set = prim.GetVariantSet(set_name)
                            self.assertGreater(
                                len(variant_set.GetVariantNames()),
                                0,
                                f"Variant set '{set_name}' on prim {path} does not contain any variants.",
                            )
                            self.assertEqual(
                                variant_set.GetVariantSelection(),
                                expected,
                                f"Incorrect variant selection for set '{set_name}' (config {allowed_config}, variant {variant})",
                            )

    async def test_create_rtx_lidar_invalid_config(self) -> None:
        """Test creating an RTX Lidar sensor with an invalid configuration."""
        translation = Gf.Vec3d(0.0, 0.0, 0.0)
        orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
        path = "/RtxLidar_InvalidConfig"
        invalid_config = "invalid_config"

        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=path,
            config=invalid_config,
            translation=translation,
            orientation=orientation,
        )

        self.assertIsNotNone(prim)
        self.assertTrue(prim.IsValid())
        self.assertTrue(prim.IsA("OmniLidar"))

    async def test_create_rtx_radar_no_config(self) -> None:
        """Test creating an RTX Radar sensor without a configuration."""
        translation = Gf.Vec3d(0.0, 1.0, 2.0)
        orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)

        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxRadar",
            path="/RtxRadar",
            translation=translation,
            orientation=orientation,
        )

        self.assertIsNotNone(prim)
        self.assertTrue(prim.IsValid())
        self.assertTrue(prim.IsA("OmniRadar"))

    async def test_create_rtx_radar_with_config(self) -> None:
        """Test creating an RTX Radar sensor with a configuration."""
        translation = Gf.Vec3d(0.0, 1.0, 2.0)
        orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
        config = "test_radar_config"

        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxRadar",
            path="/RtxRadar",
            config=config,
            translation=translation,
            orientation=orientation,
        )

        self.assertIsNotNone(prim)
        self.assertTrue(prim.IsValid())
        self.assertTrue(prim.IsA("OmniRadar"))

    async def test_create_rtx_ids_default_config(self) -> None:
        """Test creating an RTX IDS sensor with default configuration."""
        translation = Gf.Vec3d(0.0, 0.0, 1.0)
        orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)

        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxIDS",
            path="/RtxIDS",
            translation=translation,
            orientation=orientation,
        )

        self.assertIsNotNone(prim)
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetPath(), Sdf.Path("/RtxIDS"))
        self.assertEqual(prim.GetTypeName(), "Camera")

        # Verify sensor type and default config
        sensor_type_attr = prim.GetAttribute("cameraSensorType")
        self.assertTrue(sensor_type_attr.IsValid())
        self.assertEqual(sensor_type_attr.Get(), "ids")

        config_attr = prim.GetAttribute("sensorModelConfig")
        self.assertTrue(config_attr.IsValid())
        self.assertEqual(config_attr.Get(), "idsoccupancy")

        plugin_attr = prim.GetAttribute("sensorModelPluginName")
        self.assertTrue(plugin_attr.IsValid())
        self.assertEqual(plugin_attr.Get(), "omni.sensors.nv.ids.ids.plugin")

    async def test_create_rtx_ids_with_config(self) -> None:
        """Test creating an RTX IDS sensor with a specific configuration."""
        translation = Gf.Vec3d(0.0, 0.0, 1.0)
        orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
        config = "test_ids_config"

        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxIDS",
            path="/RtxIDS",
            config=config,
            translation=translation,
            orientation=orientation,
        )

        self.assertIsNotNone(prim)
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetTypeName(), "Camera")

        sensor_type_attr = prim.GetAttribute("cameraSensorType")
        self.assertTrue(sensor_type_attr.IsValid())
        self.assertEqual(sensor_type_attr.Get(), "ids")

        config_attr = prim.GetAttribute("sensorModelConfig")
        self.assertTrue(config_attr.IsValid())
        self.assertEqual(config_attr.Get(), config)

        plugin_attr = prim.GetAttribute("sensorModelPluginName")
        self.assertTrue(plugin_attr.IsValid())
        self.assertEqual(plugin_attr.Get(), "omni.sensors.nv.ids.ids.plugin")

    async def test_create_rtx_ultrasonic_default(self) -> None:
        """Test creating an RTX Ultrasonic sensor with default configuration."""
        translation = Gf.Vec3d(-1.0, 0.0, 0.0)
        orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)

        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxUltrasonic",
            path="/RtxUltrasonic",
            translation=translation,
            orientation=orientation,
        )

        self.assertIsNotNone(prim)
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetPath(), Sdf.Path("/RtxUltrasonic"))
        self.assertEqual(prim.GetTypeName(), "Camera")

        # Verify sensor type
        sensor_type_attr = prim.GetAttribute("cameraSensorType")
        self.assertTrue(sensor_type_attr.IsValid())
        self.assertEqual(sensor_type_attr.Get(), "ultrasonic")

        plugin_attr = prim.GetAttribute("sensorModelPluginName")
        self.assertTrue(plugin_attr.IsValid())
        self.assertEqual(plugin_attr.Get(), "omni.sensors.nv.acoustic.wpm_ultrasonic.plugin")

    async def test_create_rtx_ultrasonic_with_config(self) -> None:
        """Test creating an RTX Ultrasonic sensor with a specific configuration."""
        translation = Gf.Vec3d(-1.0, 0.0, 0.0)
        orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
        config = "test_ultrasonic_config"

        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxUltrasonic",
            path="/RtxUltrasonic",
            config=config,
            translation=translation,
            orientation=orientation,
        )

        self.assertIsNotNone(prim)
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetTypeName(), "Camera")

        config_attr = prim.GetAttribute("sensorModelConfig")
        self.assertTrue(config_attr.IsValid())
        self.assertEqual(config_attr.Get(), config)

        plugin_attr = prim.GetAttribute("sensorModelPluginName")
        self.assertTrue(plugin_attr.IsValid())
        self.assertEqual(plugin_attr.Get(), "omni.sensors.nv.acoustic.wpm_ultrasonic.plugin")

    async def test_sensor_with_parent(self) -> None:
        """Test creating a sensor with a parent prim."""
        # Create a parent Xform
        parent_path = "/World"
        parent = UsdGeom.Xform.Define(self.stage, parent_path)

        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/RtxLidar",
            parent=parent_path,
            config="OS1",
        )

        self.assertIsNotNone(prim)

        # Find OmniLidar prim by traversing the stage
        found_lidar = False
        for prim in traverse_stage():
            if prim.IsA("OmniLidar"):
                found_lidar = True
                self.assertTrue(prim.GetPath().HasPrefix(parent_path))
                break

        self.assertTrue(found_lidar, "Failed to find OmniLidar prim with parent")

    async def test_rtx_radar_force_camera_prim_with_config(self) -> None:
        """Test RTX Radar with force_camera_prim=True and config specified."""
        translation = Gf.Vec3d(0.0, 1.0, 2.0)
        orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
        config = "test_radar_config"

        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxRadar",
            path="/RtxRadar",
            config=config,
            translation=translation,
            orientation=orientation,
            force_camera_prim=True,
        )

        self.assertIsNotNone(prim)
        self.assertTrue(prim.IsValid())
        # Verify it's a Camera prim with radar sensor type
        self.assertTrue(prim.IsA(UsdGeom.Camera))
        sensor_type_attr = prim.GetAttribute("cameraSensorType")
        self.assertTrue(sensor_type_attr.IsValid())
        self.assertEqual(sensor_type_attr.Get(), "radar")
        # Verify config is set
        config_attr = prim.GetAttribute("sensorModelConfig")
        self.assertTrue(config_attr.IsValid())
        self.assertEqual(config_attr.Get(), config)

        plugin_attr = prim.GetAttribute("sensorModelPluginName")
        self.assertTrue(plugin_attr.IsValid())
        self.assertEqual(plugin_attr.Get(), "omni.sensors.nv.radar.wpm_dmatapprox.plugin")

    async def test_rtx_lidar_force_camera_prim_with_config(self) -> None:
        """Test RTX Lidar with force_camera_prim=True and config specified."""
        translation = Gf.Vec3d(0.0, 0.0, 0.0)
        orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
        config = Path(list(SUPPORTED_LIDAR_CONFIGS.keys())[0]).stem  # Use the first available config

        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/RtxLidar",
            config=config,
            translation=translation,
            orientation=orientation,
            force_camera_prim=True,
        )

        self.assertIsNotNone(prim)
        self.assertTrue(prim.IsValid())
        # Verify it's a Camera prim with lidar sensor type
        self.assertTrue(prim.IsA(UsdGeom.Camera))

        sensor_type_attr = prim.GetAttribute("cameraSensorType")
        self.assertTrue(sensor_type_attr.IsValid())
        self.assertEqual(sensor_type_attr.Get(), "lidar")
        # Verify config is set
        config_attr = prim.GetAttribute("sensorModelConfig")
        self.assertTrue(config_attr.IsValid())
        self.assertEqual(config_attr.Get(), config)

        plugin_attr = prim.GetAttribute("sensorModelPluginName")
        self.assertTrue(plugin_attr.IsValid())
        self.assertEqual(plugin_attr.Get(), "omni.sensors.nv.lidar.lidar_core.plugin")

    async def test_rtx_lidar_default_creation(self) -> None:
        """Test RTX Lidar default creation (no config, no force_camera_prim)."""
        translation = Gf.Vec3d(0.0, 0.0, 0.0)
        orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)

        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/RtxLidar",
            translation=translation,
            orientation=orientation,
        )

        self.assertIsNotNone(prim)
        self.assertTrue(prim.IsValid())
        # Verify it's an OmniLidar prim
        self.assertTrue(prim.IsA("OmniLidar"))

    async def test_rtx_lidar_accumulate_outputs_set(self) -> None:
        """Test that IsaacSensorCreateRtxLidar sets accumulateOutputs=True on OmniLidar prims."""
        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/RtxLidar_AccumTest",
            config="Example_Rotary",
            translation=Gf.Vec3d(0.0, 0.0, 0.0),
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        )

        self.assertIsNotNone(prim)
        self.assertTrue(prim.IsValid())
        self.assertTrue(prim.IsA("OmniLidar"))

        attr = prim.GetAttribute("omni:sensor:Core:accumulateOutputs")
        self.assertTrue(attr.IsValid(), "Expected accumulateOutputs attribute on OmniLidar prim.")
        self.assertEqual(
            attr.Get(),
            True,
            "Expected accumulateOutputs to be True after command execution.",
        )

    async def test_rtx_lidar_default_creation_with_path(self) -> None:
        """Test RTX Lidar default creation (no config, no force_camera_prim)."""
        print(get_prim_at_path("/").GetAllChildren())
        translation = Gf.Vec3d(0.0, 0.0, 0.0)
        orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)

        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/RtxLidar",
            translation=translation,
            orientation=orientation,
        )

        self.assertIsNotNone(prim)
        self.assertTrue(prim.IsValid())
        # Verify it's an OmniLidar prim
        self.assertTrue(prim.IsA("OmniLidar"))
        self.assertEqual(prim.GetPath(), Sdf.Path("/RtxLidar"))
        delete_prim(prim.GetPath())

        # Create with parent
        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/RtxLidar",
            parent="/Render",
            translation=translation,
            orientation=orientation,
        )

        self.assertIsNotNone(prim)
        self.assertTrue(prim.IsValid())
        # Verify it's an OmniLidar prim
        self.assertTrue(prim.IsA("OmniLidar"))
        self.assertEqual(prim.GetPath(), Sdf.Path("/Render/RtxLidar"))
        delete_prim(prim.GetPath())

        # Create with parent in path
        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/Render/RtxLidar",
            translation=translation,
            orientation=orientation,
        )

        self.assertIsNotNone(prim)
        self.assertTrue(prim.IsValid())
        # Verify it's an OmniLidar prim
        self.assertTrue(prim.IsA("OmniLidar"))
        self.assertEqual(prim.GetPath(), Sdf.Path("/Render/RtxLidar"))

    async def test_rtx_lidar_force_camera_no_config(self) -> None:
        """Test RTX Lidar with force_camera_prim=True but no config specified."""
        translation = Gf.Vec3d(0.0, 0.0, 0.0)
        orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)

        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/RtxLidar",
            translation=translation,
            orientation=orientation,
            force_camera_prim=True,
        )

        self.assertIsNotNone(prim)
        self.assertTrue(prim.IsValid())
        # Verify it's a Camera prim with lidar sensor type
        self.assertTrue(prim.IsA(UsdGeom.Camera))
        sensor_type_attr = prim.GetAttribute("cameraSensorType")
        self.assertTrue(sensor_type_attr.IsValid())
        self.assertEqual(sensor_type_attr.Get(), "lidar")
        # Verify sensorModelConfig attribute doesn't exist
        config_attr = prim.GetAttribute("sensorModelConfig")
        self.assertFalse(config_attr.IsValid())

        plugin_attr = prim.GetAttribute("sensorModelPluginName")
        self.assertTrue(plugin_attr.IsValid())
        self.assertEqual(plugin_attr.Get(), "omni.sensors.nv.lidar.lidar_core.plugin")

    async def test_rtx_lidar_usd_path_no_variant(self) -> None:
        """Test RTX Lidar creation with usd_path (no variant)."""
        translation = Gf.Vec3d(0.0, 0.0, 0.0)
        orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
        # Use a lidar config without variants
        usd_path = get_assets_root_path() + "/Isaac/Sensors/NVIDIA/Example_Rotary.usda"

        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/RtxLidar_UsdPath",
            usd_path=usd_path,
            translation=translation,
            orientation=orientation,
        )

        self.assertIsNotNone(prim)
        self.assertTrue(prim.IsValid())
        self.assertTrue(prim.IsA("OmniLidar"))

    async def test_rtx_lidar_usd_path_with_variant(self) -> None:
        """Test RTX Lidar creation with usd_path that has variants."""
        translation = Gf.Vec3d(0.0, 0.0, 0.0)
        orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
        # Use a lidar config with variants (Ouster OS0)
        usd_path = get_assets_root_path() + "/Isaac/Sensors/Ouster/OS0/OS0.usd"

        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/RtxLidar_UsdPathVariant",
            usd_path=usd_path,
            translation=translation,
            orientation=orientation,
        )

        self.assertIsNotNone(prim)
        self.assertTrue(prim.IsValid())
        self.assertTrue(prim.IsA("OmniLidar"))

    async def test_rtx_lidar_usd_path_with_variant_selection(self) -> None:
        """Test RTX Lidar creation with usd_path and variant selection."""
        translation = Gf.Vec3d(0.0, 0.0, 0.0)
        orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
        # Use a lidar config with variants (Ouster OS0) and select a specific variant
        usd_path = get_assets_root_path() + "/Isaac/Sensors/Ouster/OS0/OS0.usd"
        variant = "OS0_REV6_128ch10hz1024res"

        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/RtxLidar_UsdPathVariantSelection",
            usd_path=usd_path,
            variant=variant,
            translation=translation,
            orientation=orientation,
        )

        self.assertIsNotNone(prim)
        self.assertTrue(prim.IsValid())
        self.assertTrue(prim.IsA("OmniLidar"))

        # Verify the variant was set correctly
        root_prim = get_prim_at_path("/RtxLidar_UsdPathVariantSelection")
        variant_set = root_prim.GetVariantSet(SUPPORTED_LIDAR_VARIANT_SET_NAME)
        self.assertGreater(len(variant_set.GetVariantNames()), 0)
        current_variant = variant_set.GetVariantSelection()
        self.assertEqual(current_variant, variant)

    async def test_rtx_lidar_config_and_usd_path_both_provided(self) -> None:
        """Test that config takes precedence over usd_path when both are provided."""
        translation = Gf.Vec3d(0.0, 0.0, 0.0)
        orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
        # Provide both config and usd_path - config should take precedence
        config = "OS1"
        usd_path = get_assets_root_path() + "/Isaac/Sensors/NVIDIA/Example_Rotary.usda"

        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/RtxLidar_BothConfigAndUsdPath",
            config=config,
            usd_path=usd_path,
            translation=translation,
            orientation=orientation,
        )

        self.assertIsNotNone(prim)
        self.assertTrue(prim.IsValid())
        self.assertTrue(prim.IsA("OmniLidar"))
        # The prim should be from the config (OS1), not the usd_path

    async def test_rtx_radar_usd_path(self) -> None:
        """Test RTX Radar creation with usd_path."""
        translation = Gf.Vec3d(0.0, 1.0, 2.0)
        orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
        # Note: There are no standard radar USD files in SUPPORTED configs,
        # so this will fall back to replicator API
        # We're testing that usd_path doesn't break the flow

        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxRadar",
            path="/RtxRadar_UsdPath",
            translation=translation,
            orientation=orientation,
        )

        self.assertIsNotNone(prim)
        self.assertTrue(prim.IsValid())
        self.assertTrue(prim.IsA("OmniRadar"))

    async def test_undo_deletes_prim(self) -> None:
        """Test that undo properly deletes the created prim."""
        translation = Gf.Vec3d(0.0, 0.0, 0.0)
        orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
        path = "/RtxLidar_UndoTest"

        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=path,
            translation=translation,
            orientation=orientation,
        )

        self.assertIsNotNone(prim)
        self.assertTrue(prim.IsValid())

        # Now undo the command
        omni.kit.undo.undo()

        # Verify the prim was deleted
        deleted_prim = get_prim_at_path(path)
        self.assertFalse(deleted_prim.IsValid())

    async def test_undo_with_config(self) -> None:
        """Test that undo properly deletes a prim created with config."""
        translation = Gf.Vec3d(0.0, 0.0, 0.0)
        orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
        path = "/RtxLidar_UndoConfigTest"

        _, prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=path,
            config="OS1",
            translation=translation,
            orientation=orientation,
        )

        self.assertIsNotNone(prim)
        self.assertTrue(prim.IsValid())

        # Now undo the command
        omni.kit.undo.undo()

        # Verify the prim was deleted
        deleted_prim = get_prim_at_path(path)
        self.assertFalse(deleted_prim.IsValid())

    async def test_rtx_radar_motion_bvh_disabled(self) -> None:
        """Test that RTX Radar creation fails when Motion BVH is disabled."""
        settings = carb.settings.get_settings()
        # Save the original setting
        original_value = settings.get("/renderer/raytracingMotion/enabled")

        try:
            # Disable Motion BVH
            settings.set("/renderer/raytracingMotion/enabled", False)

            translation = Gf.Vec3d(0.0, 1.0, 2.0)
            orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)

            _, prim = omni.kit.commands.execute(
                "IsaacSensorCreateRtxRadar",
                path="/RtxRadar_MotionBVHDisabled",
                translation=translation,
                orientation=orientation,
            )

            # Prim should be None when Motion BVH is disabled
            self.assertIsNone(prim)

            # Verify no prim was created at the path
            check_prim = get_prim_at_path("/RtxRadar_MotionBVHDisabled")
            self.assertFalse(check_prim.IsValid())
        finally:
            # Restore the original setting
            settings.set("/renderer/raytracingMotion/enabled", original_value)

    async def test_rtx_radar_motion_bvh_enabled(self) -> None:
        """Test that RTX Radar creation succeeds when Motion BVH is enabled."""
        settings = carb.settings.get_settings()
        # Save the original setting
        original_value = settings.get("/renderer/raytracingMotion/enabled")

        try:
            # Enable Motion BVH
            settings.set("/renderer/raytracingMotion/enabled", True)

            translation = Gf.Vec3d(0.0, 1.0, 2.0)
            orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)

            _, prim = omni.kit.commands.execute(
                "IsaacSensorCreateRtxRadar",
                path="/RtxRadar_MotionBVHEnabled",
                translation=translation,
                orientation=orientation,
            )

            # Prim should be created successfully when Motion BVH is enabled
            self.assertIsNotNone(prim)
            self.assertTrue(prim.IsValid())
            self.assertTrue(prim.IsA("OmniRadar"))
        finally:
            # Restore the original setting
            settings.set("/renderer/raytracingMotion/enabled", original_value)
