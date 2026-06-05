# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Kit commands for creating RTX sensors in Isaac Sim.

This module provides command classes for creating various RTX sensors
including Lidar, Radar, IDS, and Ultrasonic sensors.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import Any

import carb
import omni.isaac.IsaacSensorSchema as IsaacSensorSchema
import omni.kit.commands
import omni.kit.utils
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.utils.prims import delete_prim
from isaacsim.core.utils.stage import add_reference_to_stage, get_next_free_path
from isaacsim.core.utils.xforms import reset_and_set_xform_ops
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, Sdf, Usd, UsdGeom

from .supported_lidar_configs import SUPPORTED_LIDAR_CONFIGS, SUPPORTED_LIDAR_VARIANT_SET_NAME


class IsaacSensorCreateRtxSensor(omni.kit.commands.Command):
    """Base class for creating RTX sensors in Isaac Sim.

    This class provides functionality to create various types of RTX sensors (lidar, radar, etc.)
    in the Isaac Sim environment. It handles sensor creation through either USD references,
    Replicator API, or direct camera prim creation.

    Args:
        path: Path where the sensor will be created. If None, a default path will be used.
        parent: Parent prim path for the sensor.
        config: Configuration name for the sensor.
        usd_path: Path to a USD file containing the sensor asset. If both config and usd_path are provided, config
            takes precedence and a warning is logged.
        translation: 3D translation vector for sensor placement.
        orientation: Quaternion for sensor orientation.
        visibility: Visibility flag for the sensor.
        variant: Variant name for the sensor configuration.
        force_camera_prim: If True, forces creation of a camera prim instead of using references or Replicator API.
        **kwargs: Additional keyword arguments for prim creation.
    """

    _replicator_api: Callable[..., Any] | None = None
    """Static method reference to the Replicator API for sensor creation."""
    _sensor_type: str = "sensor"
    """String identifier for the type of sensor."""
    _supported_configs: list[Any] | dict[str, set[str]] = []
    """List of supported sensor configurations."""
    _schema: Any = None
    """Schema for the sensor type."""
    _sensor_plugin_name: str = ""
    """Name of the sensor plugin."""
    _camera_config_name: str = ""
    """Name of the camera configuration."""

    def __init__(
        self,
        path: str | None = None,
        parent: str | None = None,
        config: str | None = None,
        usd_path: str | None = None,
        translation: Gf.Vec3d | None = Gf.Vec3d(0, 0, 0),
        orientation: Gf.Quatd | None = Gf.Quatd(1, 0, 0, 0),
        visibility: bool = False,
        variant: str | dict[str, str] | None = None,
        force_camera_prim: bool = False,
        **kwargs: Any,
    ) -> None:
        self._parent = parent
        self._config = config
        self._usd_path = usd_path
        self._translation = translation
        self._orientation = orientation
        self._visibility = visibility
        self._variant = variant
        self._force_camera_prim = force_camera_prim
        self._prim_creation_kwargs = kwargs
        self._prim = None
        self._path = path or f"/Rtx{self._sensor_type.capitalize()}"
        self._desired_prim_type = f"Omni{self._sensor_type.capitalize()}"
        self._camera_config = self._config

        # Warn if both config and usd_path are provided
        if self._config and self._usd_path:
            carb.log_warn(
                f"Both 'config' and 'usd_path' provided for Omni{self._sensor_type.capitalize()}. "
                f"Using 'config' ({self._config}) and ignoring 'usd_path' ({self._usd_path})."
            )
            self._usd_path = None

    def _add_reference(self) -> Usd.Prim | None:
        """Add a reference to the stage if a config or usd_path is provided.

        If a config is provided, this method looks up the corresponding USD path from
        supported configs. If usd_path is provided directly, it uses that path.
        The method adds a reference to the stage and sets the prim's variant if provided.
        It also handles finding the correct sensor prim within referenced assets.

        Returns:
            The created or found prim, or None if no config/usd_path was provided or found.
        """
        usd_path_to_load = None
        prim_type = "Xform"

        if self._config:
            found_config = False
            for config in self._supported_configs:
                config_path = Path(config)
                vendor_name = config_path.parts[3]
                config_name = config_path.stem
                config_name_without_vendor = config_name
                if config_name.startswith(vendor_name):
                    config_name_without_vendor = config_name[len(vendor_name) + 1 :]
                if (
                    self._config == config_name
                    or self._config == config_name.replace("_", " ")
                    or self._config == config_name_without_vendor
                    or self._config == config_name_without_vendor.replace("_", " ")
                ):
                    found_config = True
                    usd_path_to_load = get_assets_root_path() + config
                    prim_type = self._desired_prim_type if config.endswith(".usda") else "Xform"
                    break
            if not found_config:
                carb.log_warn(
                    f"Config '{self._config}' not found for Omni{self._sensor_type.capitalize()} at {self._prim_path}."
                )
                return None
        elif self._usd_path:
            usd_path_to_load = self._usd_path
            prim_type = self._desired_prim_type if self._usd_path.endswith(".usda") else "Xform"

        if usd_path_to_load is None:
            return None

        prim = add_reference_to_stage(
            usd_path=usd_path_to_load,
            prim_path=self._prim_path,
            prim_type=prim_type,
        )
        reset_and_set_xform_ops(prim.GetPrim(), self._translation, self._orientation)

        if self._variant:
            # Check if the variant is in the allowed list (if using config with supported configs)
            if isinstance(self._supported_configs, dict) and self._config:
                config_key = None
                for config in self._supported_configs:
                    if usd_path_to_load.endswith(config) or usd_path_to_load == get_assets_root_path() + config:
                        config_key = config
                        break
                if config_key:
                    allowed_variants = self._supported_configs[config_key]
                    if self._variant not in allowed_variants:
                        carb.log_warn(
                            f"Variant '{self._variant}' not found for Omni{self._sensor_type.capitalize()} at {self._prim_path}. Allowed variants: {allowed_variants}."
                        )

            pairs = (
                list(self._variant.items())
                if isinstance(self._variant, dict)
                else [(SUPPORTED_LIDAR_VARIANT_SET_NAME, self._variant)]
            )
            for set_name, selection in pairs:
                variant_set = prim.GetVariantSet(set_name)
                if len(variant_set.GetVariantNames()) == 0:
                    carb.log_warn(
                        f"Variant set {set_name} for Omni{self._sensor_type.capitalize()} at {self._prim_path} does not contain any variants."
                    )
                elif not variant_set.SetVariantSelection(selection):
                    carb.log_warn(
                        f"Variant '{selection}' not found in set '{set_name}' for Omni{self._sensor_type.capitalize()} at {self._prim_path}. Available variants: {variant_set.GetVariantNames()}."
                    )

        # If necessary, traverse children of referenced asset to find OmniSensor prim
        # Note: if multiple children of the referenced asset are OmniSensor types, this will select the first one
        if prim.GetTypeName() == "Xform":
            found_sensor = False
            for child in Usd.PrimRange(prim):
                if child.GetTypeName() == self._desired_prim_type:
                    carb.log_info(f"Using {self._desired_prim_type} prim at path {child.GetPath()}")
                    prim = child
                    found_sensor = True
                    break
            if not found_sensor:
                carb.log_error(
                    f"No {self._desired_prim_type} prim found in referenced asset at {self._prim_path}. "
                    f"USD path: {usd_path_to_load}"
                )
                self.undo()
                return None

        for attr, value in self._prim_creation_kwargs.items():
            if prim.HasAttribute(attr):
                prim.GetAttribute(attr).Set(value)
        return prim

    def _call_replicator_api(self) -> Usd.Prim | None:
        """Create a sensor using the Replicator API.

        Converts position and orientation into the format required by the Replicator API
        and creates the sensor prim.

        Returns:
            The created prim, or None if no Replicator API is available.
        """
        if self._replicator_api is not None and self._translation is not None and self._orientation is not None:
            # Convert position and orientation into tuples for Replicator API.
            position = (self._translation[0], self._translation[1], self._translation[2])
            rotation = Gf.Rotation(self._orientation)
            euler_angles_as_vec = rotation.Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())
            euler_angles = (euler_angles_as_vec[0], euler_angles_as_vec[1], euler_angles_as_vec[2])

            # First, construct the full path
            full_path = self._parent or ""
            full_path += "" if self._path.startswith("/") else "/"
            full_path += self._path

            # Then, split the full path into components
            components = full_path.split("/")
            if len(components) > 2 or components[0]:
                self._parent = "/".join(components[:-1])
            self._path = components[-1]

            return self._replicator_api(
                position=position,
                rotation=euler_angles,
                name=self._path,
                parent=self._parent,
                **self._prim_creation_kwargs,
            )
        return None

    def _create_camera_prim(self) -> Usd.Prim:
        """Create a camera prim for the sensor.

        This method is deprecated as of Isaac Sim 5.0. It creates a basic camera prim
        with sensor-specific attributes.

        Returns:
            The created camera prim.
        """
        carb.log_warn(
            "Support for creating RTX sensors as camera prims is deprecated as of Isaac Sim 5.0, and support will be removed in a future release. Please use an OmniSensor prim instead."
        )
        prim = UsdGeom.Camera.Define(self._stage, Sdf.Path(self._prim_path)).GetPrim()
        if self._schema:
            self._schema.Apply(prim)
        camSensorTypeAttr = prim.CreateAttribute("cameraSensorType", Sdf.ValueTypeNames.Token, False)
        camSensorTypeAttr.Set(self._sensor_type)
        tokens = camSensorTypeAttr.GetMetadata("allowedTokens")
        prim.CreateAttribute("sensorModelPluginName", Sdf.ValueTypeNames.String, False).Set(self._sensor_plugin_name)
        if not tokens:
            camSensorTypeAttr.SetMetadata("allowedTokens", ["camera", "radar", "lidar", "ids", "ultrasonic"])
        if self._camera_config:
            prim.CreateAttribute("sensorModelConfig", Sdf.ValueTypeNames.String, False).Set(self._camera_config)
        return prim

    def do(self) -> Usd.Prim:
        """Execute the sensor creation command.

        Creates the sensor using the most appropriate method based on the configuration
        and available APIs.

        Returns:
            The created sensor prim.
        """
        self._stage = omni.usd.get_context().get_stage()
        if not self._path.startswith("/"):
            self._path = "/" + self._path
        self._prim_path = get_next_free_path(self._path, self._parent)
        self._prim = (
            not self._force_camera_prim and (self._add_reference() or self._call_replicator_api())
        ) or self._create_camera_prim()
        return self._prim

    def undo(self) -> None:
        """Undo the sensor creation command by deleting the created prim."""
        if self._prim_path:
            delete_prim(self._prim_path)


class IsaacSensorCreateRtxLidar(IsaacSensorCreateRtxSensor):
    """Command class for creating RTX Lidar sensors.

    This class specializes the base RTX sensor creation for Lidar sensors, providing
    specific configuration and plugin settings for Lidar functionality.

    Args:
        **kwargs: Keyword arguments passed to the parent class constructor.
            See IsaacSensorCreateRtxSensor for available parameters.
    """

    _replicator_api: Callable[..., Any] = staticmethod(rep.functional.create.omni_lidar)
    """Static method reference to the Lidar Replicator API for creating Lidar sensors."""
    _sensor_type: str = "lidar"
    """String identifier set to "lidar" for the sensor type."""
    _supported_configs: dict[str, set[str]] = SUPPORTED_LIDAR_CONFIGS
    """Dictionary mapping supported Lidar configurations to their available variants."""
    _schema: Any = IsaacSensorSchema.IsaacRtxLidarSensorAPI
    """Schema for Lidar sensors using IsaacRtxLidarSensorAPI."""
    _sensor_plugin_name: str = "omni.sensors.nv.lidar.lidar_core.plugin"
    """Name of the Lidar sensor plugin "omni.sensors.nv.lidar.lidar_core.plugin"."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        if self._config and self._config.startswith("OS") and len(self._config) > 3:
            carb.log_warn(
                "Support for adding variant lidar models to the stage via config name only has been deprecated in Isaac Sim 5.0. Please use the config (model name) and variant (model variant) arguments instead."
            )
            # In Isaac Sim 5.0, Ouster lidar configs were implemented as variants on the same prim in the main USD file
            self._variant = self._config
            self._config = self._config[:3]  # truncate to OS0, OS1, or OS2
            self._camera_config = self._variant
            carb.log_warn(
                f"Example: omni.kit.commands.execute('IsaacSensorCreateRtxLidar', config='{self._config}', variant='{self._variant}')"
            )

    def do(self) -> Usd.Prim:
        """Execute the Lidar sensor creation command.

        Returns:
            The created Lidar sensor prim.
        """
        prim = super().do()
        if prim.IsValid():
            if prim.HasAttribute("omni:sensor:Core:skipDroppingInvalidPoints"):
                prim.GetAttribute("omni:sensor:Core:skipDroppingInvalidPoints").Set(False)
            if prim.HasAttribute("omni:sensor:Core:accumulateOutputs"):
                prim.GetAttribute("omni:sensor:Core:accumulateOutputs").Set(True)
            # WAR: Expose auxOutputType as "channels" for Replicator API to assign to RenderVar as attribute
            aux_output_type = None
            if "omni:sensor:Core:auxOutputType" in self._prim_creation_kwargs:
                attr_value = self._prim_creation_kwargs["omni:sensor:Core:auxOutputType"]
                aux_output_type = [str(attr_value)]
            if prim.HasAttribute("_replicator:rendervar:GenericModelOutput:omni:sensor:Core:auxOutputType"):
                attr_value = prim.GetAttribute(
                    "_replicator:rendervar:GenericModelOutput:omni:sensor:Core:auxOutputType"
                ).Get()
                prim.RemoveProperty("_replicator:rendervar:GenericModelOutput:omni:sensor:Core:auxOutputType")
                aux_output_type = [str(attr_value)]
            if aux_output_type is not None:
                prim.CreateAttribute(
                    "_replicator:rendervar:GenericModelOutput:channels", Sdf.ValueTypeNames.StringArray, True
                ).Set(aux_output_type)
        return prim


class IsaacSensorCreateRtxRadar(IsaacSensorCreateRtxSensor):
    """Command class for creating RTX Radar sensors.

    This class specializes the base RTX sensor creation for Radar sensors, providing
    specific configuration and plugin settings for Radar functionality.

    RTX Radar requires Motion BVH to be enabled. If Motion BVH is not enabled,
    the command will warn the user and not create the prim.

    Args:
        **kwargs: Keyword arguments passed to the parent class constructor.
            See IsaacSensorCreateRtxSensor for available parameters.
    """

    _replicator_api: Callable[..., Any] = staticmethod(rep.functional.create.omni_radar)
    """Static method reference to the Radar Replicator API."""
    _sensor_type: str = "radar"
    """Set to "radar"."""
    _schema: Any = IsaacSensorSchema.IsaacRtxRadarSensorAPI
    """Schema for Radar sensors."""
    _sensor_plugin_name: str = "omni.sensors.nv.radar.wpm_dmatapprox.plugin"
    """Name of the Radar sensor plugin."""

    def do(self) -> Usd.Prim | None:
        """Execute the Radar sensor creation command.

        Checks if Motion BVH settings are enabled before creating the radar sensor.
        If Motion BVH is not enabled, logs a warning and returns None without
        creating the prim.

        Returns:
            The created Radar sensor prim, or None if Motion BVH is not enabled.
        """
        settings = carb.settings.get_settings()
        motion_bvh_enabled = settings.get("/renderer/raytracingMotion/enabled")

        if not motion_bvh_enabled:
            carb.log_warn(
                "RTX Radar requires Motion BVH to be enabled. "
                "Please enable Motion BVH by setting '/renderer/raytracingMotion/enabled' to true. "
                "Radar sensor was not created."
            )
            return None

        prim = super().do()
        if prim.IsValid():
            # WAR: Expose auxOutputType as "channels" for Replicator API to assign to RenderVar as attribute
            aux_output_type = None
            if "omni:sensor:WpmDmat:auxOutputType" in self._prim_creation_kwargs:
                attr_value = self._prim_creation_kwargs["omni:sensor:WpmDmat:auxOutputType"]
                aux_output_type = [str(attr_value)]
            if prim.HasAttribute("_replicator:rendervar:GenericModelOutput:omni:sensor:WpmDmat:auxOutputType"):
                attr_value = prim.GetAttribute(
                    "_replicator:rendervar:GenericModelOutput:omni:sensor:WpmDmat:auxOutputType"
                ).Get()
                prim.RemoveProperty("_replicator:rendervar:GenericModelOutput:omni:sensor:WpmDmat:auxOutputType")
                aux_output_type = [str(attr_value)]
            if aux_output_type is not None:
                prim.CreateAttribute(
                    "_replicator:rendervar:GenericModelOutput:channels", Sdf.ValueTypeNames.StringArray, True
                ).Set(aux_output_type)
        return prim


class IsaacSensorCreateRtxIDS(IsaacSensorCreateRtxSensor):
    """Command class for creating RTX Idealized Depth Sensors (IDSs).

    This class specializes the base RTX sensor creation for IDSs, providing
    specific configuration and plugin settings for IDS functionality.

    Sets default configuration to "idsoccupancy" if no config is provided.

    Args:
        **kwargs: Keyword arguments passed to the parent class constructor.
            See IsaacSensorCreateRtxSensor for available parameters.
    """

    _sensor_type: str = "ids"
    """String identifier for the type of sensor."""
    _sensor_plugin_name: str = "omni.sensors.nv.ids.ids.plugin"
    """Name of the sensor plugin."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        if self._config is None:
            self._config = "idsoccupancy"
            self._camera_config = "idsoccupancy"


class IsaacSensorCreateRtxUltrasonic(IsaacSensorCreateRtxSensor):
    """Command class for creating RTX Ultrasonic sensors.

    This class specializes the base RTX sensor creation for Ultrasonic sensors, providing
    specific configuration and plugin settings for Ultrasonic functionality.

    Args:
        **kwargs: Keyword arguments passed to the parent class constructor.
            See IsaacSensorCreateRtxSensor for available parameters.
    """

    _sensor_type: str = "ultrasonic"
    """String identifier for the type of sensor."""
    _sensor_plugin_name: str = "omni.sensors.nv.acoustic.wpm_ultrasonic.plugin"
    """Name of the Ultrasonic sensor plugin."""


omni.kit.commands.register_all_commands_in_module(__name__)
