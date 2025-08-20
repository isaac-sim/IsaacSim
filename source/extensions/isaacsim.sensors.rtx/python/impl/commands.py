# SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from pathlib import Path
from typing import Optional

import carb
import omni.isaac.IsaacSensorSchema as IsaacSensorSchema
import omni.kit.commands
import omni.kit.utils
import omni.replicator.core as rep
import omni.usd
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

    Attributes:
        _replicator_api: Static method reference to the Replicator API for sensor creation.
        _sensor_type: String identifier for the type of sensor.
        _supported_configs: List of supported sensor configurations.
        _schema: Schema for the sensor type.
        _sensor_plugin_name: Name of the sensor plugin.
        _camera_config_name: Name of the camera configuration.
    """

    _replicator_api = None
    _sensor_type = "sensor"
    _supported_configs = []
    _schema = None
    _sensor_plugin_name = ""
    _camera_config_name = ""

    def __init__(
        self,
        path: Optional[str] = None,
        parent: Optional[str] = None,
        config: Optional[str] = None,
        translation: Optional[Gf.Vec3d] = Gf.Vec3d(0, 0, 0),
        orientation: Optional[Gf.Quatd] = Gf.Quatd(1, 0, 0, 0),
        visibility: Optional[bool] = False,
        variant: Optional[str] = None,
        force_camera_prim: bool = False,
        **kwargs,
    ):
        """Initialize the RTX sensor creation command.

        Args:
            path: Optional path where the sensor will be created. If None, a default path will be used.
            parent: Optional parent prim path for the sensor.
            config: Optional configuration name for the sensor.
            translation: Optional 3D translation vector for sensor placement.
            orientation: Optional quaternion for sensor orientation.
            visibility: Optional visibility flag for the sensor.
            variant: Optional variant name for the sensor configuration.
            force_camera_prim: If True, forces creation of a camera prim instead of using references or Replicator API.
            **kwargs: Additional keyword arguments for prim creation.
        """
        self._parent = parent
        self._config = config
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

    def _add_reference(self) -> Usd.Prim:
        """Adds a reference to the stage if a config is provided.

        If a config is provided, this method adds a reference to the stage and sets the prim's
        variant if provided. It also handles finding the correct sensor prim within referenced assets.

        Returns:
            Usd.Prim: The created or found prim, or None if no config was provided or found.
        """
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
                    prim = add_reference_to_stage(
                        usd_path=get_assets_root_path() + config,
                        prim_path=self._prim_path,
                        prim_type=self._desired_prim_type if config.endswith(".usda") else "Xform",
                    )
                    reset_and_set_xform_ops(prim.GetPrim(), self._translation, self._orientation)
                    if self._variant:
                        allowed_variants = self._supported_configs[config]
                        if self._variant not in allowed_variants:
                            carb.log_warn(
                                f"Variant '{self._variant}' not found for Omni{self._sensor_type.capitalize()} at {self._prim_path}. Allowed variants: {allowed_variants}."
                            )
                        else:
                            variant_set = prim.GetVariantSet(SUPPORTED_LIDAR_VARIANT_SET_NAME)
                            if len(variant_set.GetVariantNames()) == 0:
                                carb.log_warn(
                                    f"Variant set {SUPPORTED_LIDAR_VARIANT_SET_NAME} for Omni{self._sensor_type.capitalize()} at {self._prim_path} does not contain any variants."
                                )
                            elif not variant_set.SetVariantSelection(self._variant):
                                carb.log_warn(
                                    f"Variant '{self._variant}' not found for Omni{self._sensor_type.capitalize()} at {self._prim_path}. Mismatch between allowed variant list and available variants."
                                )
                    # If necessary, traverse children of referenced asset to find OmniSensor prim
                    # Note: if multiple children of the referenced asset are OmniSensor types, this will select the first one
                    if prim.GetTypeName() == "Xform":
                        for child in Usd.PrimRange(prim):
                            if child.GetTypeName() == self._desired_prim_type:
                                carb.log_info(f"Using {self._desired_prim_type} prim at path {child.GetPath()}")
                                prim = child
                    for attr, value in self._prim_creation_kwargs.items():
                        if prim.HasAttribute(attr):
                            prim.GetAttribute(attr).Set(value)
                    return prim
            if not found_config:
                carb.log_warn(
                    f"Config '{self._config}' not found for Omni{self._sensor_type.capitalize()} at {self._prim_path}."
                )
        return None

    def _call_replicator_api(self) -> Usd.Prim:
        """Creates a sensor using the Replicator API.

        Converts position and orientation into the format required by the Replicator API
        and creates the sensor prim.

        Returns:
            Usd.Prim: The created prim, or None if no Replicator API is available.
        """
        if self._replicator_api is not None:
            # Convert position and orientation into tuples for Replicator API.
            position = (self._translation[0], self._translation[1], self._translation[2])
            rotation = Gf.Rotation(self._orientation)
            euler_angles_as_vec = rotation.Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())
            euler_angles = (euler_angles_as_vec[0], euler_angles_as_vec[1], euler_angles_as_vec[2])
            # Construct prim
            if self._prim_path.startswith("/"):
                self._prim_path = self._prim_path[1:]
            return self._replicator_api(
                position=position,
                rotation=euler_angles,
                name=self._prim_path,
                parent=self._parent,
                **self._prim_creation_kwargs,
            )
        return None

    def _create_camera_prim(self) -> Usd.Prim:
        """Creates a camera prim for the sensor.

        This method is deprecated as of Isaac Sim 5.0. It creates a basic camera prim
        with sensor-specific attributes.

        Returns:
            Usd.Prim: The created camera prim.

        Raises:
            Warning: If called, as this functionality is deprecated.
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
        """Executes the sensor creation command.

        Creates the sensor using the most appropriate method based on the configuration
        and available APIs.

        Returns:
            Usd.Prim: The created sensor prim.
        """
        self._stage = omni.usd.get_context().get_stage()
        self._prim_path = get_next_free_path(self._path, self._parent)
        self._prim = (
            not self._force_camera_prim and (self._add_reference() or self._call_replicator_api())
        ) or self._create_camera_prim()
        return self._prim

    def undo(self) -> None:
        """Undoes the sensor creation command.

        Currently not implemented.
        """
        pass


class IsaacSensorCreateRtxLidar(IsaacSensorCreateRtxSensor):
    """Command class for creating RTX Lidar sensors.

    This class specializes the base RTX sensor creation for Lidar sensors, providing
    specific configuration and plugin settings for Lidar functionality.

    Attributes:
        _replicator_api: Static method reference to the Lidar Replicator API.
        _sensor_type: Set to "lidar".
        _supported_configs: List of supported Lidar configurations.
        _schema: Schema for Lidar sensors.
        _sensor_plugin_name: Name of the Lidar sensor plugin.
    """

    _replicator_api = staticmethod(rep.functional.create.omni_lidar)
    _sensor_type = "lidar"
    _supported_configs = SUPPORTED_LIDAR_CONFIGS
    _schema = IsaacSensorSchema.IsaacRtxLidarSensorAPI
    _sensor_plugin_name = "omni.sensors.nv.lidar.lidar_core.plugin"

    def __init__(self, **kwargs):
        """Initialize the RTX Lidar sensor creation command.
        Args:
            **kwargs: Keyword arguments passed to the parent class constructor.
        """
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
        """Executes the sensor creation command."""
        prim = super().do()
        if prim.IsValid() and prim.HasAttribute("omni:sensor:Core:skipDroppingInvalidPoints"):
            prim.GetAttribute("omni:sensor:Core:skipDroppingInvalidPoints").Set(True)
        return prim


class IsaacSensorCreateRtxRadar(IsaacSensorCreateRtxSensor):
    """Command class for creating RTX Radar sensors.

    This class specializes the base RTX sensor creation for Radar sensors, providing
    specific configuration and plugin settings for Radar functionality.

    Attributes:
        _replicator_api: Static method reference to the Radar Replicator API.
        _sensor_type: Set to "radar".
        _schema: Schema for Radar sensors.
        _sensor_plugin_name: Name of the Radar sensor plugin.
    """

    _replicator_api = staticmethod(rep.functional.create.omni_radar)
    _sensor_type = "radar"
    _schema = IsaacSensorSchema.IsaacRtxRadarSensorAPI
    _sensor_plugin_name = "omni.sensors.nv.radar.wpm_dmatapprox.plugin"


class IsaacSensorCreateRtxIDS(IsaacSensorCreateRtxSensor):
    """Command class for creating RTX Idealized Depth Sensors (IDSs).

    This class specializes the base RTX sensor creation for IDSs, providing
    specific configuration and plugin settings for IDS functionality.

    Attributes:
        _sensor_type: Set to "ids".
        _sensor_plugin_name: Name of the IDS sensor plugin.
    """

    _sensor_type = "ids"
    _sensor_plugin_name = "omni.sensors.nv.ids.ids.plugin"

    def __init__(self, **kwargs):
        """Initialize the RTX IDS sensor creation command.

        Sets default configuration to "idsoccupancy" if no config is provided.

        Args:
            **kwargs: Keyword arguments passed to the parent class constructor.
        """
        super().__init__(**kwargs)
        if self._config is None:
            self._config = "idsoccupancy"
            self._camera_config = "idsoccupancy"


class IsaacSensorCreateRtxUltrasonic(IsaacSensorCreateRtxSensor):
    """Command class for creating RTX Ultrasonic sensors.

    This class specializes the base RTX sensor creation for Ultrasonic sensors, providing
    specific configuration and plugin settings for Ultrasonic functionality.

    Attributes:
        _sensor_type: Set to "ultrasonic".
        _sensor_plugin_name: Name of the Ultrasonic sensor plugin.
    """

    _sensor_type = "ultrasonic"
    _sensor_plugin_name = "omni.sensors.nv.ultrasonic.wpm_ultrasonic.plugin"


omni.kit.commands.register_all_commands_in_module(__name__)
