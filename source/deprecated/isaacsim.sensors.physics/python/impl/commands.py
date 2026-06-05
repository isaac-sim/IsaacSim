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

"""Provide commands for creating Isaac physics sensors in USD stages."""

__all__ = ["IsaacSensorCreatePrim", "IsaacSensorCreateContactSensor", "IsaacSensorCreateImuSensor"]

import carb
import omni.isaac.IsaacSensorSchema as IsaacSensorSchema
import omni.kit.commands
import omni.kit.utils
import omni.usd
from isaacsim.core.utils.prims import delete_prim
from isaacsim.core.utils.stage import get_next_free_path
from isaacsim.core.utils.xforms import reset_and_set_xform_ops
from pxr import Gf, PhysxSchema


class IsaacSensorCreatePrim(omni.kit.commands.Command):
    """Command for creating Isaac sensor prims in the USD stage.

    This command creates a new sensor prim using the specified Isaac sensor schema type and applies the provided
    transformation properties. The sensor is created at the specified path with the given translation and orientation.
    It serves as a base command for creating various types of Isaac sensors.

    Args:
        path: USD path where the sensor prim will be created.
        parent: Parent prim path under which the sensor will be created.
        translation: Translation vector for positioning the sensor in 3D space.
        orientation: Quaternion rotation for orienting the sensor.
        schema_type: Isaac sensor schema type to apply to the created prim.
    """

    def __init__(
        self,
        path: str = "",
        parent: str = "",
        translation: Gf.Vec3d = Gf.Vec3d(0, 0, 0),
        orientation: Gf.Quatd = Gf.Quatd(1, 0, 0, 0),
        schema_type: type = IsaacSensorSchema.IsaacBaseSensor,
    ) -> None:
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._prim_path = None

    def do(self) -> object:
        """Create an Isaac sensor prim on the USD stage.

        Generates the next available path, defines the sensor with the specified schema type, enables the sensor,
        and sets the transformation operations.

        Returns:
            The created Isaac sensor prim.
        """
        self._stage = omni.usd.get_context().get_stage()

        self._prim_path = get_next_free_path(self._path, self._parent)
        self._prim = self._schema_type.Define(self._stage, self._prim_path)
        IsaacSensorSchema.IsaacBaseSensor(self._prim).CreateEnabledAttr(True)
        reset_and_set_xform_ops(self._prim.GetPrim(), self._translation, self._orientation)

        return self._prim

    def undo(self) -> object:
        """Undoes the sensor prim creation by deleting the created prim.

        Returns:
            The result of the prim deletion operation if a prim path exists.
        """
        if self._prim_path is not None:
            return delete_prim(self._prim_path)


class IsaacSensorCreateContactSensor(omni.kit.commands.Command):
    """Create an Isaac Contact Sensor prim in the USD stage.

    This command creates a contact sensor that detects physical contact between objects in a simulation.
    The sensor monitors contact forces and reports when they fall within specified threshold ranges.
    It automatically applies the PhysX Contact Report API to the parent prim to enable contact detection.

    Args:
        path: USD path where the contact sensor prim will be created.
        parent: USD path of the parent prim. Must be specified for successful sensor creation.
        min_threshold: Minimum contact force threshold for detection.
        max_threshold: Maximum contact force threshold for detection.
        color: RGBA color values for sensor visualization.
        radius: Detection radius for the contact sensor. Negative values use default behavior.
        sensor_period: Data collection frequency in seconds. Negative values use default behavior.
        translation: 3D translation offset from the parent prim's origin.
    """

    def __init__(
        self,
        path: str = "/Contact_Sensor",
        parent: str = None,
        min_threshold: float = 0,
        max_threshold: float = 100000,
        color: Gf.Vec4f = Gf.Vec4f(1, 1, 1, 1),
        radius: float = -1,
        sensor_period: float = -1,
        translation: Gf.Vec3d = Gf.Vec3d(0, 0, 0),
    ) -> None:
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._prim = None

    def do(self) -> object:
        """Create a contact sensor prim with the specified parameters.

        Creates an Isaac contact sensor prim at the specified path under the parent prim, applies contact sensor
        schema attributes, and ensures the parent prim has contact report API applied.

        Returns:
            The created contact sensor prim, or None if creation failed.
        """
        if self._parent is None:
            carb.log_error("Valid parent prim must be selected before creating contact sensor prim.")
            return None

        success, self._prim = omni.kit.commands.execute(
            "IsaacSensorCreatePrim",
            path=self._path,
            parent=self._parent,
            schema_type=IsaacSensorSchema.IsaacContactSensor,
            translation=self._translation,
        )
        if success and self._prim:
            self._prim.CreateThresholdAttr().Set((self._min_threshold, self._max_threshold))
            self._prim.CreateColorAttr().Set(self._color)
            self._prim.CreateSensorPeriodAttr().Set(self._sensor_period)
            self._prim.CreateRadiusAttr().Set(self._radius)

            # Ensure parent has contact report API in it.
            stage = omni.usd.get_context().get_stage()
            parent_prim = stage.GetPrimAtPath(self._parent)
            contact_report = PhysxSchema.PhysxContactReportAPI.Apply(parent_prim)
            contact_report.CreateThresholdAttr(self._min_threshold)

            return self._prim

        else:
            carb.log_error("Could not create contact sensor prim")
            return None

    def undo(self) -> None:
        """Undoes the contact sensor creation operation.

        Currently a no-op placeholder for the undo functionality.
        """
        # undo must be defined even if empty


class IsaacSensorCreateImuSensor(omni.kit.commands.Command):
    """Command for creating an IMU (Inertial Measurement Unit) sensor prim in the stage.

    This command creates an Isaac IMU sensor that can measure linear acceleration, angular velocity, and
    orientation. The sensor supports configurable filter sizes for smoothing the measured values and can be
    positioned and oriented relative to its parent prim.

    Args:
        path: USD path where the IMU sensor prim will be created.
        parent: USD path of the parent prim to attach the sensor to.
        sensor_period: Sensor update period in simulation time. Negative values use the default period.
        translation: Local translation offset from the parent prim.
        orientation: Local rotation offset from the parent prim as a quaternion (w, x, y, z).
        linear_acceleration_filter_size: Number of samples to use for linear acceleration filtering.
        angular_velocity_filter_size: Number of samples to use for angular velocity filtering.
        orientation_filter_size: Number of samples to use for orientation filtering.
    """

    def __init__(
        self,
        path: str = "/Imu_Sensor",
        parent: str = None,
        sensor_period: float = -1,
        translation: Gf.Vec3d = Gf.Vec3d(0, 0, 0),
        orientation: Gf.Quatd = Gf.Quatd(1, 0, 0, 0),
        linear_acceleration_filter_size: int = 1,
        angular_velocity_filter_size: int = 1,
        orientation_filter_size: int = 1,
    ) -> None:
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._prim = None

    def do(self) -> object:
        """Create an IMU sensor prim with the specified configuration.

        Executes the IMU sensor creation command with the parameters provided during initialization,
        including sensor period and filter settings for linear acceleration, angular velocity, and orientation.

        Returns:
            The created IMU sensor prim if successful, None otherwise.
        """
        success, self._prim = omni.kit.commands.execute(
            "IsaacSensorCreatePrim",
            path=self._path,
            parent=self._parent,
            schema_type=IsaacSensorSchema.IsaacImuSensor,
            translation=self._translation,
            orientation=self._orientation,
        )

        if success and self._prim:
            self._prim.CreateSensorPeriodAttr().Set(self._sensor_period)
            self._prim.CreateLinearAccelerationFilterWidthAttr().Set(self._linear_acceleration_filter_size)
            self._prim.CreateAngularVelocityFilterWidthAttr().Set(self._angular_velocity_filter_size)
            self._prim.CreateOrientationFilterWidthAttr().Set(self._orientation_filter_size)

            return self._prim
        else:
            carb.log_error("Could not create Imu sensor prim")
            return None

    def undo(self) -> None:
        """Undoes the IMU sensor creation operation.

        This method is required for command pattern implementation but currently performs no action.
        """
        # undo must be defined even if empty


omni.kit.commands.register_all_commands_in_module(__name__)
