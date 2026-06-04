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

"""Provides command classes for creating and managing PhysX range sensor prims in Isaac Sim."""

from typing import Any, Optional

import carb
import omni.isaac.IsaacSensorSchema as IsaacSensorSchema
import omni.isaac.RangeSensorSchema as RangeSensorSchema
import omni.kit.commands
from isaacsim.core.utils.stage import get_next_free_path
from isaacsim.core.utils.xforms import reset_and_set_xform_ops
from pxr import Gf, UsdGeom


def setup_base_prim(
    prim: object,
    schema_type: type,
    enabled: bool,
    draw_points: bool,
    draw_lines: bool,
    min_range: float,
    max_range: float,
) -> None:
    """Set up base attributes for a range sensor prim.

    Args:
        prim: The USD prim to set up.
        schema_type: The schema type class to use for attribute creation.
        enabled: Whether the sensor is enabled.
        draw_points: Whether to draw points for visualization.
        draw_lines: Whether to draw lines for visualization.
        min_range: Minimum range of the sensor.
        max_range: Maximum range of the sensor.
    """
    # Check if the schema type has the required methods before calling them
    if hasattr(schema_type(prim), "CreateEnabledAttr"):
        schema_type(prim).CreateEnabledAttr(enabled)

    if hasattr(schema_type(prim), "CreateDrawPointsAttr"):
        schema_type(prim).CreateDrawPointsAttr(draw_points)

    if hasattr(schema_type(prim), "CreateDrawLinesAttr"):
        schema_type(prim).CreateDrawLinesAttr(draw_lines)

    if hasattr(schema_type(prim), "CreateMinRangeAttr"):
        schema_type(prim).CreateMinRangeAttr(min_range)

    if hasattr(schema_type(prim), "CreateMaxRangeAttr"):
        schema_type(prim).CreateMaxRangeAttr(max_range)


class RangeSensorCreatePrim(omni.kit.commands.Command):
    """Base command for creating range sensor prims.

    This command is used to create each range sensor prim and handles undo operations
    so that individual prim commands don't have to implement their own undo logic.

    Args:
        path: Path for the new prim.
        parent: Parent prim path.
        schema_type: Schema type to use for the prim.
        translation: Translation vector for the prim.
        orientation: Orientation quaternion for the prim.
        visibility: Whether the prim is visible.
        min_range: Minimum range of the sensor.
        max_range: Maximum range of the sensor.
        draw_points: Whether to draw points for visualization.
        draw_lines: Whether to draw lines for visualization.
    """

    def __init__(
        self,
        path: str = "",
        parent: str = "",
        schema_type: type = RangeSensorSchema.Lidar,  # Default to Lidar instead of non-existent RangeSensor
        translation: Optional[Gf.Vec3d] = Gf.Vec3d(0, 0, 0),
        orientation: Optional[Gf.Quatd] = Gf.Quatd(1, 0, 0, 0),
        visibility: Optional[bool] = False,
        min_range: Optional[float] = 0.4,
        max_range: Optional[float] = 100.0,
        draw_points: Optional[bool] = False,
        draw_lines: Optional[bool] = False,
    ) -> None:
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._prim_path = None

    def do(self) -> Any:
        """Execute the command to create the range sensor prim.

        Returns:
            The created USD prim.
        """
        self._stage = omni.usd.get_context().get_stage()
        # make prim path unique
        self._prim_path = get_next_free_path(self._path, self._parent)
        schema_obj = self._schema_type.Define(self._stage, self._prim_path)
        setup_base_prim(
            schema_obj, self._schema_type, True, self._draw_points, self._draw_lines, self._min_range, self._max_range
        )

        # rotate sensor to align correctly if stage is y up (only if user didn't specify custom orientation)
        if UsdGeom.GetStageUpAxis(self._stage) == UsdGeom.Tokens.y and self._orientation == Gf.Quatd(1, 0, 0, 0):
            # Only apply default Y-up correction if user didn't specify a custom orientation
            self._orientation = Gf.Quatd(0.707, 0.0, 0.707, 0.0)  # 90 degree rotation around Y axis
            carb.log_info("Applied default Y-up orientation correction.")
        elif UsdGeom.GetStageUpAxis(self._stage) == UsdGeom.Tokens.y:
            carb.log_info(f"Using user-specified orientation: {self._orientation}")

        # Get the actual USD prim from the schema object
        self._prim = schema_obj.GetPrim()
        self._schema_obj = schema_obj

        reset_and_set_xform_ops(self._prim, self._translation, self._orientation)

        # Set visibility
        if self._visibility:
            self._prim.GetAttribute("visibility").Set("invisible")
        else:
            self._prim.GetAttribute("visibility").Set("inherited")

        return self._schema_obj

    def undo(self) -> Any:
        """Undo the command by removing the created prim.

        Returns:
            Result of the undo operation.
        """
        if self._prim_path is not None:
            return self._stage.RemovePrim(self._prim_path)


class RangeSensorCreateLidar(omni.kit.commands.Command):
    """Command class to create a lidar sensor.

    Typical usage example:

    .. code-block:: python

        result, prim = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/Lidar",
            parent=None,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            min_range=0.4,
            max_range=100.0,
            draw_points=False,
            draw_lines=False,
            horizontal_fov=360.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=20.0,
            high_lod=False,
            yaw_offset=0.0,
            enable_semantics=False,
        )

    Args:
        path: Path for the new lidar sensor prim.
        parent: Parent prim path.
        translation: Translation vector for the lidar sensor.
        orientation: Orientation quaternion for the lidar sensor.
        min_range: Minimum range of the sensor.
        max_range: Maximum range of the sensor.
        draw_points: Whether to draw points for visualization.
        draw_lines: Whether to draw lines for visualization.
        horizontal_fov: Horizontal field of view in degrees.
        vertical_fov: Vertical field of view in degrees.
        horizontal_resolution: Horizontal resolution in degrees per sample.
        vertical_resolution: Vertical resolution in degrees per sample.
        rotation_rate: Rotation rate of the sensor in Hz.
        high_lod: Whether to enable high level of detail rendering.
        yaw_offset: Yaw offset in degrees.
        enable_semantics: Whether to enable semantic segmentation.
    """

    def __init__(
        self,
        path: str = "/Lidar",
        parent: object = None,
        translation: Gf.Vec3d = Gf.Vec3d(0, 0, 0),
        orientation: Gf.Quatd = Gf.Quatd(1, 0, 0, 0),
        min_range: float = 0.4,
        max_range: float = 100.0,
        draw_points: bool = False,
        draw_lines: bool = False,
        horizontal_fov: float = 360.0,
        vertical_fov: float = 30.0,
        horizontal_resolution: float = 0.4,
        vertical_resolution: float = 4.0,
        rotation_rate: float = 20.0,
        high_lod: bool = False,
        yaw_offset: float = 0.0,
        enable_semantics: bool = False,
    ) -> None:
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._prim = None
        self._prim_path = None

    def do(self) -> Any:
        """Execute the command to create the lidar sensor.

        Returns:
            The created USD prim, or None if creation failed.
        """
        success, schema_obj = omni.kit.commands.execute(
            "RangeSensorCreatePrim",
            path=self._path,
            parent=self._parent,
            schema_type=RangeSensorSchema.Lidar,
            translation=self._translation,
            orientation=self._orientation,
            draw_points=self._draw_points,
            draw_lines=self._draw_lines,
            min_range=self._min_range,
            max_range=self._max_range,
        )

        if success and schema_obj:
            self._prim = schema_obj.GetPrim()
            self._prim_path = str(self._prim.GetPath())
            self._schema_obj = schema_obj

            # Set lidar-specific attributes
            lidar_schema = RangeSensorSchema.Lidar(self._prim)
            lidar_schema.CreateHorizontalFovAttr().Set(self._horizontal_fov)
            lidar_schema.CreateVerticalFovAttr().Set(self._vertical_fov)
            lidar_schema.CreateHorizontalResolutionAttr().Set(self._horizontal_resolution)
            lidar_schema.CreateVerticalResolutionAttr().Set(self._vertical_resolution)
            lidar_schema.CreateRotationRateAttr().Set(self._rotation_rate)
            lidar_schema.CreateHighLodAttr().Set(self._high_lod)
            lidar_schema.CreateYawOffsetAttr().Set(self._yaw_offset)
            lidar_schema.CreateEnableSemanticsAttr().Set(self._enable_semantics)

            return self._schema_obj
        else:
            carb.log_error("Failed to create lidar sensor prim")
            return None

    def undo(self) -> Any:
        """Undo the command by removing the created prim.

        Returns:
            Result of the undo operation.
        """
        if self._prim_path is not None:
            stage = omni.usd.get_context().get_stage()
            return stage.RemovePrim(self._prim_path)


class RangeSensorCreateGeneric(omni.kit.commands.Command):
    """Command class to create a generic range sensor.

    Typical usage example:

    .. code-block:: python

        result, prim = omni.kit.commands.execute(
            "RangeSensorCreateGeneric",
            path="/GenericSensor",
            parent=None,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            min_range=0.4,
            max_range=100.0,
            draw_points=False,
            draw_lines=False,
            sampling_rate=60,
        )

    Args:
        path: Path for the new prim.
        parent: Parent prim path.
        translation: Translation vector for the prim.
        orientation: Orientation quaternion for the prim.
        min_range: Minimum range of the sensor.
        max_range: Maximum range of the sensor.
        draw_points: Whether to draw points for visualization.
        draw_lines: Whether to draw lines for visualization.
        sampling_rate: Sampling rate of the sensor in Hz.
    """

    def __init__(
        self,
        path: str = "/GenericSensor",
        parent: object = None,
        translation: Gf.Vec3d = Gf.Vec3d(0, 0, 0),
        orientation: Gf.Quatd = Gf.Quatd(1, 0, 0, 0),
        min_range: float = 0.4,
        max_range: float = 100.0,
        draw_points: bool = False,
        draw_lines: bool = False,
        sampling_rate: int = 60,
    ) -> None:
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._prim = None
        self._prim_path = None

    def do(self) -> Any:
        """Execute the command to create the generic range sensor.

        Returns:
            The created USD prim, or None if creation failed.
        """
        success, schema_obj = omni.kit.commands.execute(
            "RangeSensorCreatePrim",
            path=self._path,
            parent=self._parent,
            schema_type=RangeSensorSchema.Generic,
            translation=self._translation,
            orientation=self._orientation,
            draw_points=self._draw_points,
            draw_lines=self._draw_lines,
            min_range=self._min_range,
            max_range=self._max_range,
        )

        if success and schema_obj:
            self._prim = schema_obj.GetPrim()
            self._prim_path = str(self._prim.GetPath())
            self._schema_obj = schema_obj

            # Set generic-specific attributes
            generic_schema = RangeSensorSchema.Generic(self._prim)
            generic_schema.CreateSamplingRateAttr().Set(self._sampling_rate)

            return self._schema_obj
        else:
            carb.log_error("Failed to create generic sensor prim")
            return None

    def undo(self) -> Any:
        """Undo the command by removing the created prim.

        Returns:
            Result of the undo operation.
        """
        if self._prim_path is not None:
            stage = omni.usd.get_context().get_stage()
            return stage.RemovePrim(self._prim_path)


class IsaacSensorCreateLightBeamSensor(omni.kit.commands.Command):
    """Command class to create a light beam sensor.

    Args:
        path: Path for the new prim.
        parent: Parent prim path.
        translation: Translation vector for the prim.
        orientation: Orientation quaternion for the prim.
        num_rays: Number of rays for the light beam sensor.
        curtain_length: Length of the curtain for multi-ray sensors.
        forward_axis: Forward direction axis.
        curtain_axis: Curtain direction axis.
        min_range: Minimum range of the sensor.
        max_range: Maximum range of the sensor.
        draw_points: Whether to draw points for visualization.
        draw_lines: Whether to draw lines for visualization.
        **kwargs: Additional keyword arguments.
    """

    def __init__(
        self,
        path: str = "/LightBeam_Sensor",
        parent: str = None,
        translation: Gf.Vec3d = Gf.Vec3d(0, 0, 0),
        orientation: Gf.Quatd = Gf.Quatd(1, 0, 0, 0),
        num_rays: int = 1,
        curtain_length: float = 0.0,
        forward_axis: Gf.Vec3d = Gf.Vec3d(1, 0, 0),  # default to x axis
        curtain_axis: Gf.Vec3d = Gf.Vec3d(0, 0, 1),  # default to z axis
        min_range: float = 0.4,
        max_range: float = 100.0,
        draw_points: bool = False,
        draw_lines: bool = False,
        **kwargs: Any,
    ) -> None:
        # condensed way to copy all input arguments into self with an underscore prefix
        for name, value in vars().items():
            if name != "self":
                setattr(self, f"_{name}", value)
        self._prim = None
        self._prim_path = None

    def do(self) -> Any:
        """Execute the command to create the light beam sensor.

        Returns:
            The created USD prim, or None if creation failed.
        """
        if self._num_rays > 1 and self._curtain_length == 0:
            carb.log_error("Must specify curtain length if num rays > 1")
            return None

        success, schema_obj = omni.kit.commands.execute(
            "RangeSensorCreatePrim",
            path=self._path,
            parent=self._parent,
            schema_type=IsaacSensorSchema.IsaacLightBeamSensor,
            translation=self._translation,
            orientation=self._orientation,
            draw_points=self._draw_points,
            draw_lines=self._draw_lines,
            min_range=self._min_range,
            max_range=self._max_range,
        )

        if success and schema_obj:
            self._prim = schema_obj.GetPrim()
            self._prim_path = str(self._prim.GetPath())
            self._schema_obj = schema_obj

            # Set light beam sensor-specific attributes
            light_beam_schema = IsaacSensorSchema.IsaacLightBeamSensor(self._prim)
            light_beam_schema.CreateNumRaysAttr().Set(self._num_rays)
            light_beam_schema.CreateCurtainLengthAttr().Set(self._curtain_length)
            light_beam_schema.CreateForwardAxisAttr().Set(self._forward_axis)
            light_beam_schema.CreateCurtainAxisAttr().Set(self._curtain_axis)

            return self._schema_obj
        else:
            carb.log_error("Failed to create light beam sensor prim")
            return None

    def undo(self) -> Any:
        """Undo the command by removing the created prim.

        Returns:
            Result of the undo operation.
        """
        if self._prim_path is not None:
            stage = omni.usd.get_context().get_stage()
            return stage.RemovePrim(self._prim_path)


omni.kit.commands.register_all_commands_in_module(__name__)
