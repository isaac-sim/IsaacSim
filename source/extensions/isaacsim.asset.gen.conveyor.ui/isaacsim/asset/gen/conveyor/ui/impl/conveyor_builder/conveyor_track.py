"""Conveyor track types and data model for conveyor builder assets."""

# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from __future__ import annotations

import asyncio
from enum import Enum

from omni.kit.widget.filebrowser import find_thumbnails_for_files_async


class CheckEnum(Enum):
    """Enhanced enumeration base class with value validation.

    Extends the standard Python Enum class with a class method to check if a value exists
    in the enumeration. This provides a convenient way to validate values before creating
    enum members or performing enum operations.
    """

    @classmethod
    def has_value(cls, value: object) -> bool:
        """Checks if a value exists in the enum.

        Args:
            value: The value to check for existence in the enum.

        Returns:
            True if the value exists in the enum, False otherwise.
        """
        return value in cls._value2member_map_


class Style(CheckEnum):
    """Enumeration for conveyor belt styles."""

    BELT = 0
    """Belt conveyor style."""
    ROLLER = 1
    """Roller conveyor style."""
    DUAL = 2
    """Dual conveyor style."""


class Angle(CheckEnum):
    """Enumeration defining angle configurations for conveyor tracks.

    This enum specifies the rotational orientation options available for conveyor track segments,
    ranging from no rotation to full rotational configurations. The angle setting affects how
    conveyor tracks are oriented and connected within the conveyor system layout.
    """

    NONE = 0
    """No angle applied."""
    HALF = 1
    """Half angle applied."""
    FULL = 2
    """Full angle applied."""


class Curvature(CheckEnum):
    """Enumeration defining curvature levels for conveyor track segments.

    Provides standardized curvature options for conveyor tracks, ranging from straight sections to various
    degrees of curved paths. Each curvature level corresponds to different turning radii and bend angles
    for conveyor track construction.

    Inherits from CheckEnum which provides validation methods for checking if values exist in the enumeration.
    """

    NONE = 0
    """No curvature applied to the conveyor track."""
    SMALL = 1
    """Small curvature applied to the conveyor track."""
    MEDIUM = 2
    """Medium curvature applied to the conveyor track."""
    LARGE = 3
    """Large curvature applied to the conveyor track."""


class Ramp(CheckEnum):
    """Enumeration for conveyor track ramp configurations.

    Defines the available ramp height levels for conveyor tracks, ranging from flat surfaces to multi-level
    elevation changes. Each value represents the number of height increments the conveyor track rises or falls.

    The ramp configuration affects the vertical positioning and connectivity of conveyor track segments in a
    conveyor system layout.
    """

    FLAT = 0
    """Flat conveyor track with no elevation change (value: 0)."""
    ONE = 1
    """Conveyor track with one-unit elevation change (value: 1)."""
    TWO = 2
    """Conveyor track with two-unit elevation change (value: 2)."""
    THREE = 3
    """Conveyor track with three-unit elevation change (value: 3)."""
    FOUR = 4
    """Conveyor track with four-unit elevation change (value: 4)."""


class Type(CheckEnum):
    """Enumeration defining conveyor track types for different connection and layout configurations.

    Defines the structural and functional type of a conveyor track segment, determining how it connects
    to other track segments and its role in the conveyor system layout. Each type represents a specific
    track configuration with distinct connection properties and behaviors.
    """

    START = 0
    """Starting point of a conveyor track segment."""
    STRAIGHT = 1
    """Straight conveyor track segment."""
    Y_MERGE = 2
    """Y-shaped merge point where two tracks converge."""
    T_MERGE = 3
    """T-shaped merge point where tracks join at right angles."""
    FORK_MERGE = 4
    """Fork merge point where tracks split or converge."""
    END = 5
    """Ending point of a conveyor track segment."""


class ConveyorTrack:
    """Base Conveyor track class, every track type should derive from this.

    Args:
        base_usd: Base USD file path for the conveyor track.
        style: Visual style of the conveyor track.
        angle: Angle configuration for the track.
        curvature: Curvature setting for curved tracks.
        ramp: Ramp configuration for inclined tracks.
        type: Type of conveyor track (start, straight, merge, etc.).
        start_level: Starting level for the conveyor track.
        direction: Direction of the track. For curves, -1 is left, 1 is right. For ramps, 1 is up, -1 is down.
        anchors: List of anchor points for track connections.
        thumb_loaded_callback: Callback function called when thumbnail loading is complete.
        **kwargs: Additional keyword arguments.

    Keyword Args:
        conveyor_nodes: Physics authoring configuration nodes.
    """

    def __init__(
        self,
        base_usd: str,
        style: Style = Style.BELT,
        angle: Angle = Angle.NONE,
        curvature: Curvature = Curvature.SMALL,
        ramp: Ramp = Ramp.FLAT,
        type: Type = Type.STRAIGHT,
        start_level: int = 0,
        direction: int = 1,
        anchors: list[str] | None = None,
        thumb_loaded_callback: object = None,
        **kwargs: object,
    ) -> None:
        if anchors is None:
            anchors = [""]
        self._style = Style[style]
        self._angle = Angle[angle]
        self._curvature = Curvature[curvature]
        self._ramp = Ramp[ramp]
        self._type = Type[type]
        self._start_level = start_level  # At which level it starts the convey (for dual belts it's always zero)
        self._direction = (
            direction  # For curves, -1 is left, 1 is right, for ramps 1 is up, -1 is down. No effect on other assets.
        )

        # self._reference_asset = args.get("asset", None)
        self.base_usd = base_usd
        self._prim = None
        self.thumb = None
        self.thumb_task = asyncio.ensure_future(self.get_thumb_async())
        self.thumb_task.add_done_callback(self.thumb_callback)
        self._anchors = anchors
        self._parent_tracks = []  # For dual tracks there should be two parents, one for each track
        self._child_tracks = []  # For dual tracks there can be two children, one for each track.
        self._thumb_callback = thumb_loaded_callback

        # physics authoring configs
        self.conveyor_nodes = kwargs.get("conveyor_nodes", {})

    async def get_thumb_async(self) -> None:
        """Asynchronously loads the thumbnail for the conveyor track's base USD file."""
        thumb = await find_thumbnails_for_files_async([self.base_usd])
        if self.base_usd in thumb:
            self.thumb = thumb[self.base_usd]
        return

    def thumb_callback(self, task: asyncio.Task) -> None:
        """Callback executed when the thumbnail loading task completes.

        Args:
            task: The completed asyncio task.
        """
        if self._thumb_callback:
            self._thumb_callback(self)

    def get_thumb(self) -> str:
        """Gets the loaded thumbnail path for the conveyor track.

        Returns:
            The thumbnail path if loaded, empty string otherwise.
        """
        if self.thumb:
            return self.thumb
        return ""

    def get_anchors(self, direction: int = 1) -> list[str]:
        """Gets the anchor points for the conveyor track.

        Args:
            direction: Direction to retrieve anchors. 1 for forward order, other values for reversed order.

        Returns:
            List of anchor point names in the specified order.
        """
        if direction == 1:
            new_anchors = list(self._anchors)
        else:
            new_anchors = list(self._anchors.__reversed__())
        return new_anchors

    @property
    def style(self) -> Style:
        """Style of the conveyor track (belt, roller, or dual).

        Returns:
            The current style setting.
        """
        return self._style

    @style.setter
    def style(self, value: int) -> None:
        """Set the conveyor visual style."""
        if Style.has_value(value):
            self._style = Style(value)

    @property
    def angle(self) -> Angle:
        """Angle configuration of the conveyor track.

        Returns:
            The current angle setting.
        """
        return self._angle

    @angle.setter
    def angle(self, value: int) -> None:
        """Set the angle configuration."""
        if Angle.has_value(value):
            self._angle = Angle(value)

    @property
    def curvature(self) -> Curvature:
        """Curvature setting of the conveyor track.

        Returns:
            The current curvature setting.
        """
        return self._curvature

    @curvature.setter
    def curvature(self, value: int) -> None:
        """Set the curvature configuration."""
        if Curvature.has_value(value):
            self._angle = Curvature(value)

    @property
    def ramp(self) -> Ramp:
        """Ramp configuration of the conveyor track.

        Returns:
            The current ramp setting.
        """
        return self._ramp

    @ramp.setter
    def ramp(self, value: int) -> None:
        """Set the ramp configuration."""
        if Ramp.has_value(value):
            self._ramp = Ramp(value)

    @property
    def type(self) -> Type:
        """Type of the conveyor track (start, straight, merge, fork, or end).

        Returns:
            The current track type.
        """
        return self._type

    @type.setter
    def type(self, value: int) -> None:
        """Set the track type."""
        if Type.has_value(value):
            self._type = Type(value)

    def start_level(self, trackIndex: int = 0, direction: int = 1) -> int:
        """Gets the starting level of the conveyor track based on direction.

        Args:
            trackIndex: Index of the track for dual-style conveyors.
            direction: Direction to determine level. 1 returns start level, other values return end level.

        Returns:
            The level at the specified position and direction.
        """
        if direction == 1:
            return self.get_start()
        else:
            return self.get_end()

    def get_start(self, trackIndex: int = 0) -> int:
        """Get the starting level for the conveyor track.

        Args:
            trackIndex: Index of the track. For dual style tracks, returns the track index.
                For other styles, this parameter is ignored.

        Returns:
            The starting level of the track.
        """
        if self.style == Style.DUAL:
            return trackIndex
        else:
            return self._start_level

    def get_end(self, trackIndex: int = 0) -> int:
        """Get the ending level for the conveyor track.

        Args:
            trackIndex: Index of the track. For dual style tracks, returns the track index.
                For other styles, this parameter is ignored.

        Returns:
            The ending level of the track.
        """
        if self.style == Style.DUAL:
            return trackIndex
        else:
            return self._start_level + int(self.ramp.value)

    def end_level(self, trackIndex: int = 0, direction: int = 1) -> int:
        """Get the ending level of the track based on direction.

        Args:
            trackIndex: Index of the track.
            direction: Direction of travel. If 1 (forward), returns the end level.
                If not 1 (reverse), returns the start level.

        Returns:
            The level at the end of the track in the specified direction.
        """
        if direction == 1:
            return self.get_end(trackIndex)
        else:
            return self.get_start(trackIndex)

    def get_config(self) -> dict:
        """Get the configuration dictionary for the conveyor track.

        Returns:
            Dictionary containing the track configuration including style, angle, curvature,
            ramp, start_level, and type.
        """
        config = {}
        config["style"] = self.style
        config["angle"] = self.angle
        config["curvature"] = self.curvature
        config["ramp"] = self.ramp
        config["start_level"] = self._start_level
        config["type"] = self.type
