# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Module providing a viewport manipulator for visualizing ROS 2 TF (Transform) relationships in Isaac Sim."""

import copy

import numpy as np
import omni.ui as ui
import omni.usd
from omni.ui import color as cl
from omni.ui import scene as sc
from pxr import UsdGeom
from scipy.spatial.transform import Rotation


class ViewManipulator(sc.Manipulator):
    """A viewport manipulator for visualizing ROS 2 TF (Transform) relationships in the Isaac Sim viewport.

    This class provides real-time visualization of coordinate frames, their relationships, and transformations
    in 3D space. It displays frame names, coordinate axes, connecting arrows between related frames, and frame
    markers. The visualization is highly configurable, allowing users to customize colors, sizes, and visibility
    of each component.

    The manipulator supports dynamic updates of frame transforms and automatically redraws when the configuration
    changes or new transform data is received.

    Args:
        **kwargs: Additional keyword arguments passed to the parent class.
    """

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)

        self._relations = []
        self._transforms = {}

        # configuration
        self.cfg_root_frame = "World"

        self.cfg_frames_show = True
        self.cfg_frames_color = [1.0, 1.0, 1.0, 1.0]
        self.cfg_frames_size = 5

        self.cfg_names_show = True
        self.cfg_names_color = [1.0, 1.0, 0.0, 1.0]
        self.cfg_names_size = 20

        self.cfg_axes_show = True
        self.cfg_axes_length = 0.1
        self.cfg_axes_thickness = 4

        self.cfg_arrows_show = True
        self.cfg_arrows_color = [0.0, 1.0, 1.0, 1.0]
        self.cfg_arrows_thickness = 4

    def update_transforms(self, transforms: dict, relations: list) -> None:
        """Updates the transforms and relations for the view manipulator.

        Args:
            transforms: Dictionary mapping frame names to their transform data (position and quaternion).
            relations: List of relations between frames.
        """
        self._relations = relations
        self._transforms = transforms
        # redraw all
        self.invalidate()

    def set_root_frame(self, value: str) -> None:
        """Sets the root frame for the view manipulator.

        Args:
            value: Name of the root frame.
        """
        self.cfg_root_frame = value

    def set_frames_show(self, value: bool) -> None:
        """Sets the visibility of frames.

        Args:
            value: Whether to show frames.
        """
        self.cfg_frames_show = value

    def set_frames_color(self, channel: int, value: float) -> None:
        """Sets the color channel value for frames.

        Args:
            channel: Color channel index (0-3 for RGBA).
            value: Color value clamped to range [0, 1].
        """
        if channel >= 0 and channel <= 3:
            self.cfg_frames_color[int(channel)] = max(min(value, 1), 0)

    def set_frames_size(self, value: float) -> None:
        """Sets the size of frames.

        Args:
            value: Size multiplier for frames.
        """
        self.cfg_frames_size = value * 30

    def set_names_show(self, value: bool) -> None:
        """Sets the visibility of frame names.

        Args:
            value: Whether to show frame names.
        """
        self.cfg_names_show = value

    def set_names_color(self, channel: int, value: float) -> None:
        """Sets the color channel value for frame names.

        Args:
            channel: Color channel index (0-3 for RGBA).
            value: Color value clamped to range [0, 1].
        """
        if channel >= 0 and channel <= 3:
            self.cfg_names_color[int(channel)] = max(min(value, 1), 0)

    def set_names_size(self, value: float) -> None:
        """Sets the size of frame names.

        Args:
            value: Size multiplier for frame names.
        """
        self.cfg_names_size = value * 50

    def set_axes_show(self, value: bool) -> None:
        """Sets the visibility of axes.

        Args:
            value: Whether to show axes.
        """
        self.cfg_axes_show = value

    def set_axes_length(self, value: float) -> None:
        """Sets the length of axes.

        Args:
            value: Length of axes in stage units.
        """
        stage_unit = UsdGeom.GetStageMetersPerUnit(omni.usd.get_context().get_stage())
        self.cfg_axes_length = value / stage_unit

    def set_axes_thickness(self, value: float) -> None:
        """Sets the thickness of the coordinate axes.

        Args:
            value: Thickness value, scaled by 20 for rendering.
        """
        self.cfg_axes_thickness = value * 20

    def set_arrows_show(self, value: bool) -> None:
        """Sets the visibility of arrows representing frame relations.

        Args:
            value: Whether to show arrows connecting related frames.
        """
        self.cfg_arrows_show = value

    def set_arrows_color(self, channel: int, value: float) -> None:
        """Sets a color channel for the arrows.

        Args:
            channel: Color channel index (0-3 for RGBA).
            value: Color value, clamped to [0, 1].
        """
        if channel >= 0 and channel <= 3:
            self.cfg_arrows_color[int(channel)] = max(min(value, 1), 0)

    def set_arrows_thickness(self, value: float) -> None:
        """Sets the thickness of the arrows.

        Args:
            value: Thickness value, scaled by 20 for rendering.
        """
        self.cfg_arrows_thickness = value * 20

    def clear(self) -> None:
        """Clears all transforms and relations from the manipulator."""
        self.update_transforms({}, [])

    def on_build(self) -> None:
        """Builds the scene representation of frames, arrows, names, and coordinate axes."""
        if not self._transforms:
            return

        transforms = copy.deepcopy(self._transforms)
        relations = copy.deepcopy(self._relations)

        names = list(transforms.keys())
        positions = [transform[0] for transform in transforms.values()]
        quaternions = [transform[1] for transform in transforms.values()]

        # draw arrows (relations)
        if self.cfg_arrows_show:
            for r in relations:
                if r[0] in transforms and r[1] in transforms:
                    sc.Line(
                        transforms[r[0]][0],
                        transforms[r[1]][0],
                        color=cl(*self.cfg_arrows_color),
                        thickness=self.cfg_arrows_thickness,
                    )

        # draw frames
        if self.cfg_frames_show:
            sc.Points(
                positions,
                colors=[cl(*self.cfg_frames_color)] * len(positions),
                sizes=[self.cfg_frames_size] * len(positions),
            )

        # draw names and axes
        T = np.eye(4)
        for name, position, quaternion in zip(names, positions, quaternions):

            # names
            T[:3, 3] = position
            if self.cfg_names_show:
                with sc.Transform(transform=sc.Matrix44(*T.T.flatten())):
                    sc.Label(
                        name,
                        alignment=ui.Alignment.CENTER_TOP,
                        color=cl(*self.cfg_names_color),
                        size=self.cfg_names_size,
                    )

            # axes
            if self.cfg_axes_show:
                T[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
                with sc.Transform(transform=sc.Matrix44(*T.T.flatten())):
                    k = self.cfg_axes_length
                    sc.Line([0, 0, 0], [k, 0, 0], color=cl("#ff0000"), thickness=self.cfg_axes_thickness)
                    sc.Line([0, 0, 0], [0, k, 0], color=cl("#00ff00"), thickness=self.cfg_axes_thickness)
                    sc.Line([0, 0, 0], [0, 0, k], color=cl("#0000ff"), thickness=self.cfg_axes_thickness)
