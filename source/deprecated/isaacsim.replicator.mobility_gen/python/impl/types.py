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

"""Defines geometric data types for 2D and 3D spatial representations used in mobility generation."""

from dataclasses import dataclass

import numpy as np


@dataclass
class Point2d:
    """Point2d(x: float, y: float).

    Args:
        x: The x-coordinate of the point.
        y: The y-coordinate of the point.
    """

    x: float
    y: float


@dataclass
class Pose2d(Point2d):
    """A 2D pose representation with position and orientation.

    Inherits from Point2d and adds rotation angle.

    Args:
        x: X coordinate position.
        y: Y coordinate position.
        theta: Rotation angle in radians.
    """

    theta: float


@dataclass
class Pose3d:
    """Pose3d(position: numpy.ndarray, orientation: numpy.ndarray).

    Args:
        position: The 3D position as a numpy array.
        orientation: The 3D orientation as a numpy array.
    """

    position: np.ndarray
    orientation: np.ndarray
