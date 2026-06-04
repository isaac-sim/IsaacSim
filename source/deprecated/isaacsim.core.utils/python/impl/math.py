# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Deprecated math utility functions."""

from __future__ import annotations

import copy

import numpy as np


def radians_to_degrees(rad_angles: np.ndarray) -> np.ndarray:
    """Converts input angles from radians to degrees.

    Args:
        rad_angles: Input array of angles (in radians).

    Returns:
        Array of angles in degrees.
    """
    return rad_angles * (180.0 / np.pi)


def cross(a: np.ndarray | list, b: np.ndarray | list) -> np.ndarray:
    """Computes the cross-product between two 3-dimensional vectors.

    Args:
        a: A 3-dimensional vector.
        b: A 3-dimensional vector.

    Returns:
        Cross product between input vectors.
    """
    return np.array([a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])


def normalize(v: np.ndarray) -> np.ndarray:
    """Normalizes the vector inline (and also returns it).

    Args:
        v: The vector to normalize.

    Returns:
        The normalized vector.
    """
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    v /= norm
    return v


def normalized(v: np.ndarray | None) -> np.ndarray | None:
    """Returns a normalized copy of the provided vector.

    Args:
        v: The vector to normalize.

    Returns:
        A normalized copy of the vector, or None if input is None.
    """
    if v is None:
        return None
    return normalize(copy.deepcopy(v))
