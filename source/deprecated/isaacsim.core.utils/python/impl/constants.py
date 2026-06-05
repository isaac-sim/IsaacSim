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

"""Deprecated constant definitions."""

from pxr import UsdGeom

AXES_INDICES = {"X": 0, "x": 0, "Y": 1, "y": 1, "Z": 2, "z": 2}
"""Mapping from axis name to axis ID

Example:

.. code-block:: python

    >>> import isaacsim.core.utils.constants as constants_utils
    >>>
    >>> # get the x-axis index
    >>> constants_utils.AXES_INDICES['x']
    0
    >>> constants_utils.AXES_INDICES['X']
    0
"""

AXES_TOKEN = {
    "X": UsdGeom.Tokens.x,
    "x": UsdGeom.Tokens.x,
    "Y": UsdGeom.Tokens.y,
    "y": UsdGeom.Tokens.y,
    "Z": UsdGeom.Tokens.z,
    "z": UsdGeom.Tokens.z,
}
"""Mapping from axis name to axis USD token

    >>> import isaacsim.core.utils.constants as constants_utils
    >>>
    >>> # get the x-axis USD token
    >>> constants_utils.AXES_TOKEN['x']
    X
    >>> constants_utils.AXES_TOKEN['X']
    X
"""
