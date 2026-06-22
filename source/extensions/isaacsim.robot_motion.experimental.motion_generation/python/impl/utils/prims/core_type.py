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

"""Utilities for determining USD prim shape types in the Isaac Sim robot motion generation system."""

# TODO: Does something in the core experimental API already do this?
from isaacsim.core.experimental.objects import Capsule, Cone, Cube, Cylinder, Mesh, Plane, Shape, Sphere


def get_shape_type(prim_path: str) -> type[Shape]:
    """Return the supported Shape type for a prim path.

    Args:
        prim_path: USD prim path to classify.

    Returns:
        Shape type associated with the prim path.

    Raises:
        RuntimeError: Raised when the prim type is not supported.

    Example:

    .. code-block:: python

        >>> from isaacsim.robot_motion.experimental.motion_generation.utils.prims import get_shape_type
        >>> prim_path = "/World/SomeShape"
        >>> _ = get_shape_type(prim_path)  # doctest: +SKIP
    """
    if Sphere.are_of_type(prim_path).numpy().item():
        return Sphere

    if Cube.are_of_type(prim_path).numpy().item():
        return Cube

    if Cone.are_of_type(prim_path).numpy().item():
        return Cone

    if Plane.are_of_type(prim_path).numpy().item():
        return Plane

    if Capsule.are_of_type(prim_path).numpy().item():
        return Capsule

    if Cylinder.are_of_type(prim_path).numpy().item():
        return Cylinder

    if Mesh.are_of_type(prim_path).numpy().item():
        return Mesh

    raise RuntimeError(
        f"Prim path {prim_path} does not point to a supported shape type. Supported shape types are: Sphere, Cube, Cone, Plane, Capsule, Cylinder, Mesh."
    )
