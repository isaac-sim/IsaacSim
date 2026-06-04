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

"""Internal fabric utilities for USDRT stage operations and data transformations between Warp and fabric arrays."""

import usdrt
import warp as wp


def update_fabric_selection(*, stage: usdrt.Usd.Stage, data: dict, device: wp.Device, attr: str, count: int) -> bool:
    """Update fabric selection and create index mapping for prims.

    Args:
        stage: The USDRT stage to select prims from.
        data: Dictionary containing selection data and specifications.
        device: The Warp device to use for operations.
        attr: The attribute name to read from selected prims.
        count: Expected number of selected prims.

    Returns:
        True if selection was successfully updated, False otherwise.
    """
    # update selection
    selection = data["selection"]
    if selection is None or device.alias.startswith("cuda"):
        selection = stage.SelectPrims(
            require_attrs=[(usdrt.Sdf.ValueTypeNames.UInt, attr, usdrt.Usd.Access.Read), *data["spec"]],
            device=device.alias,
        )
        if selection.GetCount() == count:
            data["selection"] = selection
        else:
            data["selection"] = None
            return False
    else:
        selection.PrepareForReuse()
    # build mapping
    mapping = wp.empty((count,), dtype=wp.uint32, device=device)
    wp.launch(
        _wk_selection_mapping,
        dim=(count,),
        inputs=[wp.fabricarray(selection, attrib=attr), mapping],
        device=device,
    )
    data["mapping"] = mapping
    return True


@wp.kernel(enable_backward=False)
def _wk_selection_mapping(
    fabricarray: wp.fabricarray(dtype=wp.uint32), mapping: wp.array(ndim=1, dtype=wp.uint32)
) -> None:
    """Map fabric array indices to create a mapping array.

    Args:
        fabricarray: Warp fabric array containing indices.
        mapping: Output Warp array to store the index mapping.
    """
    i = wp.tid()
    mapping[fabricarray[i]] = wp.uint32(i)


@wp.kernel(enable_backward=False)
def wk_write_to_fabric_float(
    array: wp.array(ndim=2, dtype=wp.float32),
    fabric_array: wp.fabricarray(dtype=wp.float32),
    indices: wp.array(ndim=1, dtype=wp.int32),
    mapping: wp.array(ndim=1, dtype=wp.uint32),
    broadcast: bool,
) -> None:
    """Write float values from a Warp array to a fabric array.

    Args:
        array: Source Warp array containing float values.
        fabric_array: Target fabric array to write values to.
        indices: Array of indices to process.
        mapping: Array mapping indices to fabric array positions.
        broadcast: Whether to broadcast the first value to all target positions.
    """
    i = wp.tid()
    if broadcast:
        value = array[0, 0]
    else:
        value = array[i, 0]
    fabric_array[mapping[indices[i]]] = value


@wp.func
def _decompose(m: wp.mat44f) -> tuple[wp.vec3f, wp.quatf, wp.vec3f]:
    """Decompose a 4x4 transformation matrix into position, orientation, and scale.

    Args:
        m: The 4x4 transformation matrix to decompose.

    Returns:
        Tuple of (position, rotation, scale) where position and scale are vec3f and rotation is quatf.
    """
    # extract position
    position = wp.vec3f(m[3, 0], m[3, 1], m[3, 2])
    # extract rotation matrix components
    r00, r01, r02 = m[0, 0], m[0, 1], m[0, 2]
    r10, r11, r12 = m[1, 0], m[1, 1], m[1, 2]
    r20, r21, r22 = m[2, 0], m[2, 1], m[2, 2]
    # get scale magnitudes
    sx = wp.sqrt(r00 * r00 + r01 * r01 + r02 * r02)
    sy = wp.sqrt(r10 * r10 + r11 * r11 + r12 * r12)
    sz = wp.sqrt(r20 * r20 + r21 * r21 + r22 * r22)
    # normalize rotation matrix components
    if sx != 0.0:
        r00 /= sx
        r01 /= sx
        r02 /= sx
    if sy != 0.0:
        r10 /= sy
        r11 /= sy
        r12 /= sy
    if sz != 0.0:
        r20 /= sz
        r21 /= sz
        r22 /= sz
    # extract rotation (quaternion)
    rotation = wp.quat_from_matrix(wp.transpose(wp.mat33f(r00, r01, r02, r10, r11, r12, r20, r21, r22)))
    # extract scale
    scale = wp.vec3f(sx, sy, sz)
    return position, rotation, scale


@wp.kernel(enable_backward=False)
def wk_decompose_fabric_transformation_matrix_to_warp_arrays(
    fabric_matrices: wp.fabricarray(dtype=wp.mat44d),
    array_positions: wp.array(ndim=2, dtype=wp.float32),
    array_orientations: wp.array(ndim=2, dtype=wp.float32),
    array_scales: wp.array(ndim=2, dtype=wp.float32),
    indices: wp.array(ndim=1, dtype=wp.int32),
    mapping: wp.array(ndim=1, dtype=wp.uint32),
) -> None:
    """Decompose fabric transformation matrices into separate position, orientation, and scale arrays.

    Args:
        fabric_matrices: Input fabric array of transformation matrices.
        array_positions: Output array for positions (shape (N, 3)).
        array_orientations: Output array for orientations as quaternions (shape (N, 4)).
        array_scales: Output array for scales (shape (N, 3)).
        indices: Array of indices to process.
        mapping: Array mapping indices to fabric array positions.
    """
    # resolve array index
    index = indices[wp.tid()]
    # decompose transform matrix
    position, rotation, scale = _decompose(wp.mat44f(fabric_matrices[mapping[index]]))
    # extract position
    if array_positions:
        array_positions[index, 0] = position[0]
        array_positions[index, 1] = position[1]
        array_positions[index, 2] = position[2]
    # extract orientation (Warp quaternion is xyzw)
    if array_orientations:
        array_orientations[index, 0] = rotation[3]
        array_orientations[index, 1] = rotation[0]
        array_orientations[index, 2] = rotation[1]
        array_orientations[index, 3] = rotation[2]
    # extract scale
    if array_scales:
        array_scales[index, 0] = scale[0]
        array_scales[index, 1] = scale[1]
        array_scales[index, 2] = scale[2]


@wp.kernel(enable_backward=False)
def wk_compose_fabric_transformation_matrix_from_warp_arrays(
    fabric_matrices: wp.fabricarray(dtype=wp.mat44d),
    array_positions: wp.array(ndim=2, dtype=wp.float32),
    array_orientations: wp.array(ndim=2, dtype=wp.float32),
    array_scales: wp.array(ndim=2, dtype=wp.float32),
    broadcast_positions: bool,
    broadcast_orientations: bool,
    broadcast_scales: bool,
    indices: wp.array(ndim=1, dtype=wp.int32),
    mapping: wp.array(ndim=1, dtype=wp.uint32),
) -> None:
    """Compose fabric transformation matrices from separate position, orientation, and scale arrays.

    Args:
        fabric_matrices: Output fabric array of transformation matrices.
        array_positions: Input array of positions (shape (N, 3)).
        array_orientations: Input array of orientations as quaternions (shape (N, 4)).
        array_scales: Input array of scales (shape (N, 3)).
        broadcast_positions: Whether to broadcast the first position to all matrices.
        broadcast_orientations: Whether to broadcast the first orientation to all matrices.
        broadcast_scales: Whether to broadcast the first scale to all matrices.
        indices: Array of indices to process.
        mapping: Array mapping indices to fabric array positions.
    """
    i = wp.tid()
    # resolve fabric index
    fabric_index = mapping[indices[i]]
    # decompose transform matrix
    position, rotation, scale = _decompose(wp.mat44f(fabric_matrices[fabric_index]))
    # update position
    if array_positions:
        if broadcast_positions:
            index = 0
        else:
            index = i
        position[0] = array_positions[index, 0]
        position[1] = array_positions[index, 1]
        position[2] = array_positions[index, 2]
    # update orientation (Warp quaternion is xyzw)
    if array_orientations:
        if broadcast_orientations:
            index = 0
        else:
            index = i
        rotation[0] = array_orientations[index, 1]
        rotation[1] = array_orientations[index, 2]
        rotation[2] = array_orientations[index, 3]
        rotation[3] = array_orientations[index, 0]
    # update scale
    if array_scales:
        if broadcast_scales:
            index = 0
        else:
            index = i
        scale[0] = array_scales[index, 0]
        scale[1] = array_scales[index, 1]
        scale[2] = array_scales[index, 2]
    # set transform matrix
    fabric_matrices[fabric_index] = wp.mat44d(wp.transpose(wp.transform_compose(position, rotation, scale)))
