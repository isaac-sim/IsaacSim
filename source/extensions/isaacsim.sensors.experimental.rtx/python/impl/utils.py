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

"""Utility functions."""

from __future__ import annotations

import ctypes

import isaacsim.sensors.experimental.rtx.generic_model_output as generic_model_output
import numpy as np
import warp as wp


def parse_generic_model_output_data(data: wp.array) -> generic_model_output.GenericModelOutput:
    """Parse generic model output structure from annotator data.

    Args:
        data: generic-model-output data from an annotator.

    Returns:
        Generic model output structure.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.app as app_utils
        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>> from isaacsim.sensors.experimental.rtx import LidarSensor, parse_generic_model_output_data
        >>>
        >>> stage_utils.define_prim("/World/sphere", "Sphere") # doctest: +NO_CHECK
        >>> sensor = LidarSensor("/World/lidar", annotators=["generic-model-output"]) # doctest: +NO_CHECK
        >>>
        >>> # play the simulation so the sensor can fetch data
        >>> app_utils.play(commit=True)
        >>>
        >>> data, _ = sensor.get_data("generic-model-output") # doctest: +NO_CHECK
        >>> parse_generic_model_output_data(data) # doctest: +NO_CHECK
        <generic_model_output.GenericModelOutput object at 0x...>
    """
    import carb

    if data is None:
        carb.log_warn("parse_generic_model_output_data: No data provided")
        gmo = generic_model_output.GenericModelOutput()
    # build struct from buffer
    elif isinstance(data, wp.array):
        gmo = generic_model_output.getModelOutputFromBuffer(data.numpy())
    elif isinstance(data, np.ndarray):
        gmo = generic_model_output.getModelOutputFromBuffer(data)
    # build struct from pointer
    else:
        carb.log_warn("parse_generic_model_output_data: Data provided as pointer")
        # - read first 28 bytes (magic number, version, size and number of elements)
        header = (ctypes.c_char * 28).from_address(data)
        # - resolve size (in bytes) of the contiguous buffer of the model output (including the struct itself)
        size_in_bytes = int(np.frombuffer(bytes(header[16:24]), np.uint64)[0])
        # - build the GenericModelOutput struct
        buffer = (ctypes.c_char * size_in_bytes).from_address(data)
        gmo = generic_model_output.getModelOutputFromBuffer(buffer)
    # validate struct (getModelOutputFromBuffer warns if magic number is incorrect)
    if gmo.magicNumber != generic_model_output.getMagicNumberGMO():
        carb.log_warn("parse_generic_model_output_data: Invalid magic number")
        gmo = generic_model_output.GenericModelOutput()
        gmo.numElements = 0
    return gmo


def parse_stable_id_map_data(data: wp.array) -> dict:
    """Parse Stable ID Map data from annotator data.

    Args:
        data: stable-id-map annotator data.

    Returns:
        Dictionary mapping stable IDs to their prim paths.

    .. warning::

        Some object IDs returned by the LiDAR may not have an entry in
        this map. The renderer emits each 128-bit stable ID as a
        per-instance base ID combined with an "upper index" in the high
        32 bits — the submesh index for mesh geometry and the per-triangle
        primitive index for procedural geometry. The map registers per-
        instance entries and (when ``subsetCount > 1``) per-``GeomSubset``
        entries, but it does **not** register per-primitive entries, so
        hits on procedural geometry, on submeshes that weren't expanded,
        or on renderer-internal instances without a USD prim path will
        produce IDs with no map entry, and a direct ``map[id]`` lookup
        will raise ``KeyError``. Use ``map.get(id, "<unknown>")`` to
        handle missing IDs gracefully — see the bundled
        ``resolve_lidar_object_ids.py`` example for the recommended
        pattern.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.app as app_utils
        >>> import isaacsim.core.experimental.utils.stage as stage_utils
        >>> from isaacsim.sensors.experimental.rtx import LidarSensor, parse_stable_id_map_data
        >>>
        >>> stage_utils.define_prim("/World/sphere", "Sphere") # doctest: +NO_CHECK
        >>> sensor = LidarSensor("/World/lidar", annotators=["stable-id-map"]) # doctest: +NO_CHECK
        >>>
        >>> # play the simulation so the sensor can fetch data
        >>> app_utils.play(commit=True)
        >>>
        >>> data, _ = sensor.get_data("stable-id-map") # doctest: +NO_CHECK
        >>> parse_stable_id_map_data(data) # doctest: +NO_CHECK
        {0: '/World/sphere'}
    """
    data = (data.numpy() if isinstance(data, wp.array) else data).tobytes()
    data_type = np.dtype([("stable_id", "<u4", (4)), ("label_length", "<u4"), ("label_offset", "<u4")])
    data_length = int.from_bytes(data[-4:], byteorder="little") * data_type.itemsize
    return {
        int.from_bytes(item[:4].tobytes(), byteorder="little"): data[item[5] : item[5] + item[4]]
        .decode("utf8")
        .rstrip()
        for item in np.frombuffer(data[:data_length], "<u4").reshape(-1, 6)
    }


def parse_object_ids(obj_ids: np.ndarray) -> list[int]:
    """Parse 128-bit object IDs from a GenericModelOutput ``objId`` buffer.

    Each object ID is 16 bytes (128 bits). The returned integers match the keys
    produced by :func:`parse_stable_id_map_data`.

    Args:
        obj_ids: The object ID buffer from ``GenericModelOutput.objId``.

    Returns:
        List of object IDs as Python ints.
    """
    obj_ids = np.ascontiguousarray(obj_ids)
    if obj_ids.dtype == np.uint8:
        obj_ids = obj_ids.reshape(-1, 16)
    elif obj_ids.dtype == np.uint32:
        obj_ids = obj_ids.reshape(-1, 4)
    elif obj_ids.dtype == np.uint64:
        obj_ids = obj_ids.reshape(-1, 2)
    else:
        raise ValueError(f"Unsupported dtype for object IDs: {obj_ids.dtype}. Expected uint8, uint32, or uint64.")
    return [int.from_bytes(group.tobytes(), byteorder="little") for group in obj_ids]
