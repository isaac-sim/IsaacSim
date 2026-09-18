# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Pin the dropped-row marker for every data type the tensor API can return.

An out-of-range index is dropped rather than rejected, because rejecting it would
require reading the index values back from the device. The row it would have filled
carries a marker instead of zero, since zero is a legitimate reading for most
bindings and would be indistinguishable from real data.

The live gather paths only exercise the float bindings, so the integer and boolean
branches of the marker are pinned here against the derivation itself.
"""

from __future__ import annotations

import numpy as np
import pytest
import warp as wp
from isaacsim.physics.manager.impl.tensors import _invalid_fill_value

# Every data type the tensor API can hand back, with the value a dropped row must
# carry. The rule is one pattern -- all bits set -- but what that *means* differs by
# type, and it is the meaning a caller relies on.
_EXPECTED = [
    (wp.float32, "nan"),
    (wp.float64, "nan"),
    (wp.int8, -1),
    (wp.int16, -1),
    (wp.int32, -1),
    (wp.int64, -1),
    (wp.uint8, 255),
    (wp.uint16, 65535),
    (wp.uint32, 4294967295),
    (wp.uint64, 18446744073709551615),
]


@pytest.mark.parametrize(("dtype", "expected"), _EXPECTED)
def test_dropped_row_marker(dtype: object, expected: object) -> None:
    """The marker is NaN for float, -1 for signed, and the maximum for unsigned.

    Args:
        dtype: Warp data type of the output buffer.
        expected: Value a dropped row must carry, or ``"nan"``.

    """
    value = _invalid_fill_value(dtype)
    if expected == "nan":
        assert np.isnan(value), f"{dtype} marker must be NaN, got {value!r}"
    else:
        assert int(value) == expected, f"{dtype} marker must be {expected}, got {value!r}"


def test_dropped_row_marker_is_not_zero() -> None:
    """No data type may use zero, which is a legitimate reading for most bindings."""
    for dtype, _ in _EXPECTED:
        value = _invalid_fill_value(dtype)
        assert np.isnan(value) or int(value) != 0, f"{dtype} marker collides with a real value"


def test_dropped_row_marker_fills_a_buffer() -> None:
    """``fill_`` accepts the marker for every data type, on every available device."""
    devices = ["cpu"] + (["cuda:0"] if wp.get_cuda_device_count() > 0 else [])
    for device in devices:
        for dtype, _ in _EXPECTED:
            buffer = wp.zeros((2, 3), dtype=dtype, device=device)
            buffer.fill_(_invalid_fill_value(dtype))
            filled = buffer.numpy()
            assert np.all(np.isnan(filled)) or np.all(
                filled == filled.reshape(-1)[0]
            ), f"{dtype} on {device} did not fill uniformly"


def test_bool_marker_is_documented_as_indistinguishable() -> None:
    """``bool`` has no invalid value; the marker is a non-canonical true.

    Pinned so the exception in the index contract stays true: a caller cannot tell a
    dropped boolean row from a real ``True``.
    """
    value = _invalid_fill_value(wp.uint8)
    assert int(value) == 255
    assert bool(value) is True, "a dropped boolean row reads as true"
    assert int(value) != 1, "and is not the canonical true a caller would compare against"
