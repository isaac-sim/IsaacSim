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

"""Utility functions for creating ROS 2 image messages in tests."""

import re

import numpy as np

try:
    from sensor_msgs.msg import Image
except ImportError:
    raise ImportError(
        "sensor_msgs is required to subscribe to image topics. " "Make sure ROS2 sensor_msgs package is available."
    )


def create_image(height: int, width: int, encoding: str, step: int, data: bytes, is_bigendian: int = 0) -> Image:
    """Create a ``sensor_msgs/msg/Image`` message with the given parameters.

    Args:
        height: Image height in pixels.
        width: Image width in pixels.
        encoding: Image encoding string (e.g., ``rgb8``, ``mono8``).
        step: Full row length in bytes.
        data: Raw image data bytes.
        is_bigendian: Whether the data is big-endian.

    Returns:
        A populated ROS2 Image message.
    """
    img = Image()
    img.height = int(height)
    img.width = int(width)
    img.encoding = str(encoding)
    img.step = int(step)
    img.data = data if isinstance(data, list) else list(data)
    img.is_bigendian = int(is_bigendian)
    return img


def ros2_image_to_buffer(
    image_msg: Image,
    normalize_color_order: bool = True,
    squeeze_singleton_channel: bool = True,
    copy: bool = False,
) -> np.ndarray:
    """Convert a ROS2 ``sensor_msgs/msg/Image`` to a numpy array.

    Extracts the image buffer from a ROS2 image message and returns it as a numpy array
    compatible with Isaac Sim test utilities (e.g., image IO and comparison helpers).
    Supports encodings:
    - Color: ``rgb8``, ``rgba8``, ``rgb16``, ``rgba16``, ``bgr8``, ``bgra8``, ``bgr16``, ``bgra16``.
    - Mono: ``mono8``, ``mono16``.
    - OpenCV-style: ``8UC{1..4}``, ``8SC{1..4}``, ``16UC{1..4}``, ``16SC{1..4}``, ``32SC{1..4}``,
        ``32FC{1..4}``, ``64FC{1..4}``.
    - Bayer (raw): ``bayer_rggb8``, ``bayer_bggr8``, ``bayer_gbrg8``, ``bayer_grbg8`` and 16-bit variants.
    - YUV 4:2:2 packed: ``uyvy``, ``yuyv``, ``yuv422``, ``yuv422_yuy2`` (returned as shape ``(H, W, 2)`` bytes).
    - YUV 4:2:0 / 4:4:4 (``nv21``, ``nv24``) are not decoded and will raise ``ValueError``.

    For color BGR/BGRA (8/16-bit), channels are reordered to RGB/RGBA when ``return_rgb_order=True``.

    Args:
        image_msg: ROS2 image message (``sensor_msgs.msg.Image``-like) providing ``height``,
            ``width``, ``encoding``, ``step``, ``data`` (bytes), and optionally ``is_bigendian``.
        normalize_color_order: If True, normalize BGR/BGRA channel order to RGB/RGBA.
            Has no effect for monochrome, Bayer, OpenCV generic types where order is not BGR,
            or YUV encodings which are not converted here.
        squeeze_singleton_channel: If True, squeeze single-channel output to ``(H, W)``.
        copy: If True, return a contiguous copy. Otherwise returns a view when possible.

    Returns:
        Numpy array of shape ``(H, W, C)`` for color encodings or ``(H, W)`` for single-channel
        encodings when ``squeeze_singleton_channel=True``. Dtype depends on encoding.

    Raises:
        ValueError: If the encoding is unsupported or the buffer size/stride is inconsistent.

    Example:

    .. code-block:: python

        >>> # Suppose `msg` is a sensor_msgs.msg.Image with encoding 'bgr8'
        >>> arr = ros2_image_to_buffer(msg, normalize_color_order=True)
        >>> arr.shape  # (H, W, 3)
        ...
    """
    height = int(getattr(image_msg, "height"))
    width = int(getattr(image_msg, "width"))
    original_encoding = str(getattr(image_msg, "encoding"))
    enc_lower = original_encoding.lower()
    enc_upper = original_encoding.upper()
    step = int(getattr(image_msg, "step"))
    data = getattr(image_msg, "data")
    is_bigendian = int(getattr(image_msg, "is_bigendian", 0))

    if not isinstance(data, (bytes, bytearray, memoryview)):
        data = bytes(data)

    # 1) Fixed encodings (color/mono 8/16-bit + Bayer + YUV422 packed)
    fixed_map: dict[str, tuple[np.dtype, int, str]] = {
        # 8-bit color
        "rgb8": (np.uint8, 3, "rgb"),
        "bgr8": (np.uint8, 3, "bgr"),
        "rgba8": (np.uint8, 4, "rgba"),
        "bgra8": (np.uint8, 4, "bgra"),
        # 16-bit color
        "rgb16": (np.uint16, 3, "rgb"),
        "bgr16": (np.uint16, 3, "bgr"),
        "rgba16": (np.uint16, 4, "rgba"),
        "bgra16": (np.uint16, 4, "bgra"),
        # mono
        "mono8": (np.uint8, 1, "gray"),
        "mono16": (np.uint16, 1, "gray"),
        # bayer raw (treat as single channel)
        "bayer_rggb8": (np.uint8, 1, "bayer"),
        "bayer_bggr8": (np.uint8, 1, "bayer"),
        "bayer_gbrg8": (np.uint8, 1, "bayer"),
        "bayer_grbg8": (np.uint8, 1, "bayer"),
        "bayer_rggb16": (np.uint16, 1, "bayer"),
        "bayer_bggr16": (np.uint16, 1, "bayer"),
        "bayer_gbrg16": (np.uint16, 1, "bayer"),
        "bayer_grbg16": (np.uint16, 1, "bayer"),
        # yuv422 packed (represent as 2 bytes per pixel, 2 channels)
        "uyvy": (np.uint8, 2, "yuv422"),
        "yuyv": (np.uint8, 2, "yuv422"),
        "yuv422": (np.uint8, 2, "yuv422"),
        "yuv422_yuy2": (np.uint8, 2, "yuv422"),
    }

    dtype: np.dtype
    channels: int
    channel_order: str

    if enc_lower in fixed_map:
        dtype, channels, channel_order = fixed_map[enc_lower]
    else:
        # 2) OpenCV-style encodings like 8UC3, 16SC1, 32FC4, 64FC2...
        cv_match = re.fullmatch(r"(8|16|32|64)(U|S|F)C([0-9]+)", enc_upper)
        if cv_match:
            bits = int(cv_match.group(1))
            kind = cv_match.group(2)  # U,S,F
            channels = int(cv_match.group(3))
            if kind == "U":
                dtype = {8: np.uint8, 16: np.uint16, 32: np.uint32, 64: np.uint64}[bits]
            elif kind == "S":
                dtype = {8: np.int8, 16: np.int16, 32: np.int32, 64: np.int64}[bits]
            else:  # 'F'
                dtype = {32: np.float32, 64: np.float64}[bits]
            channel_order = "generic"
        else:
            # 3) Multi-plane YUV not supported here
            if enc_lower in ("nv21", "nv24"):
                raise ValueError(
                    f"Encoding '{original_encoding}' (multi-plane YUV) is not supported by ros2_image_to_buffer. "
                    f"Please convert to RGB(A) before calling this function."
                )
            raise ValueError(f"Unsupported ROS2 image encoding: '{original_encoding}'")

    expected_row_bytes = width * channels * dtype().nbytes

    if step < expected_row_bytes:
        raise ValueError(
            f"Inconsistent step (stride): step={step}, expected at least {expected_row_bytes} for "
            f"width={width}, channels={channels}, dtype={dtype}"
        )

    needs_byteswap = (is_bigendian == 1) and (dtype().nbytes > 1) and (np.dtype(dtype).byteorder in ("<", "="))

    if step == expected_row_bytes:
        arr = np.frombuffer(data, dtype=dtype)
        if arr.size != height * width * channels:
            raise ValueError(f"Buffer size mismatch: got {arr.size} elements, expected {height * width * channels}")
        if channels == 1:
            arr = arr.reshape((height, width))
        else:
            arr = arr.reshape((height, width, channels))
    else:
        row_arrays = []
        row_stride_bytes = step
        for r in range(height):
            start = r * row_stride_bytes
            end = start + expected_row_bytes
            row_bytes = data[start:end]
            if len(row_bytes) != expected_row_bytes:
                raise ValueError(f"Row {r} length mismatch: got {len(row_bytes)} bytes, expected {expected_row_bytes}")
            row_arr = np.frombuffer(row_bytes, dtype=dtype)
            row_arrays.append(row_arr)
        stacked = np.stack(row_arrays, axis=0)
        if channels == 1:
            arr = stacked.reshape((height, width))
        else:
            arr = stacked.reshape((height, width, channels))

    if needs_byteswap:
        arr = arr.byteswap(inplace=False)

    if normalize_color_order and channels in (3, 4) and channel_order in ("bgr", "bgra"):
        # Handle both 8-bit and 16-bit color by slicing channels only
        if channels == 3:
            if channels == 3:
                arr = arr[:, :, ::-1]  # BGR -> RGB
        else:
            rgb = arr[:, :, :3][:, :, ::-1]  # BGRA -> RGB
            alpha = arr[:, :, 3:4]
            arr = np.concatenate([rgb, alpha], axis=2)

    if channels == 1 and squeeze_singleton_channel:
        pass  # already (H, W)
    elif channels == 1:
        arr = np.expand_dims(arr, axis=2)

    if copy:
        arr = np.array(arr, copy=True)

    return arr
