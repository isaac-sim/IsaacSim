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

"""Utilities for image file input/output operations including RGB, depth, and general image reading functionality."""

import os
from typing import Any

import numpy as np


def save_rgb_image(rgb_data: np.ndarray, out_dir: str, file_name: str) -> None:
    """Save RGB image data to a file on disk.

    This function converts numpy array RGB data to a PIL Image and saves it to the
    specified directory with the given filename. For RGBA data (4 channels), the file
    must be saved as PNG format since JPEG/JPG doesn't support transparency. The output
    directory is created if it doesn't exist.

    Args:
        rgb_data: RGB or RGBA image data as a numpy array with shape (height, width, channels).
        out_dir: Output directory path where the image file will be saved.
        file_name: Name of the output image file (including extension).

    Raises:
        ValueError: If trying to save RGBA data (4 channels) in JPEG/JPG format.

    Example:

    .. code-block:: python

        >>> import numpy as np
        >>> from isaacsim.test.utils.image_io import save_rgb_image
        >>>
        >>> # Save RGB image
        >>> rgb_data = np.random.randint(0, 255, (512, 512, 3), dtype=np.uint8)
        >>> save_rgb_image(rgb_data, "/tmp/test_images", "test_image.png")
        Saved image to /tmp/test_images/test_image.png with shape (512, 512, 3)
        >>>
        >>> # Save RGBA image (must use PNG format)
        >>> rgba_data = np.random.randint(0, 255, (512, 512, 4), dtype=np.uint8)
        >>> save_rgb_image(rgba_data, "/tmp/test_images", "test_rgba.png")
        Saved image to /tmp/test_images/test_rgba.png with shape (512, 512, 4)
    """
    from PIL import Image

    # Create output directory if it doesn't exist
    os.makedirs(out_dir, exist_ok=True)

    # Check if input has alpha channel or if we're dealing with RGBA data
    has_alpha = len(rgb_data.shape) == 3 and rgb_data.shape[2] == 4

    # Get file extension to check format
    file_ext = os.path.splitext(file_name)[1].lower()

    # Verify PNG format for RGBA data
    if has_alpha and file_ext in [".jpg", ".jpeg"]:
        raise ValueError(
            f"Cannot save RGBA data (4 channels) as {file_ext.upper()} format. "
            "JPEG format doesn't support transparency. Use PNG format instead."
        )

    # Convert to appropriate format based on input data
    if has_alpha:
        rgb_img = Image.fromarray(rgb_data, mode="RGBA")
    else:
        rgb_img = Image.fromarray(rgb_data)
        if file_ext == ".png":
            rgb_img = rgb_img.convert("RGBA")  # Convert to RGBA for PNG

    file_path = os.path.join(out_dir, file_name)
    rgb_img.save(file_path)
    print(f"Saved image to {file_path} with shape {rgb_data.shape}")


def save_depth_image(
    depth_data: np.ndarray,
    out_dir: str,
    file_name: str,
    normalize: bool = False,
) -> None:
    """Save depth data as TIFF (float32) or grayscale visualization.

    This function supports two primary modes:
    1. Metric (float32 TIFF): When extension is .tif/.tiff
       - Always preserves exact depth values including NaN/Inf
       - Uses 32-bit float TIFF format with lossless compression
       - If normalize=True is passed, it will be ignored with a warning

    2. Visualization (8-bit grayscale): For all other extensions
       - Converts to 8-bit grayscale for viewing
       - Optionally normalizes values to full 0-255 range based on normalize flag

    Args:
        depth_data: Depth array with shape (H, W) or (H, W, 1).
        out_dir: Output directory path.
        file_name: Output filename with extension.
        normalize: If True, normalize valid values to 0-255 range for non-TIFF formats.
            Ignored for TIFF format which always saves raw float32 data.

    Raises:
        ValueError: If depth_data has invalid shape.

    Example:

    .. code-block:: python

        >>> import numpy as np
        >>> from isaacsim.test.utils.image_io import save_depth_image
        >>>
        >>> # Save lossless float32 depth (normalize parameter is ignored for TIFF)
        >>> depth = np.random.rand(512, 512).astype(np.float32) * 10.0
        >>> save_depth_image(depth, "/tmp", "depth.tiff", normalize=False)
        Saved metric depth (float32 TIFF) to /tmp/depth.tiff
        >>>
        >>> # TIFF with normalize=True will ignore the normalize flag
        >>> save_depth_image(depth, "/tmp", "depth2.tiff", normalize=True)
        Warning: TIFF format requested with normalize=True. TIFF is intended for lossless float32 storage, ignoring normalize parameter.
        Saved metric depth (float32 TIFF) to /tmp/depth2.tiff
        >>>
        >>> # Save normalized grayscale visualization as PNG
        >>> save_depth_image(depth, "/tmp", "depth_viz.png", normalize=True)
        Saved grayscale depth to /tmp/depth_viz.png
    """
    from PIL import Image

    # Ensure output directory exists
    os.makedirs(out_dir, exist_ok=True)

    # Validate and convert to 2D array
    if len(depth_data.shape) == 3 and depth_data.shape[2] == 1:
        depth_data = depth_data.squeeze(axis=2)
    elif len(depth_data.shape) != 2:
        raise ValueError(f"Expected depth data with shape (H, W) or (H, W, 1), got {depth_data.shape}")

    # Determine file path and extension
    file_path = os.path.join(out_dir, file_name)
    file_ext = os.path.splitext(file_name)[1].lower()

    # Route to appropriate format based on extension
    if file_ext in (".tif", ".tiff"):
        # TIFF is always saved as lossless float32, regardless of normalize flag
        if normalize:
            print(
                f"Warning: TIFF format requested with normalize=True. "
                f"TIFF is intended for lossless float32 storage, ignoring normalize parameter."
            )

        # Save as lossless float32 TIFF
        depth_f32 = depth_data.astype(np.float32, copy=False)
        img = Image.fromarray(depth_f32, mode="F")

        # Try compressed save first, fallback to uncompressed
        try:
            img.save(file_path, compression="tiff_deflate")
        except Exception:
            img.save(file_path)

        print(f"Saved metric depth (float32 TIFF) to {file_path}")
    else:
        # Convert to 8-bit grayscale for visualization
        if normalize:
            # Normalize based on valid min/max values
            valid_mask = np.isfinite(depth_data)

            if not np.any(valid_mask):
                # No valid values - use mid-gray
                depth_u8 = np.full_like(depth_data, 128, dtype=np.uint8)
            else:
                # Get min/max of valid values
                valid_values = depth_data[valid_mask]
                vmin, vmax = np.min(valid_values), np.max(valid_values)

                if vmax > vmin:
                    # Replace invalid values with vmin, then normalize
                    depth_clean = np.where(valid_mask, depth_data, vmin)
                    normalized = (depth_clean - vmin) / (vmax - vmin)
                    depth_u8 = (normalized * 255).astype(np.uint8)
                else:
                    # All values identical - use mid-gray
                    depth_u8 = np.full_like(depth_data, 128, dtype=np.uint8)
        else:
            # No normalization - direct conversion
            if depth_data.dtype in (np.float32, np.float64):
                # Assume 0-1 range for floats
                depth_u8 = (np.clip(depth_data, 0, 1) * 255).astype(np.uint8)
            else:
                # Direct conversion for integer types
                depth_u8 = depth_data.astype(np.uint8)

        # Save as grayscale image
        img = Image.fromarray(depth_u8, mode="L")
        img.save(file_path)
        print(f"Saved image as grayscale depth to {file_path}")


def save_annotator_data(data: Any, output_path: str) -> None:
    """Save annotator data returned by :func:`capture_viewport_annotator_data_async` to disk.

    Handles the three forms that annotator data can take:

    * **dict with a ``"data"`` array key** — saves ``data["data"]`` as a ``.npy`` file.
      Any other dict is serialised to JSON (extension replaced with ``.json``).
    * **numpy array** — saves as ``.npy`` via ``numpy.save``, or as a PNG image via
      :func:`save_rgb_image` when *output_path* ends with ``.png``.
    * **other** — prints the type and a short repr; nothing is written to disk.

    The output directory is created automatically if it does not exist.

    Args:
        data: Annotator data as returned by ``capture_viewport_annotator_data_async``.
        output_path: Destination file path.  The extension determines the format for
            array data (``.npy`` or ``.png``); dict data always uses ``.json``.

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils.image_io import save_annotator_data
        >>>
        >>> # Save depth data captured from the viewport
        >>> save_annotator_data(depth_data, "/tmp/depth.npy")
        Saved array: /tmp/depth.npy (shape: (720, 1280, 1), dtype: float32)
        >>>
        >>> # Save RGB data as PNG
        >>> save_annotator_data(rgb_data, "/tmp/frame.png")
        Saved image to /tmp/frame.png with shape (720, 1280, 4)
    """
    import json

    import numpy as np

    out_dir = os.path.dirname(output_path) or "."
    file_name = os.path.basename(output_path)
    os.makedirs(out_dir, exist_ok=True)

    if isinstance(data, dict):
        if "data" in data and hasattr(data["data"], "shape"):
            np.save(output_path, data["data"])
            print(f"Saved data['data'] array: {output_path} (shape: {data['data'].shape})")
        else:
            json_path = os.path.splitext(output_path)[0] + ".json"
            with open(json_path, "w") as f:
                json.dump({k: repr(v) for k, v in data.items()}, f, indent=2)
            print(f"Saved dict data: {json_path}")
    elif hasattr(data, "shape"):
        if output_path.endswith(".png"):
            save_rgb_image(data, out_dir, file_name)
        else:
            np.save(output_path, data)
            print(f"Saved array: {output_path} (shape: {data.shape}, dtype: {data.dtype})")
    else:
        print(f"Annotator returned: {type(data).__name__}: {repr(data)[:200]}")


def read_image_as_array(file_path: str, squeeze_singleton_channel: bool = True) -> np.ndarray:
    """Read an image file and return it as a numpy array.

    This function loads an image file using PIL and converts it to a numpy array,
    following the same preprocessing steps used internally by compare_images_within_tolerances.
    The resulting array can be used with compare_arrays_within_tolerances for custom
    image comparison workflows.

    Args:
        file_path: Path to the image file to read.
        squeeze_singleton_channel: If True, squeeze singleton channel dimensions (H, W, 1) to (H, W).
            This matches the behavior of compare_images_within_tolerances.

    Returns:
        Image data as a numpy array. For RGB/RGBA images, shape is (height, width, channels).
        For grayscale images, shape is (height, width) if squeeze_singleton_channel=True,
        otherwise (height, width, 1).

    Raises:
        FileNotFoundError: If the image file does not exist.
        IOError: If the image file cannot be opened or read.

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils.image_io import read_image_as_array
        >>> from isaacsim.test.utils.image_comparison import compare_arrays_within_tolerances
        >>>
        >>> # Read images as arrays
        >>> golden_array = read_image_as_array("/path/to/golden.png")
        >>> test_array = read_image_as_array("/path/to/test.png")
        >>>
        >>> # Compare arrays directly
        >>> metrics = compare_arrays_within_tolerances(
        ...     golden_array,
        ...     test_array,
        ...     mean_tolerance=5.0,
        ...     percentile_tolerance=(95, 10.0)
        ... )
        >>> metrics["passed"]
        True
        >>>
        >>> # Read without squeezing singleton channels
        >>> grayscale_array = read_image_as_array("/path/to/grayscale.png", squeeze_singleton_channel=False)
        >>> grayscale_array.shape
        (512, 512, 1)
    """
    from PIL import Image

    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Image file not found: {file_path}")

    try:
        img = Image.open(file_path)
        array = np.array(img)
    except Exception as e:
        raise IOError(f"Failed to read image file {file_path}: {e}") from e

    # Apply singleton channel squeezing if requested (matches compare_images_within_tolerances behavior)
    if squeeze_singleton_channel and array.ndim == 3 and array.shape[2] == 1:
        array = array[:, :, 0]

    return array
