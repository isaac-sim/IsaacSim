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

"""Utility functions for camera sensor tests."""

import os

import cv2
import numpy as np


def debug_draw_clear_points() -> None:
    """Clear all debug draw points from the viewport."""
    from isaacsim.util.debug_draw import _debug_draw

    draw_iface = _debug_draw.acquire_debug_draw_interface()
    draw_iface.clear_points()


def debug_draw_pointcloud(pointcloud_data: object, color: object, size: float, clear_existing: bool = False) -> None:
    """Draw a pointcloud in the viewport for visual debugging.

    Args:
        pointcloud_data: NumPy array with shape (N, 3) containing 3D points.
        color: RGBA tuple for the point color (e.g., (1, 0, 0, 1) for red).
        size: Size of the points to draw.
        clear_existing: If True, clear existing points before drawing.

    Returns:
        None.

    """
    if not (isinstance(pointcloud_data, np.ndarray) and pointcloud_data.ndim == 2 and pointcloud_data.shape[1] == 3):
        print("Warning: pointcloud_data must be a NumPy array with shape (N, 3).")
        return

    from isaacsim.util.debug_draw import _debug_draw

    draw_iface = _debug_draw.acquire_debug_draw_interface()

    points_cloud = []
    colors_cloud = []
    sizes_cloud = []
    for i in range(pointcloud_data.shape[0]):
        points_cloud.append(pointcloud_data[i].tolist())
        colors_cloud.append(color)
        sizes_cloud.append(size)

    if clear_existing:
        debug_draw_clear_points()
    draw_iface.draw_points(points_cloud, colors_cloud, sizes_cloud)


def save_image(
    image: object,
    filename: str,
    golden_dir: str,
    test_dir: str,
    save_as_golden: bool = False,
    save_as_test: bool = False,
) -> None:
    """Save an image to the golden or test directory.

    Args:
        image: The image data to save.
        filename: The filename to use when saving the image.
        golden_dir: Directory path where golden reference images are stored.
        test_dir: Directory path where test output images are stored.
        save_as_golden: If True, save the image to the golden directory.
        save_as_test: If True, save the image to the test directory.
    """
    if not filename.lower().endswith(".png"):
        filename += ".png"
    if save_as_test:
        image_path = os.path.join(test_dir, filename)
        print(f" - saving image (test): {image_path}")
        os.makedirs(test_dir, exist_ok=True)
        cv2.imwrite(image_path, image)
    if save_as_golden:
        image_path = os.path.join(golden_dir, filename)
        print(f" - saving image (golden): {image_path}")
        os.makedirs(golden_dir, exist_ok=True)
        cv2.imwrite(image_path, image)


def compare_images(src1: object, src2: object, ksize: int = 5, thresh: int = 30, hist_div: int = 1) -> tuple:
    """Compare two images using histogram correlation and background subtraction.

    Args:
        src1: The first image to compare.
        src2: The second image to compare.
        ksize: Kernel size used for median blur and erosion operations.
        thresh: Threshold value for binary thresholding in background subtraction.
        hist_div: Divisor applied to histogram bin counts to reduce resolution.

    Returns:
        A tuple of (combined_score, histogram_score, background_subtraction_score).
    """

    def compare_histograms(src1: object, src2: object) -> float:
        # gray scale images
        if src1.ndim == 2:
            # calculate histograms
            hist1 = cv2.calcHist([src1], [0], None, [int(256 / hist_div)], [0, 256])
            hist2 = cv2.calcHist([src2], [0], None, [int(256 / hist_div)], [0, 256])
        # colored images
        elif src1.ndim == 3 and src1.shape[2] == 3:
            # convert to HSV color space
            hsv1 = cv2.cvtColor(src1, cv2.COLOR_BGR2HSV)
            hsv2 = cv2.cvtColor(src2, cv2.COLOR_BGR2HSV)
            # calculate histograms
            hist1 = cv2.calcHist([hsv1], [0, 1], None, [int(180 / hist_div), int(256 / hist_div)], [0, 180, 0, 256])
            hist2 = cv2.calcHist([hsv2], [0, 1], None, [int(180 / hist_div), int(256 / hist_div)], [0, 180, 0, 256])
        else:
            raise ValueError(f"Unknown format. Shape: {src1.shape}")
        # normalize histograms
        cv2.normalize(hist1, hist1)
        cv2.normalize(hist2, hist2)
        # compare histograms
        return cv2.compareHist(hist1, hist2, cv2.HISTCMP_CORREL)

    def background_subtraction(src1: object, src2: object) -> float:
        # gray scale images
        if src1.ndim == 2:
            pass
        # colored images
        elif src1.ndim == 3 and src1.shape[2] == 3:
            # convert to gray scale
            src1 = cv2.cvtColor(src1, cv2.COLOR_BGR2GRAY)
            src2 = cv2.cvtColor(src2, cv2.COLOR_BGR2GRAY)
        else:
            raise ValueError(f"Unknown format. Shape: {src1.shape}")
        # compute naive background subtraction
        diff = cv2.absdiff(src1, src2)
        _, diff = cv2.threshold(diff, thresh, 255, cv2.THRESH_BINARY)  # + cv2.THRESH_OTSU)
        diff = cv2.erode(diff, np.ones((ksize, ksize), np.uint8))
        # scale mask
        return np.exp(-10 * np.sum(diff == 255) / diff.size)

    # pre-process images
    # - remove dimensions with shape 1
    src1 = np.squeeze(src1)
    src2 = np.squeeze(src2)
    # - check dims and shape/channels
    assert src1.ndim == src2.ndim, f"Number of dimensions does not match: {src1.ndim} != {src2.ndim}"
    if src1.ndim == 3:
        assert src1.shape[2] == src2.shape[2], f"Number of channels does not match: {src1.shape[2]} != {src2.shape[2]}"
    assert src1.shape[:2] == src2.shape[:2], f"Shapes does not match: {src1.shape[:2]} != {src2.shape[:2]}"
    # - apply filter to "smooth" images
    src1 = cv2.medianBlur(src1, ksize)
    src2 = cv2.medianBlur(src2, ksize)

    h_score = compare_histograms(src1, src2)
    bs_score = background_subtraction(src1, src2)
    return (h_score * bs_score, h_score, bs_score)
