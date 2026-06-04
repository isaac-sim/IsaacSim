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

"""Image comparison utilities for testing and validation."""

import os
import re

import numpy as np
from PIL import Image


def compute_difference_metrics(
    golden_array: np.ndarray,
    test_array: np.ndarray,
    *,
    percentiles: list[float] | tuple[float, ...] = (50, 75, 90, 95, 99),
    allclose_rtol: float = 1e-05,
    allclose_atol: float = 1e-08,
    ignore_blank_pixels: bool = False,
) -> dict[str, object]:
    """Compute difference metrics between two image arrays.

    Computes a reusable set of metrics for image comparison, including ``np.allclose``
    with the provided tolerances, mean and max absolute differences, RMSE, and a set
    of requested percentiles of absolute differences.

    Args:
        golden_array: Reference image array.
        test_array: Test image array to compare.
        percentiles: Sequence of percentiles to compute for absolute differences.
        allclose_rtol: Relative tolerance for the allclose metric.
        allclose_atol: Absolute tolerance for the allclose metric.
        ignore_blank_pixels: If True, ignore blank pixels in the comparison.

    Returns:
        A dictionary containing the computed metrics: ``allclose``, ``rtol``, ``atol``,
        ``mean_abs``, ``max_abs``, ``rmse``, and ``percentiles`` (a mapping from percentile
        to value).

    Raises:
        ValueError: If image shapes do not match.

    Example:

    .. code-block:: python

        >>> import numpy as np
        >>> from isaacsim.test.utils.image_comparison import compute_difference_metrics
        >>> a = np.zeros((2, 2), dtype=np.uint8)
        >>> b = np.zeros((2, 2), dtype=np.uint8)
        >>> metrics = compute_difference_metrics(a, b)
        >>> bool(metrics["allclose"])  # True if arrays are close with defaults
        True
    """
    return _compute_difference_metrics_impl(
        golden_array=golden_array,
        test_array=test_array,
        percentiles=percentiles,
        allclose_rtol=allclose_rtol,
        allclose_atol=allclose_atol,
        ignore_blank_pixels=ignore_blank_pixels,
    )


def _compute_difference_metrics_impl(
    golden_array: np.ndarray,
    test_array: np.ndarray,
    *,
    percentiles: list[float] | tuple[float, ...],
    allclose_rtol: float,
    allclose_atol: float,
    ignore_blank_pixels: bool,
) -> dict[str, object]:
    """Internal implementation of compute_difference_metrics.

    Args:
        golden_array: Reference image array.
        test_array: Test image array to compare.
        percentiles: Sequence of percentiles to compute for absolute differences.
        allclose_rtol: Relative tolerance for the allclose metric.
        allclose_atol: Absolute tolerance for the allclose metric.
        ignore_blank_pixels: If True, ignore blank pixels in the comparison.

    Returns:
        A dictionary containing the computed metrics: ``allclose``, ``rtol``, ``atol``,
        ``mean_abs``, ``max_abs``, ``rmse``, and ``percentiles`` (a mapping from percentile
        to value).

    Raises:
        ValueError: If image shapes do not match.
    """
    if golden_array.shape != test_array.shape:
        raise ValueError(f"Image shapes do not match: golden {golden_array.shape} vs test {test_array.shape}")

    # Promote to float for numeric stability, keep NaNs to leverage equal_nan in allclose.
    golden_float = golden_array.astype(np.float64, copy=False)
    test_float = test_array.astype(np.float64, copy=False)

    # Build mask to compute stats on valid numeric entries only.
    valid_mask = np.isfinite(golden_float) & np.isfinite(test_float)
    if ignore_blank_pixels:
        valid_mask = valid_mask & (golden_float != 0) & (test_float != 0)
    if np.any(valid_mask):
        diff_vals = np.abs(golden_float[valid_mask] - test_float[valid_mask])
        rmse = float(np.sqrt(np.mean((golden_float[valid_mask] - test_float[valid_mask]) ** 2)))
        mean_diff = float(np.mean(diff_vals))
        max_diff = float(np.max(diff_vals))
    else:
        # No valid numeric elements; define neutral statistics.
        diff_vals = np.array([0.0], dtype=np.float64)
        rmse = 0.0
        mean_diff = 0.0
        max_diff = 0.0

    # Compute allclose across the full arrays (NaN == NaN when equal_nan=True).
    allclose_result = np.allclose(golden_float, test_float, rtol=allclose_rtol, atol=allclose_atol, equal_nan=True)

    # Compute requested percentiles.
    percentile_values: dict[float, float] = {}
    for p in sorted(set(percentiles)):
        percentile_values[p] = float(np.percentile(diff_vals, p))

    return {
        "allclose": bool(allclose_result),
        "rtol": float(allclose_rtol),
        "atol": float(allclose_atol),
        "mean_abs": float(mean_diff),
        "max_abs": float(max_diff),
        "rmse": float(rmse),
        "percentiles": percentile_values,
    }


def print_difference_statistics(metrics: dict[str, object]) -> None:
    """Pretty-print image difference metrics.

    Prints a stable summary of the metrics computed by ``compute_difference_metrics``.

    Args:
        metrics: Dictionary returned by ``compute_difference_metrics``.

    Example:

    .. code-block:: python

        >>> import numpy as np
        >>> from isaacsim.test.utils.image_comparison import (
        ...     compute_difference_metrics, print_difference_statistics
        ... )
        >>> a = np.zeros((2, 2), dtype=np.uint8)
        >>> b = np.ones((2, 2), dtype=np.uint8)
        >>> m = compute_difference_metrics(a, b)
        >>> print_difference_statistics(m)  # doctest: +ELLIPSIS
        ...
    """
    header = "=== All metrics ==="
    print(f"\n{header}")

    print(f"Allclose (rtol={metrics['rtol']}, atol={metrics['atol']}): " f"{'PASS' if metrics['allclose'] else 'FAIL'}")
    print(f"Mean absolute difference: {metrics['mean_abs']:.3f}")
    print(f"Max absolute difference: {metrics['max_abs']:.3f}")
    print(f"Root Mean Square Error (RMSE): {metrics['rmse']:.3f}")

    for p in sorted(metrics.get("percentiles", {}).keys()):
        p_value = metrics["percentiles"][p]
        print(f"{int(p)}th percentile of differences: {p_value:.3f}")

    print("=" * len(header) + "\n")


def compare_arrays_within_tolerances(
    golden_array: np.ndarray,
    test_array: np.ndarray,
    allclose_rtol: float | None = 1e-05,
    allclose_atol: float | None = 1e-08,
    mean_tolerance: float | None = None,
    max_tolerance: float | None = None,
    absolute_tolerance: float | None = None,
    percentile_tolerance: tuple | None = None,
    rmse_tolerance: float | None = None,
    print_all_stats: bool = False,
) -> dict[str, object]:
    """Compare two image arrays against tolerance-based criteria.

    Args:
        golden_array: Reference image array.
        test_array: Test image array to compare.
        allclose_rtol: Relative tolerance for np.allclose. Pass None to
            disable allclose check (requires allclose_atol=None too).
        allclose_atol: Absolute tolerance for np.allclose. Pass None to
            disable allclose check (requires allclose_rtol=None too).
        mean_tolerance: Maximum acceptable mean absolute difference (optional).
        max_tolerance: Maximum acceptable max absolute difference (optional).
        absolute_tolerance: Maximum absolute difference for any pixel (optional).
        percentile_tolerance: Tuple of (percentile, tolerance) for percentile-based comparison (optional).
        rmse_tolerance: Maximum acceptable root mean square error (optional).
        print_all_stats: If True, compute and print all metrics regardless of criteria.

    Returns:
        A dictionary with keys: ``passed`` (bool), ``criteria`` (mapping of criterion
        to bool), ``metrics`` (the computed metrics), and ``thresholds`` (configured values).

    Raises:
        ValueError: If image shapes do not match.

    Example:

    .. code-block:: python

        >>> import numpy as np
        >>> from isaacsim.test.utils.image_comparison import compare_arrays_within_tolerances
        >>> a = np.zeros((2, 2), dtype=np.uint8)
        >>> b = np.ones((2, 2), dtype=np.uint8)
        >>> res = compare_arrays_within_tolerances(a, b, mean_tolerance=0.1)
        >>> sorted(res["criteria"].keys())  # doctest: +ELLIPSIS
        ...
    """
    if golden_array.shape != test_array.shape:
        raise ValueError(f"Image shapes do not match: golden {golden_array.shape} vs test {test_array.shape}")

    if golden_array.dtype != test_array.dtype:
        test_array = test_array.astype(golden_array.dtype)

    default_percentiles = [50, 75, 90, 95, 99]
    extra_percentiles = []
    if percentile_tolerance is not None:
        if isinstance(percentile_tolerance, (tuple, list)) and len(percentile_tolerance) == 2:
            extra_percentiles = [float(percentile_tolerance[0])]
        else:
            raise ValueError("percentile_tolerance must be a tuple of (percentile, tolerance)")
    percentiles_to_compute = sorted(set(default_percentiles + extra_percentiles))

    metrics_rtol = allclose_rtol if allclose_rtol is not None else 1e-05
    metrics_atol = allclose_atol if allclose_atol is not None else 1e-08

    metrics = compute_difference_metrics(
        golden_array=golden_array,
        test_array=test_array,
        percentiles=percentiles_to_compute,
        allclose_rtol=metrics_rtol,
        allclose_atol=metrics_atol,
    )

    criteria_results: dict[str, bool] = {}

    if not (allclose_rtol is None and allclose_atol is None):
        criteria_results["allclose"] = bool(metrics["allclose"])  # evaluated with metrics_rtol/metrics_atol

    if mean_tolerance is not None:
        criteria_results["mean"] = float(metrics["mean_abs"]) <= float(mean_tolerance)

    if max_tolerance is not None:
        criteria_results["max"] = float(metrics["max_abs"]) <= float(max_tolerance)

    if absolute_tolerance is not None:
        criteria_results["absolute"] = float(metrics["max_abs"]) <= float(absolute_tolerance)

    if percentile_tolerance is not None:
        percentile, tolerance = float(percentile_tolerance[0]), float(percentile_tolerance[1])
        percentile_value = float(metrics["percentiles"].get(float(percentile), np.nan))
        criteria_results[f"percentile_{int(percentile)}"] = percentile_value <= tolerance

    if rmse_tolerance is not None:
        criteria_results["rmse"] = float(metrics["rmse"]) <= float(rmse_tolerance)

    passed = all(criteria_results.values()) if criteria_results else True

    thresholds: dict[str, object] = {
        "rtol": float(metrics_rtol),
        "atol": float(metrics_atol),
    }
    if mean_tolerance is not None:
        thresholds["mean_tolerance"] = float(mean_tolerance)
    if max_tolerance is not None:
        thresholds["max_tolerance"] = float(max_tolerance)
    if absolute_tolerance is not None:
        thresholds["absolute_tolerance"] = float(absolute_tolerance)
    if percentile_tolerance is not None:
        thresholds["percentile_tolerance"] = (float(percentile_tolerance[0]), float(percentile_tolerance[1]))
    if rmse_tolerance is not None:
        thresholds["rmse_tolerance"] = float(rmse_tolerance)

    if print_all_stats:
        print_difference_statistics(metrics)

    return {"passed": passed, "criteria": criteria_results, "metrics": metrics, "thresholds": thresholds}


def compare_images_within_tolerances(
    golden_file_path: str,
    test_file_path: str,
    allclose_rtol: float | None = 1e-05,
    allclose_atol: float | None = 1e-08,
    mean_tolerance: float | None = None,
    max_tolerance: float | None = None,
    absolute_tolerance: float | None = None,
    percentile_tolerance: tuple | None = None,
    rmse_tolerance: float | None = None,
    print_all_stats: bool = False,
) -> dict[str, object]:
    """Compare two image files against tolerance-based criteria.

    Args:
        golden_file_path: Path to the reference image file.
        test_file_path: Path to the test image file to compare.
        allclose_rtol: Relative tolerance for np.allclose. Pass None to
            disable allclose check (requires allclose_atol=None too).
        allclose_atol: Absolute tolerance for np.allclose. Pass None to
            disable allclose check (requires allclose_rtol=None too).
        mean_tolerance: Maximum acceptable mean absolute difference (optional).
        max_tolerance: Maximum acceptable max absolute difference (optional).
        absolute_tolerance: Maximum absolute difference for any pixel (optional).
        percentile_tolerance: Tuple of (percentile, tolerance) for percentile-based comparison (optional).
        rmse_tolerance: Maximum acceptable root mean square error (optional).
        print_all_stats: If True, compute and print all metrics regardless of criteria.

    Returns:
        A dictionary with keys: ``passed`` (bool), ``criteria`` (mapping of criterion
        to bool), ``metrics`` (the computed metrics), and ``thresholds`` (configured values).

    Raises:
        FileNotFoundError: If either image file does not exist.

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils.image_comparison import compare_images_within_tolerances
        >>> res = compare_images_within_tolerances("golden.png", "test.png", mean_tolerance=1.0)
        >>> isinstance(res["passed"], bool)
        True
    """
    if not os.path.exists(golden_file_path):
        raise FileNotFoundError(f"Golden image file not found: {golden_file_path}")
    if not os.path.exists(test_file_path):
        raise FileNotFoundError(f"Test image file not found: {test_file_path}")

    golden_img = Image.open(golden_file_path)
    test_img = Image.open(test_file_path)

    golden_array = np.array(golden_img)
    test_array = np.array(test_img)

    def _squeeze_singleton_channel(arr: np.ndarray) -> np.ndarray:
        if arr.ndim == 3 and arr.shape[2] == 1:
            return arr[:, :, 0]
        return arr

    golden_array = _squeeze_singleton_channel(golden_array)
    test_array = _squeeze_singleton_channel(test_array)

    if (
        golden_array.ndim == 3
        and test_array.ndim == 3
        and golden_array.shape[0] == test_array.shape[0]
        and golden_array.shape[1] == test_array.shape[1]
        and {golden_array.shape[2], test_array.shape[2]} == {3, 4}
    ):
        if golden_array.shape[2] == 4:
            golden_array = golden_array[:, :, :3]
        if test_array.shape[2] == 4:
            test_array = test_array[:, :, :3]

    return compare_arrays_within_tolerances(
        golden_array=golden_array,
        test_array=test_array,
        allclose_rtol=allclose_rtol,
        allclose_atol=allclose_atol,
        mean_tolerance=mean_tolerance,
        max_tolerance=max_tolerance,
        absolute_tolerance=absolute_tolerance,
        percentile_tolerance=percentile_tolerance,
        rmse_tolerance=rmse_tolerance,
        print_all_stats=print_all_stats,
    )


def compare_images_in_directories(
    golden_dir: str,
    test_dir: str,
    path_pattern: str | None = None,
    allclose_rtol: float | None = 1e-05,
    allclose_atol: float | None = 1e-08,
    mean_tolerance: float | None = None,
    max_tolerance: float | None = None,
    absolute_tolerance: float | None = None,
    percentile_tolerance: tuple | None = None,
    rmse_tolerance: float | None = None,
    print_all_stats: bool = False,
    print_per_file_results: bool = True,
) -> dict[str, object]:
    r"""Compare matching image files in two directories against tolerance-based criteria.

    This function finds all image files matching the specified pattern in both directories,
    compares them pairwise, and returns comprehensive results for all comparisons.

    The function only compares files that exist in both directories. If the golden directory
    has files not present in the test directory, they are listed in ``golden_only_files`` and
    contribute to ``file_list_match`` being False. Similarly, files only in the test directory
    are listed in ``test_only_files``.

    Args:
        golden_dir: Path to the directory containing reference images.
        test_dir: Path to the directory containing test images.
        path_pattern: RegEx (Regular Expression) pattern to match filenames.
            If None, all files are considered.
        allclose_rtol: Relative tolerance for np.allclose. Pass None to
            disable allclose check (requires allclose_atol=None too).
        allclose_atol: Absolute tolerance for np.allclose. Pass None to
            disable allclose check (requires allclose_rtol=None too).
        mean_tolerance: Maximum acceptable mean absolute difference (optional).
        max_tolerance: Maximum acceptable max absolute difference (optional).
        absolute_tolerance: Maximum absolute difference for any pixel (optional).
        percentile_tolerance: Tuple of (percentile, tolerance) for percentile-based comparison (optional).
        rmse_tolerance: Maximum acceptable root mean square error (optional).
        print_all_stats: If True, print detailed statistics for each comparison.
        print_per_file_results: If True, print a summary for each file comparison.

    Returns:
        A dictionary with keys: ``all_passed`` (bool indicating if all files passed and file lists
        match), ``file_results`` (mapping from filename to comparison result), ``passed_count`` (int),
        ``failed_count`` (int), ``golden_files`` (list of golden filenames), ``test_files`` (list of
        test filenames), ``file_list_match`` (bool indicating if file lists match exactly),
        ``golden_only_files`` (list of files only in golden directory), and ``test_only_files``
        (list of files only in test directory).

    Raises:
        FileNotFoundError: If either directory does not exist.

    Example:

    .. code-block:: python

        >>> import os
        >>> from isaacsim.test.utils.image_comparison import compare_images_in_directories
        >>>
        >>> result = compare_images_in_directories(
        ...     golden_dir="/path/to/golden",
        ...     test_dir="/path/to/test",
        ...     path_pattern=r"^rgb.*\.png$",
        ...     mean_tolerance=10.0,
        ... )
        ...
        >>> result["all_passed"]
        True
        >>> result["golden_only_files"]
        []
        >>> result["test_only_files"]
        []
    """
    if not os.path.exists(golden_dir):
        raise FileNotFoundError(f"Golden directory not found: {golden_dir}")
    if not os.path.exists(test_dir):
        raise FileNotFoundError(f"Test directory not found: {test_dir}")

    pattern_re = re.compile(path_pattern) if path_pattern is not None else None

    def matches_pattern(filename: str) -> bool:
        if pattern_re is None:
            return True
        return pattern_re.search(filename) is not None

    golden_files = sorted([f for f in os.listdir(golden_dir) if matches_pattern(f)])
    test_files = sorted([f for f in os.listdir(test_dir) if matches_pattern(f)])

    golden_only = sorted(set(golden_files) - set(test_files))
    test_only = sorted(set(test_files) - set(golden_files))
    file_list_match = golden_files == test_files

    if not file_list_match:
        pattern_desc = f"pattern '{path_pattern}'" if path_pattern is not None else "all files"
        print(f"WARNING: File lists do not match for {pattern_desc}:")
        if golden_only:
            print(f"  Files only in golden dir: {golden_only}")
        if test_only:
            print(f"  Files only in test dir: {test_only}")

    file_results: dict[str, dict[str, object]] = {}
    passed_count = 0
    failed_count = 0

    common_files = sorted(set(golden_files) & set(test_files))
    if not common_files:
        pattern_desc = f"pattern '{path_pattern}'" if path_pattern is not None else "all files"
        print(f"No common files found matching {pattern_desc} in both directories.")
        return {
            "all_passed": False,
            "file_results": {},
            "passed_count": 0,
            "failed_count": 0,
            "golden_files": golden_files,
            "test_files": test_files,
            "file_list_match": file_list_match,
            "golden_only_files": golden_only,
            "test_only_files": test_only,
        }

    if print_per_file_results:
        tolerance_info = []
        if mean_tolerance is not None:
            tolerance_info.append(f"mean_tolerance={mean_tolerance}")
        if max_tolerance is not None:
            tolerance_info.append(f"max_tolerance={max_tolerance}")
        if absolute_tolerance is not None:
            tolerance_info.append(f"absolute_tolerance={absolute_tolerance}")
        if percentile_tolerance is not None:
            tolerance_info.append(f"percentile_tolerance={percentile_tolerance}")
        if rmse_tolerance is not None:
            tolerance_info.append(f"rmse_tolerance={rmse_tolerance}")
        if not tolerance_info and allclose_rtol is not None and allclose_atol is not None:
            tolerance_info.append(f"rtol={allclose_rtol}, atol={allclose_atol}")

        tolerance_str = ", ".join(tolerance_info) if tolerance_info else "default allclose"
        pattern_desc = f"pattern '{path_pattern}'" if path_pattern is not None else "all files"
        print(f"Comparing images matching {pattern_desc} with {tolerance_str}")
        print(f"  Golden dir: {golden_dir}")
        print(f"  Test dir:   {test_dir}")

    for file_name in common_files:
        golden_file_path = os.path.join(golden_dir, file_name)
        test_file_path = os.path.join(test_dir, file_name)

        result = compare_images_within_tolerances(
            golden_file_path=golden_file_path,
            test_file_path=test_file_path,
            allclose_rtol=allclose_rtol,
            allclose_atol=allclose_atol,
            mean_tolerance=mean_tolerance,
            max_tolerance=max_tolerance,
            absolute_tolerance=absolute_tolerance,
            percentile_tolerance=percentile_tolerance,
            rmse_tolerance=rmse_tolerance,
            print_all_stats=print_all_stats,
        )

        file_results[file_name] = result

        if result["passed"]:
            passed_count += 1
            if print_per_file_results:
                metrics = result["metrics"]
                if mean_tolerance is not None:
                    print(f"\t'{file_name}': PASSED (mean_diff={metrics['mean_abs']:.3f})")
                else:
                    print(f"\t'{file_name}': PASSED")
        else:
            failed_count += 1
            if print_per_file_results:
                metrics = result["metrics"]
                print(f"\t'{file_name}': FAILED")
                print(f"\t  Golden: {golden_file_path}")
                print(f"\t  Test:   {test_file_path}")
                if mean_tolerance is not None:
                    print(f"\t  Expected mean_diff <= {mean_tolerance}, got {metrics['mean_abs']:.3f}")
                if max_tolerance is not None:
                    print(f"\t  Expected max_diff <= {max_tolerance}, got {metrics['max_abs']:.3f}")
                if rmse_tolerance is not None:
                    print(f"\t  Expected RMSE <= {rmse_tolerance}, got {metrics['rmse']:.3f}")

    all_passed = (failed_count == 0) and file_list_match

    return {
        "all_passed": all_passed,
        "file_results": file_results,
        "passed_count": passed_count,
        "failed_count": failed_count,
        "golden_files": golden_files,
        "test_files": test_files,
        "file_list_match": file_list_match,
        "golden_only_files": golden_only,
        "test_only_files": test_only,
    }
