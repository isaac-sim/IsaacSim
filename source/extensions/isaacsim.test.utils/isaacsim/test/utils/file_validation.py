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

"""Utilities for validating file system contents and structures in Isaac Sim tests."""

from collections import Counter
from pathlib import Path


def validate_folder_contents(
    path: str | Path,
    expected_counts: dict[str, int],
    *,
    recursive: bool = False,
    fail_on_empty_files: bool = False,
    allowed_extra_extensions: set[str] | None = None,
    min_file_size_bytes: int = 0,
    exact_match: bool = True,
) -> bool:
    """Validate folder contents against expected file extension counts.

    This function checks if a directory contains the expected number of files
    for each specified file extension. It supports recursive directory traversal,
    empty file detection, and flexible matching criteria.

    Args:
        path: Path to the directory to validate.
        expected_counts: Dictionary mapping file extensions (without dots) to expected counts.
        recursive: If True, search subdirectories recursively.
        fail_on_empty_files: If True, return False if any files are empty (0 bytes).
        allowed_extra_extensions: Set of file extensions that are allowed to exist
            beyond those in expected_counts. If None, any extra extensions are allowed.
        min_file_size_bytes: Minimum file size in bytes. Files smaller than this are considered invalid.
        exact_match: If True, the folder must contain exactly the expected counts.
            If False, the folder must contain at least the expected counts.

    Returns:
        True if validation passes, False otherwise.

    Example:

    .. code-block:: python

        >>> import tempfile
        >>> import os
        >>> from isaacsim.test.utils.file_validation import validate_folder_contents
        >>>
        >>> # Create a test directory with some files
        >>> with tempfile.TemporaryDirectory() as temp_dir:
        ...     # Create test files
        ...     with open(os.path.join(temp_dir, "image1.png"), "w") as f:
        ...         f.write("test")
        ...     with open(os.path.join(temp_dir, "image2.png"), "w") as f:
        ...         f.write("test")
        ...     with open(os.path.join(temp_dir, "data.json"), "w") as f:
        ...         f.write("{}")
        ...
        ...     # Validate exact counts
        ...     result = validate_folder_contents(temp_dir, {"png": 2, "json": 1})
        ...     print(result)
        True
        >>>
        >>> # Example with recursive search
        >>> with tempfile.TemporaryDirectory() as temp_dir:
        ...     sub_dir = os.path.join(temp_dir, "subdir")
        ...     os.makedirs(sub_dir)
        ...     with open(os.path.join(sub_dir, "nested.txt"), "w") as f:
        ...         f.write("content")
        ...
        ...     # Search recursively
        ...     result = validate_folder_contents(temp_dir, {"txt": 1}, recursive=True)
        ...     print(result)
        True
    """
    path = Path(path)

    if not path.exists() or not path.is_dir():
        return False

    # Collect all files based on recursive setting
    if recursive:
        all_files = [f for f in path.rglob("*") if f.is_file()]
    else:
        all_files = [f for f in path.iterdir() if f.is_file()]

    # Filter files that have extensions
    files_with_extensions = [f for f in all_files if f.suffix]

    # Check for empty files if required
    if fail_on_empty_files or min_file_size_bytes > 0:
        for file_path in files_with_extensions:
            try:
                file_size = file_path.stat().st_size
                if fail_on_empty_files and file_size == 0:
                    return False
                if file_size < min_file_size_bytes:
                    return False
            except (OSError, IOError):
                # If we can't read file stats, consider it invalid
                return False

    # Count files by extension (remove the leading dot)
    file_counts = Counter(f.suffix[1:].lower() for f in files_with_extensions)

    # Check expected counts
    for ext, expected_count in expected_counts.items():
        ext_lower = ext.lower()
        actual_count = file_counts.get(ext_lower, 0)

        if exact_match:
            if actual_count != expected_count:
                return False
        else:
            if actual_count < expected_count:
                return False

    # Check for unexpected extensions if allowed_extra_extensions is specified
    if allowed_extra_extensions is not None:
        expected_extensions = {ext.lower() for ext in expected_counts}
        allowed_extensions = {ext.lower() for ext in allowed_extra_extensions}
        all_allowed = expected_extensions | allowed_extensions

        for found_ext in file_counts:
            if found_ext not in all_allowed:
                return False

    return True


def get_folder_file_summary(
    path: str | Path,
    *,
    recursive: bool = False,
    include_file_sizes: bool = False,
) -> dict[str, int | dict[str, list]]:
    """Get a summary of files in a folder by extension.

    This function provides a detailed breakdown of files in a directory,
    including counts by extension and optionally file sizes and paths.

    Args:
        path: Path to the directory to analyze.
        recursive: If True, search subdirectories recursively.
        include_file_sizes: If True, include file sizes and paths in the output.

    Returns:
        Dictionary containing file summary with the following structure:
        - 'total_files': Total number of files found
        - 'extension_counts': Dictionary mapping extensions to counts
        - 'file_details': (if include_file_sizes=True) Dictionary mapping extensions
          to lists of dictionaries containing 'path' and 'size_bytes' for each file

    Example:

    .. code-block:: python

        >>> import tempfile
        >>> import os
        >>> from isaacsim.test.utils.file_validation import get_folder_file_summary
        >>>
        >>> # Create test directory
        >>> with tempfile.TemporaryDirectory() as temp_dir:
        ...     with open(os.path.join(temp_dir, "test.png"), "w") as f:
        ...         f.write("image data")
        ...     with open(os.path.join(temp_dir, "data.json"), "w") as f:
        ...         f.write("{}")
        ...
        ...     summary = get_folder_file_summary(temp_dir)
        ...     print(summary['total_files'])
        2
        >>>
        >>> # Get detailed summary with file sizes
        >>> with tempfile.TemporaryDirectory() as temp_dir:
        ...     with open(os.path.join(temp_dir, "test.txt"), "w") as f:
        ...         f.write("content")
        ...
        ...     summary = get_folder_file_summary(temp_dir, include_file_sizes=True)
        ...     print(len(summary['file_details']['txt']))
        1
    """
    path = Path(path)

    if not path.exists() or not path.is_dir():
        return {"total_files": 0, "extension_counts": {}, "file_details": {} if include_file_sizes else None}

    # Collect all files
    if recursive:
        all_files = [f for f in path.rglob("*") if f.is_file()]
    else:
        all_files = [f for f in path.iterdir() if f.is_file()]

    # Process files with extensions
    files_with_extensions = [f for f in all_files if f.suffix]

    # Count by extension
    extension_counts = Counter(f.suffix[1:].lower() for f in files_with_extensions)

    result = {
        "total_files": len(all_files),
        "extension_counts": dict(extension_counts),
    }

    if include_file_sizes:
        file_details = {}
        for file_path in files_with_extensions:
            ext = file_path.suffix[1:].lower()
            if ext not in file_details:
                file_details[ext] = []

            try:
                size = file_path.stat().st_size
            except (OSError, IOError):
                size = -1  # Indicate error reading file size

            file_details[ext].append({"path": str(file_path), "size_bytes": size})

        result["file_details"] = file_details

    return result


def validate_file_list(
    file_paths: list[str | Path],
    *,
    fail_on_missing: bool = True,
    fail_on_empty_files: bool = False,
    min_file_size_bytes: int = 0,
) -> dict[str, bool | list[str]]:
    """Validate a list of file paths against various criteria.

    Args:
        file_paths: List of file paths to validate.
        fail_on_missing: If True, validation fails if any files don't exist.
        fail_on_empty_files: If True, validation fails if any files are empty.
        min_file_size_bytes: Minimum file size in bytes for validation.

    Returns:
        Dictionary with validation results:
        - 'passed': Boolean indicating if all validations passed
        - 'missing_files': List of missing file paths
        - 'empty_files': List of empty file paths (if fail_on_empty_files=True)
        - 'undersized_files': List of files smaller than min_file_size_bytes

    Example:

    .. code-block:: python

        >>> import tempfile
        >>> import os
        >>> from isaacsim.test.utils.file_validation import validate_file_list
        >>>
        >>> # Create test files
        >>> with tempfile.TemporaryDirectory() as temp_dir:
        ...     existing_file = os.path.join(temp_dir, "exists.txt")
        ...     with open(existing_file, "w") as f:
        ...         f.write("content")
        ...
        ...     missing_file = os.path.join(temp_dir, "missing.txt")
        ...     files_to_check = [existing_file, missing_file]
        ...
        ...     result = validate_file_list(files_to_check)
        ...     print(len(result['missing_files']))
        1
    """
    file_paths = [Path(p) for p in file_paths]

    missing_files = []
    empty_files = []
    undersized_files = []

    for file_path in file_paths:
        if not file_path.exists():
            missing_files.append(str(file_path))
            continue

        if not file_path.is_file():
            missing_files.append(str(file_path))  # Treat directories as missing files
            continue

        try:
            file_size = file_path.stat().st_size

            if fail_on_empty_files and file_size == 0:
                empty_files.append(str(file_path))

            if file_size < min_file_size_bytes:
                undersized_files.append(str(file_path))

        except (OSError, IOError):
            missing_files.append(str(file_path))  # Treat unreadable files as missing

    passed = True
    if fail_on_missing and missing_files:
        passed = False
    if fail_on_empty_files and empty_files:
        passed = False
    if min_file_size_bytes > 0 and undersized_files:
        passed = False

    return {
        "passed": passed,
        "missing_files": missing_files,
        "empty_files": empty_files,
        "undersized_files": undersized_files,
    }
