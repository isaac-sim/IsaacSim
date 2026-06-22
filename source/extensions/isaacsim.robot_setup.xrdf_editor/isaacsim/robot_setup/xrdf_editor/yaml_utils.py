# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""YAML helpers used by both XRDF and Lula robot description I/O."""

from __future__ import annotations

import os
from collections.abc import Iterable
from typing import Any

import carb
import yaml


def recursive_cast_to_float(d: dict) -> None:
    """Recursively convert numeric strings to floats in a nested mapping.

    Walks the dictionary in-place, converting any ``str`` value (or string
    inside a list value) that parses as a Python float. Non-numeric strings
    are left unchanged.

    Args:
        d: Dictionary to mutate in place.
    """
    for k, v in d.items():
        if isinstance(v, str):
            try:
                d[k] = float(v)
            except ValueError:
                pass
        elif isinstance(v, dict):
            recursive_cast_to_float(v)
        elif isinstance(v, Iterable):
            new_list = []
            for item in v:
                if isinstance(item, str):
                    try:
                        item = float(item)
                    except ValueError:
                        pass
                new_list.append(item)
            d[k] = new_list


def safe_load_yaml(path: str | os.PathLike[str]) -> dict[str, Any]:
    """Load a YAML file, casting numeric strings to floats.

    Args:
        path: Path to the YAML file.

    Returns:
        Parsed YAML content as a dictionary. Returns an empty dict if the
        file cannot be parsed.
    """
    with open(path) as stream:
        try:
            parsed_file = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            carb.log_error(f"Attempted to load invalid yaml file {exc}")
            return {}

    if not isinstance(parsed_file, dict):
        # safe_load may return None (empty file) or a list/scalar (non-mapping
        # root); recursive_cast_to_float assumes a dict and would crash. The
        # condition is recoverable (callers receive {}), so warn — matches the
        # log level used by is_valid_xrdf_file for the analogous case.
        if parsed_file is not None:
            carb.log_warn(f"YAML file {path} does not contain a top-level mapping; got {type(parsed_file).__name__}")
        return {}

    recursive_cast_to_float(parsed_file)
    return parsed_file
