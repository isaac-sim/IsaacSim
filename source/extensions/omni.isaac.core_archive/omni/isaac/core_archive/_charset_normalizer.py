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

"""Keep charset-normalizer submodules on Python's standard package import path."""

from __future__ import annotations

import sys
from collections.abc import Sequence
from importlib.abc import MetaPathFinder
from importlib.machinery import ModuleSpec, PathFinder
from types import ModuleType


class _CharsetNormalizerFinder(MetaPathFinder):
    """Resolve charset-normalizer submodules consistently within their selected package."""

    def find_spec(
        self, fullname: str, path: Sequence[str] | None = None, target: ModuleType | None = None
    ) -> ModuleSpec | None:
        """Use standard native-module precedence without changing top-level package selection.

        Args:
            fullname: Fully qualified module name.
            path: Search locations supplied by the parent package.
            target: Existing module when reloading.

        Returns:
            The standard import specification for a charset-normalizer submodule, or None for other modules.
        """
        if path is not None and fullname.startswith("charset_normalizer."):
            return PathFinder.find_spec(fullname, path, target)
        return None


_FINDER = _CharsetNormalizerFinder()


def install() -> None:
    """Keep Kit's fast importer from mixing native and Python charset-normalizer implementations."""
    # Kit's directory scan can cache `cd.so` with `md.py`, whose class layouts are incompatible.
    # Leave package selection and all other imports with Kit; resolve only this package's submodules normally.
    if _FINDER not in sys.meta_path:
        sys.meta_path.insert(0, _FINDER)
