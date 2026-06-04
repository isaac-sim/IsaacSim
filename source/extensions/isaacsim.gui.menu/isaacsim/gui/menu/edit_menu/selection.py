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

"""Selection data model for recent and saved selections."""

__all__ = ["Selection"]

import time


class Selection:
    """Encapsulate selection data with a timestamp.

    Args:
        description: Human-readable label for the selection.
        selection: Selection data or paths to store.
    """

    def __init__(self, description: str, selection: list[str]) -> None:
        self.time = time.monotonic()
        self.description = description
        self.selection = selection

    def touch(self) -> None:
        """Update the timestamp for the selection.

        Example:
            .. code-block:: python

                selection = Selection("My Set", ["/World/Cube"])
                selection.touch()
        """
        self.time = time.monotonic()
