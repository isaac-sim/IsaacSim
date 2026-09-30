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

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable

__all__ = ["InputValueType", "Instance", "OutputValueType", "PathType", "ReadFunction", "WriteFunction"]

#: One prim path or a collection of prim paths.
PathType = str | list[str]
#: Attribute values accepted by data providers.
InputValueType = object
#: Attribute values returned by data providers.
OutputValueType = object

#: Read an attribute for one or more prim paths.
ReadFunction = Callable[[PathType, str, float | None], OutputValueType]
#: Write an attribute for one or more prim paths.
WriteFunction = Callable[[PathType, str, InputValueType, float | None], None]


class Instance(ABC):
    """Interface for reading and writing attributes on one or more prim paths.

    Args:
        paths: Prim paths managed by the instance.
    """

    @abstractmethod
    def __init__(self, paths: PathType) -> None: ...

    @property
    @abstractmethod
    def paths(self) -> list[str]:
        """Get the managed prim paths.

        Returns:
            Managed prim paths.
        """
        ...

    @abstractmethod
    def read(self, attribute_name: str, *, timestamp: float | None = None) -> OutputValueType:
        """Read an attribute value.

        Args:
            attribute_name: Attribute to read.
            timestamp: Simulation timestamp to query, or None to use the current time.

        Returns:
            Attribute value for the managed prim paths.
        """
        ...

    @abstractmethod
    def write(self, attribute_name: str, value: InputValueType, *, timestamp: float | None = None) -> None:
        """Write an attribute value.

        Args:
            attribute_name: Attribute to write.
            value: Value to assign to the managed prim paths.
            timestamp: Simulation timestamp to modify, or None to use the current time.
        """
        ...
