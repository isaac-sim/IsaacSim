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

"""Registry utility for managing named class collections."""

from __future__ import annotations

from collections import OrderedDict
from collections.abc import Callable, Iterable
from typing import Generic, TypeVar

T = TypeVar("T")


class Registry(Generic[T]):
    """A generic registry mapping class names to class objects."""

    def __init__(self) -> None:
        self.items: OrderedDict[str, type[T]] = OrderedDict()

    def register(self) -> Callable[[type[T]], type[T]]:
        """Return a decorator that registers a class in the registry.

        Returns:
            A decorator that registers the decorated class and returns it unchanged.
        """

        def _register(cls: type[T]) -> type[T]:
            self.items[cls.__name__] = cls
            return cls

        return _register

    def names(self) -> Iterable[str]:
        """Return the names of all registered classes.

        Returns:
            An iterable of registered class name strings.
        """
        return self.items.keys()

    def get(self, name: str) -> type[T]:
        """Return the class registered under the given name.

        Args:
            name: The registered class name.

        Returns:
            The class registered under the given name.
        """
        return self.items[name]

    def get_index(self, index: int) -> type[T]:
        """Return the class at the given registration index.

        Args:
            index: The zero-based index of the registered class.

        Returns:
            The class at the given index.
        """
        return list(self.items.values())[index]
