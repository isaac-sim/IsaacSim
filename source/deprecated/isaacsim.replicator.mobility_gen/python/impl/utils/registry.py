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

"""A generic registry system for managing and organizing registered classes by name or index."""

from collections import OrderedDict
from typing import Generic, TypeVar

T = TypeVar("T")


class Registry(Generic[T]):
    """A generic registry for managing and organizing registered classes.

    This class provides a centralized system for registering, storing, and retrieving classes by name or index.
    It maintains an ordered collection of registered items and offers a decorator-based registration mechanism
    for easy class registration.

    The registry supports type-safe retrieval operations and maintains insertion order through an OrderedDict.
    Classes can be registered using the register decorator, then accessed by name or numeric index.
    """

    def __init__(self) -> None:
        self.items = OrderedDict()

    def register(self) -> object:
        """Decorator for registering classes in the registry.

        Returns:
            A decorator function that registers a class by its name.
        """

        def _register(cls: type) -> type:
            self.items[cls.__name__] = cls
            return cls

        return _register

    def names(self) -> object:
        """Names of all registered items in the registry.

        Returns:
            The keys representing registered item names.
        """
        return self.items.keys()

    def get(self, name: str) -> T:
        """Retrieves a registered item by name.

        Args:
            name: The name of the item to retrieve.

        Returns:
            The registered item.
        """
        return self.items[name]

    def get_index(self, index: int) -> T:
        """Retrieves a registered item by index.

        Args:
            index: The index of the item to retrieve.

        Returns:
            The registered item at the specified index.
        """
        return list(self.items.values())[index]
