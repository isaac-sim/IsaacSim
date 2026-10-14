# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Example catalog and readiness records shared by installer frontends."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Readiness:
    """Backend-owned prerequisite status and user-facing recovery instructions.

    ``ready`` describes the shared environment, not an individual example build.
    ``can_prepare`` indicates whether the backend can prepare missing prerequisites.
    ``runnable_examples`` is an authoritative snapshot when supplied, including
    an empty set when no examples are runnable. ``None`` means the backend does
    not report disk build state, so the UI retains its session build results
    while the shared environment remains ready.
    """

    ready: bool
    message: str = ""
    instructions: str = ""
    details: str = ""
    can_prepare: bool = False
    runnable_examples: frozenset[str] | None = None


@dataclass(frozen=True)
class ModuleRequirement:
    """An SDK requirement displayed in example details."""

    name: str
    surfaces: tuple[str, ...]


@dataclass(frozen=True)
class Example:
    """A runnable example, identified by an opaque backend-owned ID."""

    id: str
    title: str
    summary: str
    path: str
    build_adapter: str
    run_adapter: str
    owners: tuple[str, ...] = ()
    categories: tuple[str, ...] = ()
    topics: tuple[str, ...] = ()
    system_requirements: tuple[str, ...] = ()
    module_requirements: tuple[ModuleRequirement, ...] = ()
    tests: tuple[str, ...] = ()


@dataclass(frozen=True)
class SeriesStep:
    """An ordered example reference and learning level."""

    example: str
    level: str


@dataclass(frozen=True)
class Series:
    """An ordered learning path."""

    id: str
    title: str
    summary: str
    steps: tuple[SeriesStep, ...]


@dataclass(frozen=True)
class Catalog:
    """An immutable snapshot returned by a backend."""

    examples: tuple[Example, ...]
    series: tuple[Series, ...] = ()
