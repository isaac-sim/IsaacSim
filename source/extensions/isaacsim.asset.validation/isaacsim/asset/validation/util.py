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

"""Utility functions for asset validation."""

import omni.asset_validator.core as _av_core


def _resolve_at(at: object) -> str:
    """Resolve a USD object reference to a stable string key for dedup purposes.

    Uses duck-typing so this module never imports pxr, keeping the companion
    unit test suite pure stdlib.
    """
    if at is None:
        return ""
    if hasattr(at, "GetRootLayer"):  # Usd.Stage
        return at.GetRootLayer().identifier
    if hasattr(at, "GetPath"):  # Usd.Prim, Usd.Attribute, Usd.Relationship
        return str(at.GetPath())
    return str(at)


class DedupMixin:
    """Mixin that suppresses duplicate checker messages within a single rule instance.

    Prefer ``DedupBaseRuleChecker`` as the direct base class for new rules. This
    mixin is exposed for callers that need custom MRO.

    MRO guarantees DedupMixin._AddError/Warning/Info run before the base class
    methods, so the first occurrence is forwarded and subsequent identical calls
    are dropped.  Each rule instance gets its own ``_seen`` set, so dedup state
    never leaks across consecutive validation runs in batch mode.

    Dedup key: ``(rule_class_name, resolved_at, severity, message)``
    """

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        self._seen: set[tuple[str, str, str, str]] = set()

    def _AddError(self, message: str, **kwargs: object) -> None:  # noqa: N802
        key = (type(self).__name__, _resolve_at(kwargs.get("at")), "error", message)
        if key not in self._seen:
            self._seen.add(key)
            super()._AddError(message, **kwargs)

    def _AddWarning(self, message: str, **kwargs: object) -> None:  # noqa: N802
        key = (type(self).__name__, _resolve_at(kwargs.get("at")), "warning", message)
        if key not in self._seen:
            self._seen.add(key)
            super()._AddWarning(message, **kwargs)

    def _AddInfo(self, message: str, **kwargs: object) -> None:  # noqa: N802
        key = (type(self).__name__, _resolve_at(kwargs.get("at")), "info", message)
        if key not in self._seen:
            self._seen.add(key)
            super()._AddInfo(message, **kwargs)


class DedupBaseRuleChecker(DedupMixin, _av_core.BaseRuleChecker):
    """BaseRuleChecker with automatic per-instance event deduplication.

    Use this as the direct base class for any rule that may fan out across
    variantSet combinations (CheckPrim-based rules) so identical events on the
    same prim from different variant passes collapse to a single emission.

    Equivalent to ``class Foo(DedupMixin, BaseRuleChecker):`` but makes the
    intent explicit and avoids repeating the multi-inheritance boilerplate in
    every rule module.
    """


def is_relationship_prepended(relationship: object) -> bool:
    """Check if a relationship is prepended in the layer stack.

    Examines the property stack of the relationship to determine if it uses
    prepended items rather than explicit items in the target path list.

    Args:
        relationship: The USD relationship to check.

    Returns:
        True if the relationship is prepended, False if it uses explicit items.
    """
    rel_stack = relationship.GetPropertyStack()
    return all(not spec.targetPathList.isExplicit for spec in rel_stack)


def make_relationship_prepended(relationship: object) -> bool:
    """Convert a relationship to use prepended items in the layer stack.

    Modifies the relationship's property specs to use prepended items instead of
    explicit items, which allows for composition with stronger layers.

    Args:
        relationship: The USD relationship to convert.

    Returns:
        True if the operation was successful.
    """
    rel_stack = relationship.GetPropertyStack()
    for spec in rel_stack:
        if spec.targetPathList.isExplicit:
            items = list(spec.targetPathList.explicitItems)
            spec.targetPathList.prependedItems = items
            spec.targetPathList.explicitItems = []
    return True
