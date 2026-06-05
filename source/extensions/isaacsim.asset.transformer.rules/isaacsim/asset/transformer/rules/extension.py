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

"""Register transformer rules with the global registry."""

from __future__ import annotations

import asyncio
import importlib
import inspect
import logging
import pkgutil

try:
    import omni.ext

    _ExtBase = omni.ext.IExt
except ImportError:

    class _ExtBase:
        def on_startup(self, ext_id: str) -> None: ...
        def on_shutdown(self) -> None: ...


from isaacsim.asset.transformer import RuleInterface, RuleRegistry

from .utils import refresh_builtin_mdl_cache_async

logger = logging.getLogger(__name__)


_EXCLUDED_SUBPACKAGES = frozenset({"tests"})
"""Top-level subpackages of :mod:`isaacsim.asset.transformer.rules` that are
skipped when discovering rule classes (e.g. test fixtures)."""


def discover_rule_classes() -> list[type[RuleInterface]]:
    """Return every concrete :class:`RuleInterface` subclass in the extension package.

    Discovery is dynamic: any rule module added under
    :mod:`isaacsim.asset.transformer.rules` (in any current or future
    subpackage that is not in :data:`_EXCLUDED_SUBPACKAGES`) is picked up
    automatically without needing to edit this file.

    Returns:
        List of rule classes, sorted by fully qualified class name for
        deterministic registration order.

    """
    import isaacsim.asset.transformer.rules as _rules_pkg

    package_root = _rules_pkg.__name__
    discovered: dict[str, type[RuleInterface]] = {}

    for module_info in pkgutil.walk_packages(_rules_pkg.__path__, prefix=f"{package_root}."):
        rel = module_info.name[len(package_root) + 1 :]
        top = rel.split(".", 1)[0]
        if top in _EXCLUDED_SUBPACKAGES:
            continue

        try:
            module = importlib.import_module(module_info.name)
        except Exception as exc:
            logger.warning(
                "[isaacsim.asset.transformer.rules] Skipping %s during rule discovery: %s",
                module_info.name,
                exc,
            )
            continue

        for _, cls in inspect.getmembers(module, inspect.isclass):
            if cls is RuleInterface:
                continue
            if not issubclass(cls, RuleInterface):
                continue
            if inspect.isabstract(cls):
                continue
            # Skip classes re-exported from other modules.
            if cls.__module__ != module.__name__:
                continue
            fqcn = f"{cls.__module__}.{cls.__qualname__}"
            discovered[fqcn] = cls

    return [discovered[name] for name in sorted(discovered)]


def register_all_rules() -> None:
    """Register all built-in rule implementations with the global :class:`RuleRegistry`.

    This is called automatically by the Kit extension on startup. Standalone
    callers (outside Kit) should invoke this once before running the asset
    transformer pipeline.

    Rules are discovered dynamically via :func:`discover_rule_classes`, so
    new rule modules added to the extension are registered automatically.
    """
    registry = RuleRegistry()
    rules = discover_rule_classes()
    for rule_cls in rules:
        registry.register(rule_cls)
    logger.info("[isaacsim.asset.transformer.rules] Registered %d rule(s)", len(rules))


class Extension(_ExtBase):
    """Extension that registers transformation rules."""

    def on_startup(self, ext_id: str) -> None:
        """Register rule implementations and kick off built-in MDL discovery.

        Args:
            ext_id: Fully qualified extension identifier.

        """
        self._ext_id = ext_id
        logger.info(f"[isaacsim.asset.transformer.rules] Startup: {ext_id}")
        register_all_rules()
        # Best-effort upgrade of the built-in MDL cache from
        # omni.kit.material.library. The cache is already initialized to a
        # hardcoded fallback at module import; refresh swaps in the live Kit
        # data when the optional dependency is loaded. Failures are logged
        # by refresh itself, so we do not need to wrap it.
        self._mdl_cache_task = asyncio.ensure_future(refresh_builtin_mdl_cache_async())

    def on_shutdown(self) -> None:
        """Log shutdown for the rules extension."""
        task = getattr(self, "_mdl_cache_task", None)
        if task is not None and not task.done():
            task.cancel()
        logger.info(f"[isaacsim.asset.transformer.rules] Shutdown: {getattr(self, '_ext_id', '')}")
