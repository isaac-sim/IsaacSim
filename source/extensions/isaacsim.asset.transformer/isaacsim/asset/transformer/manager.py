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

"""Run asset transformer rule profiles against USD stages."""

from __future__ import annotations

import logging
import os
import shutil
from collections.abc import Callable
from typing import Any, TypeVar

from pxr import Gf, Sdf, Usd, UsdUtils

from .models import ExecutionReport, RuleExecutionResult, RuleProfile
from .rule_interface import RuleInterface
from .utils import make_explicit_relative

_LOGGER = logging.getLogger(__name__)


def _collect_source_anchor_dirs(source_path: str) -> list[str]:
    """Return candidate anchor directories for resolving relative asset paths.

    When the source stage is composed from multiple sublayers, a relative
    asset path written in a sublayer (e.g. ``../textures/foo.png``) is
    anchored at THAT sublayer's directory, not at the root layer's. After
    flattening, the asset path string survives verbatim in the output but
    the sublayer-anchor information is lost.

    To recover the original anchor without round-tripping every path through
    USD's resolver one by one, the output remap step tries each candidate
    anchor in priority order: root layer dir first (most common case), then
    every sublayer dir present in the source stage's used-layers list.

    Args:
        source_path: Absolute path to the source root layer.

    Returns:
        Ordered list of unique anchor directories (root first, then sublayers).
        Falls back to ``[os.path.dirname(source_path)]`` if the source stage
        cannot be opened.
    """
    root_dir = os.path.dirname(source_path)
    ordered: list[str] = [root_dir]
    seen: set[str] = {root_dir}
    try:
        # Open the stage without populating: we only want the layer stack.
        source_stage = Usd.Stage.Open(source_path, load=Usd.Stage.LoadNone)
    except Exception:  # noqa: BLE001
        return ordered
    if source_stage is None:
        return ordered
    for layer in source_stage.GetUsedLayers(includeClipLayers=True):
        layer_real = getattr(layer, "realPath", "") or ""
        if not layer_real:
            continue
        d = os.path.dirname(layer_real)
        if d and d not in seen:
            ordered.append(d)
            seen.add(d)
    return ordered


def _collect_assets(layer: Sdf.Layer, package_root: str, source_layer_path: str | None = None) -> None:
    """Copy external assets to package_root and update layer paths.

    Args:
        layer: The USD layer to process. Asset paths are rewritten in place
            to point at the copied assets, relative to the layer's directory.
        package_root: Destination directory for collected assets.
        source_layer_path: Original source layer location used to anchor
            relative asset path resolution. When ``layer`` was produced by
            exporting a source layer to a different filesystem depth, the
            relative asset path strings inside it still refer to locations
            anchored at the source layer's (or its sublayers') directory.
            Falls back to ``layer.realPath`` when not supplied.

    """
    layer_path = layer.realPath
    # Anchor dependency discovery and relative-path resolution to the source
    # layer so that paths copied verbatim from the source (e.g. "../foo.png")
    # resolve to the file the source meant, not whatever happens to sit at
    # that relative location from the output directory.
    source_path = source_layer_path or layer_path
    layer_dir = os.path.dirname(layer_path)
    assets_dir = os.path.join(package_root, "source_assets")
    copied_assets: dict[str, str] = {}

    # Compute all asset dependencies from the source layer
    _, all_assets, _ = UsdUtils.ComputeAllDependencies(source_path)

    for asset_path in all_assets:
        resolved = str(asset_path.GetResolvedPath()) if hasattr(asset_path, "GetResolvedPath") else str(asset_path)
        if not resolved or not os.path.isfile(resolved):
            continue

        # Determine local destination preserving filename
        filename = os.path.basename(resolved)
        local_path = os.path.join(assets_dir, filename)

        # Handle duplicate filenames by appending suffix
        if local_path in copied_assets.values() and copied_assets.get(resolved) != local_path:
            base, ext = os.path.splitext(filename)
            counter = 1
            while local_path in copied_assets.values():
                local_path = os.path.join(assets_dir, f"{base}_{counter}{ext}")
                counter += 1

        if resolved not in copied_assets:
            os.makedirs(assets_dir, exist_ok=True)
            shutil.copy2(resolved, local_path)
            copied_assets[resolved] = local_path

    # Candidate anchor dirs in priority order (root first, then sublayer dirs).
    # A relative asset path in the output layer is resolved by trying each
    # anchor until one produces a path present in ``copied_assets``.
    candidate_anchor_dirs = _collect_source_anchor_dirs(source_path)

    def remap_path(original_path: str) -> str:
        """Remap asset paths to collected local copies.

        Args:
            original_path: Asset path from the layer metadata.

        Returns:
            Updated asset path relative to the output layer, or the original
            path if it does not correspond to a copied asset.

        """
        if not original_path:
            return original_path
        # Preserve URI-style asset paths (e.g. ``omni://``, ``http://``,
        # ``file://``). They are not filesystem paths and the source-relative
        # resolution below would produce nonsense. Note: Windows drive-letter
        # paths like ``C:\foo`` do NOT contain ``://`` and fall through to the
        # ``os.path.isabs`` branch, which correctly handles them.
        if "://" in original_path:
            return original_path
        if os.path.isabs(original_path):
            resolved = original_path
            if resolved in copied_assets:
                return make_explicit_relative(os.path.relpath(copied_assets[resolved], layer_dir))
            return original_path
        # Relative path: try each candidate anchor dir in priority order.
        # Root first (most common); sublayer dirs after, for paths authored
        # in sublayers that survived flatten with their original strings.
        for anchor_dir in candidate_anchor_dirs:
            resolved = os.path.normpath(os.path.join(anchor_dir, original_path))
            if resolved in copied_assets:
                return make_explicit_relative(os.path.relpath(copied_assets[resolved], layer_dir))
        return original_path

    UsdUtils.ModifyAssetPaths(layer, remap_path)
    layer.Save()


_QUAT_ZERO_THRESH: float = 1e-7


def _canonicalize_orient_quats(layer: Sdf.Layer) -> None:
    """Canonicalize every xformOp:orient (Quatd) in the layer for idempotent round-trip.

    Near-zero components are clamped to 0 and the sign is normalized (real >= 0).

    Args:
        layer: The USD layer whose orient quaternions should be canonicalized.
    """
    for prim_spec in layer.rootPrims.values():
        _canonicalize_orient_quats_recursive(prim_spec)


def _canonicalize_orient_quats_recursive(prim_spec: Sdf.PrimSpec) -> None:
    """Canonicalize ``xformOp:orient`` quaternions on *prim_spec* and descendants.

    Args:
        prim_spec: Root prim spec whose attribute and name-children hierarchy is
            processed recursively.
    """
    if "xformOp:orient" in prim_spec.attributes:
        attr = prim_spec.attributes["xformOp:orient"]
        if attr.typeName == Sdf.ValueTypeNames.Quatd and attr.default:
            q = attr.default
            real = q.GetReal()
            imag = q.GetImaginary()
            real = 0.0 if abs(real) < _QUAT_ZERO_THRESH else real
            i0 = 0.0 if abs(imag[0]) < _QUAT_ZERO_THRESH else imag[0]
            i1 = 0.0 if abs(imag[1]) < _QUAT_ZERO_THRESH else imag[1]
            i2 = 0.0 if abs(imag[2]) < _QUAT_ZERO_THRESH else imag[2]
            negate = False
            if real < 0:
                negate = True
            elif real == 0:
                for c in (i0, i1, i2):
                    if c != 0:
                        negate = c < 0
                        break
            if negate:
                real, i0, i1, i2 = -real, -i0, -i1, -i2
            attr.default = Gf.Quatd(real, i0, i1, i2)
    for child in prim_spec.nameChildren.values():
        _canonicalize_orient_quats_recursive(child)


T = TypeVar("T")


def Singleton(class_: type[T]) -> Callable[..., T]:  # noqa: N802
    """Create a singleton factory for a class.

    Args:
        class_: Class to wrap with a singleton factory.

    Returns:
        Callable that returns the singleton instance.

    Example:

    .. code-block:: python

        @Singleton
        class Registry:
            pass

        registry = Registry()

    """
    instances: dict[type[Any], object] = {}

    def getinstance(*args: object, **kwargs: object) -> T:
        if class_ not in instances:
            instances[class_] = class_(*args, **kwargs)
        return instances[class_]

    return getinstance


@Singleton
class RuleRegistry:
    """In-memory registry mapping rule names to implementation classes."""

    def __init__(self) -> None:
        self._type_to_cls: dict[str, type[RuleInterface]] = {}

    def register(self, rule_cls: type[RuleInterface]) -> None:
        """Register a rule implementation class using its fully qualified name.

        Args:
            rule_cls: Concrete subclass of :class:`RuleInterface`. The registry
                key is computed as ``{rule_cls.__module__}.{rule_cls.__qualname__}``.

        Raises:
            TypeError: If rule_cls does not inherit from RuleInterface.

        Example:

        .. code-block:: python

            registry.register(MyRule)

        """
        if not issubclass(rule_cls, RuleInterface):
            raise TypeError("rule_cls must subclass RuleInterface")
        fqcn = f"{rule_cls.__module__}.{rule_cls.__qualname__}"
        self._type_to_cls[fqcn] = rule_cls

    def get(self, rule_type: str) -> type[RuleInterface] | None:
        """Resolve a rule implementation class by fully qualified class name.

        Args:
            rule_type: Fully qualified class name stored in :class:`RuleSpec.type`.

        Returns:
            The registered class, or ``None`` if not found.

        Example:

        .. code-block:: python

            rule_cls = registry.get("my.module.MyRule")

        """
        return self._type_to_cls.get(rule_type)

    def clear(self) -> None:
        """Clear all registered rule mappings.

        Example:

        .. code-block:: python

            registry.clear()

        """
        self._type_to_cls.clear()

    def list_rules(self) -> dict[str, type[RuleInterface]]:
        """Return a copy of the registered rule mapping.

        Returns:
            Mapping of fully qualified rule type names to classes.

        Example:

        .. code-block:: python

            rules = registry.list_rules()

        """
        return dict(self._type_to_cls)

    def list_rule_types(self) -> list[str]:
        """Return sorted registered rule type names.

        Returns:
            List of fully qualified rule type names.

        Example:

        .. code-block:: python

            types = registry.list_rule_types()

        """
        return sorted(self._type_to_cls.keys())


class AssetTransformerManager:
    """Coordinates execution of a :class:`RuleProfile` over USD stages.

    The manager creates a flattened and collected copy of the input stage at
    ``{package_root}/base.usda``. External assets are copied to
    ``{package_root}/assets/`` with paths updated to local references. All
    rules execute against this self-contained working copy.

    Args:
        registry: Optional registry instance. Currently ignored in favor of the
            global singleton registry.

    """

    def __init__(self, registry: RuleRegistry | None = None) -> None:
        self._registry = RuleRegistry()

    @property
    def registry(self) -> RuleRegistry:
        """Return the singleton rule registry.

        Returns:
            The manager's internal :class:`RuleRegistry` instance.

        Example:

        .. code-block:: python

            registry = manager.registry

        """
        return self._registry

    def run(
        self,
        input_stage: str | Usd.Stage,
        profile: RuleProfile,
        package_root: str | None = None,
    ) -> ExecutionReport:
        """Execute a rule profile against a USD stage and return an execution report.

        Args:
            input_stage: Path to the source USD stage or layer, or an
                already-opened :class:`pxr.Usd.Stage`. When a stage object is
                passed it is used directly and its root layer path is used as
                the internal ``input_stage``.
            profile: Rule profile specifying ordered rules to run.
            package_root: Destination root directory for outputs.

        Returns:
            Execution report including per-rule logs and status.

        Raises:
            KeyError: If a rule type is not registered.
            RuntimeError: If stage loading or export fails.

        Example:

        .. code-block:: python

            manager = AssetTransformerManager()
            report = manager.run("input.usd", profile, package_root="/tmp/package")
        """
        # Accept either a file path string or an already-opened Usd.Stage.
        if isinstance(input_stage, str):
            input_stage_path: str = input_stage
            stage_obj: Usd.Stage | None = Usd.Stage.Open(input_stage_path)
        else:
            stage_obj = input_stage
            input_stage_path = input_stage.GetRootLayer().realPath

        package_root_final = package_root or profile.output_package_root or ""
        report = ExecutionReport(
            profile=profile,
            input_stage_path=input_stage_path,
            package_root=package_root_final,
        )

        source_stage = stage_obj
        if source_stage is None:
            report.close()
            raise RuntimeError(f"Failed to open source stage: {input_stage_path}")
        base_name = profile.base_name or "base.usd"
        # Create flattened copy at destination as base.usda. Use forward slashes so the layer
        # identifier and any downstream relative-path computations are platform-independent.
        base_usda_path = os.path.join(package_root_final, "payloads", base_name).replace(os.sep, "/")
        os.makedirs(package_root_final, exist_ok=True)
        if profile.flatten_source:
            flattened_layer = source_stage.Flatten()
            if not flattened_layer.Export(base_usda_path):
                report.close()
                raise RuntimeError(f"Failed to export flattened stage to: {base_usda_path}")
        else:
            source_stage.GetRootLayer().Export(base_usda_path)

        # Collect external assets and update paths in base.usda
        base_layer = Sdf.Layer.FindOrOpen(base_usda_path)
        if base_layer:
            _collect_assets(base_layer, package_root_final, source_layer_path=input_stage_path)
            _canonicalize_orient_quats(base_layer)
            base_layer.Save()

        working_stage = Usd.Stage.Open(base_usda_path)
        if working_stage is None:
            report.close()
            raise RuntimeError(f"Failed to open flattened stage: {base_usda_path}")

        for spec in profile.rules:
            if not spec.enabled:
                _LOGGER.info("Skipping disabled rule: %s", spec.name)
                continue

            result = RuleExecutionResult(rule=spec, success=False)
            report.results.append(result)

            try:
                impl_cls = self._registry.get(spec.type)
                if impl_cls is None:
                    raise KeyError(f"No rule implementation registered for type '{spec.type}'")

                destination_path = spec.destination or ""
                rule: RuleInterface = impl_cls(
                    working_stage,
                    package_root_final,
                    destination_path,
                    {
                        "params": spec.params,
                        "interface_asset_name": profile.interface_asset_name,
                        "input_stage_path": input_stage_path,
                        "input_stage": stage_obj,
                    },
                )

                returned_stage_path = rule.process_rule()

                # Update working stage if the rule returned a different stage path
                if returned_stage_path is not None:
                    current_path = working_stage.GetRootLayer().realPath
                    if returned_stage_path != current_path:
                        # Release the old stage before opening the new one
                        # The rule is responsible for saving its changes before returning a new path
                        del working_stage

                        # Open the new stage
                        new_stage = Usd.Stage.Open(returned_stage_path)
                        if new_stage is not None:
                            working_stage = new_stage
                            _LOGGER.info("Switched working stage to: %s", returned_stage_path)
                        else:
                            _LOGGER.warning("Failed to open returned stage: %s", returned_stage_path)
                            # Re-open the original if we can't open the new one
                            working_stage = Usd.Stage.Open(current_path)

                # Collect logs and affected stages from the rule.
                for entry in rule.get_operation_log():
                    result.log.append({"message": entry})
                result.affected_stages = rule.get_affected_stages()
                result.success = True

            except Exception as exc:  # noqa: BLE001
                _LOGGER.exception("Rule '%s' failed", spec.name)
                result.error = str(exc)
                result.success = False
            finally:
                result.close()

        # Save the working stage's root layer if it has unsaved changes
        root_layer = working_stage.GetRootLayer()
        if root_layer and root_layer.dirty:
            root_layer.Save()

        # Record the final working stage path so callers know the output asset
        report.output_stage_path = root_layer.realPath if root_layer else None

        report.close()
        return report
