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

import filecmp
import gc
import logging
import os
import shutil
import tempfile
import time
from collections.abc import Callable
from typing import Any, TypeVar

from pxr import Gf, Sdf, Usd, UsdUtils

from .models import ExecutionReport, RuleExecutionResult, RuleProfile
from .rule_interface import RuleInterface
from .utils import make_explicit_relative

_LOGGER = logging.getLogger(__name__)


def _relocate_path(path: str, source_root: str, destination_root: str) -> str:
    """Map a local path inside the staging tree to its published location.

    Args:
        path: Path or identifier reported by a rule.
        source_root: Absolute staging directory.
        destination_root: Absolute publication directory.

    Returns:
        Published path for staged files, otherwise the original identifier.
    """
    absolute = os.path.abspath(path)
    try:
        if os.path.commonpath((absolute, source_root)) == source_root:
            return os.path.join(destination_root, os.path.relpath(absolute, source_root))
    except ValueError:
        pass  # Different drives cannot identify a staged file.
    return path


def _publish_staged_package(staging_root: str, output_root: str, output_stage_path: str | None) -> bool:
    """Prepare relocated files, then publish with rollback on failure.

    Args:
        staging_root: Directory containing this conversion's outputs.
        output_root: Destination package directory.
        output_stage_path: Final working stage in the staging directory.

    Returns:
        Whether the final composition diagnostic was incomplete.

    Raises:
        RuntimeError: If a generated layer cannot be prepared or refreshed.
        OSError: If publication or rollback fails.
    """
    transaction_root = tempfile.mkdtemp(prefix=".asset-publication-", dir=output_root)
    preserve_backup = False
    try:
        pending = []
        cached_layers = []
        for directory, _, filenames in os.walk(staging_root):
            for filename in filenames:
                source = os.path.join(directory, filename)
                relative = os.path.relpath(source, staging_root)
                destination = os.path.join(output_root, relative)
                prepared = os.path.join(transaction_root, "prepared", relative)
                backup = os.path.join(transaction_root, "backup", relative)
                os.makedirs(os.path.dirname(prepared), exist_ok=True)
                if os.path.splitext(filename)[1].lower() in (".usd", ".usda", ".usdc"):
                    source_layer = Sdf.Layer.FindOrOpen(source)
                    if source_layer is None:
                        raise RuntimeError(f"Cannot publish generated USD layer: {source}")
                    layer = Sdf.Layer.CreateAnonymous()
                    layer.TransferContent(source_layer)
                    source_layer = None

                    def relocate_asset(asset_path: str) -> str:
                        """Re-anchor a path from the generated layer to its final location.

                        Args:
                            asset_path: Authored asset path or layer identifier.

                        Returns:
                            Identifier resolving to the same asset after publication.
                        """
                        if not asset_path or "://" in asset_path or asset_path.startswith("anon:"):
                            return asset_path
                        path, arguments = Sdf.Layer.SplitIdentifier(asset_path)
                        absolute = os.path.normpath(os.path.join(directory, path))
                        relocated = _relocate_path(absolute, staging_root, output_root)
                        if not os.path.isabs(path):
                            relocated = make_explicit_relative(os.path.relpath(relocated, os.path.dirname(destination)))
                        return Sdf.Layer.CreateIdentifier(relocated.replace(os.sep, "/"), arguments)

                    UsdUtils.ModifyAssetPaths(layer, relocate_asset)
                    if not layer.Export(prepared):
                        raise RuntimeError(f"Failed to prepare generated USD layer: {destination}")
                    layer = None
                    existing = Sdf.Layer.Find(destination)
                    if existing is not None:
                        snapshot = Sdf.Layer.CreateAnonymous()
                        snapshot.TransferContent(existing)
                        cached_layers.append((existing, snapshot))
                else:
                    shutil.copy2(source, prepared)
                if os.path.lexists(destination):
                    os.makedirs(os.path.dirname(backup), exist_ok=True)
                    shutil.copy2(destination, backup, follow_symlinks=False)
                else:
                    backup = None
                pending.append((prepared, destination, backup))

        published = []
        diagnostics_truncated = False
        try:
            for prepared, destination, backup in pending:
                os.makedirs(os.path.dirname(destination), exist_ok=True)
                try:
                    os.replace(prepared, destination)
                except PermissionError as error:
                    raise PermissionError(
                        error.errno,
                        "Cannot publish the generated file. Close stages or applications holding the destination "
                        "file, check write permissions, and retry, or choose a different package_root.",
                        destination,
                    ) from error
                published.append((destination, backup))
            for existing, _ in cached_layers:
                if not existing.Reload(force=True):
                    raise RuntimeError(f"Failed to refresh published USD layer: {existing.identifier}")
            if output_stage_path:
                final_path = _relocate_path(output_stage_path, staging_root, output_root)
                published_stage = Usd.Stage.Open(final_path)
                try:
                    errors, diagnostics_truncated = _collect_composition_errors(published_stage)
                except Exception:  # noqa: BLE001
                    _LOGGER.exception("Composition error collection failed for %s", final_path)
                    errors = []
                    diagnostics_truncated = True
                finally:
                    published_stage = None
                if errors:
                    raise RuntimeError(
                        f"Replacement would produce invalid USD composition at {final_path}: {errors[0]}. "
                        "When exporting into the source package, flatten its composition first or use a new destination."
                    )
        except Exception:  # noqa: BLE001
            # Restore filesystem contents before cached layers so composition
            # resolves against the previous complete package during rollback.
            preserve_backup = True
            rollback_errors = []
            for destination, backup in reversed(published):
                try:
                    if backup is None:
                        os.remove(destination)
                    else:
                        os.replace(backup, destination)
                except OSError as error:
                    rollback_errors.append(str(error))
            for existing, snapshot in cached_layers:
                try:
                    existing.TransferContent(snapshot)
                except Exception as error:  # noqa: BLE001
                    rollback_errors.append(f"{existing.identifier}: {error}")
            if rollback_errors:
                raise RuntimeError(
                    f"Package publication and rollback failed. Recovery files are preserved at {transaction_root}: "
                    + "; ".join(rollback_errors)
                ) from None
            preserve_backup = False
            raise
        return diagnostics_truncated
    finally:
        if not preserve_backup:
            shutil.rmtree(transaction_root)


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


def _collect_assets(
    layer: Sdf.Layer,
    package_root: str,
    source_layer_path: str | None = None,
    reanchor_uncollected: bool = False,
) -> None:
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
        reanchor_uncollected: Rewrite relative paths that were *not* collected
            into the package so they still resolve, by pointing them back at
            the source file they named. Required whenever ``layer`` retains
            composition arcs: only non-layer assets are collected, so a
            sublayer, reference or payload arc would otherwise keep a path
            anchored at the source directory while sitting in the output
            directory, silently dropping everything composed through it. A
            flattened layer has no arcs left, so leaving this ``False`` keeps
            that path byte-identical to before.

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
    # `filecmp` caches by (path, size, mtime), which an in-place edit can defeat.
    filecmp.clear_cache()

    # Compute all asset dependencies from the source layer
    _, all_assets, _ = UsdUtils.ComputeAllDependencies(source_path)

    def has_different_content(candidate: str, source: str) -> bool:
        """Check whether `candidate` exists and holds content other than `source`.

        Args:
            candidate: Existing destination path to compare.
            source: Source asset path.

        Returns:
            Whether the destination exists and its bytes differ from the source.
        """
        return os.path.exists(candidate) and not filecmp.cmp(source, candidate, shallow=False)

    for asset_path in all_assets:
        resolved = str(asset_path.GetResolvedPath()) if hasattr(asset_path, "GetResolvedPath") else str(asset_path)
        if not resolved or not os.path.isfile(resolved):
            continue

        # Determine local destination preserving filename
        filename = os.path.basename(resolved)
        local_path = os.path.join(assets_dir, filename)

        # Avoid overwriting an existing same-named asset with different content.
        collided = copied_assets.get(resolved) != local_path and has_different_content(local_path, resolved)
        if collided:
            base, ext = os.path.splitext(filename)
            counter = 1
            while has_different_content(local_path, resolved):
                local_path = os.path.join(assets_dir, f"{base}_{counter}{ext}")
                counter += 1

        if resolved not in copied_assets:
            os.makedirs(assets_dir, exist_ok=True)
            # Anything still here is byte-identical.
            reused = os.path.exists(local_path)
            if not reused:
                shutil.copy2(resolved, local_path)
            copied_assets[resolved] = local_path
            if collided:
                if reused:
                    _LOGGER.warning(
                        "%s already holds different content; reusing existing compatible copy %s for %s",
                        os.path.join(assets_dir, filename),
                        local_path,
                        resolved,
                    )
                else:
                    _LOGGER.warning(
                        "%s already holds different content; collecting %s as %s",
                        os.path.join(assets_dir, filename),
                        resolved,
                        os.path.basename(local_path),
                    )
            elif reused:
                _LOGGER.warning("%s already holds identical content; reusing it for %s", local_path, resolved)

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
        if reanchor_uncollected:
            # ``UsdUtils.ModifyAssetPaths`` also visits composition arcs, and
            # those targets are never in ``copied_assets``. Re-anchor to the
            # file the source named so the arc keeps resolving from the output
            # directory; returning the path unchanged would leave it dangling.
            #
            # Only the root layer's own directory is a valid anchor here. This
            # branch runs exclusively when ``flatten_source`` is False, where the
            # export copies the root layer verbatim, so every relative string it
            # carries was authored against that single directory. The sublayer
            # anchors exist for the flatten case above; reusing them here would
            # let a genuinely broken arc resolve to a same-named file under an
            # unrelated sublayer directory, silently substituting wrong content
            # for an error the report would otherwise surface.
            resolved = os.path.normpath(os.path.join(candidate_anchor_dirs[0], original_path))
            if os.path.isfile(resolved):
                try:
                    return make_explicit_relative(os.path.relpath(resolved, layer_dir))
                except ValueError:
                    # Windows refuses to relate paths across drive letters or a
                    # UNC share, which is an ordinary layout when the source
                    # tree and the output package sit on different volumes. An
                    # absolute target still resolves; aborting the transform
                    # over a path-formatting limit does not.
                    return resolved.replace(os.sep, "/")
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


def _run_deferred_deletions(paths: list[str], keep: str | None = None) -> None:
    """Best-effort delete of files a rule deferred to the manager.

    A rule cannot delete a file that backs the working stage: the manager (not
    the rule) owns the working :class:`pxr.Usd.Stage` and its root
    :class:`pxr.Sdf.Layer`, and on Windows an open USD layer is an OS-level file
    lock, so ``os.remove`` fails with ``PermissionError: [WinError 5]`` until
    that handle is dropped. The manager calls this only after it has released the
    relevant working stage, so the file pointer is normally already gone.

    This never calls ``Sdf.Layer.Clear()`` and never raises: clearing a layer
    that is still part of a live composition would corrupt the working stage,
    and a failed cleanup of an intermediate file must not fail the rule or
    cascade to later rules. A file that cannot be removed (still referenced or
    locked) is left in place and logged at warning level.

    Args:
        paths: Filesystem paths the rule requested via
            :meth:`RuleInterface.request_deletion`.
        keep: Path that must never be deleted -- the file backing the live
            working stage. Compared with :func:`os.path.normcase` /
            :func:`os.path.abspath` so it is robust on case-insensitive
            filesystems.
    """
    keep_norm = os.path.normcase(os.path.abspath(keep)) if keep else None
    for path in paths:
        if not path:
            continue
        if keep_norm is not None and os.path.normcase(os.path.abspath(path)) == keep_norm:
            _LOGGER.warning("Refusing to delete file backing the active working stage: %s", path)
            continue

        last_error: OSError | None = None
        for _ in range(10):
            # The caller already dropped its stage handle. Force a collection
            # so CPython releases any lingering layer wrapper (and the crate
            # memory-map / OS handle on Windows) before each unlink attempt.
            gc.collect()
            try:
                os.remove(path)
                last_error = None
                break
            except FileNotFoundError:
                last_error = None
                break
            except OSError as exc:
                last_error = exc
                time.sleep(0.1)
        if last_error is not None:
            # Leave the intermediate file in place rather than failing the rule.
            _LOGGER.warning("Could not delete intermediate file %s: %s", path, last_error)


#: Upper bound on variant selections probed by :func:`_collect_composition_errors`.
#: Each probe recomposes the stage, so an asset with many variant sets would
#: otherwise make reporting cost more than the transform itself.
_MAX_VARIANT_SELECTIONS_PROBED = 128


def _collect_composition_errors(stage: Usd.Stage) -> tuple[list[str], bool]:
    """Return composition errors for a stage, including unselected variants.

    :meth:`Usd.Stage.GetCompositionErrors` only reports arcs that the *current*
    variant selections actually compose. An arc broken inside a variant option
    nobody selected stays invisible, so a package can lose every mesh behind
    that option while the report still reads clean.

    To surface those, each variant set is re-selected one option at a time and
    the stage re-queried. Sets are probed independently rather than as a
    cartesian product, so the cost stays linear in the number of options rather
    than exponential in the number of sets. Selections are authored into the
    stage's session layer and rolled back, so the exported package on disk is
    never touched.

    The sweep is bounded by ``_MAX_VARIANT_SELECTIONS_PROBED`` selections. It
    reports whether that bound was reached so callers never mistake an
    abandoned sweep for a clean one.

    Two coverage limits remain. Instance proxies cannot carry variant
    selections, so the traversal uses :class:`Usd.PrimRange`'s default
    predicate, which stops at instance boundaries; a variant set that exists
    only inside an instance prototype is not probed. Candidate selections are
    also gathered once, from the composition as the rules left it, so a variant
    set that exists only *inside* a non-default option of another set is never
    discovered.

    Args:
        stage: Stage to inspect. Unmodified on return.

    Returns:
        A ``(errors, truncated)`` pair. ``errors`` is sorted and de-duplicated;
        entries that only appear under a non-default selection are prefixed
        with the prim path and the selection that exposed them, for example
        ``[/robot{Configuration=skirt_only}] Could not open asset ...``. An
        error already visible in the default composition is reported once,
        unprefixed, rather than again for every selection that also sees it.
        ``truncated`` is True when the selection bound was reached, meaning
        options went unprobed and ``errors`` is a lower bound.
    """
    baseline = {str(error) for error in stage.GetCompositionErrors()}
    errors = set(baseline)

    # Sweep from the pseudo-root, not the default prim: narrowing to the default
    # prim would skip every sibling root (``/Environment``, ``/Looks``, a physics
    # scene) and report the resulting partial sweep as a complete one.
    root = stage.GetPseudoRoot()
    if not root:
        return sorted(errors), False

    # Record paths rather than prim handles: changing a selection recomposes the
    # stage and can invalidate any prim gathered before the change.
    probes: list[tuple[Sdf.Path, str, str, list[str]]] = []
    for prim in Usd.PrimRange(root):
        variant_sets = prim.GetVariantSets()
        for set_name in variant_sets.GetNames():
            variant_set = variant_sets.GetVariantSet(set_name)
            selected = variant_set.GetVariantSelection()
            unselected = [name for name in variant_set.GetVariantNames() if name != selected]
            if unselected:
                probes.append((prim.GetPath(), set_name, selected, unselected))

    if not probes:
        return sorted(errors), False

    session_layer = stage.GetSessionLayer()
    snapshot = session_layer.ExportToString()
    probed = 0
    truncated = False
    try:
        with Usd.EditContext(stage, session_layer):
            for prim_path, set_name, selected, unselected in probes:
                for option in unselected:
                    if probed >= _MAX_VARIANT_SELECTIONS_PROBED:
                        _LOGGER.warning(
                            "Stopped probing variant selections after %d; "
                            "composition errors inside the remaining options are not reported",
                            probed,
                        )
                        truncated = True
                        break
                    prim = stage.GetPrimAtPath(prim_path)
                    if not prim:
                        break
                    variant_set = prim.GetVariantSets().GetVariantSet(set_name)
                    if not variant_set.SetVariantSelection(option):
                        continue
                    probed += 1
                    prefix = f"[{prim_path}{{{set_name}={option}}}] "
                    errors.update(
                        prefix + str(error) for error in stage.GetCompositionErrors() if str(error) not in baseline
                    )
                # Put this set back before moving to the next, so each set is
                # probed against the selections the rules actually left behind.
                prim = stage.GetPrimAtPath(prim_path)
                if prim:
                    prim.GetVariantSets().GetVariantSet(set_name).SetVariantSelection(selected)
                if truncated:
                    break
    finally:
        session_layer.ImportFromString(snapshot)

    return sorted(errors), truncated


class AssetTransformerManager:
    """Coordinates execution of a ``RuleProfile`` over USD stages.

    The manager writes a working copy of the input stage to
    ``{package_root}/payloads/{base_name}`` and runs every rule against it.
    External assets are copied to ``{package_root}/source_assets/`` with paths
    rewritten to local references.

    :attr:`RuleProfile.flatten_source` decides how composition is handled:

    - ``True`` flattens the stage first, baking every sublayer, reference and
      payload into the exported layer. The result is self-contained.
    - ``False`` exports the root layer and keeps its arcs as arcs. Their targets
      are not copied into the package, so the arcs are re-anchored to the source
      files they name. Composed content is preserved, but the output keeps
      referring to the original source tree; use ``True`` for a package that
      stands alone.

    Unresolved arcs on the final stage are reported via
    :attr:`ExecutionReport.composition_errors` rather than passing silently.
    Each variant set is re-selected one option at a time while collecting them,
    so an arc broken inside an option nobody selected is still reported. That
    search is bounded; :attr:`ExecutionReport.composition_errors_truncated`
    records whether it ran to completion, so a sweep that gave up is never
    mistaken for a clean result.

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
            The manager's internal ``RuleRegistry`` instance.

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
            package_root: Destination root directory for outputs. Generate in
                an isolated directory and publish only generated files after
                successful execution, even for new or empty destinations.
                Unrelated files are preserved and cannot influence folder-based
                rule discovery.

        Returns:
            Execution report including per-rule logs and status. A conversion
            failure leaves the previous package intact and sets
            ``output_stage_path`` to ``None``.

        Raises:
            KeyError: If a rule type is not registered.
            RuntimeError: If stage loading or export fails.

        Example:

        .. code-block:: python

            manager = AssetTransformerManager()
            report = manager.run("input.usd", profile, package_root="/tmp/package")
        """
        output_root = os.path.realpath(package_root or profile.output_package_root or os.curdir)
        os.makedirs(output_root, exist_ok=True)

        # Rules discover and reopen files in the package directory. Even an
        # empty destination can retain cached layers from a deleted export,
        # so every conversion must use fresh working paths.
        with tempfile.TemporaryDirectory(prefix=".asset-transformer-", dir=output_root) as staging_root:
            try:
                report = self._run_in_directory(input_stage, profile, staging_root)
                # Previous clean runs can also leave unreachable crate owners.
                # Drop these before publication tries to replace their files.
                gc.collect()
                report.package_root = package_root or profile.output_package_root or ""
                if any(not result.success for result in report.results) or report.composition_errors:
                    # Keep the previous package intact when a rule or composition
                    # fails; no output from the temporary directory is published.
                    report.output_stage_path = None
                    return report
                diagnostics_truncated = _publish_staged_package(staging_root, output_root, report.output_stage_path)
                report.composition_errors_truncated |= diagnostics_truncated
                if report.output_stage_path:
                    report.output_stage_path = _relocate_path(report.output_stage_path, staging_root, output_root)
                for result in report.results:
                    result.affected_stages = [
                        _relocate_path(path, staging_root, output_root) for path in result.affected_stages
                    ]
                return report
            finally:
                # Recursive rule closures can keep stages and crate mappings
                # alive after execution returns. Release those cycles before
                # TemporaryDirectory removes files (Windows denies unlinking
                # an open crate), including when conversion reports failure.
                gc.collect()

    def _run_in_directory(
        self,
        input_stage: str | Usd.Stage,
        profile: RuleProfile,
        package_root: str | None,
    ) -> ExecutionReport:
        """Execute the profile in its working directory.

        Args:
            input_stage: Source path or opened stage.
            profile: Ordered transformation rules.
            package_root: Working directory for generated files.

        Returns:
            Execution report for this directory.

        Raises:
            RuntimeError: If source loading or export fails.
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
        elif not source_stage.GetRootLayer().Export(base_usda_path):
            # Without this check a stale base layer left by an earlier run is
            # opened below and transformed in place of the source.
            report.close()
            raise RuntimeError(f"Failed to export source stage to: {base_usda_path}")

        # Collect external assets and update paths in base.usda
        base_layer = Sdf.Layer.FindOrOpen(base_usda_path)
        if base_layer:
            _collect_assets(
                base_layer,
                package_root_final,
                source_layer_path=input_stage_path,
                # A non-flattened export still carries its composition arcs,
                # whose targets are not collected, so they need re-anchoring to
                # stay resolvable from the output directory.
                reanchor_uncollected=not profile.flatten_source,
            )
            _canonicalize_orient_quats(base_layer)
            base_layer.Save()
        # Drop the manager's direct handle to the base layer. Holding it would
        # keep payloads/<base>.usd open for the whole run, so a rule that
        # converts that file to .usda (and defers its deletion back to the
        # manager) could never have the original removed on Windows. After this
        # the working stage opened below is the sole manager-side handle to the
        # file until a rule returns a replacement stage.
        base_layer = None

        working_stage = Usd.Stage.Open(base_usda_path)
        if working_stage is None:
            report.close()
            raise RuntimeError(f"Failed to open working stage: {base_usda_path}")

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

                # A rule may defer file deletions to the manager via
                # RuleInterface.request_deletion. It cannot perform them itself:
                # the manager owns the working stage handed to the rule as
                # source_stage, so the file stays open until the manager drops
                # that handle (on Windows an open USD layer is an OS-level lock
                # and os.remove fails with WinError 5).
                deferred_deletions = rule.get_pending_deletions()

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

                # The manager has now released any superseded working stage, so
                # honor the rule's deferred deletions (e.g. the original .usd
                # converted in place to .usda). Best-effort and never deletes the
                # file backing the live working stage.
                if deferred_deletions:
                    live_path = working_stage.GetRootLayer().realPath if working_stage else None
                    _run_deferred_deletions(deferred_deletions, keep=live_path)

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

        # An unresolved arc drops every prim beneath it from the output while
        # leaving each rule successful, so nothing else in this report would
        # reveal it. USD's own diagnostics cannot be relied on to surface it
        # either: Kit mutes them by default via
        # ``/persistent/app/usd/muteUsdDiagnostics``.
        #
        # The output is already saved above, so a failure here must not cost the
        # caller a report it has otherwise earned: record the sweep as
        # inconclusive instead of propagating out of a successful transform.
        # Every entry in ``composition_errors`` names an unresolved arc, so the
        # failure is reported through the truncation flag alone; a synthetic
        # entry would read as data loss the output may not actually have.
        try:
            report.composition_errors, report.composition_errors_truncated = _collect_composition_errors(working_stage)
        except Exception:  # noqa: BLE001
            _LOGGER.exception("Composition error collection failed for %s", report.output_stage_path)
            report.composition_errors = []
            report.composition_errors_truncated = True
        if report.composition_errors:
            _LOGGER.warning(
                "Output stage %s has %d unresolved composition arc(s)%s; composed content is missing. First: %s",
                report.output_stage_path,
                len(report.composition_errors),
                " (sweep truncated, more may exist)" if report.composition_errors_truncated else "",
                "; ".join(report.composition_errors[:5]),
            )

        report.close()
        return report
