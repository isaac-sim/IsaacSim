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

"""Atomic authored-asset rules for RTX Sensor Processing Graphs (SPG)."""

from __future__ import annotations

import re
from collections import defaultdict, deque
from pathlib import Path, PurePosixPath
from typing import Any

import usd_validation_nvidia
from pxr import Ar, Sdf, Usd, UsdRender, UsdShade

from .sensor_rule_utils import RuleRegistrar, fail, scoped_prims, validation_roots, warn

_CATEGORY = "IsaacSim.Sensor.SPG"
_RULES = RuleRegistrar(_CATEGORY)
_RULES.policy("SPG-003")
_SOURCE_ASSET = "info:spg:sourceAsset"
_SUB_IDENTIFIER = "info:spg:sourceAsset:subIdentifier"
_RTX_AOV = "omni:rtx:aov"
_SUPPORTED_SOURCE_EXTENSIONS = frozenset({"cu", "ptx", "cubin", "fatbin", "slang"})
_CUDA_SOURCE_EXTENSIONS = frozenset({"cu", "ptx", "cubin", "fatbin"})
_SPG_ID_PREFIXES = ("spg:", "rtx.spg.", "omni.rtx.spg.")


def _outside_material(prim: Usd.Prim) -> bool:
    parent = prim.GetParent()
    while parent and not parent.IsPseudoRoot():
        if parent.IsA(UsdShade.Material):
            return False
        parent = parent.GetParent()
    return True


def _instanceable_ancestor(prim: Usd.Prim) -> Usd.Prim:
    parent = prim.GetParent()
    while parent and not parent.IsPseudoRoot():
        if parent.IsInstanceable():
            return parent
        parent = parent.GetParent()
    return Usd.Prim()


def _is_shader(prim: Usd.Prim) -> bool:
    return bool(prim and prim.IsA(UsdShade.Shader))


def _implementation_source(prim: Usd.Prim) -> str:
    return str(UsdShade.NodeDefAPI(prim).GetImplementationSource()) if _is_shader(prim) else ""


def _shader_id(prim: Usd.Prim) -> str:
    value = prim.GetAttribute("info:id").Get() if prim.GetAttribute("info:id") else None
    return str(value) if value is not None else ""


def _has_spg_id_prefix(value: str) -> bool:
    return value.startswith(_SPG_ID_PREFIXES)


def _valid_spg_id(value: str) -> bool:
    return any(value.startswith(prefix) and len(value) > len(prefix) for prefix in _SPG_ID_PREFIXES)


def _has_source_form(prim: Usd.Prim) -> bool:
    # NodeDefAPI::GetSourceTypes classifies the `info:spg:sourceAsset`
    # namespace even when the asset value is empty.  Match the 110.4
    # SpgPopulator classification boundary; later rules validate the value.
    return _implementation_source(prim) == "sourceAsset" and bool(prim.GetAttribute(_SOURCE_ASSET))


def _has_id_form(prim: Usd.Prim) -> bool:
    return _implementation_source(prim) == "id" and _has_spg_id_prefix(_shader_id(prim))


def _is_spg_shader(prim: Usd.Prim) -> bool:
    return (
        _is_shader(prim)
        and _outside_material(prim)
        and not _instanceable_ancestor(prim)
        and (_has_source_form(prim) or _has_id_form(prim))
    )


def _has_spg_metadata(prim: Usd.Prim) -> bool:
    return _is_shader(prim) and (bool(prim.GetAttribute(_SOURCE_ASSET)) or _has_spg_id_prefix(_shader_id(prim)))


def _metadata_shaders(stage: Usd.Stage) -> list[Usd.Prim]:
    return [prim for prim in scoped_prims(stage) if _has_spg_metadata(prim)]


def _spg_shaders(stage: Usd.Stage) -> list[Usd.Prim]:
    return [prim for prim in scoped_prims(stage) if _is_spg_shader(prim)]


def _source_shaders(stage: Usd.Stage) -> list[Usd.Prim]:
    return [prim for prim in _spg_shaders(stage) if _has_source_form(prim)]


def _input_attributes(prim: Usd.Prim) -> list[Usd.Attribute]:
    return [attribute for attribute in prim.GetAttributes() if attribute.GetName().startswith("inputs:")]


def _output_attributes(prim: Usd.Prim) -> list[Usd.Attribute]:
    return [attribute for attribute in prim.GetAttributes() if attribute.GetName().startswith("outputs:")]


def _connections(attribute: Usd.Attribute) -> list[Sdf.Path]:
    return list(attribute.GetConnections()) if attribute else []


def _source_attribute(stage: Usd.Stage, path: Sdf.Path) -> Usd.Attribute:
    return stage.GetAttributeAtPath(path) if path.IsPropertyPath() else Usd.Attribute()


def _is_opaque(attribute: Usd.Attribute) -> bool:
    return bool(attribute) and attribute.GetTypeName() == Sdf.ValueTypeNames.Opaque


def _is_render_var_aov(attribute: Usd.Attribute) -> bool:
    return bool(attribute) and attribute.GetPrim().IsA(UsdRender.Var) and attribute.GetName() == _RTX_AOV


def _is_spg_output(attribute: Usd.Attribute) -> bool:
    return bool(attribute) and _is_spg_shader(attribute.GetPrim()) and attribute.GetName().startswith("outputs:")


def _is_resource_input(stage: Usd.Stage, attribute: Usd.Attribute) -> bool:
    if _is_opaque(attribute):
        return True
    for path in _connections(attribute):
        source = _source_attribute(stage, path)
        source_prim = stage.GetPrimAtPath(path.GetPrimPath())
        if _is_render_var_aov(source) or _is_spg_output(source):
            return True
        if source_prim and source_prim.IsA(UsdRender.Var) and path.name == _RTX_AOV:
            return True
        if _has_spg_metadata(source_prim) and path.name.startswith("outputs:"):
            return True
    return False


def _resource_inputs(stage: Usd.Stage) -> list[Usd.Attribute]:
    return [
        attribute
        for shader in _spg_shaders(stage)
        for attribute in _input_attributes(shader)
        if _is_resource_input(stage, attribute)
    ]


def _bound_resource_output_paths(stage: Usd.Stage) -> set[Sdf.Path]:
    """Return output ports consumed by a shader or bound to a RenderVar."""
    paths: set[Sdf.Path] = set()
    for shader in _spg_shaders(stage):
        for attribute in _input_attributes(shader):
            paths.update(path for path in _connections(attribute) if path.name.startswith("outputs:"))
    for prim in scoped_prims(stage):
        if prim.IsA(UsdRender.Var):
            paths.update(
                path
                for path in _connections(prim.GetAttribute(_RTX_AOV))
                if path.IsPropertyPath() and path.name.startswith("outputs:")
            )
    return paths


def _resource_output_paths(stage: Usd.Stage) -> set[Sdf.Path]:
    paths = _bound_resource_output_paths(stage)
    paths.update(
        attribute.GetPath()
        for shader in _spg_shaders(stage)
        for attribute in _output_attributes(shader)
        if _is_opaque(attribute)
    )
    return paths


def _resource_outputs(stage: Usd.Stage) -> list[Usd.Attribute]:
    attributes: list[Usd.Attribute] = []
    for path in _resource_output_paths(stage):
        attribute = _source_attribute(stage, path)
        if attribute and _is_spg_shader(attribute.GetPrim()):
            attributes.append(attribute)
    return attributes


def _internal_edges(
    stage: Usd.Stage,
) -> list[tuple[Usd.Attribute, Sdf.Path, Usd.Attribute]]:
    """Return authored shader-port connections as (destination, path, source)."""
    edges: list[tuple[Usd.Attribute, Sdf.Path, Usd.Attribute]] = []
    metadata_paths = {prim.GetPath() for prim in _metadata_shaders(stage)}
    for shader in _spg_shaders(stage):
        for destination in (*_input_attributes(shader), *_output_attributes(shader)):
            for path in _connections(destination):
                if path.GetPrimPath() in metadata_paths:
                    edges.append((destination, path, _source_attribute(stage, path)))
    return edges


def _valid_directed_edges(stage: Usd.Stage) -> list[tuple[Usd.Prim, Usd.Prim]]:
    edges: list[tuple[Usd.Prim, Usd.Prim]] = []
    for destination, _path, source in _internal_edges(stage):
        if (
            source
            and _is_spg_shader(source.GetPrim())
            and destination.GetName().startswith("inputs:")
            and source.GetName().startswith("outputs:")
        ):
            edges.append((source.GetPrim(), destination.GetPrim()))
    return edges


def _components(stage: Usd.Stage) -> tuple[list[set[Sdf.Path]], dict[Sdf.Path, int]]:
    adjacency: dict[Sdf.Path, set[Sdf.Path]] = {shader.GetPath(): set() for shader in _spg_shaders(stage)}
    for source, destination in _valid_directed_edges(stage):
        adjacency[source.GetPath()].add(destination.GetPath())
        adjacency[destination.GetPath()].add(source.GetPath())
    components: list[set[Sdf.Path]] = []
    component_by_path: dict[Sdf.Path, int] = {}
    for root in adjacency:
        if root in component_by_path:
            continue
        index = len(components)
        component: set[Sdf.Path] = set()
        pending = [root]
        while pending:
            path = pending.pop()
            if path in component:
                continue
            component.add(path)
            component_by_path[path] = index
            pending.extend(adjacency[path] - component)
        components.append(component)
    return components, component_by_path


def _produced_render_vars(stage: Usd.Stage) -> list[Usd.Prim]:
    result: list[Usd.Prim] = []
    metadata_paths = {prim.GetPath() for prim in _metadata_shaders(stage)}
    for prim in scoped_prims(stage):
        if not prim.IsA(UsdRender.Var):
            continue
        if any(
            path.IsPropertyPath() and path.GetPrimPath() in metadata_paths and path.name.startswith("outputs:")
            for path in _connections(prim.GetAttribute(_RTX_AOV))
        ):
            result.append(prim)
    return result


def _asset_value(shader: Usd.Prim) -> Sdf.AssetPath | None:
    attribute = shader.GetAttribute(_SOURCE_ASSET)
    value = attribute.Get() if attribute else None
    return value if isinstance(value, Sdf.AssetPath) and value.path else None


def _resolved_asset_path(shader: Usd.Prim) -> str:
    value = _asset_value(shader)
    if value is None:
        return ""
    if value.resolvedPath:
        return value.resolvedPath
    raw_path = value.path
    layer = shader.GetStage().GetRootLayer()
    try:
        raw_path = Sdf.ComputeAssetPathRelativeToLayer(layer, raw_path)
    except Exception:
        pass
    try:
        return str(Ar.GetResolver().Resolve(raw_path))
    except Exception:
        return ""


def _asset_exists(shader: Usd.Prim) -> bool:
    resolved = _resolved_asset_path(shader)
    if not resolved:
        return False
    if Path(resolved).is_file():
        return True
    try:
        return Ar.GetResolver().OpenAsset(Ar.ResolvedPath(resolved)) is not None
    except Exception:
        return False


def _read_asset(shader: Usd.Prim, *, launch: bool = False) -> bytes | None:
    path = _resolved_asset_path(shader)
    if launch:
        path = _launch_path(path)
    if not path:
        return None
    local_path = Path(path)
    if local_path.is_file():
        try:
            return local_path.read_bytes()
        except OSError:
            return None
    try:
        asset = Ar.GetResolver().OpenAsset(Ar.ResolvedPath(path))
        return bytes(asset.GetBuffer()) if asset else None
    except Exception:
        return None


def _source_extension(shader: Usd.Prim) -> str:
    value = _asset_value(shader)
    path = value.resolvedPath or value.path if value else ""
    if "]" in path:
        path = path[: path.rfind("]")]
    return PurePosixPath(path).suffix.removeprefix(".").lower()


def _launch_path(source_path: str) -> str:
    close_bracket = source_path.rfind("]")
    if close_bracket >= 0:
        return f"{source_path[:close_bracket]}.lua]"
    return f"{source_path}.lua"


def _entry_point(shader: Usd.Prim) -> str:
    attribute = shader.GetAttribute(_SUB_IDENTIFIER)
    if attribute and attribute.HasAuthoredValueOpinion():
        value = attribute.Get()
        return str(value) if value is not None else ""
    return shader.GetName()


def _decode_source(data: bytes | None) -> str | None:
    if data is None:
        return None
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError:
        return None


def _strip_c_comments(source: str) -> str:
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.DOTALL)
    return re.sub(r"//.*", "", source)


def _strip_lua_comments(source: str) -> str:
    source = re.sub(r"--\[\[.*?\]\]", "", source, flags=re.DOTALL)
    return re.sub(r"--.*", "", source)


def _entry_exists_in_source(shader: Usd.Prim, source: str, entry: str) -> bool:
    extension = _source_extension(shader)
    source = _strip_c_comments(source)
    escaped = re.escape(entry)
    if extension == "cu":
        direct = re.search(
            rf'extern\s+"C"\s+[^;{{}}]*\b__global__\b[^;{{}}]*\b{escaped}\s*\(',
            source,
        )
        linkage_block = re.search(
            rf'extern\s+"C"\s*\{{[^}}]*\b__global__\b[^;{{}}]*\b{escaped}\s*\(',
            source,
        )
        return bool(direct or linkage_block)
    if extension == "ptx":
        return bool(re.search(rf"(?:^|\s)\.entry\s+{escaped}\s*\(", source))
    if extension == "slang":
        return bool(re.search(rf"\b[A-Za-z_][A-Za-z0-9_]*(?:\s*<[^;{{}}]+>)?\s+{escaped}\s*\(", source))
    return False


def _lua_entry_exists(source: str, entry: str) -> bool:
    source = _strip_lua_comments(source)
    escaped = re.escape(entry)
    return bool(
        re.search(rf"\bfunction\s+{escaped}\s*\(", source) or re.search(rf"\b{escaped}\s*=\s*function\s*\(", source)
    )


def _lua_entry_source(source: str, entry: str) -> str:
    """Return the selected top-level Lua function, excluding inactive entry points."""
    source = _strip_lua_comments(source)
    escaped = re.escape(entry)
    start = re.search(
        rf"(?m)^\s*(?:function\s+{escaped}\s*\(|{escaped}\s*=\s*function\s*\()",
        source,
    )
    if not start:
        return ""
    next_function = re.search(r"(?m)^\s*function\s+[A-Za-z_]", source[start.end() :])
    end = start.end() + next_function.start() if next_function else len(source)
    return source[start.start() : end]


def _source_name(render_var: Usd.Prim) -> str:
    value = UsdRender.Var(render_var).GetSourceNameAttr().Get()
    return str(value) if value else render_var.GetName()


@_RULES.rule("SPG-001")
def _spg_identification(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for root in validation_roots(stage):
        for shader in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
            if not _has_spg_metadata(shader):
                continue
            instanceable = _instanceable_ancestor(shader)
            if instanceable:
                fail(
                    checker,
                    requirement,
                    f"SPG Shader {shader.GetPath()} is beneath instanceable prim {instanceable.GetPath()}; "
                    "Kit does not populate prototype Shaders into SPG",
                    shader,
                )
            elif not _is_spg_shader(shader):
                fail(
                    checker,
                    requirement,
                    f"Shader {shader.GetPath()} carries SPG metadata but is not eligible for SPG population; "
                    "expected a Shader outside a Material with implementationSource='sourceAsset' and "
                    f"{_SOURCE_ASSET}, or implementationSource='id' and info:id prefixed by "
                    f"one of {_SPG_ID_PREFIXES}",
                    shader,
                )


@_RULES.rule("SPG-005")
def _cross_product_warning(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for render_var in _produced_render_vars(stage):
        consumer = render_var.GetParent() if render_var.GetParent().IsA(UsdRender.Product) else Usd.Prim()
        for path in _connections(render_var.GetAttribute(_RTX_AOV)):
            source = _source_attribute(stage, path)
            if not _is_spg_output(source):
                continue
            # FabricToSpgTranslator compares the Shader's direct parent path
            # with the RenderVar's RenderProduct path.
            producer_path = source.GetPrim().GetParent().GetPath()
            if consumer and producer_path != consumer.GetPath():
                warn(
                    checker,
                    requirement,
                    f"SPG output {path} crosses from shader parent {producer_path} to {consumer.GetPath()}",
                    render_var,
                )


@_RULES.rule("SPG-006")
def _effective_entry_point(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for shader in _source_shaders(stage):
        if not _entry_point(shader):
            fail(
                checker,
                requirement,
                f"SPG Shader {shader.GetPath()} has no effective entry point; author a nonempty "
                f"{_SUB_IDENTIFIER} or omit it to use the Shader prim name",
                shader,
            )


@_RULES.rule("SPG-007")
def _unique_component_names(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    components, _mapping = _components(stage)
    for component in components:
        by_name: dict[str, list[Sdf.Path]] = defaultdict(list)
        for path in component:
            by_name[path.name].append(path)
        for name, paths in by_name.items():
            if len(paths) > 1:
                fail(
                    checker,
                    requirement,
                    f"SPG component contains duplicate shader basename {name!r}: {', '.join(map(str, paths))}",
                    stage.GetPrimAtPath(paths[0]),
                )


@_RULES.rule("SPG-008")
def _one_implementation(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for shader in _metadata_shaders(stage):
        forms = int(_has_source_form(shader)) + int(_has_id_form(shader) and _valid_spg_id(_shader_id(shader)))
        if forms != 1:
            fail(
                checker,
                requirement,
                f"SPG Shader {shader.GetPath()} resolves {forms} implementation forms; expected exactly one of "
                f"sourceAsset ({_SOURCE_ASSET}) or id (info:id with prefix in {_SPG_ID_PREFIXES})",
                shader,
            )


@_RULES.rule("SPG-009")
def _source_asset_exists(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for shader in _source_shaders(stage):
        if not _asset_exists(shader):
            fail(
                checker,
                requirement,
                f"SPG source asset for {shader.GetPath()} does not resolve",
                shader,
            )


@_RULES.rule("SPG-010")
def _source_extension_supported(
    checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any
) -> None:
    for shader in _source_shaders(stage):
        extension = _source_extension(shader)
        if extension not in _SUPPORTED_SOURCE_EXTENSIONS:
            fail(
                checker,
                requirement,
                f"SPG source asset for {shader.GetPath()} has unsupported extension {extension!r}; "
                f"expected one of {sorted(_SUPPORTED_SOURCE_EXTENSIONS)}",
                shader,
            )


@_RULES.rule("SPG-011")
def _launch_asset_exists(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for shader in _source_shaders(stage):
        source_path = _resolved_asset_path(shader)
        launch_path = _launch_path(source_path) if source_path else ""
        exists = Path(launch_path).is_file() if launch_path else False
        if launch_path and not exists:
            try:
                exists = bool(Ar.GetResolver().OpenAsset(Ar.ResolvedPath(launch_path)))
            except Exception:
                exists = False
        if not exists:
            fail(
                checker,
                requirement,
                f"SPG launch asset {_launch_path(source_path or '<unresolved>')} does not resolve",
                shader,
            )


@_RULES.rule("SPG-012")
def _explicit_subidentifier(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for shader in _source_shaders(stage):
        attribute = shader.GetAttribute(_SUB_IDENTIFIER)
        if attribute and attribute.HasAuthoredValueOpinion() and not str(attribute.Get() or ""):
            fail(
                checker,
                requirement,
                f"SPG Shader {shader.GetPath()} authors an empty subIdentifier",
                attribute,
            )


@_RULES.rule("SPG-013")
def _render_var_exposes_aov(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for attribute in _resource_inputs(stage):
        for path in _connections(attribute):
            prim = stage.GetPrimAtPath(path.GetPrimPath())
            if prim and prim.IsA(UsdRender.Var) and not prim.GetAttribute(_RTX_AOV):
                fail(
                    checker,
                    requirement,
                    f"RenderVar {prim.GetPath()} supplying {attribute.GetPath()} does not expose {_RTX_AOV}",
                    prim,
                )


@_RULES.rule("SPG-014")
def _opaque_resource_inputs(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for attribute in _resource_inputs(stage):
        if not _is_opaque(attribute):
            fail(
                checker,
                requirement,
                f"SPG resource input {attribute.GetPath()} is not opaque",
                attribute,
            )


@_RULES.rule("SPG-015")
def _resource_input_cardinality(
    checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any
) -> None:
    for attribute in _resource_inputs(stage):
        count = len(_connections(attribute))
        if count > 1:
            fail(
                checker,
                requirement,
                f"SPG resource input {attribute.GetPath()} has {count} sources",
                attribute,
            )


@_RULES.rule("SPG-016")
def _resource_input_source(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for attribute in _resource_inputs(stage):
        for path in _connections(attribute):
            source = _source_attribute(stage, path)
            if not (_is_render_var_aov(source) or _is_spg_output(source)):
                fail(
                    checker,
                    requirement,
                    f"SPG resource input {attribute.GetPath()} has unsupported or missing source {path}; "
                    f"expected a RenderVar {_RTX_AOV} attribute or an SPG Shader outputs:* attribute",
                    attribute,
                )


@_RULES.rule("SPG-017")
def _typed_input_cardinality(
    checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any
) -> None:
    for shader in _spg_shaders(stage):
        for attribute in _input_attributes(shader):
            if _is_opaque(attribute):
                continue
            count = len(_connections(attribute))
            if count > 1:
                fail(
                    checker,
                    requirement,
                    f"Typed SPG input {attribute.GetPath()} has {count} sources",
                    attribute,
                )


@_RULES.rule("SPG-018")
def _typed_input_compatibility(
    checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any
) -> None:
    for shader in _spg_shaders(stage):
        for attribute in _input_attributes(shader):
            if _is_opaque(attribute):
                continue
            for path in _connections(attribute):
                source = _source_attribute(stage, path)
                if not source or source.GetTypeName() != attribute.GetTypeName():
                    actual = source.GetTypeName() if source else "missing"
                    fail(
                        checker,
                        requirement,
                        f"Typed SPG input {attribute.GetPath()} ({attribute.GetTypeName()}) has incompatible "
                        f"source {path} ({actual})",
                        attribute,
                    )


@_RULES.rule("SPG-019")
def _legacy_parameter_warning(
    checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any
) -> None:
    for shader in _spg_shaders(stage):
        legacy = [attribute for attribute in shader.GetAttributes() if attribute.GetName().startswith("params:")]
        if legacy:
            warn(
                checker,
                requirement,
                f"SPG Shader {shader.GetPath()} authors legacy params:* attributes "
                f"({', '.join(attribute.GetName() for attribute in legacy)}); use inputs:* for new assets",
                shader,
            )


@_RULES.rule("SPG-020")
def _produced_direct_child(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for render_var in _produced_render_vars(stage):
        if not render_var.GetParent().IsA(UsdRender.Product):
            fail(
                checker,
                requirement,
                f"SPG-produced RenderVar {render_var.GetPath()} is not a direct child of RenderProduct",
                render_var,
            )


@_RULES.rule("SPG-021")
def _produced_connection_count(
    checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any
) -> None:
    for render_var in _produced_render_vars(stage):
        attribute = render_var.GetAttribute(_RTX_AOV)
        count = len(_connections(attribute))
        if not attribute or not attribute.HasAuthoredConnections() or count != 1:
            fail(
                checker,
                requirement,
                f"SPG-produced RenderVar {render_var.GetPath()} authors {count} {_RTX_AOV} connections; expected one",
                render_var,
            )


@_RULES.rule("SPG-022")
def _produced_source(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for render_var in _produced_render_vars(stage):
        for path in _connections(render_var.GetAttribute(_RTX_AOV)):
            if not _is_spg_output(_source_attribute(stage, path)):
                fail(
                    checker,
                    requirement,
                    f"SPG-produced RenderVar {render_var.GetPath()} targets missing or invalid output {path}; "
                    "expected an outputs:* attribute on an SPG Shader",
                    render_var,
                )


@_RULES.rule("SPG-023")
def _produced_source_name(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for render_var in _produced_render_vars(stage):
        if not _source_name(render_var):
            fail(
                checker,
                requirement,
                f"SPG-produced RenderVar {render_var.GetPath()} has empty sourceName",
                render_var,
            )


@_RULES.rule("SPG-024")
def _unique_produced_source_name(
    checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any
) -> None:
    by_product_and_name: dict[tuple[Sdf.Path, str], list[Usd.Prim]] = defaultdict(list)
    for render_var in _produced_render_vars(stage):
        parent = render_var.GetParent()
        if parent.IsA(UsdRender.Product) and _source_name(render_var):
            by_product_and_name[(parent.GetPath(), _source_name(render_var))].append(render_var)
    for (product_path, name), render_vars in by_product_and_name.items():
        if len(render_vars) > 1:
            warn(
                checker,
                requirement,
                f"RenderProduct {product_path} has duplicate SPG output sourceName {name!r}",
                render_vars[0],
            )


@_RULES.rule("SPG-027")
def _opaque_resource_outputs(
    checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any
) -> None:
    for attribute in _resource_outputs(stage):
        if not _is_opaque(attribute):
            fail(
                checker,
                requirement,
                f"SPG resource output {attribute.GetPath()} is not opaque",
                attribute,
            )


@_RULES.rule("SPG-028")
def _nonopaque_resource_output(
    checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any
) -> None:
    for attribute in _resource_outputs(stage):
        if not _is_opaque(attribute):
            fail(
                checker,
                requirement,
                f"SPG resource output {attribute.GetPath()} is non-opaque and not portable",
                attribute,
            )


@_RULES.rule("SPG-029")
def _edge_components(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    _components_list, component_by_path = _components(stage)
    for destination, path, source in _internal_edges(stage):
        source_component = component_by_path.get(path.GetPrimPath())
        destination_component = component_by_path.get(destination.GetPrim().GetPath())
        if source_component is None or source_component != destination_component:
            fail(
                checker,
                requirement,
                f"SPG edge {path} -> {destination.GetPath()} does not resolve within one component",
                destination,
            )


@_RULES.rule("SPG-030")
def _edge_ports_exist(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for destination, path, source in _internal_edges(stage):
        if not destination or not source:
            fail(
                checker,
                requirement,
                f"SPG edge {path} -> {destination.GetPath()} has a missing port endpoint",
                destination.GetPrim(),
            )


@_RULES.rule("SPG-031")
def _edge_direction(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for destination, path, source in _internal_edges(stage):
        if not destination.GetName().startswith("inputs:") or not path.name.startswith("outputs:"):
            fail(
                checker,
                requirement,
                f"SPG edge {path} -> {destination.GetPath()} must connect an outputs:* port to an inputs:* port",
                destination,
            )


@_RULES.rule("SPG-032")
def _acyclic_components(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    adjacency: dict[Sdf.Path, set[Sdf.Path]] = {shader.GetPath(): set() for shader in _spg_shaders(stage)}
    indegree = {path: 0 for path in adjacency}
    for source, destination in _valid_directed_edges(stage):
        source_path = source.GetPath()
        destination_path = destination.GetPath()
        if destination_path not in adjacency[source_path]:
            adjacency[source_path].add(destination_path)
            indegree[destination_path] += 1
    pending = deque(path for path, count in indegree.items() if count == 0)
    visited = 0
    while pending:
        path = pending.popleft()
        visited += 1
        for destination in adjacency[path]:
            indegree[destination] -= 1
            if indegree[destination] == 0:
                pending.append(destination)
    if visited != len(adjacency):
        cycle_nodes = [path for path, count in indegree.items() if count > 0]
        fail(
            checker,
            requirement,
            f"SPG shader graph contains a cycle involving {', '.join(map(str, cycle_nodes))}",
            stage.GetPrimAtPath(cycle_nodes[0]),
        )


def _intended_shaders(stage: Usd.Stage) -> set[Sdf.Path]:
    upstream: dict[Sdf.Path, set[Sdf.Path]] = defaultdict(set)
    for source, destination in _valid_directed_edges(stage):
        upstream[destination.GetPath()].add(source.GetPath())
    intended: set[Sdf.Path] = set()
    pending: list[Sdf.Path] = []
    for render_var in _produced_render_vars(stage):
        for path in _connections(render_var.GetAttribute(_RTX_AOV)):
            if _is_spg_output(_source_attribute(stage, path)):
                pending.append(path.GetPrimPath())
    while pending:
        path = pending.pop()
        if path in intended:
            continue
        intended.add(path)
        pending.extend(upstream[path] - intended)
    return intended


def _valid_resource_bindings(stage: Usd.Stage, shader: Usd.Prim) -> list[Usd.Attribute]:
    valid: list[Usd.Attribute] = []
    for attribute in _input_attributes(shader):
        connections = _connections(attribute)
        if (
            _is_opaque(attribute)
            and len(connections) == 1
            and (
                _is_render_var_aov(_source_attribute(stage, connections[0]))
                or _is_spg_output(_source_attribute(stage, connections[0]))
            )
        ):
            valid.append(attribute)
    output_paths = _bound_resource_output_paths(stage)
    valid.extend(
        attribute
        for attribute in _output_attributes(shader)
        if _is_opaque(attribute) and attribute.GetPath() in output_paths
    )
    return valid


@_RULES.rule("SPG-035")
def _cuda_bound_resource(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    intended = _intended_shaders(stage)
    for shader in _source_shaders(stage):
        if (
            shader.GetPath() in intended
            and _source_extension(shader) in _CUDA_SOURCE_EXTENSIONS
            and not _valid_resource_bindings(stage, shader)
        ):
            fail(
                checker,
                requirement,
                f"CUDA SPG node {shader.GetPath()} is in an executable path but has no valid bound resource; "
                f"expected an opaque input bound to a RenderVar {_RTX_AOV} or SPG outputs:* port, "
                "or an opaque output consumed by a Shader or RenderVar",
                shader,
            )


@_RULES.rule("SPG-036")
def _source_entry_exists(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for shader in _source_shaders(stage):
        extension = _source_extension(shader)
        entry = _entry_point(shader)
        if extension in {"cubin", "fatbin"}:
            warn(
                checker,
                requirement,
                f"Entry point {entry!r} in binary {extension} source for {shader.GetPath()} requires binary inspection",
                shader,
            )
            continue
        source = _decode_source(_read_asset(shader))
        if source is None:
            fail(
                checker,
                requirement,
                f"Cannot read SPG source for {shader.GetPath()} to verify {entry!r}",
                shader,
            )
        elif not _entry_exists_in_source(shader, source, entry):
            fail(
                checker,
                requirement,
                f"SPG source for {shader.GetPath()} does not define entry {entry!r}",
                shader,
            )


@_RULES.rule("SPG-037")
def _lua_entry(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for shader in _source_shaders(stage):
        entry = _entry_point(shader)
        source = _decode_source(_read_asset(shader, launch=True))
        if source is None:
            fail(
                checker,
                requirement,
                f"Cannot read SPG launch asset for {shader.GetPath()}",
                shader,
            )
        elif not _lua_entry_exists(source, entry):
            fail(
                checker,
                requirement,
                f"SPG launch asset for {shader.GetPath()} does not define {entry!r}",
                shader,
            )


_LUA_REFERENCE = re.compile(r"\b(inputs|outputs|params)\s*\[\s*(['\"])([A-Za-z_][A-Za-z0-9_]*)\2\s*\]")
_LUA_DYNAMIC_REFERENCE = re.compile(r"\b(inputs|outputs|params)\s*\[\s*(?!['\"])")
_LUA_LAUNCH_CALL = re.compile(r"\b(?:cuda\.kernel|slang\.dispatch)\s*([({])")
_LUA_WRAPPED_REFERENCE = re.compile(
    r"\b(?:cuda|slang)\.([A-Za-z_][A-Za-z0-9_]*)\s*\(\s*"
    r"(inputs|outputs|params)\s*\[\s*(['\"])([A-Za-z_][A-Za-z0-9_]*)\3\s*\]"
)
_INPUT_RESOURCE_WRAPPERS = frozenset({"TextureObject", "Texture2D"})
_OUTPUT_RESOURCE_WRAPPERS = frozenset({"SurfaceObject", "RWTexture2D"})


def _lua_launch_bodies(source: str) -> list[str]:
    """Return statically balanced CUDA and Slang launch-call bodies."""
    bodies: list[str] = []
    closing = {"(": ")", "{": "}", "[": "]"}
    for match in _LUA_LAUNCH_CALL.finditer(source):
        stack = [closing[match.group(1)]]
        quote = ""
        escaped = False
        for index in range(match.end(), len(source)):
            char = source[index]
            if quote:
                if escaped:
                    escaped = False
                elif char == "\\":
                    escaped = True
                elif char == quote:
                    quote = ""
            elif char in "'\"":
                quote = char
            elif char in closing:
                stack.append(closing[char])
            elif char in (")", "}", "]"):
                if char != stack.pop():
                    break
                if not stack:
                    bodies.append(source[match.end() : index])
                    break
    return bodies


def _declared_lua_attributes(shader: Usd.Prim) -> dict[str, dict[str, Usd.Attribute]]:
    inputs = {attribute.GetName().split(":", 1)[1]: attribute for attribute in _input_attributes(shader)}
    legacy = {
        attribute.GetName().split(":", 1)[1]: attribute
        for attribute in shader.GetAttributes()
        if attribute.GetName().startswith("params:") and not _is_opaque(attribute)
    }
    outputs = {attribute.GetName().split(":", 1)[1]: attribute for attribute in _output_attributes(shader)}
    value_inputs = {name: attribute for name, attribute in inputs.items() if not _is_opaque(attribute)}
    return {"inputs": legacy | inputs, "params": legacy | value_inputs, "outputs": outputs}


@_RULES.rule("SPG-038")
def _lua_arguments(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
    for shader in _source_shaders(stage):
        source = _decode_source(_read_asset(shader, launch=True))
        if source is None:
            continue
        source = _lua_entry_source(source, _entry_point(shader))
        if not source:
            continue
        declared = _declared_lua_attributes(shader)
        for body in _lua_launch_bodies(source):
            for table, _quote, name in _LUA_REFERENCE.findall(body):
                if name in declared[table]:
                    continue
                message = f"SPG launch asset for {shader.GetPath()} references undeclared {table}[{name!r}]"
                if table == "inputs":
                    warn(checker, requirement, f"{message}; confirm whether SPG supplies it implicitly", shader)
                else:
                    fail(checker, requirement, f"{message}; expected an authored Shader port", shader)
            for wrapper, table, _quote, name in _LUA_WRAPPED_REFERENCE.findall(body):
                attribute = declared[table].get(name)
                if not attribute:
                    continue
                if wrapper in _INPUT_RESOURCE_WRAPPERS:
                    valid, expected = table == "inputs" and _is_opaque(attribute), "an opaque inputs:* port"
                elif wrapper in _OUTPUT_RESOURCE_WRAPPERS:
                    valid, expected = table == "outputs" and _is_opaque(attribute), "an opaque outputs:* port"
                elif wrapper in {"array", "static", "ParameterBlock"}:
                    continue  # These accept both resource and value ports.
                else:
                    valid, expected = table in {"inputs", "params"} and not _is_opaque(attribute), "a typed value input"
                if not valid:
                    fail(
                        checker,
                        requirement,
                        f"SPG launch wrapper {wrapper} expects {expected}, but {attribute.GetPath()} "
                        f"has type {attribute.GetTypeName()} and direction {table}",
                        attribute,
                    )
            if _LUA_DYNAMIC_REFERENCE.search(body):
                warn(
                    checker,
                    requirement,
                    f"SPG launch asset for {shader.GetPath()} uses dynamic table indexing in launch arguments; "
                    "review port and wrapper compatibility",
                    shader,
                )
