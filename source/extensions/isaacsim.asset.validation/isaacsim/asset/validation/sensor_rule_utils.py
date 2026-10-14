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

"""Shared helpers for atomic authored-sensor requirement validators."""

from __future__ import annotations

import math
import re
from collections.abc import Callable, Iterable
from dataclasses import dataclass
from typing import Any

import usd_validation_nvidia
from pxr import Sdf, Usd, UsdRender

from .sensor_capabilities import get_requirement

CheckFunction = Callable[[usd_validation_nvidia.BaseRuleChecker, Usd.Stage, Any], None]

RULES_BY_REQUIREMENT_ID: dict[str, type[usd_validation_nvidia.BaseRuleChecker]] = {}

SENSOR_TYPE_NAMES = frozenset({"Camera", "OmniLidar", "OmniRadar"})
SENSOR_PRIM_TYPE_PARAMETER = "SENSOR_PRIM_TYPE"


@dataclass(frozen=True)
class RuleRegistrar:
    """Bind atomic rule registration to one Asset Validator category."""

    category: str

    def check(self, requirement_id: str, check: CheckFunction) -> type[usd_validation_nvidia.BaseRuleChecker]:
        return register_stage_check(requirement_id, check, category=self.category)

    def rule(self, requirement_id: str) -> Callable[[CheckFunction], CheckFunction]:
        """Register a named stage check while preserving its function binding.

        Args:
            requirement_id: Short spreadsheet requirement ID.

        Returns:
            Decorator that registers and returns the original check function.
        """

        def decorate(check: CheckFunction) -> CheckFunction:
            self.check(requirement_id, check)
            return check

        return decorate

    def policy(self, requirement_id: str) -> type[usd_validation_nvidia.BaseRuleChecker]:
        return register_policy_check(requirement_id, category=self.category)

    def policies(self, *requirement_ids: str) -> None:
        """Register non-constraining requirements with no check function.

        Args:
            *requirement_ids: Short spreadsheet requirement IDs.
        """
        for requirement_id in requirement_ids:
            self.policy(requirement_id)

    def manual(
        self, requirement_id: str, applies: Callable[[Usd.Stage], Iterable[Any]], *, reason: str
    ) -> type[usd_validation_nvidia.BaseRuleChecker]:
        return register_manual_check(requirement_id, applies, category=self.category, reason=reason)


@dataclass(frozen=True)
class AttributeNamespace:
    """Read attributes from a fixed schema namespace."""

    prefix: str

    def attribute(self, prim: Usd.Prim, short_name: str, instance: str | None = None) -> Usd.Attribute:
        suffix = f"{instance}:{short_name}" if instance is not None else short_name
        return prim.GetAttribute(f"{self.prefix}{suffix}")

    def value(self, prim: Usd.Prim, short_name: str, instance: str | None = None) -> Any:
        attribute = self.attribute(prim, short_name, instance)
        return attribute.Get() if attribute else None


def make_bound_check(
    select_prims: Callable[[Usd.Stage], Iterable[Usd.Prim]],
    namespace: AttributeNamespace,
    name: str,
    predicate: Callable[[Any], bool],
    description: str,
    *,
    condition: Callable[[Usd.Prim], bool] | None = None,
    message: Callable[[Usd.Prim, str, Any, str], str] | None = None,
) -> CheckFunction:
    """Create a stage check for one bounded attribute on selected prims.

    Args:
        select_prims: Select prims to validate from a composed stage.
        namespace: Resolve the attribute namespace on each prim.
        name: Attribute name within the namespace.
        predicate: Accept valid authored values.
        description: Describe the required value in a failure.
        condition: Restrict the check to prims where this returns true.
        message: Format a failure when the default message is unsuitable.

    Returns:
        Stage check for the selected attribute.

    Example:
        .. code-block:: python

            check = make_bound_check(
                scoped_prims,
                AttributeNamespace("omni:sensor:Core:"),
                "maxReturns",
                lambda value: value > 0,
                "greater than zero",
            )
    """

    def check(checker: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage, requirement: Any) -> None:
        for prim in select_prims(stage):
            if condition is not None and not condition(prim):
                continue
            attribute = namespace.attribute(prim, name)
            value = attribute.Get() if attribute else None
            if value is None or not predicate(value):
                failure = (
                    message(prim, name, value, description)
                    if message is not None
                    else f"{namespace.prefix}{name} on {prim.GetPath()} must be {description}"
                )
                fail(checker, requirement, failure, attribute or prim)

    return check


def register_stage_check(
    requirement_id: str,
    check: CheckFunction,
    *,
    category: str,
) -> type[usd_validation_nvidia.BaseRuleChecker]:
    """Register one stage callback as one SimReady requirement validator.

    Args:
        requirement_id: Short spreadsheet requirement ID.
        check: Callback implementing only that requirement.
        category: Asset Validator rule category.

    Returns:
        Registered checker class.
    """
    requirement = get_requirement(requirement_id)
    class_name = "".join(part.title() for part in re.split(r"[^A-Za-z0-9]+", requirement_id)) + "Checker"

    def check_stage(self: usd_validation_nvidia.BaseRuleChecker, stage: Usd.Stage) -> None:
        self._sensor_seen_compositions = set()
        # Empty stages have no prim callback, so they are handled here.
        if not stage.GetPseudoRoot().GetChildren():
            check(self, stage, requirement)

    def check_prim(self: usd_validation_nvidia.BaseRuleChecker, prim: Usd.Prim) -> None:
        roots = validation_roots(prim.GetStage())
        if not roots or prim != roots[0]:
            # Nested variant selections do not revisit their asset root.
            if not prim.GetVariantSets().GetNames() or not any(
                prim.GetPath().HasPrefix(root.GetPath()) for root in roots
            ):
                return
        selections = tuple(
            (scoped.GetPath(), name, scoped.GetVariantSets().GetVariantSet(name).GetVariantSelection())
            for scoped in scoped_prims(prim.GetStage())
            for name in scoped.GetVariantSets().GetNames()
        )
        if selections not in self._sensor_seen_compositions:
            self._sensor_seen_compositions.add(selections)
            check(self, prim.GetStage(), requirement)

    checker = type(
        class_name,
        (usd_validation_nvidia.BaseRuleChecker,),
        {
            "__doc__": requirement.message or requirement.display_name,
            "__module__": check.__module__,
            "CheckStage": check_stage,
            "CheckPrim": check_prim,
            "REQUIREMENT": requirement,
        },
    )
    checker = usd_validation_nvidia.register_requirements(requirement, override=True)(checker)
    checker = usd_validation_nvidia.register_rule(category)(checker)
    RULES_BY_REQUIREMENT_ID[requirement_id] = checker
    return checker


def register_policy_check(requirement_id: str, *, category: str) -> type[usd_validation_nvidia.BaseRuleChecker]:
    """Register a non-constraining policy statement as an implemented no-op.

    These requirements constrain validator behavior rather than asset content.
    Registering them prevents feature validation from treating the deliberately
    non-constraining statement as a missing implementation.

    Args:
        requirement_id: Short spreadsheet requirement ID.
        category: Asset Validator rule category.

    Returns:
        Registered checker class.
    """

    def check(_checker: usd_validation_nvidia.BaseRuleChecker, _stage: Usd.Stage, _requirement: Any) -> None:
        return

    return register_stage_check(requirement_id, check, category=category)


def register_manual_check(
    requirement_id: str,
    applies: Callable[[Usd.Stage], Iterable[Any]],
    *,
    category: str,
    reason: str,
) -> type[usd_validation_nvidia.BaseRuleChecker]:
    """Register a requirement that needs evidence unavailable to static USD validation.

    Unlike a policy no-op, this emits an attributable warning for every applicable
    object.  A capability therefore never silently claims that the requirement was
    proven.

    Args:
        requirement_id: Short spreadsheet requirement ID.
        applies: Returns the objects for which manual review is required.
        category: Asset Validator rule category.
        reason: Concise explanation of the missing static evidence.

    Returns:
        Registered checker class.
    """

    def check(
        checker: usd_validation_nvidia.BaseRuleChecker,
        stage: Usd.Stage,
        requirement: Any,
    ) -> None:
        for item in applies(stage):
            warn(checker, requirement, reason, item)

    return register_stage_check(requirement_id, check, category=category)


def fail(
    checker: usd_validation_nvidia.BaseRuleChecker,
    requirement: Any,
    message: str,
    at: Any,
) -> None:
    """Emit a requirement-attributed failure."""
    checker._AddFailedCheck(message=message, at=at, requirement=requirement)


def warn(
    checker: usd_validation_nvidia.BaseRuleChecker,
    requirement: Any,
    message: str,
    at: Any,
) -> None:
    """Emit a requirement-attributed warning."""
    checker._AddWarning(message=message, at=at, requirement=requirement)


def validation_roots(stage: Usd.Stage) -> list[Usd.Prim]:
    """Return explicitly selected roots, otherwise the asset's default prim.

    A missing default prim is invalid for whole-stage sensor validation.  Its
    root prims remain available for independently enabled diagnostic checks.
    """
    paths = stage.GetPopulationMask().GetPaths()
    if Sdf.Path.absoluteRootPath not in paths:
        return [prim for path in paths if (prim := stage.GetPrimAtPath(path))]
    default_prim = stage.GetDefaultPrim()
    return [default_prim] if default_prim else stage.GetPseudoRoot().GetChildren()


def scoped_prims(stage: Usd.Stage) -> list[Usd.Prim]:
    """Return composed prims below the selected or default asset root."""
    return [prim for root in validation_roots(stage) for prim in Usd.PrimRange(root)]


def prims(stage: Usd.Stage, type_names: Iterable[str] | None = None) -> list[Usd.Prim]:
    """Return scoped, composed prims, optionally filtered by type name."""
    allowed = set(type_names) if type_names is not None else None
    return [prim for prim in scoped_prims(stage) if allowed is None or prim.GetTypeName() in allowed]


def sensor_prims(stage: Usd.Stage) -> list[Usd.Prim]:
    """Return supported authored sensor prims from the composed stage."""
    return prims(stage, SENSOR_TYPE_NAMES)


def feature_sensor_type(checker: usd_validation_nvidia.BaseRuleChecker) -> str | None:
    """Return the feature-scoped sensor type, or ``None`` for direct capability checks."""
    if SENSOR_PRIM_TYPE_PARAMETER not in checker.parameters:
        return None
    value = checker.parameters[SENSOR_PRIM_TYPE_PARAMETER].assigned_value
    return str(value) if value in SENSOR_TYPE_NAMES else None


def feature_sensor_prims(
    checker: usd_validation_nvidia.BaseRuleChecker,
    stage: Usd.Stage,
) -> list[Usd.Prim]:
    """Return sensors covered by the selected feature when it supplies a type."""
    type_name = feature_sensor_type(checker)
    return prims(stage, {type_name} if type_name else SENSOR_TYPE_NAMES)


def render_products(stage: Usd.Stage) -> list[Usd.Prim]:
    """Return RenderProduct prims below the selected or default asset root."""
    return [prim for prim in scoped_prims(stage) if prim.IsA(UsdRender.Product)]


def render_products_targeting(
    stage: Usd.Stage, *, sensor_paths: Iterable[Sdf.Path] = (), sensor_type_names: Iterable[str] = ()
) -> list[Usd.Prim]:
    """Return RenderProducts targeting any selected path or typed sensor."""
    paths, type_names = set(sensor_paths), set(sensor_type_names)
    result: list[Usd.Prim] = []
    for product in render_products(stage):
        for path in UsdRender.Product(product).GetCameraRel().GetTargets():
            target = stage.GetPrimAtPath(path)
            if path in paths or (target and target.GetTypeName() in type_names):
                result.append(product)
                break
    return result


def ordered_render_vars(render_product_prim: Usd.Prim) -> list[tuple[Sdf.Path, Usd.Prim]]:
    """Return every orderedVars target and its resolved prim, including invalid prims."""
    relationship = UsdRender.Product(render_product_prim).GetOrderedVarsRel()
    targets = relationship.GetTargets() if relationship else []
    stage = render_product_prim.GetStage()
    return [(target, stage.GetPrimAtPath(target)) for target in targets]


def native_render_vars(render_product_prims: Iterable[Usd.Prim]) -> list[tuple[Usd.Prim, Usd.Prim]]:
    """Return ordered native RenderVars, excluding SPG-produced AOVs."""
    result: list[tuple[Usd.Prim, Usd.Prim]] = []
    for product in render_product_prims:
        for _path, render_var in ordered_render_vars(product):
            aov = render_var.GetAttribute("omni:rtx:aov") if render_var else None
            if render_var and render_var.IsA(UsdRender.Var) and not (aov and aov.GetConnections()):
                result.append((product, render_var))
    return result


def render_var_source_name(render_var_prim: Usd.Prim) -> str:
    """Return a RenderVar's effective sourceName, or an empty string."""
    if not render_var_prim or not render_var_prim.IsA(UsdRender.Var):
        return ""
    value = UsdRender.Var(render_var_prim).GetSourceNameAttr().Get()
    return str(value) if value is not None else ""


def as_sequence(value: Any) -> list[Any]:
    """Materialize an array-like value without treating scalar strings as arrays."""
    if value is None or isinstance(value, (str, bytes, Sdf.AssetPath)):
        return []
    try:
        return list(value)
    except TypeError:
        return []


def numeric_values_are_finite(value: Any, *, missing: bool = False) -> bool:
    """Return whether a numeric scalar or sequence contains only finite values."""
    if value is None:
        return missing
    if isinstance(value, (float, int)):
        return math.isfinite(value)
    return all(isinstance(item, (float, int)) and math.isfinite(item) for item in as_sequence(value))


def render_var_channels(render_var: Usd.Prim) -> list[str]:
    """Return a RenderVar's effective channels as strings."""
    attribute = render_var.GetAttribute("channels")
    return [str(value) for value in as_sequence(attribute.Get() if attribute else None)]


def attribute_value(prim: Usd.Prim, name: str) -> Any:
    """Return an effective attribute value, or ``None`` when unavailable."""
    attribute = prim.GetAttribute(name)
    return attribute.Get() if attribute else None


def is_finite(value: Any) -> bool:
    """Return whether every numeric scalar or sequence component is finite."""
    if isinstance(value, bool):
        return True
    if isinstance(value, (int, float)):
        return math.isfinite(value)
    if isinstance(value, (str, Sdf.AssetPath)) or value is None:
        return True
    try:
        return all(is_finite(component) for component in value)
    except TypeError:
        return True


def applied_instances(prim: Usd.Prim, schema_name: str) -> list[str]:
    """Return composed multiple-apply instance names for ``schema_name``."""
    prefix = f"{schema_name}:"
    return [schema[len(prefix) :] for schema in prim.GetAppliedSchemas() if schema.startswith(prefix)]


def is_valid_instance_name(value: str) -> bool:
    """Return whether a multiple-apply instance name is a valid nonnumeric identifier."""
    return bool(value and not value[0].isdigit() and Sdf.Path.IsValidIdentifier(value))
