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

"""Selective Newton actuator integration for the Isaac Sim SysID bridge.

Explicit actuators are an overlay: only selected DOFs are transferred from
implicit USD drives.  Candidate parameters remain in runtime Warp arrays and
accepted parameters alone are authored back to USD.
"""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import PurePosixPath
from types import SimpleNamespace
from typing import TYPE_CHECKING, Any

import numpy as np

from .actuator_compatibility import ACTUATOR_RUNTIME_EXPLICIT

if TYPE_CHECKING:
    from pxr import Sdf, Usd

_ACTUATOR_SCOPE_NAME = "__sysid_actuators"


@dataclass(frozen=True)
class ExplicitPdActuatorDescriptor:
    """USD-authored baseline for one supported explicit actuator."""

    prim_path: str
    target_path: str
    dof_name: str
    kp: float
    kd: float
    const_effort: float
    delay_steps: int
    max_effort: float
    controller_kind: str = "pd"
    integral_gain: float = 0.0
    integral_max: float = float("inf")
    clamp_kind: str = "max_effort"
    max_motor_effort: float = float("inf")
    saturation_effort: float = float("inf")
    velocity_limit: float = float("inf")


@dataclass(frozen=True)
class TemporaryActuatorPromotion:
    """One actuator prim authored into the stage session layer."""

    prim_path: str
    target_path: str


def _drive_baseline(joint_prim: Usd.Prim) -> tuple[float, float, float]:
    """Read a joint's authored implicit-drive baseline without creating an API.

    Args:
        joint_prim: Joint prim containing the implicit drive.

    Returns:
        The stiffness, damping, and maximum effort.
    """
    from pxr import UsdPhysics

    drive_name = "linear" if joint_prim.IsA(UsdPhysics.PrismaticJoint) else "angular"
    drive = UsdPhysics.DriveAPI.Get(joint_prim, drive_name)
    if not drive or not drive.GetPrim().IsValid():
        return 0.0, 0.0, float("inf")

    def _value(attr: Any, default: float) -> float:
        value = attr.Get() if attr else None
        return float(default if value is None else value)

    stiffness = max(_value(drive.GetStiffnessAttr(), 0.0), 0.0)
    damping = max(_value(drive.GetDampingAttr(), 0.0), 0.0)
    if drive_name == "angular":
        # USD angular drives are authored per degree. Newton explicit actuators
        # consume effective gains per radian.
        degrees_per_radian = 180.0 / math.pi
        stiffness *= degrees_per_radian
        damping *= degrees_per_radian
    return stiffness, damping, max(_value(drive.GetMaxForceAttr(), float("inf")), 0.0)


def promote_selected_dofs_to_session_actuators(
    stage: Usd.Stage,
    articulation_root: str,
    selected_dof_paths: Iterable[str],
    *,
    controller_kind: str = "pd",
    include_delay: bool = False,
) -> list[TemporaryActuatorPromotion]:
    """Author only missing selected actuators into the session layer.

    Args:
        stage: Stage receiving the temporary actuator prims.
        articulation_root: Root path of the target articulation.
        selected_dof_paths: Joint paths selected for explicit control.
        controller_kind: Controller type to author, either ``pd`` or ``pid``.
        include_delay: Whether to author the actuator delay API.

    Returns:
        The temporary actuator promotions that were created.
    """
    from pxr import Sdf, Usd

    root = stage.GetPrimAtPath(articulation_root)
    if not root or not root.IsValid():
        raise ValueError(f"Invalid articulation root: {articulation_root}")
    kind = str(controller_kind).lower()
    if kind not in ("pd", "pid"):
        raise ValueError(f"Only PD and PID joints can be promoted; got {controller_kind!r}.")

    existing_targets = {target for prim in Usd.PrimRange(root) for target in _target_paths(prim)}
    session_layer = stage.GetSessionLayer()
    promotions: list[TemporaryActuatorPromotion] = []
    with Usd.EditContext(stage, session_layer):
        for target_path in [str(path) for path in selected_dof_paths]:
            if target_path in existing_targets:
                continue
            joint = stage.GetPrimAtPath(target_path)
            if not joint or not joint.IsValid():
                raise ValueError(f"Cannot promote missing joint prim: {target_path}")
            kp, kd, max_effort = _drive_baseline(joint)
            base = target_path.rsplit("/", 1)[-1]
            scope = f"{articulation_root.rstrip('/')}/{_ACTUATOR_SCOPE_NAME}"
            candidate = f"{scope}/{base}"
            suffix = 1
            while stage.GetPrimAtPath(candidate).IsValid():
                candidate = f"{scope}/{base}_{suffix}"
                suffix += 1
            prim = stage.DefinePrim(candidate, "Xform")
            prim.AddAppliedSchema("NewtonPIDControlAPI" if kind == "pid" else "NewtonPDControlAPI")
            prim.AddAppliedSchema("NewtonMaxEffortClampingAPI")
            if include_delay:
                prim.AddAppliedSchema("NewtonActuatorDelayAPI")
            prim.CreateRelationship("newton:targets").SetTargets([Sdf.Path(target_path)])
            _set_attr(prim, "newton:kp", Sdf.ValueTypeNames.Float, kp)
            _set_attr(prim, "newton:kd", Sdf.ValueTypeNames.Float, kd)
            _set_attr(prim, "newton:constEffort", Sdf.ValueTypeNames.Float, 0.0)
            _set_attr(prim, "newton:maxEffort", Sdf.ValueTypeNames.Float, max_effort)
            if kind == "pid":
                _set_attr(prim, "newton:ki", Sdf.ValueTypeNames.Float, 0.0)
                _set_attr(prim, "newton:integralMax", Sdf.ValueTypeNames.Float, float("inf"))
            if include_delay:
                _set_attr(prim, "newton:delaySteps", Sdf.ValueTypeNames.Int, 0)
            promotions.append(TemporaryActuatorPromotion(candidate, target_path))
    return promotions


def _active_edit_layer(stage: Usd.Stage) -> Sdf.Layer | None:
    """Return the layer the stage is currently authoring to, if it exposes one.

    Args:
        stage: Stage whose edit target should be resolved.

    Returns:
        Active edit-target layer, or None when the stage exposes no edit target.
    """
    getter = getattr(stage, "GetEditTarget", None)
    if not callable(getter):
        return None
    target = getter()
    layer_getter = getattr(target, "GetLayer", None)
    return layer_getter() if callable(layer_getter) else None


def discard_session_actuator_promotions(stage: Usd.Stage, promotions: Iterable[TemporaryActuatorPromotion]) -> None:
    """Remove temporary session-layer prim specs without touching persistent USD.

    Args:
        stage: Stage containing the temporary prims.
        promotions: Temporary promotions to remove.
    """
    from pxr import Sdf, Usd

    layer = stage.GetSessionLayer()
    scope_paths: set[Sdf.Path] = set()
    with Usd.EditContext(stage, layer):
        for item in promotions:
            path = Sdf.Path(item.prim_path)
            if layer.GetPrimAtPath(path) is not None:
                stage.RemovePrim(path)
            parent = path.GetParentPath()
            if parent.name == _ACTUATOR_SCOPE_NAME:
                scope_paths.add(parent)
        # Leaving the emptied scope behind would make a discarded run look like
        # it had promoted actuators.
        for scope_path in scope_paths:
            scope_spec = layer.GetPrimAtPath(scope_path)
            if scope_spec is not None and not scope_spec.nameChildren:
                stage.RemovePrim(scope_path)


def commit_session_actuator_promotions(
    stage: Usd.Stage,
    promotions: Iterable[TemporaryActuatorPromotion],
    *,
    target_layer: Sdf.Layer | None = None,
) -> bool:
    """Copy accepted temporary actuator prims to a persistent stage layer.

    Args:
        stage: Stage containing the temporary prims.
        promotions: Temporary promotions to persist.
        target_layer: Optional destination layer; defaults to the root layer.

    Returns:
        Whether every promotion was copied successfully.
    """
    from pxr import Sdf

    source = stage.GetSessionLayer()
    target = target_layer or stage.GetRootLayer()
    if target is source:
        # The promotions already live in the session layer. Copying it onto
        # itself would succeed and the follow-up discard would then delete the
        # only copy, so fall back to a durable layer.
        target = stage.GetRootLayer()
    for item in promotions:
        path = Sdf.Path(item.prim_path)
        if target.GetPrimAtPath(path.GetParentPath()) is None:
            Sdf.CreatePrimInLayer(target, path.GetParentPath())
        if source.GetPrimAtPath(path) is None or not Sdf.CopySpec(source, path, target, path):
            return False
    return True


def _applied_schemas(prim: Usd.Prim) -> set[str]:
    try:
        return {str(name) for name in prim.GetAppliedSchemas()}
    except Exception:
        return set()


def _target_paths(prim: Usd.Prim) -> list[str]:
    try:
        rel = prim.GetRelationship("newton:targets")
        if not rel:
            return []
        return [str(path) for path in rel.GetTargets()]
    except Exception:
        return []


def _attr_float(prim: Usd.Prim, name: str, default: float) -> float:
    try:
        attr = prim.GetAttribute(name)
        value = attr.Get() if attr else default
        result = float(value)
    except (TypeError, ValueError, RuntimeError):
        result = float(default)
    return result


def _attr_int(prim: Usd.Prim, name: str, default: int) -> int:
    try:
        attr = prim.GetAttribute(name)
        value = attr.Get() if attr else default
        result = int(value)
    except (TypeError, ValueError, RuntimeError):
        result = int(default)
    return result


def discover_explicit_pd_actuators(
    stage: Usd.Stage,
    articulation_root: str,
    *,
    selected_dof_paths: Iterable[str] | None = None,
) -> list[ExplicitPdActuatorDescriptor]:
    """Read and validate the supported explicit-PD surface from USD.

    When ``selected_dof_paths`` is supplied, the returned descriptors follow
    that exact order and every selected DOF must have one actuator.

    Args:
        stage: Stage containing the articulation.
        articulation_root: Root path of the articulation to inspect.
        selected_dof_paths: Optional ordered subset of joint paths.

    Returns:
        Validated actuator descriptors in requested DOF order.
    """
    from pxr import Usd

    root = stage.GetPrimAtPath(articulation_root)
    if not root or not root.IsValid():
        raise ValueError(f"Invalid articulation root: {articulation_root}")

    selected_paths = None if selected_dof_paths is None else [str(path) for path in selected_dof_paths]
    selected_path_set = None if selected_paths is None else set(selected_paths)
    by_target: dict[str, ExplicitPdActuatorDescriptor] = {}
    controller_schemas = {
        "NewtonPDControlAPI",
        "NewtonPIDControlAPI",
        "NewtonNeuralControlAPI",
    }
    unsupported_clamps = {"NewtonPositionBasedClampingAPI"}
    for prim in Usd.PrimRange(root):
        targets = _target_paths(prim)
        schemas = _applied_schemas(prim)
        if not targets and not (schemas & controller_schemas):
            continue
        if selected_path_set is not None and not any(target in selected_path_set for target in targets):
            continue
        if len(targets) != 1:
            raise ValueError(f"Explicit actuator '{prim.GetPath()}' must target exactly one joint; got {targets}.")
        target = targets[0]
        neural_controllers = schemas & {"NewtonNeuralControlAPI"}
        if not (schemas & {"NewtonPDControlAPI", "NewtonPIDControlAPI"}) or neural_controllers:
            found = sorted(schemas & controller_schemas) or ["no supported control schema"]
            raise ValueError(
                f"Explicit actuator '{prim.GetPath()}' targets '{target}' but is not a mutable PD/PID actuator "
                f"({', '.join(found)})."
            )
        if {"NewtonPDControlAPI", "NewtonPIDControlAPI"}.issubset(schemas):
            raise ValueError(f"Explicit actuator '{prim.GetPath()}' applies both PD and PID controller schemas.")
        if schemas & unsupported_clamps:
            raise ValueError(
                f"Explicit actuator '{prim.GetPath()}' uses unsupported clamping schemas: "
                + ", ".join(sorted(schemas & unsupported_clamps))
            )
        if target in by_target:
            raise ValueError(
                f"Explicit actuators '{by_target[target].prim_path}' and '{prim.GetPath()}' both target '{target}'."
            )

        kp = _attr_float(prim, "newton:kp", 0.0)
        kd = _attr_float(prim, "newton:kd", 0.0)
        const_effort = _attr_float(prim, "newton:constEffort", 0.0)
        max_effort = (
            _attr_float(prim, "newton:maxEffort", float("inf"))
            if "NewtonMaxEffortClampingAPI" in schemas
            else float("inf")
        )
        controller_kind = "pid" if "NewtonPIDControlAPI" in schemas else "pd"
        if {"NewtonMaxEffortClampingAPI", "NewtonDCMotorClampingAPI"}.issubset(schemas):
            raise ValueError(f"Explicit actuator '{prim.GetPath()}' applies multiple effort-clamping schemas.")
        clamp_kind = "dc_motor" if "NewtonDCMotorClampingAPI" in schemas else "max_effort"
        delay_steps = _attr_int(prim, "newton:delaySteps", 0) if "NewtonActuatorDelayAPI" in schemas else 0
        if not math.isfinite(kp) or kp < 0.0:
            raise ValueError(f"{prim.GetPath()}.newton:kp must be finite and non-negative.")
        if not math.isfinite(kd) or kd < 0.0:
            raise ValueError(f"{prim.GetPath()}.newton:kd must be finite and non-negative.")
        if not math.isfinite(const_effort):
            raise ValueError(f"{prim.GetPath()}.newton:constEffort must be finite.")
        if delay_steps < 0:
            raise ValueError(f"{prim.GetPath()}.newton:delaySteps must be non-negative.")
        if max_effort < 0.0 or math.isnan(max_effort):
            raise ValueError(f"{prim.GetPath()}.newton:maxEffort must be non-negative.")
        integral_gain = _attr_float(prim, "newton:ki", 0.0)
        integral_max = _attr_float(prim, "newton:integralMax", float("inf"))
        max_motor_effort = _attr_float(prim, "newton:maxMotorEffort", float("inf"))
        saturation_effort = _attr_float(prim, "newton:saturationEffort", float("inf"))
        velocity_limit = _attr_float(prim, "newton:velocityLimit", float("inf"))
        if controller_kind == "pid":
            if not math.isfinite(integral_gain) or integral_gain < 0.0:
                raise ValueError(f"{prim.GetPath()}.newton:ki must be finite and non-negative.")
            if integral_max < 0.0 or math.isnan(integral_max):
                raise ValueError(f"{prim.GetPath()}.newton:integralMax must be non-negative.")
        if clamp_kind == "dc_motor":
            for name, value in (
                ("newton:maxMotorEffort", max_motor_effort),
                ("newton:saturationEffort", saturation_effort),
                ("newton:velocityLimit", velocity_limit),
            ):
                if value < 0.0 or math.isnan(value):
                    raise ValueError(f"{prim.GetPath()}.{name} must be non-negative.")

        by_target[target] = ExplicitPdActuatorDescriptor(
            prim_path=str(prim.GetPath()),
            target_path=target,
            dof_name=PurePosixPath(target).name,
            kp=kp,
            kd=kd,
            const_effort=const_effort,
            delay_steps=delay_steps,
            max_effort=max_effort,
            controller_kind=controller_kind,
            integral_gain=integral_gain,
            integral_max=integral_max,
            clamp_kind=clamp_kind,
            max_motor_effort=max_motor_effort,
            saturation_effort=saturation_effort,
            velocity_limit=velocity_limit,
        )

    if selected_paths is None:
        return sorted(by_target.values(), key=lambda item: item.target_path)

    ordered: list[ExplicitPdActuatorDescriptor] = []
    for path in selected_paths:
        descriptor = by_target.get(str(path))
        if descriptor is None:
            raise ValueError(f"Selected DOF '{path}' has no mutable Newton PD/PID actuator.")
        ordered.append(descriptor)
    return ordered


def validate_explicit_pd_stage(
    stage: Usd.Stage,
    articulation_root: str,
    *,
    selected_dof_paths: Iterable[str] | None = None,
) -> list[str]:
    """Return validation errors without constructing runtime objects.

    Args:
        stage: Stage containing the articulation.
        articulation_root: Root path of the articulation to validate.
        selected_dof_paths: Optional joint paths that must have actuators.

    Returns:
        User-facing validation errors, or an empty list when valid.
    """
    try:
        descriptors = discover_explicit_pd_actuators(
            stage,
            articulation_root,
            selected_dof_paths=selected_dof_paths,
        )
    except ValueError as exc:
        return [str(exc)]
    if not descriptors:
        return [f"No explicit Newton PD/PID actuators were found under '{articulation_root}'."]
    return []


def _load_runtime() -> Any:
    import warp as wp
    from isaacsim.core.experimental.actuators import (
        ActuatorConfig,
        ArticulationActuators,
    )
    from newton.actuators import (
        ClampingDCMotor,
        ClampingMaxEffort,
        Delay,
        DrivePD,
        DrivePID,
    )

    return SimpleNamespace(
        wp=wp,
        ActuatorConfig=ActuatorConfig,
        ArticulationActuators=ArticulationActuators,
        DrivePD=DrivePD,
        DrivePID=DrivePID,
        Delay=Delay,
        ClampingMaxEffort=ClampingMaxEffort,
        ClampingDCMotor=ClampingDCMotor,
    )


def _actuators_by_dof_index(manager: Any) -> dict[int, Any]:
    """Map articulation DOFs to actuators using the actuators' remapped indices."""
    n_all_dofs = manager.articulation.num_dofs
    by_dof_index: dict[int, Any] = {}
    for actuator in manager.actuators:
        actuator_dof_indices = {int(index) % n_all_dofs for index in actuator.indices.numpy().reshape(-1)}
        for dof_index in actuator_dof_indices:
            existing = by_dof_index.get(dof_index)
            if existing is not None and existing is not actuator:
                raise ValueError(f"Multiple runtime actuators control DOF index {dof_index}.")
            by_dof_index[dof_index] = actuator
    return by_dof_index


def _restore_manager_drive_gains(
    manager: Any,
    *,
    stiffness: np.ndarray | None,
    damping: np.ndarray | None,
    dof_indices: list[int] | None,
) -> None:
    """Restore live drive gains displaced by ``ArticulationActuators``.

    Args:
        manager: Actuator manager whose articulation will be restored.
        stiffness: Original stiffness values.
        damping: Original damping values.
        dof_indices: Original articulation DOF indices.
    """
    if stiffness is None or damping is None or dof_indices is None:
        return
    articulation = getattr(manager, "articulation", None)
    if articulation is None:
        return
    articulation.set_dof_gains(
        stiffnesses=np.ascontiguousarray(stiffness, dtype=np.float32),
        dampings=np.ascontiguousarray(damping, dtype=np.float32),
        dof_indices=list(dof_indices),
    )


class ExplicitPdActuatorCompat:
    """Run-local PD + Delay manager with in-place per-environment updates.

    Args:
        manager: Underlying experimental actuator manager.
        descriptors: USD-derived actuator descriptors.
        actuators: Runtime actuator objects in descriptor order.
        env_count: Number of rollout environments.
        physics_dt: Physics time step in seconds.
        max_delay_steps: Allocated delay-buffer capacity.
        restore_stiffness: Implicit stiffness values to restore at close.
        restore_damping: Implicit damping values to restore at close.
        restore_dof_indices: DOF indices whose gains must be restored.
        promotions: Temporary session-layer actuator promotions.
        stage: Stage containing temporary promotions.
    """

    def __init__(
        self,
        *,
        manager: Any,
        descriptors: list[ExplicitPdActuatorDescriptor],
        actuators: list[Any],
        env_count: int,
        physics_dt: float,
        max_delay_steps: int,
        restore_stiffness: np.ndarray | None = None,
        restore_damping: np.ndarray | None = None,
        restore_dof_indices: list[int] | None = None,
        promotions: list[TemporaryActuatorPromotion] | None = None,
        stage: Usd.Stage | None = None,
    ) -> None:
        self._manager = manager
        self.descriptors = list(descriptors)
        self._actuators = list(actuators)
        self.env_count = int(env_count)
        self.physics_dt = float(physics_dt)
        self.max_delay_steps = int(max_delay_steps)
        self._restore_stiffness = (
            None if restore_stiffness is None else np.asarray(restore_stiffness, dtype=np.float32).copy()
        )
        self._restore_damping = (
            None if restore_damping is None else np.asarray(restore_damping, dtype=np.float32).copy()
        )
        self._restore_dof_indices = None if restore_dof_indices is None else list(restore_dof_indices)
        self._promotions = list(promotions or [])
        self._stage = stage
        self._promotions_committed = False
        self._closed = False
        self._baseline_kp = np.asarray([item.kp for item in descriptors], dtype=np.float32)
        self._baseline_kd = np.asarray([item.kd for item in descriptors], dtype=np.float32)
        self._baseline_delay_steps = np.asarray([item.delay_steps for item in descriptors], dtype=np.int32)
        self._baseline_ki = np.asarray([item.integral_gain for item in descriptors], dtype=np.float32)

    @classmethod
    def create(
        cls,
        *,
        stage: Usd.Stage,
        articulation_root: str,
        articulation_paths: str,
        selected_dof_paths: list[str],
        env_count: int,
        physics_dt: float,
        max_delay_seconds: float,
        device: str | None = None,
        runtime: Any | None = None,
        restore_stiffness: np.ndarray | None = None,
        restore_damping: np.ndarray | None = None,
        restore_dof_indices: list[int] | None = None,
        promotions: list[TemporaryActuatorPromotion] | None = None,
    ) -> "ExplicitPdActuatorCompat":
        """Create an explicit-actuator runtime from selected USD joints.

        Args:
            stage: Stage containing the articulation.
            articulation_root: Root path used for actuator discovery.
            articulation_paths: Runtime expression matching cloned articulations.
            selected_dof_paths: Ordered joint paths under explicit control.
            env_count: Number of rollout environments.
            physics_dt: Physics time step in seconds.
            max_delay_seconds: Largest candidate command delay in seconds.
            device: Optional Warp device identifier.
            runtime: Optional injected actuator runtime.
            restore_stiffness: Implicit stiffness values to restore at close.
            restore_damping: Implicit damping values to restore at close.
            restore_dof_indices: DOF indices whose gains must be restored.
            promotions: Temporary session-layer actuator promotions.

        Returns:
            The initialized compatibility manager.
        """
        runtime = runtime or _load_runtime()
        descriptors = discover_explicit_pd_actuators(
            stage,
            articulation_root,
            selected_dof_paths=selected_dof_paths,
        )
        if physics_dt <= 0.0 or not math.isfinite(float(physics_dt)):
            raise ValueError("physics_dt must be positive for explicit actuator delay quantization.")
        requested_capacity = int(math.ceil(max(float(max_delay_seconds), 0.0) / float(physics_dt) - 1e-12))
        authored_capacity = max((item.delay_steps for item in descriptors), default=0)
        max_delay_steps = max(1, requested_capacity, authored_capacity)

        wp = runtime.wp
        configs: list[tuple[Any, str]] = []
        for item in descriptors:
            controller_kwargs = {
                "kp": wp.array([item.kp] * env_count, dtype=wp.float32, device=device),
                "kd": wp.array([item.kd] * env_count, dtype=wp.float32, device=device),
                "const_effort": wp.array([item.const_effort] * env_count, dtype=wp.float32, device=device),
            }
            if item.controller_kind == "pid":
                controller = runtime.DrivePID(
                    **controller_kwargs,
                    ki=wp.array([item.integral_gain] * env_count, dtype=wp.float32, device=device),
                    integral_max=wp.array([item.integral_max] * env_count, dtype=wp.float32, device=device),
                )
            else:
                controller = runtime.DrivePD(**controller_kwargs)
            delay = runtime.Delay(
                delay_steps=wp.array([item.delay_steps] * env_count, dtype=wp.int32, device=device),
                max_delay=max_delay_steps,
            )
            clamping = []
            if item.clamp_kind == "dc_motor":
                clamping.append(
                    runtime.ClampingDCMotor(
                        saturation_effort=wp.array(
                            [item.saturation_effort] * env_count, dtype=wp.float32, device=device
                        ),
                        velocity_limit=wp.array([item.velocity_limit] * env_count, dtype=wp.float32, device=device),
                        max_motor_effort=wp.array([item.max_motor_effort] * env_count, dtype=wp.float32, device=device),
                    )
                )
            elif math.isfinite(item.max_effort):
                clamping.append(
                    runtime.ClampingMaxEffort(
                        max_effort=wp.array([item.max_effort] * env_count, dtype=wp.float32, device=device)
                    )
                )
            configs.append(
                (runtime.ActuatorConfig(controller=controller, clamping=clamping, delay=delay), item.dof_name)
            )

        manager = runtime.ArticulationActuators.from_actuators(
            articulation_paths,
            configs,
            device=device,
            auto_step_pre_physics=True,
        )
        ordered_actuators: list[Any] = []
        try:
            # Actuators retain config order; `actuated_dof_indices` is sorted independently.
            by_dof_index = _actuators_by_dof_index(manager)
            for item in descriptors:
                indices = manager.articulation.get_dof_indices(item.dof_name).numpy().reshape(-1)
                if indices.size != 1:
                    raise ValueError(f"DOF name '{item.dof_name}' did not resolve uniquely in the articulation.")
                actuator = by_dof_index.get(int(indices[0]))
                if actuator is None:
                    raise ValueError(f"No runtime actuator was constructed for DOF '{item.dof_name}'.")
                ordered_actuators.append(actuator)
        except Exception:
            manager.close()
            _restore_manager_drive_gains(
                manager,
                stiffness=restore_stiffness,
                damping=restore_damping,
                dof_indices=restore_dof_indices,
            )
            raise

        return cls(
            manager=manager,
            descriptors=descriptors,
            actuators=ordered_actuators,
            env_count=env_count,
            physics_dt=physics_dt,
            max_delay_steps=max_delay_steps,
            restore_stiffness=restore_stiffness,
            restore_damping=restore_damping,
            restore_dof_indices=restore_dof_indices,
            promotions=promotions,
            stage=stage,
        )

    @property
    def baseline_kp(self) -> np.ndarray:
        """Return baseline proportional gains for every environment."""
        return np.tile(self._baseline_kp, (self.env_count, 1))

    @property
    def baseline_kd(self) -> np.ndarray:
        """Return baseline derivative gains for every environment."""
        return np.tile(self._baseline_kd, (self.env_count, 1))

    @property
    def baseline_delay_steps(self) -> np.ndarray:
        """Return baseline command delays for every environment."""
        return np.tile(self._baseline_delay_steps, (self.env_count, 1))

    @property
    def baseline_ki(self) -> np.ndarray:
        """Return baseline integral gains for every environment."""
        return np.tile(self._baseline_ki, (self.env_count, 1))

    def apply_candidate_batch(
        self,
        *,
        stiffness_scale: np.ndarray,
        damping_scale: np.ndarray,
        delay_seconds: np.ndarray | None,
        integral_gain: np.ndarray | None = None,
    ) -> None:
        """Assign candidate values into existing controller and delay arrays.

        Args:
            stiffness_scale: Per-environment proportional-gain scales.
            damping_scale: Per-environment derivative-gain scales.
            delay_seconds: Per-environment physical command delays.
            integral_gain: Optional per-environment integral gains.
        """
        stiffness_scale = np.asarray(stiffness_scale, dtype=np.float32)
        damping_scale = np.asarray(damping_scale, dtype=np.float32)
        expected = (self.env_count, len(self.descriptors))
        if stiffness_scale.shape != expected or damping_scale.shape != expected:
            raise ValueError(
                f"Explicit actuator candidate shapes must be {expected}; got "
                f"{stiffness_scale.shape} and {damping_scale.shape}."
            )
        kp = self.baseline_kp * np.maximum(stiffness_scale, 0.0)
        kd = self.baseline_kd * np.maximum(damping_scale, 0.0)

        if delay_seconds is None:
            delay_steps = self.baseline_delay_steps
        else:
            seconds = np.asarray(delay_seconds, dtype=np.float32)
            if seconds.shape != expected:
                raise ValueError(f"Explicit actuator delay shape must be {expected}; got {seconds.shape}.")
            baseline_seconds = self.baseline_delay_steps.astype(np.float32) * self.physics_dt
            seconds = np.where(np.isfinite(seconds), seconds, baseline_seconds)
            seconds = np.maximum(seconds, 0.0)
            delay_steps = np.rint(seconds / self.physics_dt).astype(np.int32)
        if np.any(delay_steps > self.max_delay_steps):
            requested = int(np.max(delay_steps))
            raise ValueError(
                f"Candidate actuator delay needs {requested} steps, but the run allocated {self.max_delay_steps}."
            )

        for column, actuator in enumerate(self._actuators):
            actuator.drive.kp.assign(np.ascontiguousarray(kp[:, column], dtype=np.float32))
            actuator.drive.kd.assign(np.ascontiguousarray(kd[:, column], dtype=np.float32))
            if integral_gain is not None:
                values = np.asarray(integral_gain, dtype=np.float32)
                if values.shape != expected:
                    raise ValueError(f"Explicit actuator integral-gain shape must be {expected}; got {values.shape}.")
                controller_ki = getattr(actuator.drive, "ki", None)
                if controller_ki is None:
                    if np.any(np.abs(values[:, column]) > 0.0):
                        raise ValueError(f"DOF '{self.descriptors[column].dof_name}' is PD and cannot accept ki.")
                else:
                    controller_ki.assign(np.ascontiguousarray(np.maximum(values[:, column], 0.0)))
            actuator.delay.delay_steps.assign(np.ascontiguousarray(delay_steps[:, column], dtype=np.int32))
        self.reset()

    def reset(self) -> None:
        """Reset stateful actuator components."""
        reset = getattr(self._manager, "reset", None)
        if callable(reset):
            reset()

    def close(self) -> None:
        """Release runtime resources and restore implicit drive gains."""
        if self._closed:
            return
        self._closed = True
        close = getattr(self._manager, "close", None)
        try:
            if callable(close):
                close()
        finally:
            _restore_manager_drive_gains(
                self._manager,
                stiffness=self._restore_stiffness,
                damping=self._restore_damping,
                dof_indices=self._restore_dof_indices,
            )
            if self._stage is not None and self._promotions and not self._promotions_committed:
                discard_session_actuator_promotions(self._stage, self._promotions)

    def commit_promotions(self, target_layer: Sdf.Layer | None = None) -> bool:
        """Persist run-local promoted actuator topology after acceptance.

        Args:
            target_layer: Optional persistent destination layer.

        Returns:
            Whether the promoted topology was persisted successfully.
        """
        if not self._promotions:
            return True
        if self._stage is None:
            return False
        committed = commit_session_actuator_promotions(
            self._stage,
            self._promotions,
            target_layer=target_layer,
        )
        self._promotions_committed = bool(committed)
        if committed:
            discard_session_actuator_promotions(self._stage, self._promotions)
        return bool(committed)

    def write_parameters_to_usd(
        self,
        stage: Usd.Stage,
        *,
        kp: np.ndarray | None = None,
        kd: np.ndarray | None = None,
        ki: np.ndarray | None = None,
        delay_seconds: np.ndarray | None = None,
    ) -> bool:
        """Persist accepted PD gains and delay to their actuator prims.

        Args:
            stage: Stage containing the actuator prims.
            kp: Optional proportional gains in descriptor order.
            kd: Optional derivative gains in descriptor order.
            ki: Optional integral gains in descriptor order.
            delay_seconds: Optional command delays in descriptor order.

        Returns:
            Whether every supplied value was validated and authored.
        """
        from pxr import Sdf

        # `commit_session_actuator_promotions` copies layer-to-layer, so an
        # enclosing `Usd.EditContext` does not steer it. Resolve the active
        # edit target explicitly, otherwise multi-physics runs author the gains
        # into the solver layer while the actuator prims they target land in the
        # root layer and are lost if that layer is not shipped.
        if not self.commit_promotions(target_layer=_active_edit_layer(stage)):
            return False

        kp_values = None if kp is None else np.asarray(kp, dtype=np.float64).reshape(-1)
        kd_values = None if kd is None else np.asarray(kd, dtype=np.float64).reshape(-1)
        ki_values = None if ki is None else np.asarray(ki, dtype=np.float64).reshape(-1)
        delay_values = None if delay_seconds is None else np.asarray(delay_seconds, dtype=np.float64).reshape(-1)
        if kp_values is not None and kp_values.size != len(self.descriptors):
            return False
        if kd_values is not None and kd_values.size != len(self.descriptors):
            return False
        if ki_values is not None and ki_values.size != len(self.descriptors):
            return False
        if delay_values is not None and delay_values.size != len(self.descriptors):
            return False
        for values in (kp_values, kd_values, ki_values, delay_values):
            if values is not None and not np.all(np.isfinite(values)):
                return False

        for index, item in enumerate(self.descriptors):
            prim = stage.GetPrimAtPath(item.prim_path)
            if not prim or not prim.IsValid():
                return False
            if kp_values is not None:
                _set_attr(prim, "newton:kp", Sdf.ValueTypeNames.Float, max(float(kp_values[index]), 0.0))
            if kd_values is not None:
                _set_attr(prim, "newton:kd", Sdf.ValueTypeNames.Float, max(float(kd_values[index]), 0.0))
            if ki_values is not None:
                if item.controller_kind != "pid":
                    return False
                _set_attr(prim, "newton:ki", Sdf.ValueTypeNames.Float, max(float(ki_values[index]), 0.0))
            if delay_values is not None:
                prim.AddAppliedSchema("NewtonActuatorDelayAPI")
                steps = max(0, int(round(float(delay_values[index]) / self.physics_dt)))
                _set_attr(prim, "newton:delaySteps", Sdf.ValueTypeNames.Int, steps)
        return True

    @property
    def metadata(self) -> dict[str, Any]:
        """Return provenance describing the active actuator integration."""
        return {
            "integration": ACTUATOR_RUNTIME_EXPLICIT,
            "physics_dt": self.physics_dt,
            "max_delay_steps": self.max_delay_steps,
            "control_ownership": "explicit_selected_dofs_only",
            "temporary_promotions": [item.prim_path for item in self._promotions],
            "actuators": [
                {
                    "prim_path": item.prim_path,
                    "target_path": item.target_path,
                    "kp": item.kp,
                    "kd": item.kd,
                    "delay_steps": item.delay_steps,
                    "controller": item.controller_kind,
                    "clamp": item.clamp_kind,
                }
                for item in self.descriptors
            ],
        }


def _set_attr(prim: Usd.Prim, name: str, value_type: Any, value: Any) -> None:
    attr = prim.GetAttribute(name)
    if attr is None or not attr.IsValid():
        attr = prim.CreateAttribute(name, value_type, custom=True)
    attr.Set(value)
