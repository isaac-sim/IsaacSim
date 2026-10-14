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

"""Runtime wrapper that drives an `Articulation` with USD-authored or Python-built Newton actuators."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import carb
import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from pxr import Usd

_RUNTIME_LOADED = False

if TYPE_CHECKING:
    import warp as wp
    from isaacsim.core.experimental.prims import Articulation
    from isaacsim.core.simulation_manager import IsaacEvents, SimulationEvent, SimulationManager
    from newton import Model, ModelBuilder
    from newton.actuators import Actuator, ActuatorParsed, ClampingBase, Delay, DriveBase, parse_actuator_prim

    # When type checking is on, we have already imported everything.
    _RUNTIME_LOADED = True


def _ensure_runtime() -> None:
    """Import Newton, Warp, and Articulation dependencies on first use (cached)."""
    global _RUNTIME_LOADED
    global wp, Model, ModelBuilder, Actuator, ClampingBase, Delay, DriveBase, parse_actuator_prim
    global Articulation, IsaacEvents, SimulationEvent, SimulationManager

    if _RUNTIME_LOADED:
        return

    import warp as wp
    from isaacsim.core.experimental.prims import Articulation
    from isaacsim.core.simulation_manager import IsaacEvents, SimulationEvent, SimulationManager
    from newton import Model, ModelBuilder
    from newton.actuators import Actuator, ClampingBase, Delay, DriveBase, parse_actuator_prim

    _RUNTIME_LOADED = True


@dataclass
class _SimStateAdapter:
    """Articulation state, flattened to the layout Newton actuators expect."""

    joint_q: wp.array[float]
    joint_qd: wp.array[float]


@dataclass
class _SimControlAdapter:
    """Control targets + scatter-add effort buffer in the layout Newton actuators expect."""

    joint_target_q: wp.array[float]
    joint_target_qd: wp.array[float]
    joint_control_feedforward: wp.array[float]
    joint_f: wp.array[float]


def _gather_into(dst: wp.array, src: wp.array) -> wp.array:
    """Reuse ``dst`` for cross-device transfers; return ``src`` directly when same-device.

    Replaces the idiomatic ``src.to(dst.device)`` pattern, which falls back to
    ``warp.clone`` (alloc + copy) on a device mismatch.  ``wp.copy(dst, src)`` reuses
    ``dst`` so the steady-state path is allocation-free regardless of device pairing.

    ``src`` must already match ``dst``'s shape and dtype.

    Args:
        dst: Persistent destination buffer on the desired device.
        src: Source array to read from this step.

    Returns:
        ``src`` unchanged if its device matches ``dst.device``; otherwise ``dst``,
        populated via ``wp.copy``.
    """
    _ensure_runtime()
    if src.device == dst.device:
        return src
    wp.copy(dst, src)
    return dst


def _row_col_to_flat_index(
    n_rows: int,
    n_cols: int,
    columns: list[int],
    device: wp.DeviceLike,
) -> wp.array:
    """Build flattened row-major indices for selected columns across all rows.

    Args:
        n_rows: Number of rows in the flattened matrix.
        n_cols: Number of columns in each complete row.
        columns: Column indices to select, in the order expected by the actuator components.
        device: Target Warp device for the returned array.

    Returns:
        Robot-major indices with every selected column for row zero, then row one, and so on.
    """
    _ensure_runtime()
    row_offsets = np.arange(n_rows, dtype=np.uint32)[:, np.newaxis] * n_cols
    selected_columns = np.asarray(columns, dtype=np.uint32)[np.newaxis, :]
    return wp.array((row_offsets + selected_columns).reshape(-1), dtype=wp.uint32, device=device)


def _add_actuator_from_spec(builder: ModelBuilder, spec: ActuatorParsed, *, index: int) -> None:
    """Add a parsed actuator without silently discarding component semantics.

    Preserve clamping subclasses, their parameters, and their execution order.
    Newton's builder accepts only the built-in delay through ``delay_steps``;
    it cannot preserve a custom delay class or multiple delay stages.

    Args:
        builder: Newton builder that owns component grouping and construction.
        spec: Specification returned by Newton's actuator parser.
        index: Target DOF index in the articulation state layout.

    Raises:
        TypeError: If a component cannot be represented by Newton's builder API.
        ValueError: If delay parameters are invalid or multiple delays are supplied.
    """
    from newton.actuators import ClampingBase, Delay

    delay_steps = None
    clamping = []
    for component_class, parameters in spec.component_specs:
        if issubclass(component_class, ClampingBase):
            clamping.append((component_class, parameters))
        elif component_class is Delay:
            if delay_steps is not None:
                raise ValueError(f"Actuator targeting '{spec.target_path}' has multiple delay stages.")
            resolved = Delay.resolve_arguments(parameters)
            unsupported = parameters.keys() - resolved.keys()
            if unsupported:
                raise ValueError(
                    f"Actuator targeting '{spec.target_path}' has unsupported delay parameters: {sorted(unsupported)}."
                )
            delay_steps = resolved["delay_steps"]
        else:
            raise TypeError(
                f"Actuator targeting '{spec.target_path}' uses {component_class.__name__}, "
                "which Newton ModelBuilder.add_actuator cannot represent."
            )

    builder.add_actuator(
        drive_class=spec.drive_class,
        index=index,
        clamping=clamping or None,
        delay_steps=delay_steps,
        **spec.drive_kwargs,
    )


@dataclass
class ActuatorConfig:
    """Component bundle passed to `ArticulationActuators.from_actuators`.

    Holds the Newton drive, optional clamping stages, and an optional delay.
    The per-motor warp arrays inside each component must be sized for
    ``n_robots * n_target_dofs``, where ``n_target_dofs`` is one for a scalar DOF
    name or the length of the target DOF list. ``n_robots`` can be obtained from
    ``len(Articulation(paths))`` before constructing the config. The
    ``ArticulationActuators`` builds the ``Actuator`` wrapper, including index
    arrays, internally.

    Args:
        controller: Any Newton ``DriveBase`` subclass with pre-built warp arrays.
        clamping: Ordered list of ``ClampingBase`` stages applied after the drive.
        delay: Optional ``Delay`` applied to input targets before the drive.

    Raises:
        TypeError: If a component does not have the required Newton actuator type.

    Example:

    .. code-block:: python

        >>> import warp as wp
        >>> from isaacsim.core.experimental.actuators import ActuatorConfig
        >>> from newton.actuators import DrivePD

        >>> cfg = ActuatorConfig(
        ...     controller=DrivePD(
        ...         kp=wp.array([100.0], dtype=wp.float32),
        ...         kd=wp.array([10.0], dtype=wp.float32),
        ...     )
        ... )
    """

    controller: DriveBase
    clamping: list[ClampingBase] = field(default_factory=list)
    delay: Delay | None = None

    def __post_init__(self) -> None:
        """Validate actuator component types."""
        _ensure_runtime()
        if not isinstance(self.controller, DriveBase):
            controller_type_name = type(self.controller).__name__
            raise TypeError(f"Actuator controller must be a Newton DriveBase, got {controller_type_name}.")
        if not isinstance(self.clamping, list):
            raise TypeError(f"Actuator clamping must be a list, got {type(self.clamping).__name__}.")
        for clamp in self.clamping:
            if not isinstance(clamp, ClampingBase):
                raise TypeError(
                    f"Actuator clamping entries must be Newton ClampingBase objects, got {type(clamp).__name__}."
                )
        if self.delay is not None and not isinstance(self.delay, Delay):
            raise TypeError(f"Actuator delay must be a Newton Delay, got {type(self.delay).__name__}.")


class ArticulationActuators:
    """Newton actuator manager for an `Articulation` that computes and applies joint efforts each step.

    Parses the `NewtonActuator` prims of the first matched articulation, groups them
    into Newton actuator batches replicated across all N robot instances, zeros
    `UsdPhysics.DriveAPI` gains on every actuated joint, and on each physics pre-step
    reads articulation state and control targets, steps every actuator, and writes the
    resulting efforts back to the articulation. For Python-built actuators, meaning
    those supplied through `ArticulationActuators.from_actuators`, each user-provided
    ``(ActuatorConfig, dof_names)`` group becomes one Newton ``Actuator``.

    All N instances share identical actuator topology and parameters (the
    prototype-reference pattern). Per-instance USD overrides are not read.

    The pre-physics callback is enabled at construction unless ``auto_step_pre_physics=False``.
    Disabling it (at construction or via `disable_auto_step_pre_physics()`) prevents unintended
    last-writer-wins when multiple `ArticulationActuators` objects target overlapping DOFs.

    Use `articulation` to access the underlying `Articulation` for setting position/velocity
    targets, reading state, or modifying drive gains.

    Lifetime
    --------
    Each instance registers callbacks with the process-wide `SimulationManager`. Tear instances
    down explicitly via `close()` or by using them as a context manager (``with ArticulationActuators(...)
    as actuated: ...``). A `__del__` fallback also calls `close()` on garbage collection, but
    ordering is non-deterministic so it must not be relied on.

    Args:
        paths: Single path or list of paths to articulation root prims. May include regex
            matching multiple instances.
        auto_step_pre_physics: When true, register the pre-physics callback immediately so
            actuators are stepped automatically on every physics tick.
        device: Warp device on which to allocate per-actuator scratch buffers. When ``None``,
            use Warp's default device.
        _skip_discovery: Internal flag used by `from_actuators` to bypass USD discovery so
            actuators can be supplied directly from Python.

    Raises:
        ValueError: If a USD actuator target is not a DOF of this articulation,
            or if two USD actuators resolve to the same DOF index.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.actuators import ArticulationActuators

        >>> with ArticulationActuators("/World/Robot") as actuated:  # doctest: +NO_CHECK
        ...     actuated.step_actuators(step_dt=1.0 / 60.0)
    """

    @classmethod
    def from_actuators(
        cls,
        paths: str | list[str],
        actuators: list[tuple[ActuatorConfig, str | list[str]]],
        *,
        device: wp.DeviceLike = None,
        auto_step_pre_physics: bool = True,
    ) -> ArticulationActuators:
        """Construct an `ArticulationActuators` entirely from Python-built `ActuatorConfig` objects.

        No USD scanning is performed.  This is the escape hatch for cases where USD
        authoring is unavailable or impractical — the caller supplies the complete set
        of actuator configurations and their target DOF groups.

        Each entry becomes exactly one Newton ``Actuator``. Every per-motor Warp
        array inside its ``ActuatorConfig`` components must contain
        ``n_robots * n_target_dofs`` values in robot-major order: all named DOFs
        for robot zero, then all named DOFs for robot one, and so on. Component
        parameters declared as shared apply to the whole group and are not repeated
        per motor.

        `actuated_dof_indices` is always sorted by DOF index. `actuators` follows the
        supplied group order and is not positionally aligned with that DOF list. Within
        a multi-DOF group, name order determines the component-array value order.

        Args:
            paths: Single path or list of paths to articulation root prims.
            actuators: List of ``(config, dof_names)`` pairs, where ``dof_names`` is
                either one DOF name or a list of DOF names. Each name must match an
                entry in `Articulation.dof_names` (e.g. ``"shoulder_joint"``). One
                pair produces one vectorized Newton ``Actuator`` for the entire group.
            device: Warp device on which to allocate per-actuator scratch buffers. When ``None``,
                use Warp's default device.
            auto_step_pre_physics: Forwarded to the underlying ``__init__``.

        Returns:
            A fully initialised ``ArticulationActuators`` driven by the provided configs.

        Raises:
            TypeError: If a target is neither a string nor a ``list[str]``.
            ValueError: If a target group is empty, a DOF name is not found, or a DOF
                appears more than once in ``actuators``.

        Example:

        .. code-block:: python

            >>> import warp as wp
            >>> from isaacsim.core.experimental.actuators import (
            ...     ActuatorConfig,
            ...     ArticulationActuators,
            ... )
            >>> from newton.actuators import DrivePD

            >>> cfg = ActuatorConfig(
            ...     controller=DrivePD(
            ...         kp=wp.array([100.0, 200.0], dtype=wp.float32),
            ...         kd=wp.array([10.0, 20.0], dtype=wp.float32),
            ...     )
            ... )
            >>> actuated = ArticulationActuators.from_actuators(  # doctest: +NO_CHECK
            ...     "/World/Robot",
            ...     [(cfg, ["shoulder_joint", "elbow_joint"])],
            ... )
            >>> len(actuated.actuators)
            1
            >>> actuated.actuators[0].num_actuators
            2
        """
        instance = cls(paths, auto_step_pre_physics=auto_step_pre_physics, device=device, _skip_discovery=True)

        if not actuators:
            return instance

        n_robots = len(instance._articulation)
        n_all_dofs = instance._articulation.num_dofs
        groups: list[tuple[ActuatorConfig, list[int]]] = []
        seen: set[int] = set()
        for config, dof_names in actuators:
            if not isinstance(config, ActuatorConfig):
                raise TypeError(f"Expected ActuatorConfig, got {type(config).__name__}.")
            if isinstance(dof_names, str):
                dof_names = [dof_names]
            elif not isinstance(dof_names, list):
                target_type_name = type(dof_names).__name__
                raise TypeError(
                    f"Actuator target names must be a string or list[str], got {target_type_name}.",
                )
            if not all(isinstance(dof_name, str) for dof_name in dof_names):
                raise TypeError("Actuator target names must all be strings.")
            if not dof_names:
                raise ValueError("Actuator target name groups must not be empty.")

            group_dof_indices: list[int] = []
            for dof_name in dof_names:
                if dof_name not in instance._articulation.dof_names:
                    raise ValueError(
                        f"DOF name {dof_name!r} not found in articulation "
                        f"'{instance._articulation.paths[0]}'. "
                        f"Available DOF names: {instance._articulation.dof_names}."
                    )
                dof_index = int(instance._articulation.get_dof_indices(dof_name).numpy()[0])
                if dof_index in seen:
                    raise ValueError(
                        f"DOF {dof_name!r} (index {dof_index}) appears more than once in `actuators`.",
                    )
                seen.add(dof_index)
                group_dof_indices.append(dof_index)
            groups.append((config, group_dof_indices))

        n_actuated_dofs, dof_index_to_actuated = instance._setup_actuated_dof_buffers(sorted(seen), n_robots)

        for config, group_dof_indices in groups:
            sim_state_indices = _row_col_to_flat_index(n_robots, n_all_dofs, group_dof_indices, device)
            effort_columns = [dof_index_to_actuated[dof_index] for dof_index in group_dof_indices]
            effort_indices = _row_col_to_flat_index(n_robots, n_actuated_dofs, effort_columns, device)
            # Assume Newton validates per-motor component array lengths during actuator construction.
            actuator = Actuator(
                indices=sim_state_indices,
                pos_indices=sim_state_indices,
                target_pos_indices=sim_state_indices,
                drive=config.controller,
                delay=config.delay,
                clamping=config.clamping or None,
                effort_indices=effort_indices,
                control_target_pos_attr="joint_target_q",
                control_target_vel_attr="joint_target_qd",
                control_feedforward_attr="joint_control_feedforward",
            )
            instance._actuators.append(actuator)
            state = actuator.state()
            instance._cur_states.append(state)
            instance._nxt_states.append(actuator.state() if state is not None else None)

        # If physics is already up, apply physics-ready setup now that actuators are
        # populated so drive gains are zeroed and the callback is registered if opted in.
        if SimulationManager.get_physics_simulation_view() is not None:
            instance._on_physics_ready(None)

        return instance

    def __init__(
        self,
        paths: str | list[str],
        *,
        auto_step_pre_physics: bool = True,
        device: wp.DeviceLike = None,
        _skip_discovery: bool = False,
    ) -> None:
        _ensure_runtime()
        self._articulation = Articulation(paths)
        self._device = device

        n_robots = len(self._articulation)
        n_all_dofs = self._articulation.num_dofs
        flat_dof_size = n_robots * n_all_dofs

        # CPU-side authoritative buffer for target feedforward efforts.
        # Kept on CPU so `set_dof_feedforward_effort_targets` can write through a numpy view.
        self._feedforward_target_efforts = wp.zeros(
            shape=[n_robots, n_all_dofs],
            dtype=wp.float32,
            device="cpu",
        )

        # Persistent device-side mirror of `_feedforward_target_efforts` and persistent
        # destination buffers for the articulation state/control snapshots consumed by
        # `step_actuators`.  These exist so the per-step path is allocation-free on every
        # device pairing: when `Articulation.get_dof_*()` already returns data on
        # `self._device`, `_gather_into` returns it directly; otherwise it `wp.copy`s into
        # the buffer below instead of falling back to `warp.clone`.
        self._joint_q_buffer = wp.empty(flat_dof_size, dtype=wp.float32, device=self._device)
        self._joint_qd_buffer = wp.empty(flat_dof_size, dtype=wp.float32, device=self._device)
        self._joint_target_q_buffer = wp.empty(flat_dof_size, dtype=wp.float32, device=self._device)
        self._joint_target_qd_buffer = wp.empty(flat_dof_size, dtype=wp.float32, device=self._device)
        self._joint_control_feedforward_buffer = wp.empty(flat_dof_size, dtype=wp.float32, device=self._device)

        #: Finalized Newton model that owns USD-built actuators. ``None`` for Python-built configs.
        self._actuator_model: Model | None = None

        #: Newton `Actuator` objects owned by this wrapper.
        self._actuators: list[Actuator] = []

        #: Sorted unique DOF indices covered by the owned actuators.
        self._actuated_dof_indices: list[int] = []

        #: Cached `wp.array` of `_actuated_dof_indices` for indexed `set_dof_efforts` calls.
        self._actuated_dof_indices_wp: wp.array | None = None
        self._cur_states: list[Actuator.State | None] = []
        self._nxt_states: list[Actuator.State | None] = []

        #: Compact `wp.array` of shape `(n_robots * num_actuated_dofs,)` scatter-add target for each step.
        self._effort_buffer: wp.array | None = None

        self._auto_step_pre_physics = auto_step_pre_physics
        self._actuator_callback_id: int | None = None

        # When physics is ready, this callback will ensure that we:
        #   a) zero the stiffness/damping of DriveAPI gains
        #   b) register our actuator callback _if_ `_auto_step_pre_physics` is True.
        # On timeline stop, we clear the live pre-physics callback handle but preserve
        # `_auto_step_pre_physics`, so the next PHYSICS_READY re-registers if opted in.
        self._lifecycle_callback_ids: list[int] = [
            SimulationManager.register_callback(self._on_physics_ready, event=IsaacEvents.PHYSICS_READY),
            SimulationManager.register_callback(self._on_timeline_stop, event=SimulationEvent.SIMULATION_STOPPED),
        ]

        if not _skip_discovery:
            self._discover_and_build_actuators()

            # If physics is already up at construction time, run the ready handler synchronously
            # so drive gains are zeroed immediately rather than waiting for the next start cycle.
            if SimulationManager.get_physics_simulation_view() is not None:
                self._on_physics_ready(None)

    def __del__(self) -> None:
        """Call `close()` during garbage collection as a best-effort fallback.

        Do not rely on this for teardown. Garbage-collection ordering is non-deterministic
        and `__del__` may not run before interpreter shutdown. Always tear the instance
        down explicitly via `close()` or by using it as a context manager (`with ...`).
        """
        try:
            self.close()
        except Exception:  # noqa: BLE001 - best-effort during GC
            pass

    def __enter__(self) -> ArticulationActuators:
        """Return self so the instance can be used as a context manager.

        Returns:
            This `ArticulationActuators` instance, unmodified.
        """
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        """Deregister all callbacks on exit from the context-managed block.

        Args:
            exc_type: Exception class raised inside the ``with`` body, or ``None``.
            exc: Exception instance raised inside the ``with`` body, or ``None``.
            tb: Traceback associated with ``exc``, or ``None``.
        """
        self.close()

    @property
    def articulation(self) -> Articulation:
        """Get the underlying `Articulation` wrapper.

        Returns:
            The `Articulation` driven by this instance.

        Example:

        .. code-block:: python

            >>> actuated.articulation  # doctest: +NO_CHECK
            <isaacsim.core.experimental.prims.Articulation object at ...>
        """
        return self._articulation

    @property
    def actuators(self) -> list[Actuator]:
        """Get the Newton `Actuator` instances owned by this wrapper.

        One actuator can control multiple DOFs, so this list is not positionally
        aligned with `actuated_dof_indices`.

        Returns:
            A shallow copy of the owned `Actuator` list.

        Example:

        .. code-block:: python

            >>> actuated.actuators  # doctest: +NO_CHECK
            [<newton.actuators.Actuator object at ...>, ...]
        """
        return list(self._actuators)

    @property
    def actuated_dof_indices(self) -> list[int]:
        """Get the sorted unique DOF indices covered by the owned actuators.

        This contains one entry per DOF, not one entry per `Actuator`.

        Returns:
            A copy of the actuated DOF index list, sorted ascending.

        Example:

        .. code-block:: python

            >>> actuated.actuated_dof_indices  # doctest: +NO_CHECK
            [0, 1, 2]
        """
        return list(self._actuated_dof_indices)

    def close(self) -> None:
        """Deregister all callbacks held by this instance. Safe to call multiple times.

        Prefer using `ArticulationActuators` as a context manager (``with ... as
        actuated:``) so `close()` is invoked automatically on block exit; call this
        method explicitly only when the wrapper's lifetime cannot be bounded by a
        ``with`` statement.

        Example:

        .. code-block:: python

            >>> actuated.close()  # doctest: +NO_CHECK
        """
        self.disable_auto_step_pre_physics()
        if not self._lifecycle_callback_ids:
            return
        _ensure_runtime()
        for cb_id in self._lifecycle_callback_ids:
            SimulationManager.deregister_callback(cb_id)
        self._lifecycle_callback_ids = []

    def set_dof_feedforward_effort_targets(
        self,
        target_feedforward_efforts: float | list | np.ndarray | wp.array,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
        dof_indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set per-DOF feedforward effort targets consumed by `step_actuators`.

        These values are passed through the actuator pipeline (controller + clamping)
        on the next `step_actuators` call.  They have no effect on joints that do not
        have a corresponding actuator.

        Args:
            target_feedforward_efforts: Feedforward efforts [N or N·m], shape ``(N, D)``
                or broadcastable.
            indices: Prim indices subset.
            dof_indices: DOF indices subset.

        Example:

        .. code-block:: python

            >>> import numpy as np

            >>> actuated.set_dof_feedforward_effort_targets(  # doctest: +NO_CHECK
            ...     np.array([5.0, -3.0]),
            ...     dof_indices=[0, 1],
            ... )
        """
        _ensure_runtime()
        indices = ops_utils.resolve_indices(indices, count=len(self._articulation), device="cpu")
        dof_indices = ops_utils.resolve_indices(dof_indices, count=self._articulation.num_dofs, device="cpu")
        target_feedforward_efforts = ops_utils.broadcast_to(
            target_feedforward_efforts,
            shape=(indices.shape[0], dof_indices.shape[0]),
            dtype=wp.float32,
            device="cpu",
        ).numpy()

        self._feedforward_target_efforts.numpy()[
            np.ix_(indices.numpy(), dof_indices.numpy())
        ] = target_feedforward_efforts

    def enable_auto_step_pre_physics(self) -> None:
        """Opt in to the pre-physics actuator callback. Persists across stop/start cycles.

        Example:

        .. code-block:: python

            >>> actuated.enable_auto_step_pre_physics()  # doctest: +NO_CHECK
        """
        self._auto_step_pre_physics = True
        self._register_actuator_callback()

    def disable_auto_step_pre_physics(self) -> None:
        """Opt out of the pre-physics actuator callback.

        Example:

        .. code-block:: python

            >>> actuated.disable_auto_step_pre_physics()  # doctest: +NO_CHECK
        """
        self._auto_step_pre_physics = False
        self._deregister_actuator_callback()

    def reset(self) -> None:
        """Zero all stateful actuator state (PID integrals, delay buffers).

        Called automatically on each `PHYSICS_READY` event and available for manual invocation.

        Example:

        .. code-block:: python

            >>> actuated.reset()  # doctest: +NO_CHECK
        """
        for cur, nxt in zip(self._cur_states, self._nxt_states):
            if cur is not None:
                cur.reset()
                nxt.reset()

    def step_actuators(self, step_dt: float, context: Any = None) -> None:
        """Compute and apply actuator efforts for one physics timestep.

        Args:
            step_dt: Physics timestep in seconds.
            context: Unused; accepted to satisfy the `SimulationManager` callback signature.

        Example:

        .. code-block:: python

            >>> actuated.step_actuators(step_dt=1.0 / 60.0)  # doctest: +NO_CHECK
        """
        del context  # Unused; required by `SimulationManager` callback signature.
        _ensure_runtime()
        if not self._actuators:
            return

        n_robots = len(self._articulation)
        n_all_dofs = self._articulation.num_dofs
        n_actuated_dofs = len(self._actuated_dof_indices)

        self._effort_buffer.zero_()  # zero before scatter-add

        # Push the CPU-staged feedforward targets into their persistent device-side mirror.
        wp.copy(
            self._joint_control_feedforward_buffer,
            self._feedforward_target_efforts.reshape(n_robots * n_all_dofs),
        )

        # State / control arrays retain the articulation's full `(n_robots, n_all_dofs)` layout; actuators
        # read from them via their full-stride `indices`.  `_gather_into` returns the
        # articulation's array directly when its device already matches `self._device`,
        # and otherwise `wp.copy`s into the persistent per-step buffers allocated in
        # `__init__` — never `warp.clone`s, so no per-step allocations on either path.
        sim_state = _SimStateAdapter(
            joint_q=_gather_into(
                self._joint_q_buffer,
                self._articulation.get_dof_positions().reshape(n_robots * n_all_dofs),
            ),
            joint_qd=_gather_into(
                self._joint_qd_buffer,
                self._articulation.get_dof_velocities().reshape(n_robots * n_all_dofs),
            ),
        )
        sim_control = _SimControlAdapter(
            joint_target_q=_gather_into(
                self._joint_target_q_buffer,
                self._articulation.get_dof_position_targets().reshape(n_robots * n_all_dofs),
            ),
            joint_target_qd=_gather_into(
                self._joint_target_qd_buffer,
                self._articulation.get_dof_velocity_targets().reshape(n_robots * n_all_dofs),
            ),
            joint_control_feedforward=self._joint_control_feedforward_buffer,
            # compact `(n_robots * n_actuated_dofs,)` buffer — actuators scatter here via `effort_indices`.
            # Already lives on `self._device` (allocated there in `_discover_and_build_actuators` /
            # `from_actuators`), so no transfer is needed.
            joint_f=self._effort_buffer,
        )

        for actuator, cur, nxt in zip(self._actuators, self._cur_states, self._nxt_states):
            actuator.step(sim_state, sim_control, cur, nxt, dt=step_dt)

        # Swap double-buffered state for the next step.
        self._cur_states, self._nxt_states = self._nxt_states, self._cur_states

        # Indexed write: only the actuated DOF slots are touched. Safe for concurrent
        # `ArticulationActuators` objects that own disjoint DOFs on the same robot.
        self._articulation.set_dof_efforts(
            self._effort_buffer.reshape([n_robots, n_actuated_dofs]),
            dof_indices=self._actuated_dof_indices_wp,
        )

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _setup_actuated_dof_buffers(self, dof_indices: list[int], n_robots: int) -> tuple[int, dict[int, int]]:
        """Store the actuated DOFs and allocate their compact effort buffer.

        Args:
            dof_indices: Articulation DOF indices controlled by this wrapper.
            n_robots: Number of matched articulation instances.

        Returns:
            The number of actuated DOFs and a mapping from articulation DOF index to compact buffer column.
        """
        self._actuated_dof_indices = sorted(dof_indices)
        n_actuated_dofs = len(self._actuated_dof_indices)
        dof_index_to_actuated = {
            dof_index: actuated_index for actuated_index, dof_index in enumerate(self._actuated_dof_indices)
        }
        self._effort_buffer = wp.zeros(n_robots * n_actuated_dofs, dtype=wp.float32, device=self._device)
        self._actuated_dof_indices_wp = wp.array(self._actuated_dof_indices, dtype=wp.int32, device=self._device)
        return n_actuated_dofs, dof_index_to_actuated

    def _discover_and_build_actuators(self) -> None:
        """Parse USD-authored actuators and group them through Newton's ``ModelBuilder``.

        Only `NewtonActuator` prims under the first matched articulation are read, and
        their targets are mapped directly to articulation DOF indices. The actuators are
        added to a template builder that holds no bodies or joints, replicated once per
        matched robot, and finalized so Newton groups compatible actuators into batches.
        """
        _ensure_runtime()

        n_robots = len(self._articulation)
        n_all_dofs = self._articulation.num_dofs

        stage = stage_utils.get_current_stage(backend="usd")
        root_path = self._articulation.paths[0]
        root_prim = stage.GetPrimAtPath(root_path)
        if not root_prim.IsValid():
            carb.log_warn(f"ArticulationActuators: root prim '{root_path}' is invalid; no actuators discovered")
            return

        dof_paths = self._articulation.dof_paths[0] if self._articulation.dof_paths else []
        dof_path_to_index = {dof_path: dof_index for dof_index, dof_path in enumerate(dof_paths)}

        # Parse the first robot's actuator prims into a template of articulation DOF indices.
        template_builder = ModelBuilder()
        dof_index_to_prim: dict[int, str] = {}
        for prim in Usd.PrimRange(root_prim):
            parsed = parse_actuator_prim(prim)
            if parsed is None:
                continue
            target_path = parsed.target_path

            prim_path = str(prim.GetPath())
            dof_index = dof_path_to_index.get(target_path)
            if dof_index is None:
                raise ValueError(
                    f"Actuator '{prim_path}' targets '{target_path}', which is not "
                    f"a DOF of articulation '{root_path}'."
                )
            if dof_index in dof_index_to_prim:
                raise ValueError(
                    f"Actuator '{prim_path}' targets '{target_path}', which is already "
                    f"targeted by '{dof_index_to_prim[dof_index]}'. Actuators support a 1-to-1 mapping "
                    "between actuators and joints."
                )
            dof_index_to_prim[dof_index] = prim_path

            _add_actuator_from_spec(template_builder, parsed, index=dof_index)

        if not dof_index_to_prim:
            return

        n_actuated_dofs, dof_index_to_actuated = self._setup_actuated_dof_buffers(list(dof_index_to_prim), n_robots)

        # The template holds actuators only, so `replicate` would offset every robot by its
        # zero joint count. Use the articulation DOF count as the per-robot stride instead,
        # so robot r's actuator indices land at `r * n_all_dofs + dof_index`.
        template_builder.joint_dof_count = n_all_dofs
        template_builder.joint_coord_count = n_all_dofs

        # Replicate the template and let Newton finalize its optimized actuator batches.
        builder = ModelBuilder()
        builder.replicate(template_builder, n_robots)
        self._actuator_model = builder.finalize(device=self._device)
        self._actuators = self._actuator_model.actuators

        dof_to_actuated_lookup = np.array(
            [dof_index_to_actuated.get(index, -1) for index in range(n_all_dofs)], dtype=np.int64
        )

        # Indices are already in the flattened articulation layout; map them to effort-buffer slots.
        for actuator in self._actuators:
            robot_indices, dof_indices = np.divmod(actuator.indices.numpy(), n_all_dofs)
            effort_indices = robot_indices * n_actuated_dofs + dof_to_actuated_lookup[dof_indices]
            actuator.effort_indices = wp.array(effort_indices, dtype=wp.uint32, device=actuator.device)
            actuator.control_feedforward_attr = "joint_control_feedforward"

            state = actuator.state()
            self._cur_states.append(state)
            self._nxt_states.append(actuator.state() if state is not None else None)

    def _on_physics_ready(self, event: object) -> None:
        """Zero drive gains on actuated joints, re-register the pre-physics callback if enabled, reset state.

        Args:
            event: Opaque event payload (unused).
        """
        del event
        if self._actuated_dof_indices:
            self._articulation.set_dof_gains(
                stiffnesses=0.0,
                dampings=0.0,
                dof_indices=self._actuated_dof_indices,
            )
        if self._auto_step_pre_physics:
            self._register_actuator_callback()
        self.reset()

    def _on_timeline_stop(self, event: object) -> None:
        """Clear the live pre-physics callback handle on timeline stop.

        `_auto_step_pre_physics` is preserved so the next `PHYSICS_READY`
        re-registers the callback if the user had previously opted in via
        `enable_auto_step_pre_physics()`.

        Args:
            event: Opaque event payload (unused).
        """
        del event
        self._deregister_actuator_callback()

    def _register_actuator_callback(self) -> None:
        """Register the pre-physics callback if not already registered."""
        if self._actuator_callback_id is not None:
            return
        _ensure_runtime()
        self._actuator_callback_id = SimulationManager.register_callback(
            self.step_actuators,
            event=SimulationEvent.PHYSICS_PRE_STEP,
        )

    def _deregister_actuator_callback(self) -> None:
        """Deregister the pre-physics callback if currently registered."""
        if self._actuator_callback_id is None:
            return
        SimulationManager.deregister_callback(self._actuator_callback_id)
        self._actuator_callback_id = None
