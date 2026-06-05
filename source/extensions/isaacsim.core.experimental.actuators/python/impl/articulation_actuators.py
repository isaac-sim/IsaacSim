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

"""Runtime wrapper that drives an `Articulation` via explicit Newton actuators parsed from USD."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import carb
import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import newton.actuators as na
import numpy as np
import warp as wp
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import IsaacEvents, SimulationEvent, SimulationManager
from newton.actuators import Actuator, ActuatorParsed, Clamping, Controller, Delay
from pxr import Usd

from .clamping_builders import build_clamping
from .controller_builders import build_controller
from .delay_builder import build_delay


@dataclass
class _SimStateAdapter:
    """Articulation state, flattened to the layout Newton actuators expect."""

    joint_q: wp.array[float]
    joint_qd: wp.array[float]


@dataclass
class _SimControlAdapter:
    """Control targets + scatter-add effort buffer in the layout Newton actuators expect."""

    joint_target_pos: wp.array[float]
    joint_target_vel: wp.array[float]
    joint_control_feedforward: wp.array[float]
    joint_f: wp.array[float]


def _row_major_indices(n_rows: int, stride: int, column: int, device: str | None) -> wp.array:
    """Build strided indices into a row-major ``(n_rows, stride)`` flat buffer.

    The ``i``-th element of the returned array is ``i * stride + column`` — i.e.
    the flat-buffer index of ``column`` in row ``i``.  Used to build per-actuator
    gather/scatter indices for the articulation's full
    ``(n_robots, n_all_dofs)`` state and the compact
    ``(n_robots, n_actuated_dofs)`` effort buffer.

    Args:
        n_rows: Number of rows in the flat buffer (``n_robots``).
        stride: Row stride (``n_all_dofs`` or ``n_actuated_dofs``).
        column: Column to pick from each row (a DOF index or actuated-DOF slot).
        device: Target ``warp`` device for the returned array.

    Returns:
        ``wp.array`` of shape ``(n_rows,)`` and dtype ``wp.uint32``.
    """
    return wp.array(
        np.arange(n_rows, dtype=np.uint32) * stride + column,
        dtype=wp.uint32,
        device=device,
    )


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
    if src.device == dst.device:
        return src
    wp.copy(dst, src)
    return dst


@dataclass
class ActuatorConfig:
    """Component bundle passed to `ArticulationActuators.from_actuators`.

    Holds the Newton controller, optional clamping stages, and an optional delay.
    The warp arrays inside each component must be sized for ``n_robots``, which
    can be obtained from ``len(Articulation(paths))`` before constructing this
    config.  The ``ArticulationActuators`` builds the ``Actuator`` wrapper
    (including index arrays) internally.

    Args:
        controller: Any Newton ``Controller`` subclass with pre-built warp arrays.
        clamping: Ordered list of ``Clamping`` stages applied after the controller.
        delay: Optional ``Delay`` applied to input targets before the controller.

    Example:

    .. code-block:: python

        >>> import warp as wp
        >>> from isaacsim.core.experimental.actuators import ActuatorConfig
        >>> from newton.actuators import ControllerPD

        >>> cfg = ActuatorConfig(
        ...     controller=ControllerPD(
        ...         kp=wp.array([100.0], dtype=wp.float32),
        ...         kd=wp.array([10.0], dtype=wp.float32),
        ...     )
        ... )
    """

    controller: Controller
    clamping: list[Clamping] = field(default_factory=list)
    delay: Delay | None = None


class ArticulationActuators:
    """Newton actuator manager for an `Articulation` that computes and applies joint efforts each step.

    Parses `NewtonActuator` prims from the USD subtree under the first matched articulation
    instance, constructs one `newton.actuators.Actuator` per prim (fanned out to all N robot
    instances via `indices`), zeros `UsdPhysics.DriveAPI` gains on every actuated joint, and
    on each physics pre-step reads articulation state + control targets, steps every actuator,
    and writes the resulting efforts back to the articulation.

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
            use the articulation's device.
        _skip_discovery: Internal flag used by `from_actuators` to bypass USD discovery so
            actuators can be supplied directly from Python.

    Raises:
        ValueError: If an `ActuatorParsed` target path is not a DOF of this articulation,
            or if two actuators resolve to the same DOF index.

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
        actuators: list[tuple[ActuatorConfig, str]],
        *,
        auto_step_pre_physics: bool = True,
    ) -> ArticulationActuators:
        """Construct an `ArticulationActuators` entirely from Python-built `ActuatorConfig` objects.

        No USD scanning is performed.  This is the escape hatch for cases where USD
        authoring is unavailable or impractical — the caller supplies the complete set
        of actuator configurations and their target DOF names.

        The warp arrays inside each ``ActuatorConfig``'s controller (e.g. ``kp``, ``kd``)
        must be sized for ``n_robots = len(Articulation(paths))``.  The ``Actuator``
        wrapper (including index arrays) is built internally by this method.

        The order of entries in ``actuators`` does not affect the result — the set is
        always stored sorted by DOF index.

        Args:
            paths: Single path or list of paths to articulation root prims.
            actuators: List of ``(config, dof_name)`` pairs.  ``dof_name`` is the
                last path segment of the target joint (e.g. ``"RevoluteJoint"``).
            auto_step_pre_physics: Forwarded to the underlying ``__init__``.

        Returns:
            A fully initialised ``ArticulationActuators`` driven by the provided configs.

        Raises:
            ValueError: If a ``dof_name`` is not found or matches multiple DOFs.
            ValueError: If the same DOF name appears more than once in ``actuators``.

        Example:

        .. code-block:: python

            >>> import warp as wp
            >>> from isaacsim.core.experimental.actuators import (
            ...     ActuatorConfig,
            ...     ArticulationActuators,
            ... )
            >>> from newton.actuators import ControllerPD

            >>> cfg = ActuatorConfig(
            ...     controller=ControllerPD(
            ...         kp=wp.array([100.0], dtype=wp.float32),
            ...         kd=wp.array([10.0], dtype=wp.float32),
            ...     )
            ... )
            >>> actuated = ArticulationActuators.from_actuators(  # doctest: +NO_CHECK
            ...     "/World/Robot",
            ...     [(cfg, "RevoluteJoint")],
            ... )
        """
        instance = cls(paths, auto_step_pre_physics=auto_step_pre_physics, _skip_discovery=True)

        if not actuators:
            return instance

        n_robots = len(instance._articulation)
        n_all_dofs = instance._articulation.num_dofs

        pairs: list[tuple[int, ActuatorConfig]] = []
        seen: set[int] = set()
        for config, dof_name in actuators:
            if dof_name not in instance._articulation.dof_names:
                raise ValueError(
                    f"DOF name {dof_name!r} not found in articulation "
                    f"'{instance._articulation.paths[0]}'. "
                    f"Available DOF names: {instance._articulation.dof_names}."
                )
            dof_index = int(instance._articulation.get_dof_indices(dof_name).numpy()[0])
            if dof_index in seen:
                raise ValueError(f"DOF {dof_name!r} (index {dof_index}) appears more than once in `actuators`.")
            seen.add(dof_index)
            pairs.append((dof_index, config))

        sorted_pairs = sorted(pairs)
        new_dof_indices = [dof_idx for dof_idx, _ in sorted_pairs]
        n_actuated_dofs = len(new_dof_indices)
        dof_index_to_actuated = {dof_idx: pos for pos, dof_idx in enumerate(new_dof_indices)}

        instance._actuated_dof_indices = new_dof_indices
        instance._effort_buffer = wp.zeros(n_robots * n_actuated_dofs, dtype=wp.float32, device=instance._device)
        instance._actuated_dof_indices_wp = wp.array(new_dof_indices, dtype=wp.int32, device=instance._device)

        # Build each Actuator wrapper from the config's components with correct index arrays.
        # controller.finalize(device, n_robots) is called inside Actuator.__init__, which is
        # idempotent for stateless controllers and correctly re-sizes scratch buffers for
        # stateful ones (e.g. PID integral).
        for dof_index, cfg in sorted_pairs:
            actuated_dof_index = dof_index_to_actuated[dof_index]
            sim_state_indices = _row_major_indices(n_robots, n_all_dofs, dof_index, instance._device)
            effort_indices = _row_major_indices(n_robots, n_actuated_dofs, actuated_dof_index, instance._device)
            actuator = Actuator(
                indices=sim_state_indices,
                controller=cfg.controller,
                delay=cfg.delay,
                clamping=cfg.clamping or None,
                effort_indices=effort_indices,
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
        device: str | None = None,
        _skip_discovery: bool = False,
    ) -> None:
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
        self._joint_target_pos_buffer = wp.empty(flat_dof_size, dtype=wp.float32, device=self._device)
        self._joint_target_vel_buffer = wp.empty(flat_dof_size, dtype=wp.float32, device=self._device)
        self._joint_control_feedforward_buffer = wp.empty(flat_dof_size, dtype=wp.float32, device=self._device)

        #: Newton `Actuator` objects owned by this wrapper, one per discovered prim.
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
            joint_target_pos=_gather_into(
                self._joint_target_pos_buffer,
                self._articulation.get_dof_position_targets().reshape(n_robots * n_all_dofs),
            ),
            joint_target_vel=_gather_into(
                self._joint_target_vel_buffer,
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

    def _discover_and_build_actuators(self) -> None:
        """Parse `NewtonActuator` prims under the prototype instance and instantiate them.

        Runs a two-pass scan: the first pass parses prims and collects the sorted DOF
        indices so each actuator's effort can be scattered into a compact ``(n_robots, A)``
        buffer; the second pass constructs the `Actuator` objects with both full-stride
        ``indices`` (into the articulation's ``(n_robots, n_all_dofs)`` state/control layout) and
        compact ``effort_indices`` (into the effort buffer).
        """
        n_robots = len(self._articulation)
        n_all_dofs = self._articulation.num_dofs

        stage = stage_utils.get_current_stage(backend="usd")

        # NOTE: here we have built in the assumption that ALL robots
        # in a single Articulation are built from the same USD (which is true)
        root_path = self._articulation.paths[0]
        root_prim = stage.GetPrimAtPath(root_path)
        if not root_prim.IsValid():
            carb.log_warn(f"ArticulationActuators: root prim '{root_path}' is invalid; no actuators discovered")
            return

        dof_paths_prototype = self._articulation.dof_paths[0] if self._articulation.dof_paths else []
        dof_path_to_index = {p: i for i, p in enumerate(dof_paths_prototype)}

        # Pass 1: parse every prim, validate DOF mapping, collect (prim_path, parsed, dof_index).
        dof_index_to_prim: dict[int, str] = {}
        collected: list[tuple[str, ActuatorParsed, int]] = []

        for prim in Usd.PrimRange(root_prim):
            # NOTE: parse_actuator_prim uses the usd function rel.GetTargets(), which always
            # returns an absolute SDF path.
            parsed: ActuatorParsed | None = na.parse_actuator_prim(prim)
            if parsed is None:
                continue

            if parsed.target_path not in dof_path_to_index:
                raise ValueError(
                    f"Actuator '{prim.GetPath()}' targets '{parsed.target_path}', which is not "
                    f"a DOF of articulation '{root_path}'."
                )
            dof_index = dof_path_to_index[parsed.target_path]

            if dof_index in dof_index_to_prim:
                raise ValueError(
                    f"Actuator '{prim.GetPath()}' targets '{parsed.target_path}', which is already "
                    f"targeted by '{dof_index_to_prim[dof_index]}'. Actuators support a 1-to-1 mapping "
                    "between actuators and joints."
                )
            dof_index_to_prim[dof_index] = str(prim.GetPath())
            collected.append((str(prim.GetPath()), parsed, dof_index))

        if not collected:
            return

        # Assign each owned DOF a slot in the compact effort buffer. Sort by DOF index
        # for stable ordering — this same order is passed to `set_dof_efforts(dof_indices=...)`.
        # `dof_index` is the index into the articulation's full DOF list; `actuated_dof_index`
        # is the position of that DOF in the actuated-only subset (also the column index in
        # the compact effort buffer).
        self._actuated_dof_indices = sorted(dof_index_to_prim.keys())
        dof_index_to_actuated_dof_index = {
            dof_index: actuated_dof_index for actuated_dof_index, dof_index in enumerate(self._actuated_dof_indices)
        }
        n_actuated_dofs = len(self._actuated_dof_indices)

        # Flat buffer to work with newton-actuators library:
        self._effort_buffer = wp.zeros(n_robots * n_actuated_dofs, dtype=wp.float32, device=self._device)
        self._actuated_dof_indices_wp = wp.array(self._actuated_dof_indices, dtype=wp.int32, device=self._device)

        # Pass 2: build Actuator objects with full-stride `indices` for state reads and
        # compact `effort_indices` for effort writes.
        for _prim_path, parsed, dof_index in collected:
            actuated_dof_index = dof_index_to_actuated_dof_index[dof_index]
            sim_state_indices = _row_major_indices(n_robots, n_all_dofs, dof_index, self._device)
            effort_indices = _row_major_indices(n_robots, n_actuated_dofs, actuated_dof_index, self._device)

            controller = build_controller(parsed.controller_class, parsed.controller_kwargs, n_robots, self._device)

            delay: Delay | None = None
            clamping: list[Clamping] = []
            for comp_class, comp_kwargs in parsed.component_specs:
                if issubclass(comp_class, Delay):
                    delay = build_delay(comp_kwargs, n_robots, self._device)
                else:
                    clamping.append(build_clamping(comp_class, comp_kwargs, n_robots, self._device))

            actuator = Actuator(
                indices=sim_state_indices,
                controller=controller,
                delay=delay,
                clamping=clamping if clamping else None,
                effort_indices=effort_indices,
                control_feedforward_attr="joint_control_feedforward",
            )
            self._actuators.append(actuator)

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
