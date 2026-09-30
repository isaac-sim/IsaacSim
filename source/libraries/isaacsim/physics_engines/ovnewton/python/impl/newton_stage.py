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

"""Manage the Newton state associated with an attached ovstage.

The driver borrows a caller-owned ovstage, builds the Newton model through
``ovnewton.attach_ovstage``, advances simulation state, and dispatches registered
callbacks. Newton, Warp, ovstage, and ovnewton imports are deferred so importing
the backend does not require those optional runtime packages.
"""

from __future__ import annotations

from collections.abc import Callable
from contextlib import nullcontext
from typing import Any

from isaacsim.common.logging import Logger

from .newton_config import NewtonConfig

_log = Logger("isaacsim.physics_engines.ovnewton.impl")


def _is_maximal_solver(solver: Any) -> bool:
    """Return whether a solver uses maximal coordinates.

    Maximal-coordinate solvers advance body transforms without updating generalized joint coordinates. Unknown solver
    implementations are treated as generalized-coordinate solvers.

    Args:
        solver: Newton solver instance to classify.

    Returns:
        True for a supported maximal-coordinate solver, otherwise False.

    """
    import newton.solvers as solvers

    return isinstance(solver, (solvers.SolverXPBD, solvers.SolverSemiImplicit, solvers.SolverVBD))


class NewtonStage:
    """Manage Newton-engine state for a single attached ovstage.

    Args:
        config: Backend configuration, or None to use the default configuration.

    Lifecycle:
        1. Constructor receives an optional :class:`NewtonConfig`.
        2. :meth:`initialize` borrows a caller-owned ovstage, builds a Newton model
           via ``ovnewton.attach_ovstage``, and creates solver state.
        3. Subsequent calls to :meth:`simulate` advance time.
        4. :meth:`on_detach` / :meth:`close` clear the attachment and mark the stage uninitialized.

    """

    def __init__(self, config: NewtonConfig | None = None) -> None:
        self.config: NewtonConfig = config or NewtonConfig()

        # Stage state
        self._stage_id: int = 0  # 0 = no stage attached (usd_identifier republish)
        # Populated by `NewtonSimulationRegistry.register()` immediately after
        # simulation registration returns. It fills
        # `PhysicsStepContext.simulation_id` when invoking step subscribers;
        # the registration layer forwards a single subscription to all engines,
        # and the context identifies the engine firing each callback.
        self._simulation_id: int = 0
        self._initialized: bool = False

        # Step counters surfaced through the physics registration callbacks.
        self._timestamp: int = 0
        self._step_count: int = 0
        # True when the last engine step raised. The counters are held on
        # failure so a frozen state is observable rather than looking like
        # simulated progress.
        self._last_step_failed: bool = False

        # Subscriber state for contact / step events.
        self._next_subscription_id: int = 1
        self._contact_subscribers: dict[int, Callable[[Any, Any, Any], Any]] = {}
        self._step_subscribers: dict[int, tuple[bool, int, Callable[[float, Any], Any]]] = {}

        # Misc state
        self._change_tracking_paused: bool = False

        # Lazy-imported engine handles populated on first ``initialize()``.
        self._ovstage = None  # borrowed caller-owned ovstage.Stage
        self._binding = None  # ovnewton.StageBinding
        self._usd_stage = None  # optional pxr.Usd.Stage for tensor path queries
        self._model = None  # newton.Model
        self._state_0 = None
        self._state_1 = None
        self._control = None
        self._contacts = None
        self._solver = None
        self._solver_is_maximal = False
        # Mapping from USD prim path -> Newton body index (for writeback).
        self._body_index: dict[str, int] = {}

    # ------------------------------------------------------------------
    # Stage lifecycle
    # ------------------------------------------------------------------

    def on_attach(self, stage_id: int) -> bool:
        """Record the USD identifier republished for ``get_attached_stage``.

        Args:
            stage_id: Numeric identifier supplied by the physics manager.

        Returns:
            True when the nonzero identifier is accepted, otherwise False.

        """
        if stage_id == 0:
            _log.warning("on_attach called with stage_id=0; ignoring")
            return False
        if stage_id != self._stage_id:
            # The borrowed ovstage belongs to the previously attached stage. Keeping it
            # would let `start_simulation` rebuild Newton from the old scene while
            # `get_attached_stage` reports the new identifier.
            self._clear_stage_attachment()
        self._stage_id = stage_id
        self._initialized = False
        self._timestamp = 0
        self._step_count = 0
        _log.info(f"Attached to stage id {stage_id}")
        return True

    def on_detach(self) -> None:
        """Detach the backend from its current ovstage."""
        if self._ovstage is None and self._stage_id == 0 and not self._initialized:
            return
        _log.info(f"Detaching from stage id {self._stage_id}")
        self._clear_stage_attachment()
        self._stage_id = 0
        self._initialized = False

    def _clear_stage_attachment(self) -> None:
        """Drop the borrowed ovstage, its USD query stage, and all derived Newton state."""
        self._clear_model_state()
        self._ovstage = None
        self._usd_stage = None

    def get_attached_stage(self) -> int:
        """Get the attached USD stage identifier.

        Returns:
            Attached stage identifier, or zero when no stage is attached.

        """
        return self._stage_id

    def has_attached_stage(self) -> bool:
        """Report whether a caller-owned ovstage is still borrowed.

        Distinguishes a retained ovstage (including StageCache id 0) from a
        fully detached backend. Required after a failed ``initialize`` that
        keeps ``_ovstage`` so ``start_simulation`` can retry: the manager uses
        this signal for ``InitializeResult::eFailedDirty`` and owner keepalive.

        Returns:
            True when a borrowed ovstage reference is held, otherwise False.

        """
        return self._ovstage is not None

    # ------------------------------------------------------------------
    # Public accessors used by the Newton tensor adapters.
    # ------------------------------------------------------------------

    @property
    def model(self) -> Any | None:
        """Get the Newton model.

        Returns:
            Newton model after successful model construction, or None before construction or for an empty stage.

        """
        return self._model

    @property
    def state_0(self) -> Any | None:
        """Get the current Newton simulation state.

        Returns:
            Current Newton state after successful model construction, or None when no model is available.

        """
        return self._state_0

    @property
    def state_1(self) -> Any | None:
        """Get the scratch Newton simulation state.

        Returns:
            Scratch Newton state after successful model construction, or None when no model is available.

        """
        return self._state_1

    @property
    def control(self) -> Any | None:
        """Get the Newton control buffer.

        Returns:
            Control buffer after successful model construction, or None when no model is available.

        """
        return self._control

    @property
    def contacts(self) -> Any | None:
        """Get the Newton contact buffer.

        Returns:
            Contact buffer after successful model construction, or None when no model is available.

        """
        return self._contacts

    @property
    def solver(self) -> Any | None:
        """Get the Newton solver.

        Returns:
            Solver after successful model construction, or None when no model is available.

        """
        return self._solver

    @property
    def solver_is_maximal(self) -> bool:
        """Check whether the active solver uses maximal coordinates.

        Returns:
            True when the active solver advances maximal-coordinate state.

        """
        return self._solver_is_maximal

    @property
    def initialized(self) -> bool:
        """Check whether initialization has completed.

        Returns:
            True after an initialization attempt completes, including an empty-stage initialization.

        """
        return self._initialized

    @property
    def ovstage(self) -> Any | None:
        """Get the borrowed caller-owned ovstage.

        Returns:
            Attached ovstage Stage, or None when no stage is attached.

        """
        return self._ovstage

    @property
    def binding(self) -> Any | None:
        """Get the ovnewton stage binding.

        Returns:
            Active ``ovnewton.StageBinding``, or None before construction or for an empty stage.

        """
        return self._binding

    @property
    def usd_stage(self) -> Any | None:
        """Get an optional pxr USD stage for tensor path queries.

        When ``usd_identifier`` resolves through the StageCache, tensor views use
        this stage for prim-path pattern matching. Physics model construction always
        comes from the borrowed ovstage.

        Returns:
            Cached USD stage, or None when no StageCache entry is available.

        """
        return self._usd_stage

    @property
    def device(self) -> Any:
        """Get the device used by the Newton model.

        Returns:
            Model device when available, otherwise the Warp CPU device. If Warp is unavailable, returns ``"cpu"``.

        """
        if self._model is not None and hasattr(self._model, "device"):
            return self._model.device
        try:
            import warp as wp

            return wp.get_device("cpu")
        except Exception:
            return "cpu"

    # ------------------------------------------------------------------
    # Initialization (lazy ovnewton / Warp import)
    # ------------------------------------------------------------------

    def initialize(self, ovstage_obj: Any | None, stage_id: int = 0) -> bool:
        """Borrow an ovstage and build the Newton model through ovnewton.

        Args:
            ovstage_obj: Caller-owned ``ovstage.Stage``, or None for an empty scene.
            stage_id: Numeric identifier republished through :meth:`get_attached_stage`.

        Returns:
            True when initialization completes, including an empty stage or unavailable optional dependencies.
            Returns False when model construction raises an exception or a required Stage object is missing.

        """
        self._clear_model_state()
        self._ovstage = ovstage_obj
        self._usd_stage = self._resolve_usd_stage(stage_id) if stage_id else None
        self._stage_id = int(stage_id) if stage_id else 0
        self._timestamp = 0
        self._step_count = 0
        self._initialized = False

        if ovstage_obj is None:
            _log.info("Newton initialized with an empty ovstage (no physics).")
            self._initialized = True
            return True

        try:
            engine_modules = self._import_engine()
        except (ImportError, RuntimeError) as exc:
            # Newton, Warp, ovstage, and ovnewton are optional runtime dependencies:
            # keep the backend plumbing usable (no model, no physics) instead of failing
            # attach. ``ovstage.setup()`` reports a missing bundled runtime as a
            # RuntimeError rather than an ImportError, so both are treated the same way.
            # Only the import step is treated this way -- the same exception raised while
            # ovnewton builds the model is a real failure, not a missing package.
            _log.warning(f"Newton, Warp, ovstage, or ovnewton is not available: {exc}")
            self._initialized = True
            return True

        try:
            self._build_newton_model_from_ovstage(ovstage_obj, *engine_modules)
        except Exception as exc:
            _log.error(f"Failed to build Newton model from ovstage: {exc}")
            # A half-built binding/model would otherwise be reported through the
            # `binding` / `model` accessors as if the attach had succeeded. `_usd_stage`
            # is deliberately kept: like `_stage_id`, it tracks the identifier the caller
            # supplied and is independent of whether model construction succeeded.
            # Leave `_initialized` False so ``start_simulation`` can retry the build.
            self._clear_model_state()
            self._initialized = False
            return False

        self._initialized = True
        return True

    def _clear_model_state(self) -> None:
        """Drop the ovnewton binding plus Newton model, solver, and body-index state."""
        self._binding = None
        self._model = None
        self._state_0 = None
        self._state_1 = None
        self._control = None
        self._contacts = None
        self._solver = None
        self._solver_is_maximal = False
        self._body_index = {}

    @staticmethod
    def _import_engine() -> tuple[Any, Any, Any]:
        """Import the optional Newton / ovnewton / Warp runtime.

        Returns:
            The ``newton.solvers``, ``ovnewton``, and ``warp`` modules.

        Raises:
            ImportError: If any of the optional runtime packages is unavailable.
            RuntimeError: If the bundled ovstage runtime cannot be located.

        """
        from isaacsim.physics_engines.ovstage import setup

        setup()
        import newton.solvers as solvers
        import ovnewton
        import warp as wp

        return solvers, ovnewton, wp

    def _build_newton_model_from_ovstage(self, ovstage_obj: Any, solvers: Any, ovnewton: Any, wp: Any) -> None:
        """Build Newton simulation state from a populated ovstage.

        Args:
            ovstage_obj: Caller-owned ovstage Stage containing the physics scene.
            solvers: ``newton.solvers`` module.
            ovnewton: ``ovnewton`` module.
            wp: ``warp`` module.

        Raises:
            Exception: Propagates model-construction failures from ovnewton.

        """
        device = self.config.device
        device_ctx = wp.ScopedDevice(device) if device else nullcontext()
        with device_ctx:
            # Population payloads are conventionally sealed at ordinal 1 before attach.
            self._ensure_write_floor(ovstage_obj, ordinal=1)
            self._binding = ovnewton.attach_ovstage(ovstage_obj, ordinal=1)
            self._model = self._binding.model

            if self._model is None:
                _log.info("ovnewton returned no model; Newton will run empty.")
                return

            self._body_index = {
                str(label): idx for idx, label in enumerate(getattr(self._model, "body_label", []) or []) if label
            }
            self._sync_articulation_labels_from_usd()

            if getattr(self._model, "body_count", 0) == 0:
                _log.info("ovnewton parsed the ovstage but found no rigid bodies.")
                return

            # State, control, contact buffers, and the solver allocate Warp arrays. Keep
            # them inside the scoped device so they land on `config.device` alongside the
            # model instead of on Warp's process default.
            # Raw-contact tensor views consume Newton's optional force field.
            self._model.request_contact_attributes("force")
            self._state_0 = self._model.state()
            self._state_1 = self._model.state()
            self._control = self._model.control()
            self._contacts = self._model.contacts()
            self._solver = solvers.SolverXPBD(self._model, iterations=2)
            self._solver_is_maximal = _is_maximal_solver(self._solver)

    def _sync_articulation_labels_from_usd(self) -> None:
        """Replace synthetic articulation labels with USD ArticulationRoot paths.

        Tensor views match ``create_articulation_view`` patterns against
        ``model.articulation_label``. ``ovnewton.attach_ovstage`` may leave
        synthetic names such as ``articulation_0`` that never appear on the USD
        stage, so articulation views come back empty. Walk from each
        articulation's root body to the nearest ``ArticulationRootAPI`` prim
        (falling back to the root body path) so labels match USD paths.
        """
        model = self._model
        stage = self._usd_stage
        if model is None or stage is None or getattr(model, "articulation_count", 0) == 0:
            return
        try:
            from pxr import UsdPhysics
        except ImportError:
            return

        # Best effort: label layout differs across Newton versions, and a failure here
        # must not abort an otherwise valid model (it only degrades view matching).
        try:
            arti_starts = model.articulation_start.numpy().tolist()
            joint_child = model.joint_child.numpy().tolist()
            body_label = model.body_label
            for arti_idx in range(model.articulation_count):
                joint_start = arti_starts[arti_idx]
                if joint_start >= len(joint_child):
                    continue  # articulation without joints: no root body to resolve
                root_body_idx = int(joint_child[joint_start])
                if not 0 <= root_body_idx < len(body_label):
                    continue  # free-floating child (e.g. world body index -1)
                body_path = str(body_label[root_body_idx])
                if not body_path.startswith("/"):
                    continue  # synthetic label, not a USD path: nothing to resolve
                resolved = body_path
                prim = stage.GetPrimAtPath(body_path)
                while prim and prim.IsValid():
                    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                        resolved = str(prim.GetPath())
                        break
                    prim = prim.GetParent()
                model.articulation_label[arti_idx] = resolved
        except Exception as exc:  # noqa: BLE001 -- label fixup is optional
            _log.warning(f"Could not sync articulation labels to USD paths: {exc}")

    @staticmethod
    def _ensure_write_floor(ovstage_obj: Any, ordinal: int) -> None:
        """Seal the ovstage write floor through ``ordinal`` when needed.

        Args:
            ovstage_obj: Caller-owned ovstage Stage.
            ordinal: Population ordinal that ``attach_ovstage`` will read.

        """
        advance = getattr(ovstage_obj, "advance_write_floor", None)
        if advance is None:
            return
        sealed_known = True
        try:
            with ovstage_obj.get_attribute_write_floor() as query:
                sealed = int(query.fetch())
        except Exception as exc:
            # Callers conventionally seal before attach, so assume the floor is behind
            # and advance speculatively -- but record why the state was unreadable.
            _log.verbose(f"Could not read the ovstage write floor ({exc}); advancing to ordinal {ordinal}")
            sealed = 0
            sealed_known = False
        if sealed >= int(ordinal):
            return
        try:
            waitable = advance(int(ordinal))
        except Exception as exc:
            if sealed_known:
                raise
            # The floor was already sealed at or past `ordinal` and advancing it again
            # is rejected. That is the expected state for attach, so do not fail here.
            _log.verbose(f"Speculative advance_write_floor({ordinal}) was rejected: {exc}")
            return
        wait = getattr(waitable, "wait", None)
        if callable(wait):
            wait()

    # ------------------------------------------------------------------
    # Backend-side state accessors (outside the registration callback surface).
    # ------------------------------------------------------------------
    #
    # The registration layer does not commit results to a particular sink.
    # Applications and tests query the engine directly through these helpers
    # rather than introspecting Newton internals.

    def get_body_position(self, prim_path: str) -> tuple[float, float, float] | None:
        """Get the latest position of a Newton rigid body.

        Args:
            prim_path: USD prim path used to identify the body.

        Returns:
            World-space position when the body and current state are available, otherwise None.

        """
        if self._model is None or self._state_0 is None:
            return None
        idx = self._body_index.get(prim_path)
        if idx is None:
            return None
        row = self._state_0.body_q.numpy()[idx]
        return (float(row[0]), float(row[1]), float(row[2]))

    def list_body_paths(self) -> list[str]:
        """Get the USD prim paths parsed as Newton bodies.

        Returns:
            Body prim paths in parser-defined order.

        """
        return list(self._body_index.keys())

    def close(self) -> None:
        """Detach the stage and mark the simulation uninitialized."""
        self.on_detach()

    # ------------------------------------------------------------------
    # Stepping
    # ------------------------------------------------------------------

    def simulate(self, elapsed_time: float, current_time: float) -> None:
        """Advance the simulation and fetch synchronous results.

        Args:
            elapsed_time: Duration of the simulation step in seconds.
            current_time: Current simulation time supplied to the backend.

        """
        self.simulate_async(elapsed_time, current_time)
        self.fetch_results()

    def simulate_async(self, elapsed_time: float, current_time: float) -> None:
        """Advance the simulation and notify pre-step and post-step subscribers.

        Pre-step subscribers run before the solver and post-step subscribers run afterward. Subscribers within each
        phase run by ascending order value. A solver failure leaves the step counters unchanged.

        Args:
            elapsed_time: Duration of the simulation step in seconds.
            current_time: Current simulation time supplied to the backend.

        """
        if not self._initialized:
            return

        try:
            from isaacsim.physics.registration import (
                PhysicsStepContext,
                SimulationId,
            )
        except ImportError:
            PhysicsStepContext = None  # type: ignore[assignment]
            SimulationId = None  # type: ignore[assignment]

        # Populate the context so subscribers can identify the engine firing
        # this callback. `subscribePhysicsOnStepEvents` forwards one
        # subscription to every registered simulation, and simulation_id
        # distinguishes callbacks across engines. `scene_path` stays 0 because
        # each NewtonStage drives a single scene.
        ctx = None
        if PhysicsStepContext is not None:
            ctx = PhysicsStepContext()
            sim_id_int = int(self._simulation_id) if self._simulation_id is not None else 0
            if SimulationId is not None:
                ctx.simulation_id = SimulationId(sim_id_int)
            ctx.scene_path = 0

        pre_subs = []
        post_subs = []
        for _, (pre, order, fn) in self._step_subscribers.items():
            (pre_subs if pre else post_subs).append((order, fn))
        pre_subs.sort(key=lambda kv: kv[0])
        post_subs.sort(key=lambda kv: kv[0])

        for _order, fn in pre_subs:
            try:
                fn(elapsed_time, ctx) if ctx is not None else fn(elapsed_time, None)
            except Exception as exc:  # pragma: no cover
                _log.error(f"Pre-step subscriber raised; continuing: {exc}")

        # Drive the engine if we managed to build a model in initialize().
        step_failed = False
        if self._solver is not None and self._model is not None:
            try:
                self._model.collide(self._state_0, self._contacts)
                self._solver.step(self._state_0, self._state_1, self._control, self._contacts, float(elapsed_time))
                self._state_0, self._state_1 = self._state_1, self._state_0
                self._solver.update_contacts(self._contacts)
                if self._solver_is_maximal:
                    self._sync_generalized_coordinates()
            except Exception as exc:
                step_failed = True
                _log.error(f"Newton simulation step raised; state frozen and step counter held: {exc}")
            finally:
                try:
                    # Consume forces submitted between steps. After a successful
                    # swap this clears the new current state; after a failed step
                    # it clears the unchanged input state so forces cannot leak
                    # into a later step.
                    self._state_0.clear_forces()
                except Exception as exc:
                    _log.error(f"Newton state.clear_forces raised; continuing: {exc}")

        # Hold the counters on a failed step so the caller cannot mistake a
        # frozen state for progress.
        self._last_step_failed = step_failed
        if not step_failed:
            self._step_count += 1
            self._timestamp += 1

        for _order, fn in post_subs:
            try:
                fn(elapsed_time, ctx) if ctx is not None else fn(elapsed_time, None)
            except Exception as exc:  # pragma: no cover
                _log.error(f"Post-step subscriber raised; continuing: {exc}")

    def _sync_generalized_coordinates(self) -> None:
        """Recover ``joint_q`` / ``joint_qd`` from the stepped body state via IK.

        Only meaningful for maximal-coordinate solvers (XPBD, semi-implicit,
        VBD), which advance ``State.body_q`` and leave the generalized joint
        coordinates stale. The caller gates this on ``_solver_is_maximal``;
        generalized solvers (MuJoCo, Featherstone) advance ``joint_q`` directly.
        """
        if self._model is None or self._state_0 is None:
            return
        if getattr(self._model, "articulation_count", 0) == 0:
            return
        import newton

        state = self._state_0
        newton.eval_ik(self._model, state, state.joint_q, state.joint_qd)

    def fetch_results(self) -> None:
        """Dispatch an empty compatibility contact report to subscribers.

        Newton tensor views expose contact data directly. This callback notifies
        subscribers with empty header, contact-data, and friction-anchor
        collections.
        """
        if not self._initialized:
            return

        # Applications choose the simulation result sink, such as USD
        # write-back, Fabric, or a native Python tensor view. Callers query body
        # state through backend-specific accessors such as get_body_position,
        # or through separate adapters.
        try:
            from isaacsim.physics.registration import (
                ContactDataVector,
                ContactEventHeaderVector,
                FrictionAnchorsDataVector,
            )
        except ImportError:
            return

        if not self._contact_subscribers:
            return

        headers = ContactEventHeaderVector()
        data = ContactDataVector()
        anchors = FrictionAnchorsDataVector()
        for cb in list(self._contact_subscribers.values()):
            try:
                cb(headers, data, anchors)
            except Exception as exc:  # pragma: no cover
                _log.error(f"Contact subscriber raised; continuing: {exc}")

    def check_results(self) -> bool:
        """Check whether synchronous simulation results are ready.

        Returns:
            Always True because the backend completes work synchronously.

        """
        return True

    def flush_changes(self) -> None:
        """Accept a request to flush pending changes.

        The Newton backend applies no deferred modifications.
        """

        # ------------------------------------------------------------------

    # Change tracking
    # ------------------------------------------------------------------

    def pause_change_tracking(self, pause: bool) -> None:
        """Set whether change tracking is paused.

        Args:
            pause: True to pause change tracking, or False to resume it.

        """
        self._change_tracking_paused = bool(pause)

    def is_change_tracking_paused(self) -> bool:
        """Check whether change tracking is paused.

        Returns:
            True when change tracking is paused.

        """
        return self._change_tracking_paused

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def subscribe_contact_report(self, callback: Callable[[Any, Any, Any], Any]) -> int:
        """Subscribe to contact-report notifications.

        Args:
            callback: Function invoked with contact headers, contact data, and friction-anchor data.

        Returns:
            Identifier used to remove the subscription.

        """
        sub_id = self._next_subscription_id
        self._next_subscription_id += 1
        self._contact_subscribers[sub_id] = callback
        return sub_id

    def unsubscribe_contact_report(self, subscription_id: int) -> None:
        """Remove a contact-report subscription.

        Args:
            subscription_id: Identifier returned by :meth:`subscribe_contact_report`.

        """
        self._contact_subscribers.pop(subscription_id, None)

    def subscribe_step_event(self, pre_step: bool, order: int, callback: Callable[[float, Any], Any]) -> int:
        """Subscribe to simulation-step notifications.

        Args:
            pre_step: True to invoke the callback before the solver step, or False to invoke it afterward.
            order: Ordering value within the selected step phase.
            callback: Function invoked with the step duration and physics-step context.

        Returns:
            Identifier used to remove the subscription.

        """
        sub_id = self._next_subscription_id
        self._next_subscription_id += 1
        self._step_subscribers[sub_id] = (bool(pre_step), int(order), callback)
        return sub_id

    def unsubscribe_step_event(self, subscription_id: int) -> None:
        """Remove a simulation-step subscription.

        Args:
            subscription_id: Identifier returned by :meth:`subscribe_step_event`.

        """
        self._step_subscribers.pop(subscription_id, None)

    # ------------------------------------------------------------------
    # Capabilities
    # ------------------------------------------------------------------

    def is_capable_of_simulating(self, schema_names: list[str]) -> tuple[bool, list[bool]]:
        """Check support for requested USD physics schemas.

        Newton advertises rigid-body and articulation support. Soft-body, fluid, cloth, and vehicle schemas are
        unsupported.

        Args:
            schema_names: Schema type names to query.

        Returns:
            A successful-query flag and one support flag per requested schema.

        """
        supported = {
            "PhysicsRigidBodyAPI",
            "PhysicsCollisionAPI",
            "PhysicsArticulationRootAPI",
            "PhysicsRevoluteJoint",
            "PhysicsPrismaticJoint",
            "PhysicsFixedJoint",
        }
        return (True, [name in supported for name in schema_names])

    # ------------------------------------------------------------------
    # Stage lifecycle (engine-internal; called directly when driving the engine
    # outside the physics registration callbacks)
    # ------------------------------------------------------------------

    def on_update(self, current_time: float, elapsed_secs: float, enable_update: bool) -> None:
        """Advance the backend for an enabled update.

        Args:
            current_time: Current simulation time.
            elapsed_secs: Duration of the update in seconds.
            enable_update: Whether this update should advance simulation.

        """
        if not enable_update or not self._initialized:
            return
        self.simulate(elapsed_secs, current_time)

    def on_resume(self, current_time: float) -> None:  # noqa: D401
        """Record a simulation-resume event.

        Args:
            current_time: Simulation time at resume.

        """
        _log.verbose(f"on_resume(t={current_time:f})")

    def on_pause(self) -> None:
        """Record a simulation-pause event."""
        _log.verbose("on_pause")

    def on_reset(self) -> None:
        """Reset the simulation timestamp and step count."""
        self._step_count = 0
        self._timestamp = 0

    def force_load_physics_from_usd(self) -> None:
        """Rebuild physics state from the currently borrowed ovstage."""
        if self._ovstage is not None or self._stage_id != 0:
            self.initialize(self._ovstage, self._stage_id)

    def release_physics_objects(self) -> None:
        """Drop the ovnewton binding and derived Newton simulation objects."""
        self._clear_model_state()

    def reset_simulation(self) -> None:
        """Clear physics objects, counters, and initialization state."""
        self.release_physics_objects()
        self._initialized = False
        self._step_count = 0
        self._timestamp = 0

    def start_simulation(self) -> None:
        """Initialize the attached ovstage when simulation has not started."""
        if not self._initialized and (self._ovstage is not None or self._stage_id != 0):
            self.initialize(self._ovstage, self._stage_id)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _resolve_usd_stage(stage_id: int) -> Any | None:
        """Look up a pxr USD stage for tensor path queries.

        Args:
            stage_id: Numeric USD stage-cache identifier.

        Returns:
            Cached USD stage, or None when USD is unavailable or the identifier is not cached.

        """
        try:
            from pxr import Usd, UsdUtils
        except ImportError:
            _log.verbose("pxr or usd-core is not available; tensor prim-path queries are disabled")
            return None
        try:
            stage = UsdUtils.StageCache.Get().Find(Usd.StageCache.Id.FromLongInt(int(stage_id)))
        except Exception as exc:
            # The USD stage is optional, but losing it silently degrades tensor-view
            # prim-path matching, so record why it could not be resolved.
            _log.warning(f"Could not resolve USD stage {stage_id} from the StageCache: {exc}")
            return None
        if stage is None:
            _log.verbose(f"USD stage {stage_id} is not in the StageCache; tensor prim-path queries are disabled")
        return stage

    @property
    def step_count(self) -> int:
        """Get the number of completed simulation steps.

        Returns:
            Number of initialized step callbacks that did not encounter a solver error since attachment or reset.

        """
        return self._step_count

    @property
    def timestamp(self) -> int:
        """Get the simulation timestamp.

        Returns:
            Backend timestamp incremented once per initialized step callback that does not encounter a solver error.

        """
        return self._timestamp

    @property
    def last_step_failed(self) -> bool:
        """Check whether the most recent solver step failed.

        Returns:
            True when the most recent attempted solver step raised an exception.

        """
        return self._last_step_failed
