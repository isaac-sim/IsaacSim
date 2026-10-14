# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""BackendContext stub, source badge, and natural-frequency mode gating."""

from __future__ import annotations

from types import SimpleNamespace
from unittest import mock

import omni.kit.test
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot_setup import gain_tuner
from isaacsim.robot_setup.gain_tuner import GainSource
from isaacsim.robot_setup.gain_tuner.ui.backend_context import BackendContext
from isaacsim.robot_setup.gain_tuner.ui.ui_builder import TuningMode, UIBuilder
from pxr import Usd


class TestBackendContext(omni.kit.test.AsyncTestCase):
    """Derived-property behavior of the backend/gain-source stub."""

    async def test_default_stub_names_no_engine(self) -> None:
        """A context nobody has filled in reports no engine, not PhysX.

        The default used to be ``"PhysX"``, so an unfilled or unreadable context was
        indistinguishable from a running PhysX one in every derived label -- the
        header badge said PhysX directly above joint fields that had disabled
        themselves because the engine was unknown.
        """
        ctx = BackendContext()
        self.assertEqual(ctx.backend, "")
        self.assertFalse(ctx.show_solver)
        self.assertEqual(ctx.solver_label, "")
        self.assertFalse(ctx.show_ki)
        self.assertEqual(ctx.source_label, "")
        self.assertEqual(ctx.backend_display, "unknown")
        self.assertNotIn("PhysX", f"{ctx.backend_display}{ctx.solver_display}{ctx.solver_short}{ctx.source_label}")
        self.assertFalse(ctx.save_target_resolved)

    async def test_an_unknown_engine_is_never_displayed_as_physx(self) -> None:
        """Every display string agrees with the joint fields' "unsupported" state.

        The badge reads :attr:`backend_display`, the info tag reads
        :attr:`solver_display`, the table's Source column reads
        :attr:`source_label`.  All three mapped "not Newton" to PhysX, so an engine
        the panel refuses to author for was presented as the one engine it authors
        for most confidently.
        """
        for backend, badge in (("remotesim", "remotesim"), ("", "unknown")):
            ctx = BackendContext(backend=backend)
            self.assertEqual(ctx.backend_display, badge, msg=backend)
            self.assertEqual(ctx.solver_display, badge, msg=backend)
            self.assertEqual(ctx.solver_short, "-", msg=backend)
            self.assertEqual(ctx.source_label, "", msg=backend)
            self.assertFalse(gain_tuner.backend_supported(ctx.backend), msg=backend)

    async def test_newton_solver_visible(self) -> None:
        """A Newton backend with a named solver shows the solver label and badge."""
        ctx = BackendContext(backend="NewtonAPI", solver="Featherstone")
        self.assertTrue(ctx.show_solver)
        self.assertEqual(ctx.solver_label, "Solver: Featherstone")
        self.assertEqual(ctx.source_label, "Newton")

    async def test_physx_never_shows_solver(self) -> None:
        """PhysX suppresses the solver label even if a solver string is set."""
        ctx = BackendContext(backend="PhysX", solver="Featherstone")
        self.assertFalse(ctx.show_solver)
        self.assertEqual(ctx.solver_label, "")

    async def test_actuator_source_shows_ki(self) -> None:
        """The Ki column is only shown when the gain source is an actuator."""
        ctx = BackendContext(gain_source="actuator")
        self.assertTrue(ctx.show_ki)

    async def test_save_target_resolved(self) -> None:
        """A non-empty save target resolves the save row."""
        ctx = BackendContext(save_target="actuator_payload.usda")
        self.assertTrue(ctx.save_target_resolved)

    async def test_backend_and_solver_display_strings(self) -> None:
        """backend_display / solver_display / solver_short across PhysX and Newton."""
        # PhysX reports a plain PhysX tag for every display string.
        physx = BackendContext(backend="PhysX")
        self.assertEqual((physx.backend_display, physx.solver_display, physx.solver_short), ("PhysX", "PhysX", "PhysX"))
        # Newton names the specific solver (solver_display prefixes the family).
        feather = BackendContext(backend="NewtonAPI", solver="Featherstone")
        self.assertEqual(
            (feather.backend_display, feather.solver_display, feather.solver_short),
            ("Newton", "Newton Featherstone", "Featherstone"),
        )
        mujoco = BackendContext(backend="NewtonAPI", solver="MuJoCo", solver_type="mujoco")
        self.assertEqual(mujoco.solver_display, "Newton MuJoCo")
        # is_mujoco_solver gates whether mjc:* gains become the active/editable
        # source, and now reads the raw solver TOKEN rather than the display name:
        # the token is what the resolver chain is keyed on, so a stale or
        # differently-cased display string can no longer decide it.
        self.assertTrue(mujoco.is_mujoco_solver)
        self.assertFalse(feather.is_mujoco_solver)
        self.assertFalse(physx.is_mujoco_solver)
        self.assertFalse(BackendContext(backend="NewtonAPI", solver="MuJoCo").is_mujoco_solver)
        # An unknown/empty solver falls back to plain "Newton"; solver_short uses an
        # ASCII hyphen placeholder (the app font has no em-dash glyph).
        unknown = BackendContext(backend="NewtonAPI", solver="")
        self.assertEqual((unknown.solver_display, unknown.solver_short), ("Newton", "-"))

    async def test_solver_known_gates_the_resolver_chain(self) -> None:
        """Under Newton the resolver order needs a solver; under PhysX it never does."""
        self.assertTrue(BackendContext(backend="PhysX").solver_known)
        for token in ("mujoco", "xpbd", "vbd"):
            self.assertTrue(BackendContext(backend="NewtonAPI", solver_type=token).solver_known, msg=token)
        self.assertFalse(BackendContext(backend="NewtonAPI").solver_known)

    async def test_from_app_is_defensive(self) -> None:
        """from_app never raises and always returns a BackendContext."""
        for stage in (None, object()):
            ctx = BackendContext.from_app(stage)
            self.assertIsInstance(ctx, BackendContext)
            self.assertIn(ctx.backend, ("", "PhysX", "NewtonAPI", *[b for b in ctx.available_backends]))

    async def test_an_undeterminable_engine_reads_empty_from_both_readers(self) -> None:
        """``from_app`` and ``active_backend_label`` must not disagree.

        The panel compares them every frame to notice an engine switch.  If
        ``from_app`` defaulted to "PhysX" for a query that returned nothing while
        the live read honestly reported "", the two would never agree and the panel
        would rebuild forever.  Both report nothing, and the advanced joint params
        go read-only rather than authoring a guessed schema.
        """
        with mock.patch.object(SimulationManager, "get_active_physics_engine", return_value=""):
            self.assertEqual(BackendContext.active_backend_label(), "")
            self.assertEqual(BackendContext.from_app(None).backend, "")
        with mock.patch.object(SimulationManager, "get_active_physics_engine", side_effect=RuntimeError("no manager")):
            self.assertEqual(BackendContext.active_backend_label(), "")
            self.assertEqual(BackendContext.from_app(None).backend, "")

    async def test_remotesim_is_reported_as_itself_and_refused(self) -> None:
        """A real engine value with no known joint schema is named, not renamed to PhysX."""
        with mock.patch.object(SimulationManager, "get_active_physics_engine", return_value="remotesim"):
            for backend in (BackendContext.active_backend_label(), BackendContext.from_app(None).backend):
                self.assertEqual(backend, "remotesim")
                self.assertFalse(gain_tuner.backend_supported(backend))
                self.assertIsNone(gain_tuner.backend_write_schema(backend))

    async def test_from_app_reports_a_solver_token_it_can_resolve(self) -> None:
        """The solver is load-bearing, so from_app must carry the token, not just a name."""
        ctx = BackendContext.from_app(None)
        if ctx.backend != "NewtonAPI":
            self.assertEqual((ctx.solver, ctx.solver_type), ("", ""))
            return
        self.assertIn(ctx.solver_type, ("", "mujoco", "xpbd", "vbd"))
        # A resolved token always comes with a display name, and vice versa.
        self.assertEqual(bool(ctx.solver), bool(ctx.solver_type))

    async def test_a_stage_resolves_the_solver_before_the_first_play(self) -> None:
        """The stage's Newton scene answers what the live config cannot yet.

        ``NewtonSolverConfig.solver_type`` is the placeholder ``"None"`` until
        Newton initializes, which is why the solver read back empty and
        ``solver_display`` fell through to a bare "Newton".
        """
        from isaacsim.robot_setup import gain_tuner
        from isaacsim.robot_setup.gain_tuner.gain_sources import _staged_newton_solver_type
        from pxr import UsdPhysics

        try:
            from isaacsim.physics.newton import newton_solver_to_api_schema
        except ImportError:
            self.skipTest("isaacsim.physics.newton is not enabled in this app")

        stage = Usd.Stage.CreateInMemory()
        scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene").GetPrim()
        scene.ApplyAPI(newton_solver_to_api_schema[gain_tuner.SOLVER_XPBD])

        self.assertEqual(_staged_newton_solver_type(stage), gain_tuner.SOLVER_XPBD)
        # No scene, no answer -- and no exception either.
        self.assertEqual(_staged_newton_solver_type(Usd.Stage.CreateInMemory()), "")
        self.assertEqual(_staged_newton_solver_type(None), "")


class TestUIBuilderImport(omni.kit.test.AsyncTestCase):
    """Smoke check that the top-level UI entry point imports cleanly."""

    async def test_uibuilder_importable(self) -> None:
        """The UIBuilder class is importable from the extension module."""
        self.assertTrue(callable(UIBuilder))


class _DeferredRetryProbe(UIBuilder):
    """UIBuilder stub that counts deferred re-setups.

    Stubs the stage and UI half of ``_on_articulation_selection`` and delegates the
    re-arm decision to :meth:`_arm_deferred_joint_wait`.

    Args:
        joint_entries: What the stub tuner enumerates on every setup.
        num_dofs: DOF count the stub articulation reports.
    """

    def __init__(self, joint_entries: list[object], num_dofs: int = 6) -> None:
        self._awaiting_articulation_dofs = True
        self._gains_tuner = SimpleNamespace(
            _robot_prim_path="/World/Robot",
            _articulation=SimpleNamespace(num_dofs=num_dofs),
            get_joint_entries=lambda: joint_entries,
        )
        self.resetup_count = 0

    def _on_articulation_selection(self, articulation_path: str, force: bool = False) -> None:
        """Count the re-setup, then re-arm through the production decision."""
        self.resetup_count += 1
        self._arm_deferred_joint_wait()


class _TimelineStopProbe(UIBuilder):
    """UIBuilder stub that counts override restores driven by timeline events.

    Args:
        stopped: What the stub timeline reports for ``is_stopped``.
    """

    def __init__(self, stopped: bool) -> None:
        self._timeline = SimpleNamespace(is_stopped=lambda: stopped)
        self._restarting_for_override = False
        self._articulation_menu_model = None
        self.restore_count = 0

    def _restore_self_collision_override(self) -> None:
        """Count the restore call."""
        self.restore_count += 1


class TestTimelineStopRestoresOverride(omni.kit.test.AsyncTestCase):
    """Stopping the timeline ends a test run outside cancel and completion."""

    async def test_stop_drops_a_live_override(self) -> None:
        """The toolbar stop is a test exit, so it has to restore the override."""
        probe = _TimelineStopProbe(stopped=True)
        probe.on_timeline_event(None)
        self.assertEqual(probe.restore_count, 1)

    async def test_play_leaves_the_override_alone(self) -> None:
        """Starting the timeline is not a test exit."""
        probe = _TimelineStopProbe(stopped=False)
        probe.on_timeline_event(None)
        self.assertEqual(probe.restore_count, 0)

    async def test_the_overrides_own_recook_does_not_restore_itself(self) -> None:
        """The recook stops the timeline; that stop must not undo the override."""
        probe = _TimelineStopProbe(stopped=True)
        probe._restarting_for_override = True
        probe.on_timeline_event(None)
        self.assertEqual(probe.restore_count, 0)


class TestDeferredJointRepopulate(omni.kit.test.AsyncTestCase):
    """Deferred joint re-setup when the (Newton, lazy) articulation view cooks."""

    async def test_repopulate_decision(self) -> None:
        """Re-setup fires only when a prior setup found no joints AND DOFs now exist.

        Under Newton the physics-tensor articulation is built lazily, so the first
        setup() sees ``num_dofs == 0`` and enumerates no joints ("no joints found");
        once the view reports DOFs the joints must be re-enumerated.  A normal
        (already-populated) robot never has the awaiting flag set.
        """
        # Not awaiting (joints already enumerated): never re-populate, regardless of DOFs.
        self.assertFalse(UIBuilder._should_repopulate_deferred_joints(False, 0))
        self.assertFalse(UIBuilder._should_repopulate_deferred_joints(False, 6))
        # Awaiting but the view has not cooked yet (num_dofs == 0): keep waiting.
        self.assertFalse(UIBuilder._should_repopulate_deferred_joints(True, 0))
        # Awaiting and the view now reports DOFs: re-enumerate the joints.
        self.assertTrue(UIBuilder._should_repopulate_deferred_joints(True, 6))

    async def test_arming_decision(self) -> None:
        """The wait is armed by an uncooked DOF view, not by "have we retried yet".

        A robot that already reports DOFs and still enumerates no joints cannot be
        fixed by re-running the same enumeration, so it must not arm.
        """
        # No joints and no DOFs yet: the view has not cooked, so wait.
        self.assertTrue(UIBuilder._should_await_articulation_dofs(0, False))
        # No joints but DOFs exist: the schema does not list them, so do not wait.
        self.assertFalse(UIBuilder._should_await_articulation_dofs(6, False))
        # Joints enumerated: nothing to defer, whatever the DOF count says.
        self.assertFalse(UIBuilder._should_await_articulation_dofs(0, True))
        self.assertFalse(UIBuilder._should_await_articulation_dofs(6, True))

    async def test_no_retry_loop_when_joints_never_enumerate(self) -> None:
        """A robot that reports DOFs but no joints re-setups once, not every frame.

        Each re-setup restarts the timeline from ``_setup_robot_links``, so an
        unbounded retry shows up as the timeline flickering between play and stop.
        """
        probe = _DeferredRetryProbe(joint_entries=[])
        for _ in range(10):  # ten render steps
            probe._maybe_repopulate_deferred_joints()
        self.assertEqual(probe.resetup_count, 1)
        self.assertFalse(probe._awaiting_articulation_dofs)

    async def test_retry_populates_joints_when_the_view_cooks(self) -> None:
        """The deferred retry still repopulates a lazily-cooked (Newton) articulation."""
        probe = _DeferredRetryProbe(joint_entries=["joint0"])
        for _ in range(10):
            probe._maybe_repopulate_deferred_joints()
        self.assertEqual(probe.resetup_count, 1)
        self.assertFalse(probe._awaiting_articulation_dofs)

    async def test_wait_is_not_spent_while_the_view_has_no_dofs(self) -> None:
        """The wait survives an arbitrary number of uncooked frames.

        Bounding it at one attempt would trade the flicker for a joint table that
        stays empty when the view takes several frames to cook.
        """
        probe = _DeferredRetryProbe(joint_entries=[], num_dofs=0)
        for _ in range(10):
            probe._maybe_repopulate_deferred_joints()
        self.assertEqual(probe.resetup_count, 0)
        self.assertTrue(probe._awaiting_articulation_dofs)

        # The view cooks on a later frame: the deferred retry is still available.
        probe._gains_tuner._articulation.num_dofs = 6
        probe._gains_tuner.get_joint_entries = lambda: ["joint0"]
        probe._maybe_repopulate_deferred_joints()
        self.assertEqual(probe.resetup_count, 1)
        self.assertFalse(probe._awaiting_articulation_dofs)

    async def test_render_step_holds_the_retry_back_during_an_override_recook(self) -> None:
        """The recook window must not re-run selection and drop the override.

        The re-setup resets the tuner, which restores the override the recook is
        installing, leaving the test to run with self-collisions still on.
        """
        probe = _DeferredRetryProbe(joint_entries=[])
        probe._restarting_for_override = True
        probe.sync_viewport_from_scroll_frame = lambda: None
        probe._articulation_menu_model = SimpleNamespace(has_item=lambda: True)
        probe._reset_ui_next_frame = False
        # Only reached once the guard is lifted; the poll is held back with the
        # re-setup because it re-runs selection the same way.
        probe._sync_backend_if_engine_changed = lambda: None

        for _ in range(10):
            probe.on_render_step(None)

        self.assertEqual(probe.resetup_count, 0)
        self.assertTrue(probe._awaiting_articulation_dofs, "the wait is held, not spent")

        # Once the recook lands, the deferred re-setup is free to run.
        probe._restarting_for_override = False
        probe.on_render_step(None)
        self.assertEqual(probe.resetup_count, 1)

    async def test_num_dofs_read_failure_counts_as_uncooked(self) -> None:
        """An articulation whose DOF query raises is treated as not yet cooked."""

        class _Raising:
            @property
            def num_dofs(self) -> int:
                raise RuntimeError("physics view not ready")

        probe = _DeferredRetryProbe(joint_entries=[])
        probe._gains_tuner._articulation = _Raising()
        self.assertEqual(probe._articulation_num_dofs(), 0)
        probe._maybe_repopulate_deferred_joints()
        self.assertEqual(probe.resetup_count, 0)


class TestSourceBadge(omni.kit.test.AsyncTestCase):
    """UI badge mapping for the four drive/actuator gain-source combos (A)."""

    async def test_source_badge_matrix(self) -> None:
        """Every (drive, actuator, mjc) combo maps to the expected badge text/style/subtitle."""
        # (has_pd, has_act, has_mjc) -> (text, style, subtitle)
        cases = [
            (True, True, True, "Multiple", "source_newton", "PhysicsDriveAPI + Newton Actuator + MuJoCo"),
            (True, True, False, "Both available", "source_newton", "PhysicsDriveAPI + Newton Actuator"),
            (True, False, True, "Drive + MuJoCo", "source_newton", "PhysicsDriveAPI + MuJoCo-native gains"),
            (False, True, True, "Actuator + MuJoCo", "source_newton", "Newton Actuator + MuJoCo-native gains"),
            (False, True, False, "Actuator only", "source_newton", "Newton Actuator"),
            (False, False, True, "MuJoCo only", "source_newton", "MuJoCo-native gains"),
            (True, False, False, "Drive only", "source_physics", "PhysicsDriveAPI"),
            (False, False, False, "No gains", "source_physics", "-"),
        ]
        for has_pd, has_act, has_mjc, text, style, subtitle in cases:
            got = UIBuilder._source_badge(has_pd=has_pd, has_act=has_act, has_mjc=has_mjc)
            self.assertEqual(got, (text, style, subtitle), f"badge for pd={has_pd} act={has_act} mjc={has_mjc}")

    async def test_mjc_default_is_false(self) -> None:
        """Omitting has_mjc keeps the original drive/actuator behavior."""
        self.assertEqual(UIBuilder._source_badge(has_pd=True, has_act=False)[0], "Drive only")
        self.assertEqual(UIBuilder._source_badge(has_pd=True, has_act=True)[0], "Both available")


class TestNaturalFrequencyModeGating(omni.kit.test.AsyncTestCase):
    """Gating of the PhysX natural-frequency tuning-mode toggle (A + B)."""

    @staticmethod
    def _entry(applied_schemas=()):
        """A fake joint entry whose joint reports the given applied schemas."""
        joint = SimpleNamespace(GetAppliedSchemas=lambda: list(applied_schemas))
        return SimpleNamespace(joint=joint, drive_axis=None)

    @staticmethod
    def _resolved(source, *, is_active=True, has_attrs=True):
        """A minimal resolved-gains stand-in for the gating check."""
        attr = object() if has_attrs else None
        return SimpleNamespace(source=source, is_active=is_active, kp_attr=attr, kd_attr=attr)

    async def test_default_tuning_mode_is_stiffness(self) -> None:
        """The default per-joint tuning mode edits stiffness/damping directly."""
        self.assertEqual(TuningMode.STIFFNESS, TuningMode(0))
        self.assertEqual(TuningMode.NATURAL_FREQUENCY, TuningMode(1))

    async def test_physics_drive_active_supports_mode(self) -> None:
        """Active PhysX PhysicsDrive gains support the natural-frequency toggle."""
        supported = UIBuilder._supports_natural_frequency_mode(
            None, self._entry(), self._resolved(GainSource.PHYSICS_DRIVE)
        )
        self.assertTrue(supported)

    async def test_actuator_source_does_not_support_mode(self) -> None:
        """Newton actuator gains never show the natural-frequency toggle."""
        supported = UIBuilder._supports_natural_frequency_mode(None, self._entry(), self._resolved(GainSource.ACTUATOR))
        self.assertFalse(supported)

    async def test_inactive_physics_drive_does_not_support_mode(self) -> None:
        """Read-only (inactive) PhysicsDrive gains do not show the toggle."""
        supported = UIBuilder._supports_natural_frequency_mode(
            None, self._entry(), self._resolved(GainSource.PHYSICS_DRIVE, is_active=False)
        )
        self.assertFalse(supported)

    async def test_mimic_joint_does_not_support_mode(self) -> None:
        """A mimic joint keeps its own gains and hides the toggle."""
        supported = UIBuilder._supports_natural_frequency_mode(
            None, self._entry(applied_schemas=["PhysxMimicJointAPI:rotZ"]), self._resolved(GainSource.PHYSICS_DRIVE)
        )
        self.assertFalse(supported)

    async def test_no_authored_attrs_does_not_support_mode(self) -> None:
        """PhysicsDrive with no stiffness/damping attributes hides the toggle."""
        supported = UIBuilder._supports_natural_frequency_mode(
            None, self._entry(), self._resolved(GainSource.PHYSICS_DRIVE, has_attrs=False)
        )
        self.assertFalse(supported)
