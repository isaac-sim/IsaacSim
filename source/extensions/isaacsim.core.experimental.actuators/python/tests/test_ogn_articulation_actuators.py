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

"""Verifies the OgnArticulationActuators OmniGraph node driving articulated robots through Newton actuators. Covers auto-step and exec-driven execution, feedforward commands, DOF index inputs, robot path reinitialization, runtime toggles, and instance cleanup."""

from __future__ import annotations

import math

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.app
import omni.timeline
from isaacsim.core.experimental.prims import Articulation
from isaacsim.storage.native import get_assets_root_path
from pxr import Sdf

# ---------------------------------------------------------------------------
# Stage / asset constants — keep in sync with test_articulation_actuators.py
# ---------------------------------------------------------------------------

# Relative path inside the assets root for the robot USD.
_SIMPLE_ART_REL_PATH = "Isaac/Robots/IsaacSim/SimpleArticulation/simple_articulation.usd"

# Where we place the articulation root in our test stage.
_ART_ROOT = "/World/A_0"

# Absolute joint paths used for actuator authoring and DOF-index lookups.
_REVOLUTE_JOINT_PATH = "/World/A_0/Arm/RevoluteJoint"
_PRISMATIC_JOINT_PATH = "/World/A_0/Slider/PrismaticJoint"

# The fully-qualified OGN node type name.
# Derived from the extension name + the key inside the .ogn file:
#   extension:  isaacsim.core.experimental.actuators
#   .ogn key:   ArticulationActuators
_NODE_TYPE = "isaacsim.core.experimental.actuators.ArticulationActuators"


class TestOgnArticulationActuators(ogts.OmniGraphTestCase):
    """End-to-end tests for the OgnArticulationActuators wrapper node."""

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def setUp(self) -> None:
        """Prepare the OmniGraph actuator test stage."""
        # ogts.OmniGraphTestCase.setUp opens a new empty stage.  We must call
        # it before adding any prims so we start from a clean slate.
        await super().setUp()

        # Add a PhysicsScene so the physics simulator can run.
        stage_utils.define_prim("/World", "Xform")
        stage_utils.define_prim("/World/PhysicsScene", "PhysicsScene")

        # Add the SimpleArticulation reference.  It is a two-DOF robot
        # (revolute + prismatic) with no actuators authored in USD — we add
        # those in each test as needed.
        usd_path = f"{get_assets_root_path()}/{_SIMPLE_ART_REL_PATH}"
        stage_utils.add_reference_to_stage(usd_path=usd_path, path=_ART_ROOT)

        self._timeline = omni.timeline.get_timeline_interface()

    async def tearDown(self) -> None:
        """Clean up the OmniGraph actuator test stage."""
        # Stop the timeline before the stage is torn down so SimulationManager
        # callbacks are cleanly deregistered.
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        await super().tearDown()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _author_pd_actuator(
        self,
        actuator_prim_path: str,
        target_joint_path: Sdf.Path,
        *,
        kp: float = 100.0,
        kd: float = 10.0,
    ) -> None:
        """Write a NewtonActuator prim with a NewtonPDControlAPI to the stage.

        Args:
            actuator_prim_path: USD path where the NewtonActuator prim is created.
            target_joint_path: Absolute SDF path of the DOF this actuator drives.
            kp: Proportional gain (stiffness).
            kd: Derivative gain (damping).
        """
        stage = stage_utils.get_current_stage(backend="usd")
        prim = stage.DefinePrim(actuator_prim_path, "NewtonActuator")
        prim.AddAppliedSchema("NewtonPDControlAPI")
        prim.CreateAttribute("newton:kp", Sdf.ValueTypeNames.Float).Set(kp)
        prim.CreateAttribute("newton:kd", Sdf.ValueTypeNames.Float).Set(kd)
        prim.CreateRelationship("newton:targets").SetTargets([target_joint_path])

    def _build_graph(
        self,
        *,
        robot_path: str = _ART_ROOT,
        auto_step: bool = True,
        step_dt: float = 1.0 / 60.0,
        feedforward: list[float] | None = None,
        dof_indices: list[int] | None = None,
        connect_tick: bool = True,
    ) -> og.Graph:
        """Build an Action Graph containing our node and return it.

        The graph structure is:

            OnPlaybackTick ──(tick)──► ArticulationActuators

        ``OnPlaybackTick`` fires once per app update while the timeline is
        playing, so every physics step triggers one ``execIn`` pulse on our
        node.  Pass ``connect_tick=False`` to omit that connection (used to
        test the node receiving no pulses).

        Args:
            robot_path: Value written to ``inputs:robotPath``.
            auto_step: Value written to ``inputs:autoStepPrePhysics``.
            step_dt: Value written to ``inputs:stepDt``.
            feedforward: Optional default value for ``inputs:feedforwardCommand``.
            dof_indices: Optional default value for ``inputs:dofIndices``.
            connect_tick: Whether to wire OnPlaybackTick → execIn.

        Returns:
            The created ``og.Graph``.
        """
        set_values = [
            ("ArticulationActuators.inputs:robotPath", robot_path),
            ("ArticulationActuators.inputs:autoStepPrePhysics", auto_step),
            ("ArticulationActuators.inputs:stepDt", step_dt),
        ]
        if feedforward is not None:
            set_values.append(("ArticulationActuators.inputs:feedforwardCommand", feedforward))
        if dof_indices is not None:
            set_values.append(("ArticulationActuators.inputs:dofIndices", dof_indices))

        connections = []
        if connect_tick:
            connections.append(("OnPlaybackTick.outputs:tick", "ArticulationActuators.inputs:execIn"))

        graph, _nodes, _prims, _specs = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ArticulationActuators", _NODE_TYPE),
                ],
                og.Controller.Keys.SET_VALUES: set_values,
                og.Controller.Keys.CONNECT: connections,
            },
        )
        return graph

    # ------------------------------------------------------------------
    # 1. Smoke test — the node type must be registered
    # ------------------------------------------------------------------

    async def test_node_can_be_added_to_graph(self) -> None:
        """Verify that the OGN node type string resolves so the graph builds without error.

        If ``_NODE_TYPE`` is wrong (e.g. misspelled, extension not loaded), OGN
        raises a ValueError when the CREATE_NODES step runs.  A successful
        ``og.Controller.edit`` call here means the node was registered by the
        extension's build output.
        """
        # Just building the graph is the assertion.
        graph = self._build_graph()
        self.assertIsNotNone(graph)

    # ------------------------------------------------------------------
    # 2. auto_step_pre_physics=True → pre-physics callback drives motion
    # ------------------------------------------------------------------

    async def test_auto_step_true_drives_motion(self) -> None:
        """Verify that with `autoStepPrePhysics=True` the joint converges to its position target.

        Execution path in play:
          1. First OnPlaybackTick fires execIn → lazy init: ArticulationActuators
             is constructed with auto_step_pre_physics=True.
          2. PHYSICS_READY fires → the pre-physics callback is registered.
          3. Each physics tick: the pre-physics callback calls step_actuators(),
             which reads the position target from the Articulation and computes
             the PD effort.
          4. The execIn pulses from the tick are *no-ops for stepping* (auto_step
             is True) but they do call set_dof_feedforward_effort_targets if any
             feedforward is set.

        We author a PD actuator in USD so ArticulationActuators can discover it
        automatically.  The position target is set directly on the Articulation
        prim wrapper after physics starts.
        """
        # Author a stiff PD actuator on the revolute DOF.
        self._author_pd_actuator(
            f"{_ART_ROOT}/Actuators/revolute_act",
            Sdf.Path(_REVOLUTE_JOINT_PATH),
            kp=100.0,
            kd=100.0,
        )

        graph = self._build_graph(auto_step=True)
        await og.Controller.evaluate(graph)

        # Create an Articulation handle so we can set targets and read positions.
        # ArticulationActuators owns a second Articulation handle internally, but
        # this separate handle reads/writes the same USD prims.
        robot = Articulation(_ART_ROOT)
        dof_paths = robot.dof_paths  # shape: (n_robots, n_dofs)
        revolute_idx = dof_paths[0].index(_REVOLUTE_JOINT_PATH)

        app_utils.play()
        # Let the simulation run for a moment so physics initialises.
        await omni.kit.app.get_app().next_update_async()

        target_position = 0.05  # radians
        robot.set_dof_position_targets(target_position, dof_indices=revolute_idx)

        # Run 120 physics steps (~2 s at 60 Hz).  The PD controller should
        # converge the joint close to the target within this window.
        await app_utils.update_app_async(steps=120)

        final_pos = robot.get_dof_positions(dof_indices=revolute_idx).numpy().item()
        self.assertLess(
            math.fabs(final_pos - target_position),
            0.01,
            f"Revolute joint did not converge to {target_position} rad; got {final_pos}. "
            "If this fails, the pre-physics callback was likely not registered.",
        )

    # ------------------------------------------------------------------
    # 3. auto_step_pre_physics=False → execIn manually steps
    # ------------------------------------------------------------------

    async def test_auto_step_false_drives_motion_via_execin(self) -> None:
        """Verify that with `autoStepPrePhysics=False` the joint converges via manual steps on `execIn`.

        Execution path in play:
          1. First execIn → lazy init with auto_step_pre_physics=False.
          2. PHYSICS_READY fires → drive gains zeroed but pre-physics callback is
             NOT registered (auto_step is False).
          3. Each execIn pulse: node calls step_actuators(stepDt) directly.
             This is the *manual stepping* path.

        Comparing this test against test_auto_step_true_drives_motion confirms
        that both code paths produce the same physical result.
        """
        self._author_pd_actuator(
            f"{_ART_ROOT}/Actuators/revolute_act",
            Sdf.Path(_REVOLUTE_JOINT_PATH),
            kp=100.0,
            kd=100.0,
        )

        # auto_step=False → every execIn pulse calls step_actuators(stepDt).
        # We set stepDt explicitly so the controller sees a consistent dt.
        step_dt = 1.0 / 60.0
        graph = self._build_graph(auto_step=False, step_dt=step_dt)
        await og.Controller.evaluate(graph)

        robot = Articulation(_ART_ROOT)
        dof_paths = robot.dof_paths
        revolute_idx = dof_paths[0].index(_REVOLUTE_JOINT_PATH)

        app_utils.play()
        await omni.kit.app.get_app().next_update_async()

        target_position = 0.05
        robot.set_dof_position_targets(target_position, dof_indices=revolute_idx)

        await app_utils.update_app_async(steps=120)

        final_pos = robot.get_dof_positions(dof_indices=revolute_idx).numpy().item()
        self.assertLess(
            math.fabs(final_pos - target_position),
            0.01,
            f"Revolute joint did not converge in manual-step mode; got {final_pos}.",
        )

    # ------------------------------------------------------------------
    # 4. No execIn + auto_step=False → joint must not move
    # ------------------------------------------------------------------

    async def test_no_exec_in_no_auto_step_joint_stays_still(self) -> None:
        """Verify that without an `execIn` pulse and with `autoStepPrePhysics=False` the joint does not move.

        This is the *negative control* for the manual-stepping path.  It confirms
        that ``step_actuators`` is truly gated on ``execIn`` and that the node does
        not secretly drive the joint through some other code path.

        Setup
        -----
        * A PD actuator (Kp=100, Kd=10) is authored so the joint *would* converge
          if efforts were applied.
        * The graph is built with ``autoStepPrePhysics=False`` and ``connect_tick=False``,
          so ``OnPlaybackTick`` is **not** wired to ``execIn``.
        * ``feedforwardCommand=[100.0]`` is set as a static graph-attribute value
          (a large effort that would cause noticeable motion if ever applied).
        * We fire ``execIn`` exactly **once** via ``og.Controller.evaluate`` *before*
          play.  This lazy-initialises the node (constructing the
          ``ArticulationActuators``) and causes ``_on_physics_ready`` to subscribe.
          Critically, this single evaluate happens before physics starts, so
          ``_on_physics_ready`` hasn't run yet and no effort is applied.

        Why the initial evaluate is needed
        -----------------------------------
        Without it the node is never initialised and the ``_on_physics_ready``
        event subscription is never registered.  The PhysX drive gains from USD
        would then remain active and could move the joint on their own, which
        would make the "joint stays still" assertion unreliable.  By firing
        ``execIn`` once we ensure:

        1. The ``ArticulationActuators`` instance is created.
        2. ``_on_physics_ready`` fires when we call ``play()``, zeroing the PhysX
           drive gains so the joint has no hidden drive force acting on it.
        3. Because ``auto_step_pre_physics=False``, the pre-physics callback is
           *not* registered, so ``step_actuators`` is never called automatically.

        After play(), no further ``execIn`` pulses arrive (``OnPlaybackTick`` is not
        connected), so ``compute()`` is never called again, meaning:
        * ``set_dof_feedforward_effort_targets`` is never invoked.
        * ``step_actuators`` is never invoked.
        * The 100 N·m ``feedforwardCommand`` value sits in the graph as an inert
          attribute that nobody reads.

        Assertions
        ----------
        * Joint position stays near its start value (≤ 0.001 rad) — no motion.
        * ``get_dof_efforts()`` ≈ 0 — no effort was ever written to the physics
          simulation, which is the *direct cause* of no motion.  This makes the
          test self-explanatory: even if the joint had gravity or some friction
          artefact, zero efforts confirm the node never ran.
        """
        self._author_pd_actuator(
            f"{_ART_ROOT}/Actuators/revolute_act",
            Sdf.Path(_REVOLUTE_JOINT_PATH),
            kp=100.0,
            kd=10.0,
        )

        # Build the graph with manual stepping but *without* wiring OnPlaybackTick.
        # A large feedforward is pre-set so we can be sure it would move the joint
        # if it were ever applied.
        graph = self._build_graph(
            auto_step=False,
            step_dt=1.0 / 60.0,
            feedforward=[100.0],  # 100 N·m — would cause large displacement if applied
            connect_tick=False,
        )

        # Fire execIn exactly once to trigger lazy initialisation.  This happens
        # *before* play(), so _on_physics_ready has not yet zeroed the drive gains.
        # The feedforward value is read by compute() here, but step_actuators() also
        # runs once because auto_step=False.  We record the position *before* play
        # so we can confirm the joint doesn't continue moving during playback.
        await og.Controller.evaluate(graph)

        robot = Articulation(_ART_ROOT)
        revolute_idx = robot.dof_paths[0].index(_REVOLUTE_JOINT_PATH)

        app_utils.play()
        # _on_physics_ready fires here: zeros drive gains, does NOT register the
        # pre-physics callback (auto_step=False).  No further execIn pulses arrive.
        await omni.kit.app.get_app().next_update_async()

        # Record the position immediately after the first tick.  From this point
        # on, no efforts are applied and the joint should remain still.
        pos_after_init = robot.get_dof_positions(dof_indices=revolute_idx).numpy().item()

        # Run for many steps — if any stepping were occurring the joint would move.
        await app_utils.update_app_async(steps=120)

        final_pos = robot.get_dof_positions(dof_indices=revolute_idx).numpy().item()
        final_effort = robot.get_dof_efforts(dof_indices=revolute_idx).numpy().item()

        self.assertAlmostEqual(
            final_pos,
            pos_after_init,
            delta=0.001,
            msg=(
                f"Joint moved from {pos_after_init:.4f} to {final_pos:.4f} rad "
                "without any execIn pulse — step_actuators must have been called "
                "through an unexpected code path."
            ),
        )
        self.assertAlmostEqual(
            final_effort,
            0.0,
            delta=0.01,
            msg=(
                f"get_dof_efforts() returned {final_effort:.4f} N·m after playback "
                "with no execIn.  A non-zero value means step_actuators ran and "
                "wrote an effort to the physics simulation."
            ),
        )

    # ------------------------------------------------------------------
    # 5. feedforwardCommand input is forwarded to the actuator
    # ------------------------------------------------------------------

    async def test_feedforward_command_applied(self) -> None:
        """Verify that `feedforwardCommand` values reach the actuator and appear in `get_dof_efforts`.

        We author a zero-gain actuator so the only contribution to the output
        effort is the feedforward term.  Setting `feedforwardCommand=[F]` on the
        graph must result in `get_dof_efforts()` returning F (±floating-point
        tolerance).

        Execution path:
          execIn → set_dof_feedforward_effort_targets([F])
                 → step_actuators(dt)       (manual step, auto_step=False)
                 → Articulation.set_dof_efforts(F)
        """
        # A kp=kd=0 PD actuator passes the feedforward through unchanged.
        self._author_pd_actuator(
            f"{_ART_ROOT}/Actuators/revolute_act",
            Sdf.Path(_REVOLUTE_JOINT_PATH),
            kp=0.0,
            kd=0.0,
        )

        feedforward_effort = 7.5  # N·m

        # We use auto_step=False and connect to OnPlaybackTick so that each
        # physics step fires:  execIn → set_feedforward → step_actuators.
        # The feedforwardCommand default is set at graph-build time so it is
        # already in place before the first tick fires.
        graph = self._build_graph(
            auto_step=False,
            step_dt=1.0 / 60.0,
            feedforward=[feedforward_effort],
        )
        await og.Controller.evaluate(graph)

        robot = Articulation(_ART_ROOT)
        dof_paths = robot.dof_paths
        revolute_idx = dof_paths[0].index(_REVOLUTE_JOINT_PATH)

        app_utils.play()
        # Two updates: first fires the tick (init + feedforward + manual step),
        # second ensures the effort has been written back to PhysX.
        await app_utils.update_app_async(steps=2)

        applied = robot.get_dof_efforts(dof_indices=revolute_idx).numpy().item()
        self.assertAlmostEqual(
            applied,
            feedforward_effort,
            places=3,
            msg=(
                f"Expected effort {feedforward_effort} N·m from feedforward; got {applied}. "
                "The feedforwardCommand input was likely not forwarded correctly."
            ),
        )

    # ------------------------------------------------------------------
    # 6. dofIndices input narrows the feedforward target
    # ------------------------------------------------------------------

    async def test_feedforward_with_dof_indices_input(self) -> None:
        """Verify that the `dofIndices` input restricts `feedforwardCommand` to the named DOFs.

        We author zero-gain actuators on both DOFs, then send feedforward only to
        the revolute DOF using `dofIndices`.  The prismatic DOF must see zero effort.

        This exercises the path:
          set_dof_feedforward_effort_targets(values, dof_indices=dofIndices)
        inside `OgnArticulationActuators.compute`.
        """
        # Author zero-gain actuators on BOTH DOFs so both are under actuator
        # control and their drive gains are zeroed.
        self._author_pd_actuator(
            f"{_ART_ROOT}/Actuators/revolute_act",
            Sdf.Path(_REVOLUTE_JOINT_PATH),
            kp=0.0,
            kd=0.0,
        )
        self._author_pd_actuator(
            f"{_ART_ROOT}/Actuators/prismatic_act",
            Sdf.Path(_PRISMATIC_JOINT_PATH),
            kp=0.0,
            kd=0.0,
        )

        robot = Articulation(_ART_ROOT)
        dof_paths = robot.dof_paths[0]
        revolute_idx = dof_paths.index(_REVOLUTE_JOINT_PATH)
        prismatic_idx = dof_paths.index(_PRISMATIC_JOINT_PATH)

        feedforward_effort = 5.0  # N·m — applied to revolute only

        # Build the graph with:
        #   feedforwardCommand = [5.0]     (scalar, broadcast to selected DOF)
        #   dofIndices         = [revolute_idx]
        graph = self._build_graph(
            auto_step=False,
            step_dt=1.0 / 60.0,
            feedforward=[feedforward_effort],
            dof_indices=[revolute_idx],
        )
        await og.Controller.evaluate(graph)

        app_utils.play()
        await app_utils.update_app_async(steps=2)

        revolute_applied = robot.get_dof_efforts(dof_indices=revolute_idx).numpy().item()
        prismatic_applied = robot.get_dof_efforts(dof_indices=prismatic_idx).numpy().item()

        self.assertAlmostEqual(
            revolute_applied,
            feedforward_effort,
            places=3,
            msg=f"Revolute effort should be {feedforward_effort}; got {revolute_applied}.",
        )
        self.assertAlmostEqual(
            prismatic_applied,
            0.0,
            places=3,
            msg=(
                f"Prismatic effort should be 0.0 (no feedforward sent to it); got {prismatic_applied}. "
                "The dofIndices input was likely not forwarded correctly."
            ),
        )

    # ------------------------------------------------------------------
    # 7. Changing robotPath at runtime causes re-initialisation
    # ------------------------------------------------------------------

    async def test_robot_path_change_causes_reinitialization(self) -> None:
        """Verify that changing `inputs:robotPath` destroys the old `ArticulationActuators` and builds a new one.

        Procedure
        ---------
        1. Build the graph pointing at /World/A_0 (PD actuator on its revolute DOF).
        2. Play for 120 steps — the revolute joint on A_0 converges to a target.
        3. Stop.
        4. Add /World/A_1 with its own PD actuator on the revolute DOF.
        5. Change inputs:robotPath to /World/A_1.
        6. Replay — first OnPlaybackTick delivers execIn → reinit → _on_physics_ready
           zeroes A_1's drive gains.
        7. Assert that A_1's stiffness and damping are 0.0.

        Why zeroed drive gains prove re-initialisation
        -----------------------------------------------
        ``ArticulationActuators._on_physics_ready`` is the *only* thing that
        ever calls ``set_dof_gains(stiffnesses=0, dampings=0, ...)`` on A_1.
        The test framework never touches drive gains directly.  So if A_1's
        stiffness and damping are 0 after the first tick of Phase 2, a new
        ``ArticulationActuators`` was definitely constructed for A_1.

        A_1 must have at least one actuator for this to work: ``_on_physics_ready``
        only calls ``set_dof_gains`` when ``_actuated_dof_indices`` is non-empty.
        A bare articulation (no prims) would leave gains at their USD defaults
        and the assertion would never pass even if reinit happened correctly.

        This is faster than asserting on joint motion: only one physics step is
        needed in Phase 2, and there is no ambiguity from residual PhysX drives.

        Why we don't need an explicit og.Controller.evaluate() before replaying
        -----------------------------------------------------------------------
        Changing ``inputs:robotPath`` only updates the attribute value in the
        stage; ``compute()`` is not called until the next ``execIn`` pulse.  That
        pulse arrives automatically from the ``OnPlaybackTick`` connection: the
        first ``next_update_async()`` call after ``app_utils.play()`` fires the
        tick, delivers ``execIn``, and ``compute()`` detects
        ``state._robot_path != robot_path``, calls ``state.release()`` to close
        the old instance, and constructs a fresh ``ArticulationActuators`` for
        the new path.  No extra evaluate call is needed.
        """
        # Author a PD actuator on /World/A_0's revolute DOF.
        self._author_pd_actuator(
            f"{_ART_ROOT}/Actuators/revolute_act",
            Sdf.Path(_REVOLUTE_JOINT_PATH),
            kp=100.0,
            kd=100.0,
        )

        # ----- Phase 1: drive /World/A_0 (confirms the node works before the switch) -----
        graph = self._build_graph(robot_path=_ART_ROOT, auto_step=True)
        await og.Controller.evaluate(graph)

        robot_a0 = Articulation(_ART_ROOT)
        revolute_idx_a0 = robot_a0.dof_paths[0].index(_REVOLUTE_JOINT_PATH)

        app_utils.play()
        await omni.kit.app.get_app().next_update_async()  # first tick → lazy init for A_0
        robot_a0.set_dof_position_targets(0.05, dof_indices=revolute_idx_a0)
        await app_utils.update_app_async(steps=120)

        pos_a0 = robot_a0.get_dof_positions(dof_indices=revolute_idx_a0).numpy().item()
        self.assertLess(math.fabs(pos_a0 - 0.05), 0.01, f"Phase 1: A_0 did not converge; got {pos_a0}.")

        # ----- Phase 2: switch to /World/A_1 -----
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Add A_1 with a PD actuator on its revolute DOF.  The actuator is required
        # because _on_physics_ready only zeroes gains for DOFs that have an actuator
        # (it skips the set_dof_gains call when _actuated_dof_indices is empty).
        # Without it the gains would stay at their USD defaults and the assertion
        # below would always fail regardless of whether reinit happened.
        alt_root = "/World/A_1"
        alt_revolute = f"{alt_root}/Arm/RevoluteJoint"
        usd_path = f"{get_assets_root_path()}/{_SIMPLE_ART_REL_PATH}"
        stage_utils.add_reference_to_stage(usd_path=usd_path, path=alt_root)
        self._author_pd_actuator(
            f"{alt_root}/Actuators/revolute_act",
            Sdf.Path(alt_revolute),
            kp=100.0,
            kd=100.0,
        )

        robot_a1 = Articulation(alt_root)
        revolute_idx_a1 = robot_a1.dof_paths[0].index(alt_revolute)

        # initially the stiffness and damping should be non-zero
        stiffness, damping = robot_a1.get_dof_gains(dof_indices=revolute_idx_a1)
        self.assertNotEqual(stiffness.numpy().item(), 0.0, "A_1 stiffness must be non-zero")
        self.assertNotEqual(damping.numpy().item(), 0.0, "A_1 damping must be non-zero")

        app_utils.play()
        await omni.kit.app.get_app().next_update_async()
        # Point the graph at A_1.  The change is picked up on the next execIn
        # pulse
        og.Controller.attribute("inputs:robotPath", "/ActionGraph/ArticulationActuators").set(alt_root)

        # One update delivers the first OnPlaybackTick → execIn → compute() detects
        # the path change → reinitialises for A_1 → _on_physics_ready zeros gains.
        await omni.kit.app.get_app().next_update_async()

        stiffness, damping = robot_a1.get_dof_gains(dof_indices=revolute_idx_a1)
        self.assertEqual(
            stiffness.numpy().item(),
            0.0,
            "A_1 stiffness must be 0 — only ArticulationActuators._on_physics_ready "
            "zeroes drive gains.  A non-zero value means the node did not reinitialise.",
        )
        self.assertEqual(
            damping.numpy().item(),
            0.0,
            "A_1 damping must be 0 after reinitialisation.",
        )

    # ------------------------------------------------------------------
    # 8. Changing autoStepPrePhysics at runtime is reflected immediately
    # ------------------------------------------------------------------

    async def test_auto_step_toggle_at_runtime_does_not_crash(self) -> None:
        """Verify that toggling `autoStepPrePhysics` between True and False at runtime does not raise.

        This is intentionally a robustness / smoke test.  The full behavioural
        proof would require inspecting the node's internal state (which would need
        the generated OGN Database class).  Instead we verify:

        * True → False: the node detects the change, calls
          ``disable_auto_step_pre_physics()``, and then manually steps on the next
          execIn.  The joint continues to converge because manual stepping takes
          over from the pre-physics callback.
        * False → True: the pre-physics callback is re-registered and the joint
          keeps converging.
        * No exception is raised at any point.
        """
        self._author_pd_actuator(
            f"{_ART_ROOT}/Actuators/revolute_act",
            Sdf.Path(_REVOLUTE_JOINT_PATH),
            kp=100.0,
            kd=100.0,
        )

        # Start with autoStepPrePhysics=True.
        graph = self._build_graph(auto_step=True, step_dt=1.0 / 60.0)
        await og.Controller.evaluate(graph)

        robot = Articulation(_ART_ROOT)
        dof_paths = robot.dof_paths[0]
        revolute_idx = dof_paths.index(_REVOLUTE_JOINT_PATH)

        app_utils.play()
        await omni.kit.app.get_app().next_update_async()
        robot.set_dof_position_targets(0.05, dof_indices=revolute_idx)
        await app_utils.update_app_async(steps=30)

        # ----- Toggle True → False -----
        # The next execIn will see that state._auto_step_pre_physics (True) ≠
        # db.inputs.autoStepPrePhysics (now False), so it calls
        # disable_auto_step_pre_physics() and switches to manual stepping.
        og.Controller.attribute("inputs:autoStepPrePhysics", "/ActionGraph/ArticulationActuators").set(False)
        await app_utils.update_app_async(steps=30)

        # Joint should still be moving (manual stepping via execIn took over).
        pos_after_toggle = robot.get_dof_positions(dof_indices=revolute_idx).numpy().item()
        self.assertGreater(
            pos_after_toggle,
            0.0,
            "Joint must have moved after True→False toggle (manual stepping should continue).",
        )

        # ----- Toggle False → True -----
        # Re-enable: the pre-physics callback is re-registered.
        og.Controller.attribute("inputs:autoStepPrePhysics", "/ActionGraph/ArticulationActuators").set(True)
        await app_utils.update_app_async(steps=30)

        # Joint should continue converging.
        pos_final = robot.get_dof_positions(dof_indices=revolute_idx).numpy().item()
        self.assertGreater(
            pos_final,
            pos_after_toggle,
            "Joint must continue converging after False→True toggle.",
        )

    # ------------------------------------------------------------------
    # 9. release_instance tears down the inner ArticulationActuators
    # ------------------------------------------------------------------

    async def test_release_instance_closes_inner_actuators(self) -> None:
        """Verify that `release_instance` closes the inner `ArticulationActuators` deterministically.

        OmniGraph calls ``release_instance(node, graph_instance_id)`` when the node's
        graph instance is destroyed.  The hook must fetch the per-instance state
        and call ``state.release()``, which in turn calls
        ``ArticulationActuators.close()`` to deregister all `SimulationManager`
        callbacks owned by that instance.

        Procedure
        ---------
        1. Build the graph and fire one ``execIn`` pulse so lazy initialisation
           constructs the inner ``ArticulationActuators``.
        2. Cache references to the state and inner actuators *before* calling
           ``release_instance`` so we can inspect them afterwards (the state's
           own reference to ``_actuators`` is nulled by ``release()``).
        3. Sanity-check that the inner ``ArticulationActuators`` has its two
           lifecycle callbacks registered (PHYSICS_READY + SIMULATION_STOPPED)
           and that the per-instance state reports `initialized=True`.
        4. Invoke ``OgnArticulationActuators.release_instance`` directly with
           the node + graph_instance_id obtained from the graph.
        5. Assert the inner instance's `_lifecycle_callback_ids` is empty
           (`close()` was called) and the state's `_actuators` is `None`.
        """
        from isaacsim.core.experimental.actuators.nodes.OgnArticulationActuators import OgnArticulationActuators
        from isaacsim.core.experimental.actuators.ogn.OgnArticulationActuatorsDatabase import (
            OgnArticulationActuatorsDatabase,
        )

        self._author_pd_actuator(
            f"{_ART_ROOT}/Actuators/revolute_act",
            Sdf.Path(_REVOLUTE_JOINT_PATH),
            kp=100.0,
            kd=10.0,
        )

        # Build the graph and pump one app update while playing so OnPlaybackTick
        # fires once.  That pulse drives execIn on our node, which lazily
        # constructs the inner ArticulationActuators.
        graph = self._build_graph(auto_step=False, step_dt=1.0 / 60.0, connect_tick=True)
        await og.Controller.evaluate(graph)
        app_utils.play()
        await omni.kit.app.get_app().next_update_async()

        node = og.Controller.node("/ActionGraph/ArticulationActuators")
        # The per-instance state is keyed in og.Database by node.get_graph_instance_id()
        # (an opaque target id, NOT 0).  per_instance_internal_state() does that lookup
        # for us so we always read the same state object compute() populated.  We still
        # need the raw id below to invoke release_instance() with the same key the OGN
        # runtime would use.
        state = OgnArticulationActuatorsDatabase.per_instance_internal_state(node)
        graph_instance_id = node.get_graph_instance_id()

        self.assertTrue(state.initialized, "State must be initialised after the first execIn pulse.")
        inner = state._actuators
        self.assertIsNotNone(inner)
        self.assertEqual(
            len(inner._lifecycle_callback_ids),
            2,
            "Inner ArticulationActuators must have 2 lifecycle callbacks registered before release.",
        )

        # Trigger the OGN per-instance teardown hook directly.
        OgnArticulationActuators.release_instance(node, graph_instance_id)

        self.assertEqual(
            inner._lifecycle_callback_ids,
            [],
            "release_instance must call ArticulationActuators.close(), which clears _lifecycle_callback_ids.",
        )
        self.assertIsNone(
            state._actuators,
            "release_instance must null out the state's _actuators reference.",
        )
        self.assertFalse(state.initialized, "State must report initialized=False after release.")
