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

"""Substep state-buffer handling of ``NewtonStage.simulate()``.

``simulate()`` ping-pongs ``state_0``/``state_1`` across substeps. With a
captured CUDA graph the replay always reads the buffers that were live at
capture time, and tensor views cache ``state_0``'s pointers when they are
created. Both only work if ``state_0`` names the same buffer at the end of
every frame, whatever the substep count and graph mode. The captured graph
also bakes in the substep count, so a live ``num_substeps`` change must
recapture it.
"""

import copy

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.physics.newton as newton_ext
import numpy as np
import omni.kit.app
import omni.kit.test
import omni.physics.tensors as physics_tensors
import omni.usd
import warp as wp
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.physics.newton import MuJoCoSolverConfig, configure_newton, get_newton_config
from isaacsim.physics.newton.impl.newton_stage import NewtonStage
from pxr import Gf, UsdGeom, UsdPhysics

_GRAVITY = 9.81
_DT = 1.0 / 60.0
_START_Z = 50.0


class TestNewtonStageSubsteps(omni.kit.test.AsyncTestCase):
    """Physics time, buffer identity and state writes for every substep count and graph mode."""

    async def setUp(self) -> None:
        """Method called to prepare the test fixture."""
        super().setUp()
        await stage_utils.create_new_stage_async()
        self.stage = stage_utils.get_current_stage()
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        self._saved_cfg = copy.deepcopy(get_newton_config())

    async def tearDown(self) -> None:
        """Method called immediately after the test method has been called."""
        app_utils.stop()
        await omni.kit.app.get_app().next_update_async()
        if self._saved_cfg is not None:
            configure_newton(self._saved_cfg)
        await omni.usd.get_context().close_stage_async()
        super().tearDown()

    async def _play_free_fall(self, num_substeps: int, use_cuda_graph: bool) -> tuple[NewtonStage, int]:
        """Build a contact-free falling cube, play it and return the live Newton stage.

        Args:
            num_substeps: Substeps per physics step.
            use_cuda_graph: Whether the physics step is captured in a CUDA graph.

        Returns:
            Tuple of the initialized Newton stage and the cube's body index.
        """
        scene = UsdPhysics.Scene.Define(self.stage, "/World/PhysicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr(_GRAVITY)
        SimulationManager.set_default_physics_scene("/World/PhysicsScene")
        cube = UsdGeom.Cube.Define(self.stage, "/World/Cube")
        cube.GetSizeAttr().Set(0.1)
        UsdGeom.Xformable(cube.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, _START_Z))
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(1.0)
        await omni.kit.app.get_app().next_update_async()

        cfg = copy.deepcopy(get_newton_config())
        if not isinstance(cfg.solver_cfg, MuJoCoSolverConfig):
            cfg.solver_cfg = MuJoCoSolverConfig()
        cfg.num_substeps = num_substeps
        cfg.use_cuda_graph = use_cuda_graph
        configure_newton(cfg)
        app_utils.play()
        stage = newton_ext.acquire_stage()
        self.assertIsNotNone(stage)
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()
            if stage.initialized and stage.model is not None:
                break
        self.assertTrue(stage.initialized, "Newton stage failed to initialize")
        # Stop the app loop from stepping so only explicit `step_sim()` calls advance physics.
        app_utils.pause()
        await omni.kit.app.get_app().next_update_async()
        stage.playing = True
        self.assertEqual(stage.cfg.num_substeps, num_substeps)
        self.assertEqual(bool(stage.cfg.use_cuda_graph), use_cuda_graph)
        labels = [str(label) for label in stage.model.body_label]
        body = next(i for i, label in enumerate(labels) if label.endswith("/World/Cube"))
        return stage, body

    async def _check_substeps(self, num_substeps: int, use_cuda_graph: bool) -> None:
        """Assert full-rate physics, a pinned state_0 and persistent tensor-view writes.

        Args:
            num_substeps: Substeps per physics step.
            use_cuda_graph: Whether the physics step is captured in a CUDA graph.
        """
        if use_cuda_graph and not wp.get_device().is_cuda:
            self.skipTest("CUDA graph capture requires a CUDA device")
        stage, body = await self._play_free_fall(num_substeps, use_cuda_graph)
        tag = f"num_substeps={num_substeps}, use_cuda_graph={use_cuda_graph}"

        # Warm up past the uncaptured first step and the capture step.
        for _ in range(3):
            stage.step_sim(_DT)
        pinned = stage.state_0
        vz0 = float(stage.state_0.body_qd.numpy()[body][2])

        frames = 30
        same_buffer = True
        for _ in range(frames):
            stage.step_sim(_DT)
            same_buffer = same_buffer and stage.state_0 is pinned
        vz1 = float(stage.state_0.body_qd.numpy()[body][2])
        # A free body gains g*dt of downward speed per frame. A replayed substep
        # shows up as (N-1)/N of that (0.75 at N=4).
        advanced = (vz0 - vz1) / (_GRAVITY * _DT * frames)
        self.assertAlmostEqual(advanced, 1.0, delta=0.01, msg=f"physics ran at {advanced:.3f}x ({tag})")

        # A write through the tensor API must survive the following frames.
        sim = physics_tensors.create_simulation_view("warp", backend="newton", stage_id=-1)
        view = sim.create_rigid_body_view("/World/Cube")
        indices = wp.array([0], dtype=wp.int32, device=view.get_transforms().device)
        target = [1.0, 2.0, 30.0, 0.0, 0.0, 0.0, 1.0]
        view.set_transforms(wp.array([target], dtype=wp.float32, device=indices.device), indices)
        view.set_velocities(wp.zeros((1, 6), dtype=wp.float32, device=indices.device), indices)
        for frame in range(4):
            stage.step_sim(_DT)
            state_z = float(stage.state_0.body_q.numpy()[body][2])
            view_z = float(view.get_transforms().numpy().reshape(-1, 7)[0][2])
            fall = 0.5 * _GRAVITY * ((frame + 1) * _DT) ** 2
            self.assertAlmostEqual(
                state_z, target[2] - fall, delta=0.01, msg=f"teleport lost after frame {frame + 1} ({tag})"
            )
            self.assertAlmostEqual(view_z, state_z, delta=1e-4, msg=f"view reads a stale buffer ({tag})")
        np.testing.assert_allclose(stage.state_0.body_q.numpy()[body][:2], target[:2], atol=1e-4)
        self.assertTrue(same_buffer, f"state_0 changed buffer between frames ({tag})")

    # --------------------------------------------------------------------
    async def test_graph_one_substep(self) -> None:
        """Test graph capture with one substep keeps state_0 and writes."""
        await self._check_substeps(1, True)

    async def test_graph_two_substeps(self) -> None:
        """Test graph capture with an even substep count runs at full rate and keeps writes."""
        await self._check_substeps(2, True)

    async def test_graph_three_substeps(self) -> None:
        """Test graph capture with three substeps keeps state_0 and writes."""
        await self._check_substeps(3, True)

    async def test_graph_four_substeps(self) -> None:
        """Test graph capture with four substeps runs at full rate and keeps writes."""
        await self._check_substeps(4, True)

    async def test_no_graph_one_substep(self) -> None:
        """Test uncaptured stepping with one substep keeps state_0 pinned for tensor views."""
        await self._check_substeps(1, False)

    async def test_no_graph_two_substeps(self) -> None:
        """Test uncaptured stepping with two substeps keeps state_0 and writes."""
        await self._check_substeps(2, False)

    async def test_no_graph_three_substeps(self) -> None:
        """Test uncaptured stepping with three substeps keeps state_0 pinned for tensor views."""
        await self._check_substeps(3, False)

    async def test_graph_recaptured_when_num_substeps_changes(self) -> None:
        """Test a live `num_substeps` change recaptures the graph instead of replaying the old count."""
        if not wp.get_device().is_cuda:
            self.skipTest("CUDA graph capture requires a CUDA device")
        stage, body = await self._play_free_fall(1, True)
        for _ in range(3):
            stage.step_sim(_DT)
        self.assertIsNotNone(stage.graph, "graph was not captured")
        captured_graph = stage.graph
        pinned = stage.state_0

        # Only a (re)capture runs the solver from Python; a graph replay never calls it.
        substep_dts: list[float] = []
        solver_step = stage.solver.step

        def counting_step(*args: object, **kwargs: object) -> None:
            substep_dts.append(float(args[-1]))  # type: ignore[arg-type]
            solver_step(*args, **kwargs)

        stage.solver.step = counting_step
        try:
            # `get_newton_config()` returns the live configuration of the running stage.
            get_newton_config().num_substeps = 3
            stage.step_sim(_DT)
            self.assertEqual(len(substep_dts), 3, "num_substeps change did not recapture the graph")
            np.testing.assert_allclose(substep_dts, [_DT / 3.0] * 3, rtol=1e-6)
            self.assertIsNot(stage.graph, captured_graph)
            # The following steps replay the new graph.
            stage.step_sim(_DT)
            self.assertEqual(len(substep_dts), 3, "recaptured graph was not replayed")
        finally:
            stage.solver.step = solver_step

        # Full-rate physics and the pinned buffer survive the recapture (odd substep count).
        vz0 = float(stage.state_0.body_qd.numpy()[body][2])
        frames = 30
        for _ in range(frames):
            stage.step_sim(_DT)
            self.assertIs(stage.state_0, pinned, "state_0 changed buffer after recapture")
        vz1 = float(stage.state_0.body_qd.numpy()[body][2])
        advanced = (vz0 - vz1) / (_GRAVITY * _DT * frames)
        self.assertAlmostEqual(advanced, 1.0, delta=0.01, msg=f"physics ran at {advanced:.3f}x after recapture")
