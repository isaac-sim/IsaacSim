# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from __future__ import annotations

import asyncio
import time

import carb.settings
import omni.kit.test
from isaacsim.asset.gen.conveyor import create_conveyor_belt
from pxr import Gf, PhysxSchema
from pxr import Sdf as PxrSdf
from pxr import UsdGeom, UsdPhysics, UsdShade
from usdrt import Sdf, Usd


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
async def simulate_async(seconds: float, steps_per_sec: int = 60) -> None:
    """Helper function to simulate async for seconds * steps_per_sec frames.

    Args:
        seconds: Time in seconds to simulate for.
        steps_per_sec: Steps per second.
    """
    for frame in range(int(steps_per_sec * seconds)):
        await omni.kit.app.get_app().next_update_async()


def add_cube(
    stage: Usd.Stage, path: str, size: float, offset: tuple[float, float, float], physics: bool = False
) -> Usd.Prim:
    """Creates a cube geometry primitive in the USD stage.

    Args:
        stage: The USD stage to create the cube in.
        path: Path where the cube primitive will be created.
        size: Size of the cube.
        offset: Translation offset as (x, y, z) coordinates.
        physics: Whether to apply rigid body physics to the cube.

    Returns:
        The created cube primitive.
    """
    cubeGeom = UsdGeom.Cube.Define(stage, path)
    cubePrim = stage.GetPrimAtPath(path)

    cubeGeom.CreateSizeAttr(size)
    cubeGeom.AddTranslateOp().Set(offset)
    if physics:
        rigid_api = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        rigid_api.CreateRigidBodyEnabledAttr(True)

    UsdPhysics.CollisionAPI.Apply(cubePrim)
    return cubePrim


def create_physics_scene(stage: Usd.Stage, gravity: float = 9.81) -> None:
    """Creates a physics scene with gravity and PhysX settings.

    Args:
        stage: The USD stage to create the physics scene in.
        gravity: Gravity magnitude value.
    """
    scene = UsdPhysics.Scene.Define(stage, "/physics")
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(gravity)

    PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physics"))
    physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/physics")
    physxSceneAPI.CreateEnableCCDAttr(True)
    physxSceneAPI.CreateEnableStabilizationAttr(True)
    physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
    physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
    physxSceneAPI.CreateSolverTypeAttr("TGS")


class TestConveyor(omni.kit.test.AsyncTestCase):
    """Test suite for conveyor belt functionality in Isaac Sim.

    This class provides comprehensive testing for the conveyor belt system, including creation,
    velocity control, physics interactions, and performance validation. It tests both linear
    and angular conveyor belt movements, physics-enabled and physics-disabled scenarios,
    and validates that objects placed on conveyor belts move correctly according to the
    conveyor's velocity and direction settings.

    The tests cover:
    - Creating conveyor belts from cube primitives
    - Setting linear and angular velocities
    - Validating physics interactions with objects on the conveyor
    - Testing conveyor belts in different directions
    - Performance testing with multiple conveyor belts
    - Proper cleanup and teardown procedures
    """

    # Before running each test
    async def setUp(self) -> None:
        """Set up the test environment before each test.

        Initializes the conveyor node, creates a new USD stage, sets up the timeline,
        and creates a physics scene for the test.
        """
        self.conveyor_node = None
        await omni.usd.get_context().new_stage_async()
        self._stage = omni.usd.get_context().get_stage()
        self._timeline = omni.timeline.get_timeline_interface()
        create_physics_scene(self._stage)
        await omni.kit.app.get_app().next_update_async()
        pass

    # After running each test
    async def tearDown(self) -> None:
        """Clean up the test environment after each test.

        Stops the timeline, clears the conveyor node, and waits for any loading assets
        to finish before proceeding.
        """
        await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()
        self.conveyor_node = None
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        pass

    async def test_add_conveyor(self, physics: bool = True) -> None:
        """Test creating a conveyor belt with a cube primitive.

        Args:
            physics: Whether to enable physics on the cube.
        """
        cube_prim = add_cube(self._stage, "/cube", 1.00, (0, 0, 0), physics=physics)
        rigid_prim = UsdPhysics.RigidBodyAPI(cube_prim)
        if rigid_prim:
            rigid_prim.GetKinematicEnabledAttr().Set(True)
        og_prim = create_conveyor_belt(self._stage, cube_prim)
        self.assertIsNotNone(og_prim)
        self.conveyor_node = og_prim
        self.velocity_attr = self._stage.GetPrimAtPath("/ConveyorBeltGraph").GetAttribute("graph:variable:Velocity")
        self.assertTrue(self.conveyor_node.IsValid())
        pass

    async def test_add_conveyor_without_physics(self) -> None:
        """Test creating a conveyor belt without physics enabled on the cube."""
        await self.test_add_conveyor(physics=False)

    async def test_set_velocity(self, direction: list[float] | None = None) -> None:
        """Test setting the velocity of a conveyor belt.

        Creates a conveyor belt, sets its direction and velocity, plays the timeline,
        and verifies that the surface velocity matches the expected value.

        Args:
            direction: Direction vector for the conveyor belt movement.
        """
        if direction is None:
            direction = [1.0, 0.0, 0.0]
        await self.test_add_conveyor()
        dir_attr = self.conveyor_node.GetAttribute("inputs:direction")
        dir_attr.Set(Gf.Vec3f(*direction))
        attr = self.conveyor_node.GetAttribute("inputs:velocity")
        self.velocity_attr.Set(0.10)
        self.assertAlmostEqual(self.velocity_attr.Get(), 0.10, delta=1e-4)
        self._timeline.play()
        await simulate_async(0.4)
        rigid_prim = UsdPhysics.RigidBodyAPI(self._stage.GetPrimAtPath("/cube"))
        surface_velocity = PhysxSchema.PhysxSurfaceVelocityAPI(rigid_prim)
        usd_velocity = surface_velocity.GetSurfaceVelocityAttr().Get()
        self.assertAlmostEqual(usd_velocity.GetLength(), 0.10, delta=1e-4)
        self._timeline.stop()
        pass

    async def test_set_angular_velocity(self, direction: list[float] | None = None) -> None:
        """Test setting the angular velocity of a curved conveyor belt.

        Creates a conveyor belt, enables curved mode, sets its direction and velocity,
        plays the timeline, and verifies that the surface angular velocity matches
        the expected value.

        Args:
            direction: Direction vector for the conveyor belt angular movement.
        """
        if direction is None:
            direction = [0.0, 0.0, 1.0]
        await self.test_add_conveyor()
        dir_attr = self.conveyor_node.GetAttribute("inputs:curved")
        dir_attr.Set(True)
        dir_attr = self.conveyor_node.GetAttribute("inputs:direction")
        dir_attr.Set(Gf.Vec3f(*direction))
        self.velocity_attr.Set(0.10)
        self.assertAlmostEqual(self.velocity_attr.Get(), 0.10, delta=1e-4)
        self._timeline.play()
        await simulate_async(0.4)
        rigid_prim = UsdPhysics.RigidBodyAPI(self._stage.GetPrimAtPath("/cube"))
        rigid_prim.GetKinematicEnabledAttr().Set(True)
        surface_velocity = PhysxSchema.PhysxSurfaceVelocityAPI(rigid_prim)
        usd_velocity = surface_velocity.GetSurfaceAngularVelocityAttr().Get()
        self.assertAlmostEqual(usd_velocity.GetLength(), 0.10, delta=1e-4)
        self._timeline.stop()
        pass

    async def test_conveyor(self, d: list[float] | None = None) -> None:
        """Test conveyor belt functionality with a moving object.

        Creates a conveyor belt with specified direction, places a small cube on top,
        runs the simulation, and verifies that the cube moves in the expected direction
        with the correct velocity.

        Args:
            d: Direction vector for the conveyor belt movement.
        """
        if d is None:
            d = [1.0, 0.0, 0.0]
        await self.test_set_velocity(d)

        cube_prim = add_cube(self._stage, "/cube2", 0.1, (0, 0, 0.55), physics=True)
        self._timeline.play()
        await simulate_async(1)
        rigid_prim = UsdPhysics.RigidBodyAPI(cube_prim)
        rt_stage = Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
        rt_prim = rt_stage.GetPrimAtPath(Sdf.Path(str(cube_prim.GetPath())))
        # usd_velocity = rigid_prim.GetVelocityAttr().Get()
        usd_velocity = rt_prim.GetAttribute(rigid_prim.GetVelocityAttr().GetName()).Get()
        self.assertAlmostEqual(d[0] * 0.1, usd_velocity[0], delta=1e-2)
        self.assertAlmostEqual(d[1] * 0.1, usd_velocity[1], delta=1e-2)
        self.assertAlmostEqual(d[2] * 0.1, usd_velocity[2], delta=1e-2)
        pass

    async def test_conveyor_y(self) -> None:
        """Test conveyor belt functionality with movement in the Y direction."""
        await self.test_conveyor(d=[0.0, 1.0, 0.0])

    async def test_100_conveyors(self) -> None:
        """Test performance with 100 conveyor belts.

        Creates a 10x10 grid of conveyor belts, each with different directional
        properties based on their position, runs the simulation, and measures
        performance over 100 frames.
        """
        conveyor_nodes = []
        for i in range(10):
            for j in range(10):
                cube_prim = add_cube(self._stage, f"/cube_{i}_{j}", 1.00, (i, j, 0), physics=True)
                og_prim = create_conveyor_belt(self._stage, cube_prim)
                self.assertIsNotNone(og_prim)
                conveyor_nodes.append(og_prim)
                self.assertTrue(conveyor_nodes[-1].IsValid())
                dir_attr = conveyor_nodes[-1].GetAttribute("inputs:direction")
                dir_attr.Set(Gf.Vec3f(*[float(i <= j), float(i > j), 0.0]))
                attr = conveyor_nodes[-1].GetAttribute("inputs:velocity")
                attr.Set(1)
        self._timeline.play()
        await simulate_async(1.0)
        t = time.time()
        # SImulate exacly 100 frames
        for i in range(100):
            await omni.kit.app.get_app().next_update_async()


def _bind_dummy_shader_material(stage, geom_prim, mat_path: str = "/World/Looks/ConveyorMat") -> PxrSdf.Path:
    """Author a minimal UsdShade Material + Shader pair and bind it to ``geom_prim``.

    The conveyor node walks the bound material's prim subtree and creates
    ``inputs:texture_translate`` on every shader prim it finds. The shader does not need
    to consume the attribute for these tests; we only assert that the OG node authors and
    animates the value, and (under FSD) mirrors it into Fabric.

    Returns the SdfPath of the shader prim so tests can read back the attribute directly.
    """
    UsdGeom.Scope.Define(stage, PxrSdf.Path(mat_path).GetParentPath())
    material = UsdShade.Material.Define(stage, mat_path)
    shader_path = PxrSdf.Path(mat_path + "/Shader")
    shader = UsdShade.Shader.Define(stage, shader_path)
    shader.CreateIdAttr("UsdPreviewSurface")
    surface_output = shader.CreateOutput("surface", PxrSdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(surface_output)
    UsdShade.MaterialBindingAPI(geom_prim).Bind(material, UsdShade.Tokens.strongerThanDescendants)
    return shader_path


class TestConveyorTextureAnimation(omni.kit.test.AsyncTestCase):
    """Tests for the conveyor texture-animation path in OgnIsaacConveyor.

    Coverage targets the FSD/USDRT mirror code added in 1.2.1 and the stop-time restore
    contract documented in the node header. These tests do not require real MDL shaders;
    a minimal UsdShade Shader prim is enough to exercise the cpp's attribute walk.
    """

    async def setUp(self) -> None:
        await omni.usd.get_context().new_stage_async()
        self._stage = omni.usd.get_context().get_stage()
        self._timeline = omni.timeline.get_timeline_interface()
        create_physics_scene(self._stage)
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()

    async def _build_conveyor_with_material(
        self, animate: bool = True, velocity: float = 1.0
    ) -> tuple[Usd.Prim, PxrSdf.Path]:
        """Build a kinematic conveyor body, bind a shader material, configure the OG node.

        Returns the (conveyor OG node prim, shader path).
        """
        cube_prim = add_cube(self._stage, "/cube", 1.0, (0.0, 0.0, 0.0), physics=True)
        UsdPhysics.RigidBodyAPI(cube_prim).GetKinematicEnabledAttr().Set(True)
        shader_path = _bind_dummy_shader_material(self._stage, cube_prim)
        og_prim = create_conveyor_belt(self._stage, cube_prim)
        self.assertTrue(og_prim.IsValid())
        og_prim.GetAttribute("inputs:animateTexture").Set(animate)
        og_prim.GetAttribute("inputs:animateScale").Set(1.0)
        og_prim.GetAttribute("inputs:animateDirection").Set(Gf.Vec2f(1.0, 0.0))
        og_prim.GetAttribute("inputs:direction").Set(Gf.Vec3f(1.0, 0.0, 0.0))
        velocity_var = self._stage.GetPrimAtPath("/ConveyorBeltGraph").GetAttribute("graph:variable:Velocity")
        velocity_var.Set(velocity)
        return og_prim, shader_path

    def _read_translate_usd(self, shader_path: PxrSdf.Path) -> Gf.Vec2f:
        """Read ``inputs:texture_translate`` from the USD stage. Defaults to (0, 0)."""
        attr = self._stage.GetPrimAtPath(shader_path).GetAttribute("inputs:texture_translate")
        if not attr:
            return Gf.Vec2f(0.0, 0.0)
        value = attr.Get()
        return value if value is not None else Gf.Vec2f(0.0, 0.0)

    def _read_translate_usdrt(self, shader_path: PxrSdf.Path) -> tuple[float, float] | None:
        """Read ``inputs:texture_translate`` from the USDRT (Fabric) view of the stage.

        Returns a tuple of floats, or ``None`` if the attribute is absent in Fabric.
        """
        rt_stage = Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
        rt_prim = rt_stage.GetPrimAtPath(Sdf.Path(str(shader_path)))
        if not rt_prim:
            return None
        attr = rt_prim.GetAttribute("inputs:texture_translate")
        if not attr:
            return None
        value = attr.Get()
        if value is None:
            return None
        return float(value[0]), float(value[1])

    async def test_texture_animates_in_usd(self) -> None:
        """Texture translation is monotonically advanced in USD while the timeline plays.

        Guards against a regression where the texture-animation block in compute() is
        skipped (e.g. wrong gating around `state.m_velocity != 0` or `inputs.animateTexture`).
        """
        og_prim, shader_path = await self._build_conveyor_with_material(animate=True, velocity=1.0)
        self._timeline.play()
        await simulate_async(0.5)

        t0 = self._read_translate_usd(shader_path)
        await simulate_async(0.5)
        t1 = self._read_translate_usd(shader_path)

        self.assertGreater(t1[0], t0[0], "texture_translate.x should advance while the conveyor runs")
        self.assertAlmostEqual(t0[1], 0.0, delta=1e-5)
        self.assertAlmostEqual(t1[1], 0.0, delta=1e-5)

    async def test_texture_disabled_does_not_animate(self) -> None:
        """When `inputs:animateTexture` is false the texture translation must not advance."""
        og_prim, shader_path = await self._build_conveyor_with_material(animate=False, velocity=1.0)
        self._timeline.play()
        await simulate_async(0.5)
        # The cpp still authors the attribute on first start so it can restore on stop;
        # we only assert it does not change between two later samples while play continues.
        t0 = self._read_translate_usd(shader_path)
        await simulate_async(0.5)
        t1 = self._read_translate_usd(shader_path)
        self.assertAlmostEqual(t0[0], t1[0], delta=1e-6)
        self.assertAlmostEqual(t0[1], t1[1], delta=1e-6)

    async def test_zero_velocity_does_not_animate(self) -> None:
        """Velocity 0 must short-circuit the texture-animation branch."""
        og_prim, shader_path = await self._build_conveyor_with_material(animate=True, velocity=0.0)
        self._timeline.play()
        await simulate_async(0.5)
        t0 = self._read_translate_usd(shader_path)
        await simulate_async(0.5)
        t1 = self._read_translate_usd(shader_path)
        self.assertAlmostEqual(t0[0], t1[0], delta=1e-6)
        self.assertAlmostEqual(t0[1], t1[1], delta=1e-6)

    async def test_stop_restores_initial_translation(self) -> None:
        """Stopping the timeline must restore the captured initial texture_translate value.

        Regression guard: the original code relied on a `requestCompute` from the StopPlay
        callback to re-enter the OG node and run the restore branch. With scheduling =
        compute-on-request and the action-graph evaluator paused at stop, that compute was
        unreliable and the restore was effectively dead code. The fix moves the restore work
        directly into the StopPlay event callback (mirroring `BaseResetNode`'s pattern).
        """
        og_prim, shader_path = await self._build_conveyor_with_material(animate=True, velocity=1.0)
        self._timeline.play()
        await simulate_async(0.5)

        advanced = self._read_translate_usd(shader_path)
        self.assertGreater(advanced[0], 0.0, "precondition: translation must have advanced before stop")

        self._timeline.stop()
        # Pump several ticks so any "tail" tick that OnPlaybackTick may fire after stop
        # has a chance to re-enter the OG node. Without the m_isPlaying guard in compute()
        # this tail tick re-authors one dt step on top of the just-restored zero, leaving
        # a visible drift of exactly one delta translation.
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        restored = self._read_translate_usd(shader_path)
        self.assertAlmostEqual(restored[0], 0.0, delta=1e-5)
        self.assertAlmostEqual(restored[1], 0.0, delta=1e-5)

    async def test_play_pause_play_stop_restores_to_true_baseline(self) -> None:
        """Pause/Play must not re-baseline `initialValue` to the live (advanced) value.

        Regression guard: a prior implementation called `_collectShaderAttributes` on every
        Play (including resume from Pause) and re-sampled the live USD value into
        `initialValue`. After Play/Pause/Play/Stop the texture would restore to the value
        reached at the *second* Play instead of the original baseline, leaking the in-between
        drift into the resting state. The fix keys prior `initialValue`s by shader path and
        preserves them across re-collects.
        """
        og_prim, shader_path = await self._build_conveyor_with_material(animate=True, velocity=1.0)
        baseline = self._read_translate_usd(shader_path)

        self._timeline.play()
        await simulate_async(0.5)
        self._timeline.pause()
        await omni.kit.app.get_app().next_update_async()
        # Confirm pause leaves the texture advanced (precondition for the regression).
        paused = self._read_translate_usd(shader_path)
        self.assertGreater(paused[0], baseline[0])

        self._timeline.play()
        await simulate_async(0.5)
        self._timeline.stop()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        restored = self._read_translate_usd(shader_path)
        self.assertAlmostEqual(restored[0], baseline[0], delta=1e-5)
        self.assertAlmostEqual(restored[1], baseline[1], delta=1e-5)

    async def test_stop_no_off_by_one_dt_drift(self) -> None:
        """A tail tick after stop must not advance the texture translation.

        Regression guard for the post-stop "exactly one dt" leak. After stopping the
        timeline the restore has already run in the StopPlay callback; the next tick
        from OnPlaybackTick (which can still fire once after stop) must be a no-op.
        """
        og_prim, shader_path = await self._build_conveyor_with_material(animate=True, velocity=1.0)
        self._timeline.play()
        await simulate_async(0.5)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        first = self._read_translate_usd(shader_path)
        # Capture the value across multiple post-stop ticks; it must not move.
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()
        last = self._read_translate_usd(shader_path)

        self.assertAlmostEqual(first[0], 0.0, delta=1e-5)
        self.assertAlmostEqual(first[1], 0.0, delta=1e-5)
        self.assertAlmostEqual(last[0], first[0], delta=1e-6)
        self.assertAlmostEqual(last[1], first[1], delta=1e-6)

    async def test_fabric_mirror_matches_usd_under_fsd(self) -> None:
        """Under FSD the Fabric attribute value must match the authored USD value.

        The conveyor extension's test config sets ``/app/useFabricSceneDelegate=true``;
        we assert the precondition explicitly so this test is honest about its scope.
        """
        if not carb.settings.get_settings().get_as_bool("/app/useFabricSceneDelegate"):
            self.skipTest("Fabric Scene Delegate is disabled in this test environment")

        og_prim, shader_path = await self._build_conveyor_with_material(animate=True, velocity=1.0)
        self._timeline.play()
        await simulate_async(0.5)

        usd_value = self._read_translate_usd(shader_path)
        usdrt_value = self._read_translate_usdrt(shader_path)
        self.assertIsNotNone(usdrt_value, "expected USDRT attribute to exist under FSD")
        self.assertAlmostEqual(usd_value[0], usdrt_value[0], delta=1e-4)
        self.assertAlmostEqual(usd_value[1], usdrt_value[1], delta=1e-4)
