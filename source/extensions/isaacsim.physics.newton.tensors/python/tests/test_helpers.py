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

from __future__ import annotations

import math
import os

import numpy as np
import omni.kit.app
import omni.kit.test
import omni.physics.tensors as tensors
import omni.usd
import warp as wp
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from . import warp_utils


def get_asset_root() -> str:
    """Return the path to the data/usd directory shipped with this extension."""
    tests_dir = os.path.dirname(__file__)
    ext_root = os.path.abspath(os.path.join(tests_dir, "..", "..", "..", "..", ".."))
    return os.path.join(ext_root, "data", "usd")


def create_actor_from_asset(
    stage: Usd.Stage,
    actor_path: Sdf.Path | str,
    asset_path: str,
    position: Gf.Vec3f = Gf.Vec3f(0.0),
) -> Usd.Prim:
    """Load a USDA asset as a USD reference at the given path."""
    prim = stage.OverridePrim(Sdf.Path(actor_path))
    prim.GetReferences().AddReference(asset_path)
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(position)
    xf.AddOrientOp().Set(Gf.Quatf(1.0))
    return prim


def create_grid_scene(
    stage: Usd.Stage,
    asset_path: str,
    actor_child_name: str,
    num_envs: int,
    spacing: float = 2.0,
    position: Gf.Vec3f = Gf.Vec3f(0.0),
) -> Sdf.Path:
    """Build an /envTemplate + /envs/envN grid using USD inherits.

    Returns the template path and the number of envs actually created.
    """
    env_template_path = Sdf.Path("/envTemplate")
    env_template_xform = UsdGeom.Xform.Define(stage, env_template_path)
    env_template_xform.GetPrim().SetSpecifier(Sdf.SpecifierClass)

    actor_path = env_template_path.AppendChild(actor_child_name)
    create_actor_from_asset(stage, actor_path, asset_path, position)

    env_scope_path = Sdf.Path("/envs")
    UsdGeom.Scope.Define(stage, env_scope_path)

    num_rows = int(math.ceil(math.sqrt(num_envs)))
    num_cols = int(math.ceil(float(num_envs) / float(num_rows)))
    row_offset = 0.5 * spacing * (num_rows - 1)
    col_offset = 0.5 * spacing * (num_cols - 1)

    for i in range(num_envs):
        row = i // num_cols
        col = i % num_cols
        x = row_offset - row * spacing
        y = col * spacing - col_offset

        env_path = env_scope_path.AppendChild("env" + str(i))
        env_xform = UsdGeom.Xform.Define(stage, env_path)
        env_xform.GetPrim().GetInherits().AddInherit(env_template_path)
        env_xform.AddTranslateOp().Set(Gf.Vec3f(x, y, 0.0))

    return env_template_path


def create_rigid_ball(
    stage: Usd.Stage,
    path: Sdf.Path | str,
    position: Gf.Vec3f = Gf.Vec3f(0.0, 0.0, 0.5),
    radius: float = 0.15,
) -> Usd.Prim:
    """Create a rigid sphere with collision, rigid body, and mass APIs.

    Args:
        stage: USD stage.
        path: Sdf.Path or string for the sphere prim.
        position: Translation offset.
        radius: Sphere radius [m].

    Returns:
        The created UsdPrim.
    """
    sphere = UsdGeom.Sphere.Define(stage, Sdf.Path(path))
    sphere.GetRadiusAttr().Set(float(radius))
    prim = sphere.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim)
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(position)
    xf.AddOrientOp().Set(Gf.Quatf(1.0))
    return prim


class DeviceParams:
    """Describes the sim-device / view-device combination for a test.

    Args:
        sim_device: Device Newton simulation runs on (``"cpu"`` or ``"cuda:0"``).
        view_device: Device for tensor API views and user-facing arrays.
    """

    def __init__(self, sim_device: str, view_device: str):
        self.sim_device = sim_device
        self.view_device = view_device

    @property
    def suffix(self) -> str:
        s = "G" if self.sim_device.startswith("cuda") else "C"
        v = "G" if self.view_device.startswith("cuda") else "C"
        return s + v


ALL_DEVICE_CONFIGS = (
    DeviceParams("cpu", "cpu"),
    DeviceParams("cuda:0", "cpu"),
    DeviceParams("cuda:0", "cuda:0"),
)


class NewtonTensorTestBase(omni.kit.test.AsyncTestCase):
    """Base class for Newton tensor backend tests.

    Manages stage lifecycle, creates the simulation view with the Newton
    backend, and provides helpers for articulation-level and rigid-body
    assertions.

    Subclasses set ``SIM_DEVICE`` and ``DEVICE`` via the
    :func:`run_on_device_configs` decorator.
    """

    NUM_ENVS = 4
    SIM_DEVICE = "cpu"
    DEVICE = "cpu"

    async def setUp(self) -> None:
        await omni.usd.get_context().new_stage_async()
        self.stage = omni.usd.get_context().get_stage()
        self._stage_id = omni.usd.get_context().get_stage_id()

        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)

        scene = UsdPhysics.Scene.Define(self.stage, "/physicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        scene.CreateGravityMagnitudeAttr(9.81)

        self._sim = None

    async def tearDown(self) -> None:
        if self._sim is not None:
            try:
                tensors.reset()
            except Exception:
                pass
        await omni.usd.get_context().close_stage_async()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    async def create_sim(self) -> tensors.SimulationView:
        """Create a Newton-backed simulation view for the current stage."""
        await omni.kit.app.get_app().next_update_async()

        from isaacsim.physics.newton.impl.extension import acquire_stage as acquire_newton_stage

        newton_stage = acquire_newton_stage()
        if newton_stage is not None:
            newton_stage.initialize_newton(self.SIM_DEVICE)

        self._sim = tensors.create_simulation_view("warp", backend="newton", stage_id=self._stage_id)
        self.assertIsNotNone(self._sim, "Failed to create Newton simulation view")
        return self._sim

    def to_warp(self, numpy_arr: np.ndarray, dtype: type = wp.float32) -> wp.array:
        return wp.from_numpy(numpy_arr, dtype=dtype, device=self.DEVICE)

    def check_articulation_view(
        self,
        view: tensors.ArticulationView,
        expected_count: int,
        expected_max_links: int,
        expected_max_dofs: int,
    ) -> None:
        self.assertIsNotNone(view)
        self.assertEqual(view.count, expected_count)
        self.assertEqual(view.max_links, expected_max_links)
        self.assertEqual(view.max_dofs, expected_max_dofs)

    def check_rigid_body_view(self, view: tensors.RigidBodyView, expected_count: int) -> None:
        self.assertIsNotNone(view)
        self.assertEqual(view.count, expected_count)

    def start_playing(self) -> None:
        """Enable Newton simulation so step_sim() actually runs physics."""
        from isaacsim.physics.newton.impl.extension import acquire_stage as acquire_newton_stage

        newton_stage = acquire_newton_stage()
        if newton_stage is not None:
            newton_stage.playing = True

    def get_sim_dt(self) -> float:
        """Return the physics timestep used by SimulationManager if available, otherwise newton_stage.sim_dt."""
        import sys

        if "isaacsim.core.simulation_manager" in sys.modules:
            try:
                sm = sys.modules["isaacsim.core.simulation_manager"].SimulationManager
                if sm.get_active_physics_engine() == "newton":
                    return sm.get_physics_dt()
            except Exception:
                pass
        from isaacsim.physics.newton.impl.extension import acquire_stage as acquire_newton_stage

        newton_stage = acquire_newton_stage()
        return newton_stage.sim_dt if newton_stage else 1.0 / 60.0

    def step(self, n: int = 1, dt: float | None = None) -> None:
        """Step the Newton simulation n times.

        Args:
            n: Number of steps.
            dt: Per-step timestep. Defaults to Newton's configured sim_dt.
        """
        if dt is None:
            dt = self.get_sim_dt()
        for _ in range(n):
            self._sim.step(dt)

    def setup_cartpole_grid(self, num_envs: int | None = None) -> int:
        """Set up a grid of CartPole articulations from the USDA asset."""
        if num_envs is None:
            num_envs = self.NUM_ENVS
        asset_path = os.path.join(get_asset_root(), "CartPole.usda")
        create_grid_scene(self.stage, asset_path, "cartpole", num_envs, spacing=6.5, position=Gf.Vec3f(0.0, 0.0, 1.0))
        return num_envs

    def setup_ant_grid(self, num_envs: int | None = None) -> int:
        """Set up a grid of Ant articulations from the USDA asset."""
        if num_envs is None:
            num_envs = self.NUM_ENVS
        asset_path = os.path.join(get_asset_root(), "Ant.usda")
        create_grid_scene(self.stage, asset_path, "ant", num_envs, spacing=2.5, position=Gf.Vec3f(0.0, 0.0, 1.0))
        return num_envs

    def setup_humanoid_grid(self, num_envs: int | None = None) -> int:
        """Set up a grid of Humanoid articulations from the USDA asset."""
        if num_envs is None:
            num_envs = self.NUM_ENVS
        asset_path = os.path.join(get_asset_root(), "Humanoid.usda")
        create_grid_scene(self.stage, asset_path, "humanoid", num_envs, spacing=2.0, position=Gf.Vec3f(0.0, 0.0, 1.5))
        return num_envs

    def setup_ball_grid(
        self,
        num_envs: int | None = None,
        ball_name: str = "ball",
        radius: float = 0.15,
        position: Gf.Vec3f = Gf.Vec3f(0.0, 0.0, 0.5),
        spacing: float = 1.0,
    ) -> int:
        """Set up a grid of rigid balls.

        Args:
            num_envs: Number of environments.
            ball_name: Child prim name for each ball.
            radius: Sphere radius [m].
            position: Position offset for the ball within each env.
            spacing: Grid spacing [m].

        Returns:
            Number of envs created.
        """
        if num_envs is None:
            num_envs = self.NUM_ENVS

        env_template_path = Sdf.Path("/envTemplate")
        env_template_xform = UsdGeom.Xform.Define(self.stage, env_template_path)
        env_template_xform.GetPrim().SetSpecifier(Sdf.SpecifierClass)

        actor_path = env_template_path.AppendChild(ball_name)
        create_rigid_ball(self.stage, actor_path, position, radius)

        env_scope_path = Sdf.Path("/envs")
        UsdGeom.Scope.Define(self.stage, env_scope_path)

        num_rows = int(math.ceil(math.sqrt(num_envs)))
        num_cols = int(math.ceil(float(num_envs) / float(num_rows)))
        row_offset = 0.5 * spacing * (num_rows - 1)
        col_offset = 0.5 * spacing * (num_cols - 1)

        for i in range(num_envs):
            row = i // num_cols
            col = i % num_cols
            x = row_offset - row * spacing
            y = col * spacing - col_offset

            env_path = env_scope_path.AppendChild("env" + str(i))
            env_xform = UsdGeom.Xform.Define(self.stage, env_path)
            env_xform.GetPrim().GetInherits().AddInherit(env_template_path)
            env_xform.AddTranslateOp().Set(Gf.Vec3f(x, y, 0.0))

        return num_envs

    def check_rigid_contact_view(
        self, view: tensors.RigidContactView, expected_sensors: int, expected_filters: int
    ) -> None:
        self.assertIsNotNone(view)
        self.assertEqual(view.sensor_count, expected_sensors)
        self.assertEqual(view.filter_count, expected_filters)

    def setup_box_on_ground(
        self,
        num_envs: int | None = None,
        mass: float = 1.0,
        half_extent: float = 0.3,
        height: float | None = None,
        spacing: float = 2.0,
    ) -> int:
        """Set up a grid of rigid boxes above a ground plane.

        Args:
            num_envs: Number of environments.
            mass: Box mass [kg].
            half_extent: Box half-extent [m].
            height: Drop height [m]; defaults to half_extent.
            spacing: Grid spacing [m].

        Returns:
            Number of envs created.
        """
        if num_envs is None:
            num_envs = self.NUM_ENVS
        if height is None:
            height = half_extent

        create_ground_plane(self.stage)

        env_template_path = Sdf.Path("/envTemplate")
        env_template_xform = UsdGeom.Xform.Define(self.stage, env_template_path)
        env_template_xform.GetPrim().SetSpecifier(Sdf.SpecifierClass)

        box_path = env_template_path.AppendChild("box")
        box = create_rigid_box(self.stage, box_path, position=Gf.Vec3f(0.0, 0.0, height), half_extent=half_extent)
        mass_api = UsdPhysics.MassAPI(box)
        mass_api.GetMassAttr().Set(float(mass))

        env_scope_path = Sdf.Path("/envs")
        UsdGeom.Scope.Define(self.stage, env_scope_path)

        num_rows = int(math.ceil(math.sqrt(num_envs)))
        num_cols = int(math.ceil(float(num_envs) / float(num_rows)))
        row_offset = 0.5 * spacing * (num_rows - 1)
        col_offset = 0.5 * spacing * (num_cols - 1)

        for i in range(num_envs):
            row = i // num_cols
            col = i % num_cols
            x = row_offset - row * spacing
            y = col * spacing - col_offset
            env_path = env_scope_path.AppendChild("env" + str(i))
            env_xform = UsdGeom.Xform.Define(self.stage, env_path)
            env_xform.GetPrim().GetInherits().AddInherit(env_template_path)
            env_xform.AddTranslateOp().Set(Gf.Vec3f(x, y, 0.0))

        return num_envs


def create_ground_plane(stage: Usd.Stage, path: Sdf.Path | str = "/groundPlane") -> Usd.Prim:
    """Create a collision ground plane at z=0.

    Args:
        stage: USD stage.
        path: Prim path for the ground plane.

    Returns:
        The ground plane prim.
    """
    plane_geom = UsdGeom.Plane.Define(stage, path)
    plane_prim = stage.GetPrimAtPath(path)
    plane_geom.CreateAxisAttr("Z")
    UsdPhysics.CollisionAPI.Apply(plane_prim)
    return plane_prim


def create_rigid_box(
    stage: Usd.Stage,
    path: Sdf.Path | str,
    position: Gf.Vec3f = Gf.Vec3f(0.0, 0.0, 0.5),
    half_extent: float = 0.3,
) -> Usd.Prim:
    """Create a rigid box with collision, rigid body, and mass APIs.

    Args:
        stage: USD stage.
        path: Sdf.Path or string for the cube prim.
        position: Translation offset.
        half_extent: Cube half-extent [m].

    Returns:
        The created UsdPrim.
    """
    cube = UsdGeom.Cube.Define(stage, Sdf.Path(path))
    cube.GetSizeAttr().Set(float(half_extent * 2.0))
    prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim)
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(position)
    xf.AddOrientOp().Set(Gf.Quatf(1.0))
    return prim


def run_on_device_configs(configs: tuple[DeviceParams, ...] = ALL_DEVICE_CONFIGS):
    """Class decorator that generates a test class variant per :class:`DeviceParams`.

    For each config a new class is created with ``SIM_DEVICE`` and ``DEVICE``
    set accordingly and injected into the caller's module.

    Usage::

        @run_on_device_configs()
        class TestFoo(NewtonTensorTestBase):
            ...

    This produces ``TestFooCC``, ``TestFooGC``, and ``TestFooGG``.
    """
    import sys

    def decorator(cls):
        module = sys.modules[cls.__module__]
        first = None
        for cfg in configs:
            name = cls.__name__ + cfg.suffix
            variant = type(
                name,
                (cls,),
                {
                    "SIM_DEVICE": cfg.sim_device,
                    "DEVICE": cfg.view_device,
                },
            )
            variant.__module__ = cls.__module__
            if first is None:
                first = variant
            else:
                setattr(module, name, variant)
        return first

    return decorator
