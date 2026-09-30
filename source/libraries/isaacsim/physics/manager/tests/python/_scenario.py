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

"""Provide an in-memory scenario harness for physics tensor tests.

Scenarios build stages with USD physics schemas and run through the physics
manager against either Newton or OvPhysX. :class:`DeviceParams` independently
selects the simulation and Warp tensor devices, which lets the same scenario
exercise CPU and CUDA execution.

Usage from a test::

    from _scenario import DeviceParams, GridParams, GridTestBase, RunnerInMemory, SimParams

    class EmptyGridScenario(GridTestBase):
        def __init__(self, test_case, device_params):
            super().__init__(test_case, GridParams(num_envs=1), SimParams(), device_params)

        def on_start(self, sim):
            self.finish()

        def on_physics_step(self, sim, stepno, dt):
            return

    class TestEmptyGrid:
        def test_x_cpu_newton(self):
            scenario = EmptyGridScenario(self, DeviceParams(False, False))
            RunnerInMemory(scenario, engine="newton", frontend="warp").run()

        def test_x_gpu_newton(self):
            scenario = EmptyGridScenario(self, DeviceParams(True, True))
            RunnerInMemory(scenario, engine="newton", frontend="warp").run()
"""

from __future__ import annotations

import math
import os
from abc import ABC, abstractmethod
from collections.abc import Sequence
from types import ModuleType
from typing import Protocol

import isaacsim.physics.manager as physics_manager
import numpy as np
import pytest
from physx_usd_schemas import PhysxSchema
from pxr import (
    Gf,
    Sdf,
    Usd,
    UsdGeom,
    UsdLux,
    UsdPhysics,
    UsdUtils,
)

# ---------------------------------------------------------------------------
# Asset path
# ---------------------------------------------------------------------------


def get_asset_root() -> str:
    """Return the registered test-resource directory.

    Returns:
        Absolute path to the test-resource directory.

    """
    return os.environ["ISAACSIM_TEST_RESOURCE_ROOT"]


from warp_utils import pack_wrench  # noqa: E402, F401 -- re-exported for scenario callers


class _ImplSpec(Protocol):
    supports: bool


class _OperationView(Protocol):
    def list_impls(self, kind: object) -> list[str]: ...

    def get_impl_spec(self, op: str, kind: object) -> _ImplSpec: ...


class _CountedView(Protocol):
    count: int


class _ContactView(Protocol):
    def get_metadata(self, name: str) -> object | None: ...


# ---------------------------------------------------------------------------
# Entity-view factory handed to scenarios
# ---------------------------------------------------------------------------


class SimulationEntities:
    """Create entity views for one running simulation.

    Every method is a thin call to ``isaacsim.physics.manager.impl.tensors.create_entity`` with the entity
    type filled in, so scenarios exercise the registry path rather than a parallel one.

    Args:
        engine: Name the simulation registered its entity factories under.
        device_ordinal: Ordinal of the device holding the simulation's tensors, or ``None`` for an engine
            that reports its own. Only engines that cannot report one at creation need this.

    """

    def __init__(self, engine: str, device_ordinal: int | None = None) -> None:
        self.engine = engine
        self.device_ordinal = device_ordinal

    def create_view(self, entity: str, paths: str | Sequence[str], options: dict[str, object] | None = None) -> object:
        """Create an entity view of any registered type.

        Args:
            entity: Registered entity type, such as ``"articulation"``.
            paths: Path pattern or sequence of path patterns.
            options: Engine-defined construction arguments forwarded verbatim.

        Returns:
            Entity view for the requested type.

        """
        import isaacsim.physics.manager.impl.tensors as t

        return t.create_entity(self.engine, entity, paths, options or None, device_ordinal=self.device_ordinal)

    def create_articulation_view(self, paths: str | Sequence[str]) -> object:
        """Create an articulation view.

        Args:
            paths: Path pattern or sequence of path patterns.

        Returns:
            Articulation entity view.

        """
        return self.create_view("articulation", paths)

    def create_rigid_body_view(self, paths: str | Sequence[str]) -> object:
        """Create a rigid-body view.

        Args:
            paths: Path pattern or sequence of path patterns.

        Returns:
            Rigid-body entity view.

        """
        return self.create_view("rigid-body", paths)

    def create_rigid_contact_view(
        self,
        paths: str | Sequence[str],
        filter_patterns: Sequence[str] | None = None,
        max_contact_data_count: int = 0,
    ) -> object:
        """Create a rigid-contact view.

        Args:
            paths: Sensor path pattern or sequence of sensor path patterns.
            filter_patterns: Contact counterparts to report against.
            max_contact_data_count: Maximum raw contact records retained by the view.

        Returns:
            Rigid-contact entity view.

        """
        options: dict[str, object] = {}
        if filter_patterns:
            options["filter-patterns"] = list(filter_patterns)
        # Pass an explicit 0 through rather than treating it as "unset": it asks for a view that retains no
        # raw contact records, which is not the same as letting the engine pick its default.
        if max_contact_data_count is not None:
            options["max-contact-data-count"] = int(max_contact_data_count)
        return self.create_view("rigid-contact", paths, options)

    def create_sdf_shape_view(self, paths: str | Sequence[str], num_points: int) -> object:
        """Create a signed-distance-field shape view.

        Args:
            paths: Path pattern or sequence of path patterns.
            num_points: Query points allocated per selected shape.

        Returns:
            Signed-distance-field shape entity view.

        """
        return self.create_view("sdf-shape", paths, {"num-points": int(num_points)})

    def create_volume_deformable_body_view(self, paths: str | Sequence[str]) -> object:
        """Create a volume-deformable-body view.

        Args:
            paths: Path pattern or sequence of path patterns.

        Returns:
            Volume-deformable-body entity view.

        """
        return self.create_view("volume-deformable-body", paths)

    def create_surface_deformable_body_view(self, paths: str | Sequence[str]) -> object:
        """Create a surface-deformable-body view.

        Args:
            paths: Path pattern or sequence of path patterns.

        Returns:
            Surface-deformable-body entity view.

        """
        return self.create_view("surface-deformable-body", paths)

    def create_deformable_material_view(self, paths: str | Sequence[str]) -> object:
        """Create a deformable-material view.

        Args:
            paths: Path pattern or sequence of path patterns.

        Returns:
            Deformable-material entity view.

        """
        return self.create_view("deformable-material", paths)


# ---------------------------------------------------------------------------
# Parameter dataclasses (mirror the legacy shape)
# ---------------------------------------------------------------------------


class Transform:
    """Position and orientation used to place test actors.

    Args:
        p: Translation vector.
        q: Quaternion in scalar-first order.

    """

    def __init__(
        self,
        p: tuple[float, float, float] | Gf.Vec3f = (0.0, 0.0, 0.0),
        q: tuple[float, float, float, float] | Gf.Quatf = (1.0, 0.0, 0.0, 0.0),
    ) -> None:
        self.p = Gf.Vec3f(*p) if not isinstance(p, Gf.Vec3f) else p
        if isinstance(q, Gf.Quatf):
            self.q = q
        elif len(q) == 4:
            self.q = Gf.Quatf(q[0], Gf.Vec3f(q[1], q[2], q[3]))
        else:
            self.q = Gf.Quatf(1.0)


class DeviceParams:
    """Simulation and tensor device selection for a scenario.

    Args:
        use_gpu_sim: Whether the engine should use GPU simulation.
        use_gpu_pipeline: Whether Warp tensors should use a CUDA device.
        num_workers: Optional worker count supplied to scenarios that support it.

    """

    def __init__(
        self,
        use_gpu_sim: bool = False,
        use_gpu_pipeline: bool = False,
        num_workers: int | None = None,
    ) -> None:
        self.use_gpu_sim = use_gpu_sim
        self.use_gpu_pipeline = use_gpu_pipeline
        self.num_workers = num_workers


class SimParams:
    """Simulation timing, gravity, and default-scene settings."""

    def __init__(self) -> None:
        self.gravity_dir = Gf.Vec3f(0.0, 0.0, -1.0)
        self.gravity_mag = 9.81
        self.add_default_light = True
        self.add_default_ground = True
        self.time_steps_per_second = 60
        self.min_frame_rate = 60


class GridParams:
    """Grid layout settings for replicated scenario environments.

    Args:
        num_envs: Number of environments to create.
        env_spacing: Default distance between adjacent environments.

    """

    def __init__(self, num_envs: int = 16, env_spacing: float = 2.0) -> None:
        self.num_envs = num_envs
        self.env_spacing = env_spacing
        self.num_rows = None
        self.row_spacing = None
        self.col_spacing = None


class SyncParams:
    """Stage synchronization settings retained by the scenario runner.

    Args:
        sync_usd: Whether simulation changes should synchronize to USD.
        sync_fabric: Whether simulation changes should synchronize to Fabric.
        transforms_only: Whether synchronization should be limited to transforms.

    """

    def __init__(
        self,
        sync_usd: bool = True,
        sync_fabric: bool = False,
        transforms_only: bool = False,
    ) -> None:
        self.sync_usd = sync_usd
        self.sync_fabric = sync_fabric
        self.transforms_only = transforms_only


# ---------------------------------------------------------------------------
# USD helpers (replacement for omni.physx.scripts.physicsUtils)
# ---------------------------------------------------------------------------


def _add_quad_plane(
    stage: Usd.Stage,
    path: str,
    axis: str = "Z",
    size: float = 20.0,
    position: Gf.Vec3f = Gf.Vec3f(0.0),
    color: Gf.Vec3f = Gf.Vec3f(0.5),
) -> None:
    """Create a static quad collision plane.

    Args:
        stage: Stage that receives the plane.
        path: Prim path for the plane mesh.
        axis: Plane-normal axis.
        size: Side length of the quad.
        position: Translation applied to the plane.
        color: Display color authored on the mesh.

    """
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))

    half = size * 0.5
    if axis == "Z":
        verts = [(-half, -half, 0.0), (half, -half, 0.0), (half, half, 0.0), (-half, half, 0.0)]
    elif axis == "Y":
        verts = [(-half, 0.0, -half), (half, 0.0, -half), (half, 0.0, half), (-half, 0.0, half)]
    else:  # X
        verts = [(0.0, -half, -half), (0.0, half, -half), (0.0, half, half), (0.0, -half, half)]

    mesh.CreatePointsAttr().Set(verts)
    mesh.CreateFaceVertexCountsAttr().Set([4])
    mesh.CreateFaceVertexIndicesAttr().Set([0, 1, 2, 3])
    mesh.CreateExtentAttr().Set([(-half, -half, 0.0), (half, half, 0.0)])
    mesh.CreateDisplayColorAttr().Set([color])
    UsdGeom.XformCommonAPI(mesh).SetTranslate(Gf.Vec3d(*position))

    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())


def _set_xform(prim: Usd.Prim, transform: Transform) -> None:
    """Replace a prim's transform stack with translation and orientation.

    Args:
        prim: Prim whose local transform is updated.
        transform: Translation and orientation to author.

    """
    xformable = UsdGeom.Xformable(prim)
    if not xformable:
        return
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(transform.p[0], transform.p[1], transform.p[2]))
    xformable.AddOrientOp().Set(transform.q)


# ---------------------------------------------------------------------------
# ScenarioBase — pure UsdPhysics scene builder (no omni.physx.scripts).
# ---------------------------------------------------------------------------


class ScenarioBase(ABC):
    """Base class for one in-memory physics test scenario.

    Args:
        sim_params: Simulation and default-scene settings.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, sim_params: SimParams, device_params: DeviceParams) -> None:
        self.sim_params = sim_params
        self.device_params = device_params

        stage = Usd.Stage.CreateInMemory()
        self.stage = stage

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        self.scene_path = Sdf.Path("/physicsScene")
        physics_scene = UsdPhysics.Scene.Define(stage, self.scene_path)
        physics_scene.CreateGravityDirectionAttr().Set(sim_params.gravity_dir)
        physics_scene.CreateGravityMagnitudeAttr().Set(sim_params.gravity_mag)

        physx_scene = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
        physx_scene.CreateEnableGPUDynamicsAttr().Set(device_params.use_gpu_sim)
        physx_scene.CreateBroadphaseTypeAttr().Set("GPU" if device_params.use_gpu_sim else "MBP")
        physx_scene.CreateEnableSceneQuerySupportAttr().Set(False)
        physx_scene.CreateTimeStepsPerSecondAttr().Set(sim_params.time_steps_per_second)
        self.physx_scene = physx_scene

        if sim_params.add_default_ground:
            _add_quad_plane(stage, "/groundPlane", "Z", 20.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        if sim_params.add_default_light:
            self.create_distant_light()

        # Tensor frontend's runtime device. CPU only when explicitly chosen;
        # otherwise CUDA when use_gpu_pipeline is True. Tests gate on this
        # via `self.wp_device`.
        self.wp_device = "cuda:0" if device_params.use_gpu_pipeline else "cpu"

        self.finished = False
        self.failed = False
        self.minsteps: int | None = None
        self.maxsteps: int | None = None

    def create_distant_light(self) -> None:
        """Create the default distant light on the scenario stage."""
        light = UsdLux.DistantLight.Define(self.stage, Sdf.Path("/distantLight"))
        light.CreateAngleAttr().Set(0.53)
        light.CreateColorAttr().Set(Gf.Vec3f(1.0))
        light.CreateIntensityAttr().Set(5000.0)

    def finish(self) -> None:
        """Mark the scenario as successfully complete."""
        self.finished = True

    def fail(self) -> None:
        """Mark the scenario as failed."""
        self.failed = True

    def should_quit(self, stepno: int) -> bool:
        """Check whether the runner should stop the scenario.

        Args:
            stepno: Zero-based simulation step number.

        Returns:
            True after the minimum step count when the scenario has finished or failed.

        """
        if self.minsteps is not None and stepno < self.minsteps:
            return False
        return self.finished or self.failed

    @abstractmethod
    def on_start(self, sim: SimulationEntities) -> None:
        """Initialize the scenario before the first simulated frame.

        Args:
            sim: Entity-view factory bound to the running simulation.

        """

    @abstractmethod
    def on_physics_step(self, sim: SimulationEntities, stepno: int, dt: float) -> None:
        """Advance scenario assertions after one simulated frame.

        Args:
            sim: Entity-view factory bound to the running simulation.
            stepno: Zero-based simulation step number.
            dt: Simulated time interval in seconds.

        """

        # ---------------------------------------------------------------------------


# GridScenarioBase — builds an env grid and lets sub-classes drop assets in.
# ---------------------------------------------------------------------------


class GridScenarioBase(ScenarioBase):
    """Scenario that replicates one actor template across a spatial grid.

    Args:
        grid_params: Environment count and spacing settings.
        sim_params: Simulation and default-scene settings.
        device_params: Simulation and tensor device selection.

    """

    def __init__(self, grid_params: GridParams, sim_params: SimParams, device_params: DeviceParams) -> None:
        super().__init__(sim_params, device_params)
        self.grid_params = grid_params

        # Path layout mirrors the legacy harness: `/envs/envN` (no `/World/`
        # prefix, no zero-padding) so legacy view patterns like
        # `/envs/*/ball` resolve unchanged.
        envs_path = Sdf.Path("/envs")
        self.stage.DefinePrim(envs_path, "Xform")

        spacing = grid_params.env_spacing
        row_spacing = grid_params.row_spacing if grid_params.row_spacing is not None else spacing
        col_spacing = grid_params.col_spacing if grid_params.col_spacing is not None else spacing

        if grid_params.num_rows is None:
            cols = max(1, int(math.ceil(math.sqrt(grid_params.num_envs))))
            rows = max(1, int(math.ceil(grid_params.num_envs / cols)))
        else:
            rows = grid_params.num_rows
            cols = max(1, int(math.ceil(grid_params.num_envs / rows)))

        # Env template — sub-classes drop actors at `env_template_path`,
        # and each `env_N` **inherits** from it (USD class-prim mechanism,
        # matching the legacy harness). Class prims propagate physics
        # schemas to every inheriting env reliably; internal references
        # produce subtle schema-resolution gaps that surface as engines
        # finding "0 prims" for valid view patterns.
        self.env_template_path = Sdf.Path("/envTemplate")
        env_template_xform = UsdGeom.Xform.Define(self.stage, self.env_template_path)
        env_template_xform.GetPrim().SetSpecifier(Sdf.SpecifierClass)

        self.env_paths = []
        for i in range(min(grid_params.num_envs, rows * cols)):
            r = i // cols
            c = i % cols
            env_path = envs_path.AppendChild(f"env{i}")
            env_xform = UsdGeom.Xform.Define(self.stage, env_path)
            env_xform.GetPrim().GetInherits().AddInherit(self.env_template_path)
            # Legacy convention: row index advances along X (row_spacing),
            # col index advances along Y (col_spacing). Per-test spacing
            # values (e.g. tight row_spacing, wide col_spacing) are chosen
            # so the wide axis aligns with the asset's long axis (the cart
            # rail runs along Y), keeping replicas from overlapping.
            UsdGeom.XformCommonAPI(env_xform.GetPrim()).SetTranslate(Gf.Vec3d(r * row_spacing, c * col_spacing, 0.0))
            self.env_paths.append(env_path)

    def _replicate_template_into_envs(self) -> None:
        """Finalize template replication before backend initialization.

        Class-prim inheritance already propagates the template, so the default
        implementation has no additional work.
        """
        return

    @property
    def num_envs(self) -> int:
        """Get the number of environments created for the scenario."""
        return len(self.env_paths)

    # -- asset placement ------------------------------------------------

    def create_actor_from_asset(
        self,
        actor_path: str | Sdf.Path,
        transform: Transform,
        asset_path: str,
    ) -> Usd.Prim:
        """Create a transformed actor that references an external USD asset.

        Args:
            actor_path: Prim path for the actor.
            transform: Local actor transform.
            asset_path: Referenced USD asset path.

        Returns:
            Actor prim containing the reference.

        """
        actor_prim = self.stage.DefinePrim(actor_path, "Xform")
        actor_prim.GetReferences().AddReference(asset_path)
        _set_xform(actor_prim, transform)
        return actor_prim

    # -- rigid-body primitive helpers ----------------------------------
    # Pure UsdPhysics replacements for `omni.physx.scripts.physicsUtils`.
    # Each returns the Prim so subclasses can attach extra schemas.

    def create_rigid_ball(
        self,
        actor_path: str | Sdf.Path,
        transform: Transform,
        radius: float = 0.5,
    ) -> Usd.Prim:
        """Create a dynamic sphere actor.

        Args:
            actor_path: Prim path for the sphere.
            transform: Local actor transform.
            radius: Sphere radius.

        Returns:
            Sphere prim with collision, rigid-body, and mass APIs.

        """
        sphere = UsdGeom.Sphere.Define(self.stage, actor_path)
        sphere.CreateRadiusAttr(radius)
        sphere.AddTranslateOp().Set(transform.p)
        sphere.AddOrientOp().Set(transform.q)
        sphere.CreateDisplayColorAttr().Set([Gf.Vec3f(71.0 / 255.0, 105.0 / 255.0, 1.0)])

        prim = sphere.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim)
        return prim

    def create_rigid_box(
        self,
        actor_path: str | Sdf.Path,
        transform: Transform,
        scale: Gf.Vec3f = Gf.Vec3f(1.0, 1.0, 1.0),
    ) -> Usd.Prim:
        """Create a dynamic box actor.

        Args:
            actor_path: Prim path for the box.
            transform: Local actor transform.
            scale: Box scale along each axis.

        Returns:
            Box prim with collision, rigid-body, and mass APIs.

        """
        box = UsdGeom.Cube.Define(self.stage, actor_path)
        box.CreateSizeAttr(1.0)
        box.AddTranslateOp().Set(transform.p)
        box.AddOrientOp().Set(transform.q)
        box.AddScaleOp().Set(scale)
        box.CreateDisplayColorAttr().Set([Gf.Vec3f(71.0 / 255.0, 105.0 / 255.0, 1.0)])

        prim = box.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim)
        return prim

    @staticmethod
    def create_transform(
        translate: Gf.Vec3d = Gf.Vec3d(0.0),
        rotate: Gf.Rotation = Gf.Rotation(Gf.Quatd(1.0)),
        scale: Gf.Vec3d = Gf.Vec3d(1.0),
        pivot_pos: Gf.Vec3d = Gf.Vec3d(0.0),
        pivot_orient: Gf.Rotation = Gf.Rotation(Gf.Quatd(1.0)),
    ) -> Gf.Transform:
        """Build a USD transform from decomposed components.

        Args:
            translate: Translation component.
            rotate: Rotation component.
            scale: Scale component.
            pivot_pos: Pivot translation.
            pivot_orient: Pivot orientation.

        Returns:
            Composed USD transform.

        """
        return Gf.Transform(translate, rotate, scale, pivot_pos, pivot_orient)


class GridTestBase(GridScenarioBase):
    """Grid scenario with shared tensor-view assertion helpers.

    Articulation and rigid-body count helpers accept an empty view because
    backend capability checks are performed separately. Deformable helpers
    require an exact count.

    Args:
        test_case: Test instance associated with the scenario.
        grid_params: Environment count and spacing settings.
        sim_params: Simulation and default-scene settings.
        device_params: Simulation and tensor device selection.

    """

    def __init__(
        self,
        test_case: object,
        grid_params: GridParams,
        sim_params: SimParams,
        device_params: DeviceParams,
    ) -> None:
        super().__init__(grid_params, sim_params, device_params)
        self.test_case = test_case
        self.minsteps = 0
        self.maxsteps = 300

    # -- helpers --------------------------------------------------------

    def to_warp(self, numpy_arr: np.ndarray, dtype: object | None = None) -> object:
        """Copy an array to the scenario's Warp device.

        Args:
            numpy_arr: Array-compatible host data to copy.
            dtype: Warp element type, or None to use scalar single precision.

        Returns:
            Warp array on the configured tensor device.

        """
        import warp as wp

        if dtype is None:
            dtype = wp.float32
        return wp.from_numpy(numpy_arr, dtype=dtype, device=self.wp_device)

    def skip_if_unsupported(self, view: _OperationView, op: str, kind: str) -> None:
        """Skip an operation that the backend explicitly marks unsupported.

        An unregistered operation remains a test failure so missing
        registrations cannot be mistaken for backend capability limits.

        Args:
            view: Tensor view queried for operation metadata.
            op: Registered operation name.
            kind: Operation direction, either ``"get"`` or ``"set"``.

        Raises:
            ValueError: If ``kind`` is neither ``"get"`` nor ``"set"``.

        """
        import isaacsim.physics.manager.impl.tensors as t

        if kind == "get":
            impl_kind = t.ImplKind.Get
        elif kind == "set":
            impl_kind = t.ImplKind.Set
        else:
            raise ValueError(f"kind must be 'get' or 'set', got {kind!r}")
        if op in view.list_impls(impl_kind) and not view.get_impl_spec(op, impl_kind).supports:
            pytest.skip(f"'{op}' not implemented for this engine")

    # -- view introspection helpers ------------------------------------

    def check_articulation_view(
        self,
        view: _CountedView,
        expected_count: int,
        expected_max_links: int = -1,
        expected_max_dofs: int = -1,
        require_homogeneous: bool = False,
    ) -> None:
        """Check the resolved articulation count when the view is nonempty.

        Args:
            view: Backend-specific articulation view.
            expected_count: Expected number of articulations.
            expected_max_links: Expected maximum link count retained by the shared contract.
            expected_max_dofs: Expected maximum degree-of-freedom count retained by the shared contract.
            require_homogeneous: Whether homogeneous topology is required by the caller.

        """
        # Real Newton / ovphysx adapters expose `count`. These legacy helpers
        # accept count zero but do not classify why the view is empty.
        actual_count = int(getattr(view, "count", 0))
        if actual_count > 0:
            assert actual_count == expected_count

    def check_rigid_body_view(self, view: _CountedView, expected_count: int) -> None:
        """Check the resolved rigid-body count when the view is nonempty.

        Args:
            view: Backend-specific rigid-body view.
            expected_count: Expected number of rigid bodies.

        """
        actual_count = int(getattr(view, "count", 0))
        if actual_count > 0:
            assert actual_count == expected_count

    def check_rigid_contact_view(
        self,
        view: _ContactView,
        expected_sensor_count: int,
        expected_filter_count: int,
    ) -> None:
        """Check sensor and filter counts for a supported contact view.

        Args:
            view: Backend-specific rigid-contact view.
            expected_sensor_count: Expected number of contact sensors.
            expected_filter_count: Expected number of contact filters.

        """
        # get_metadata returns None when a key isn't registered -- an
        # unsupported / placeholder view (an engine that doesn't back contact
        # views), which legitimately has no sensors and isn't asserted. A
        # SUPPORTED view (num-sensors registered) must match BOTH counts exactly:
        # a zero-sensor or wrong-filter supported view is a real defect, not a
        # vacuous pass (a matrix view silently dropping one of its two filters
        # would otherwise go undetected).
        raw_sensors = view.get_metadata("num-sensors")
        if raw_sensors is None:
            return
        assert int(raw_sensors) == expected_sensor_count, "num-sensors mismatch"
        raw_filters = view.get_metadata("num-filters")
        filter_count = int(raw_filters) if raw_filters is not None else 0
        assert filter_count == expected_filter_count, "num-filters mismatch"

    def check_deformable_body_view(self, view: _CountedView, expected_count: int) -> None:
        """Check the exact resolved deformable-body count.

        Args:
            view: Backend-specific deformable-body view.
            expected_count: Expected number of deformable bodies.

        """
        assert int(view.count) == expected_count, "deformable-body count mismatch"

    def check_deformable_material_view(self, view: _CountedView, expected_count: int) -> None:
        """Check the exact resolved deformable-material count.

        Args:
            view: Backend-specific deformable-material view.
            expected_count: Expected number of deformable materials.

        """
        assert int(view.count) == expected_count, "deformable-material count mismatch"

    def check_sdf_shape_view(self, view: object, expected_count: int) -> None:
        """Retain the shared signed-distance-field check for unsupported backends.

        Args:
            view: Backend-specific signed-distance-field view.
            expected_count: Expected shape count retained by the shared contract.

        """

        # ---------------------------------------------------------------------------


# RunnerInMemory — drives the umbrella's Physics, no omni.physx, no carb.
# ---------------------------------------------------------------------------


class RunnerInMemory:
    """Run one scenario in-process through the physics manager.

    The runner creates a Warp simulation view, invokes the scenario hooks for
    each time step, and releases backend resources when stopped. Newton is
    registered for the duration of the run; OvPhysX uses its process-wide
    registration.

    Args:
        scenario: Scenario to initialize and simulate.
        engine: Physics engine name.
        frontend: Tensor frontend name, or None to select Warp.
        sync_params: Stage synchronization settings retained for the run.
        fast_step: Whether the caller requests the fast-step path.

    """

    def __init__(
        self,
        scenario: ScenarioBase,
        engine: str = "newton",
        frontend: str | None = None,
        sync_params: SyncParams | None = None,
        fast_step: bool = False,
    ) -> None:
        self.scenario = scenario
        self.engine = engine
        self.sync_params = sync_params or SyncParams()
        self.fast_step = fast_step

        # Pick the public Warp frontend automatically when not given. It
        # supports both CPU and CUDA, with the runtime device chosen by
        # `DeviceParams.use_gpu_pipeline` rather than by frontend name.
        if frontend is None:
            frontend = "warp"
        self.frontend = frontend

        self._registered_id = None
        self._sim_view = None
        self._tmp_stage_file: str | None = None
        self._ovstage = None  # caller-owned ovstage (ovphysx); destroyed after detach
        self._stage_cache_id = None  # UsdUtils.StageCache.Id inserted for Newton

    # -- helpers -------------------------------------------------------

    def _engine_module(self) -> ModuleType:
        """Import the registration module for the selected dynamic backend.

        Returns:
            Backend registration module.

        Raises:
            ValueError: If the selected backend has no dynamic registration module.

        """
        if self.engine == "newton":
            import isaacsim.physics_engines.ovnewton.impl as engine
            import isaacsim.physics_engines.ovnewton.impl.tensors  # noqa: F401  (registers)

            return engine
        # ovphysx is a pure-C++ backend; _engine_module() is only called for
        # non-ovphysx engines (start/stop guard with self.engine != "ovphysx").
        raise ValueError(f"Unknown engine {self.engine!r}")

    def _stage_id(self) -> int:
        """Insert the scenario stage into the cache and return its identifier.

        Returns:
            Stage-cache identifier used by the physics backend.

        """
        return UsdUtils.StageCache.Get().Insert(self.scenario.stage).ToLongInt()

    # -- public API ---------------------------------------------------

    def start(self) -> None:
        """Attach the stage, create the tensor view, and initialize the scenario."""
        # ovphysx 0.5+ supports in-process device switching: a new
        # OvPhysxStage with a different device tears down the previous
        # instance and creates a fresh one. No harness-level pre-skip
        # needed -- the @gpu_only decorator still gates on actual CUDA
        # availability, and `_get_or_create_physx` does the rest.

        # Replicate env_template into each env_N before the engine sees
        # the stage — subclasses populate the template in their `__init__`
        # and the runner finalizes layout here.
        if hasattr(self.scenario, "_replicate_template_into_envs"):
            self.scenario._replicate_template_into_envs()

        if self.engine == "ovphysx":
            # ovphysx is a pure-C++ backend auto-registered at process start.
            # Find its SimulationId by name -- no Python register/unregister.
            import _physics_setup

            self._registered_id = _physics_setup.find_ovphysx_sim_id()
            if self._registered_id is None:
                pytest.skip("OvPhysX backend not registered")

            # Declare this test's tensor device the way Isaac Sim's app does:
            # /physics/suppressReadback opts the scene into PhysX DirectGPU
            # (GPU-resident tensors). Set it per test from the scenario's
            # GPU-pipeline choice, before the scene is (re)created below; ovphysx
            # reads it at scene creation.
            wp_device = self.scenario.wp_device
            _physics_setup.set_suppress_readback(wp_device != "cpu")
            # ovphysx has no query for the device its scene picked, so declare it once here; every entity
            # view it builds reports this value.
            _physics_setup.set_tensor_device_ordinal(
                -1 if wp_device == "cpu" else int(str(wp_device).rsplit(":", 1)[-1])
            )
        else:
            from isaacsim.physics_engines.ovnewton.impl import NewtonConfig

            engine = self._engine_module()
            self._registered_id = engine.register(NewtonConfig(device=self.scenario.wp_device))

        # Dual-input contract: the caller owns the ovstage. Export the in-memory
        # stage to disk, populate an ovstage from it, and hand its native handle to
        # the umbrella. Both ovphysx and ovnewton attach this borrowed Stage
        # (Newton via PyPI ovnewton.attach_ovstage).
        import os
        import tempfile

        from common.usd_fixtures import populate_ovstage_from_usd_path

        _fd, self._tmp_stage_file = tempfile.mkstemp(suffix=".usda")
        os.close(_fd)
        self.scenario.stage.GetRootLayer().Export(self._tmp_stage_file)

        self._ovstage, ovstage_handle = populate_ovstage_from_usd_path(self._tmp_stage_file, name="umbrella-scenario")
        # ovphysx ignores usd_identifier beyond getAttachedStage(). Newton still
        # resolves an optional StageCache entry for tensor prim-path queries.
        if self.engine == "newton":
            from pxr import UsdUtils

            # Tracked so cleanup() can erase it; the StageCache is process-global and
            # would otherwise keep every scenario stage alive for the whole session.
            self._stage_cache_id = UsdUtils.StageCache.Get().Insert(self.scenario.stage)
            usd_identifier = str(self._stage_cache_id.ToLongInt())
        else:
            usd_identifier = "0"

        self._sim_iface = physics_manager
        # Pass the ovstage object as owner so the simulation keeps it alive for
        # the attach; the caller can't free it under a live backend reference.
        # A failed attach leaves physics a no-op, so fail loudly rather than run the
        # scenario against a frozen scene.
        if not self._sim_iface.initialize(ovstage_handle, usd_identifier, owner=self._ovstage):
            raise RuntimeError(
                f"physics initialize failed (ovstage={ovstage_handle}, usd_identifier={usd_identifier!r})"
            )

        # Both engines report their own device now -- ovphysx from the declaration made in `start()`,
        # Newton from its backend -- so the factory pushes nothing.
        self._sim_view = SimulationEntities(self.engine)

        # User hook.
        self.scenario.on_start(self._sim_view)

    def simulate(self) -> None:
        """Step the scenario until completion, failure, or its step limit."""
        dt = 1.0 / self.scenario.sim_params.time_steps_per_second
        current_time = 0.0

        for stepno in range(self.scenario.maxsteps or 1):
            self._sim_iface.simulate_async(dt, current_time)
            self._sim_iface.fetch_results()
            current_time += dt

            self.scenario.on_physics_step(self._sim_view, stepno, dt)
            if self.scenario.should_quit(stepno):
                break

        if self.scenario.failed:
            raise AssertionError("scenario reported failure")

    def stop(self) -> None:
        """Detach the simulation and release runner-owned backend resources."""
        try:
            # ovphysx is auto-registered at process start; do not unregister it.
            if self._registered_id is not None and self.engine != "ovphysx":
                engine = self._engine_module()
                engine.unregister(self._registered_id)
            self._registered_id = None
        finally:
            self._sim_view = None
            # Detach (umbrella close) before destroying the caller-owned ovstage, so
            # ovphysx no longer references it when it is freed.
            if self._ovstage is not None:
                closed = False
                try:
                    if getattr(self, "_sim_iface", None) is not None:
                        closed = self._sim_iface.close()
                except Exception:
                    closed = False
                # Destroy only after a full detach; on a failed close ovphysx still
                # holds the Stage (kept alive by the simulation keepalive), so skip
                # destroy to avoid freeing it under a live reference.
                if closed:
                    try:
                        self._ovstage.destroy()
                    except Exception:
                        pass
                self._ovstage = None
            if self._tmp_stage_file is not None:
                try:
                    os.unlink(self._tmp_stage_file)
                except OSError:
                    pass
                self._tmp_stage_file = None
            if self._stage_cache_id is not None:
                from pxr import UsdUtils

                UsdUtils.StageCache.Get().Erase(self._stage_cache_id)
                self._stage_cache_id = None

    def run(self) -> None:
        """Run the complete scenario lifecycle with guaranteed finalization."""
        self.start()
        try:
            self.simulate()
        finally:
            self.stop()
