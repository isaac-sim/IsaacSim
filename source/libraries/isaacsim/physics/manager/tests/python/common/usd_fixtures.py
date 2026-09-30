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

"""Build cached USD stages for physics-correctness tests.

Each helper returns a :class:`StageHandle` that retains the
:class:`pxr.Usd.Stage`, an optional StageCache id, and a temporary USD file
path used to populate a caller-owned ovstage for engine initialize.

Use :func:`release_stage` in test ``teardown_method`` to drop the cache entry
and let the stage be garbage-collected.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdUtils


@dataclass
class StageHandle:
    """Cached USD stage and the resources needed to release it.

    Args:
        stage: Disk-backed stage instance.
        stage_id: Integer stage-cache identifier accepted by physics backends.
        cache_id: Stage-cache key used to erase the stage.
        file_path: Temporary USD file path, or None when already removed.
        ovstage: Caller-owned ovstage Stage when populated, otherwise None.
        ovstage_handle: Native ovstage address for ``physics_manager.initialize``.

    """

    stage: Usd.Stage
    stage_id: int
    cache_id: UsdUtils.StageCache.Id
    file_path: str | None = None  # populated when `save_to_file` is true
    ovstage: Any | None = None
    ovstage_handle: int = 0

    def release(self) -> None:
        """Erase the stage-cache entry and remove its temporary file."""
        UsdUtils.StageCache.Get().Erase(self.cache_id)
        if self.ovstage is not None:
            try:
                self.ovstage.destroy()
            except Exception:
                pass
            self.ovstage = None
            self.ovstage_handle = 0
        if self.file_path:
            import os

            try:
                os.unlink(self.file_path)
            except OSError:
                pass


def populate_ovstage_from_usd_path(usd_path: str, *, name: str = "umbrella-physics-test") -> tuple[Any, int]:
    """Create and populate a caller-owned ovstage from an on-disk USD file.

    Args:
        usd_path: Absolute path to a USD stage file.
        name: Optional ovstage instance name.

    Returns:
        ``(ovstage, native_handle)`` ready for ``physics_manager.initialize``.

    """
    from isaacsim.physics_engines.ovstage import get_native_handle, setup

    setup()
    import ovstage

    ov = ovstage.Stage(name)
    ovstage.population.open_usd(
        ov,
        usd_path,
        ordinal=1,
        time_code=0.0,
        domains=ovstage.PopulationDomain.RENDERING | ovstage.PopulationDomain.PHYSICS,
    )
    # Seal population payloads before ovnewton.attach_ovstage reads ordinal 1.
    ov.advance_write_floor(1).wait()
    return ov, get_native_handle(ov)


def populate_ovstage(stage_handle: StageHandle, *, name: str = "umbrella-physics-test") -> StageHandle:
    """Populate a caller-owned ovstage from a StageHandle's on-disk USD file.

    Args:
        stage_handle: Handle whose ``file_path`` supplies the USD scene.
        name: Optional ovstage instance name.

    Returns:
        The same handle with ``ovstage`` / ``ovstage_handle`` filled in.

    Raises:
        ValueError: If the handle has no on-disk USD path.

    """
    if not stage_handle.file_path:
        raise ValueError("StageHandle.file_path is required to populate an ovstage")

    if stage_handle.ovstage is not None:
        try:
            stage_handle.ovstage.destroy()
        except Exception:
            pass
    # Drop the previous references before repopulating: if Stage()/open_usd() below
    # raises, the handle must not survive as a dangling address that a later
    # `initialize` would hand to a backend, and `release()` must not double-destroy.
    stage_handle.ovstage = None
    stage_handle.ovstage_handle = 0

    stage_handle.ovstage, stage_handle.ovstage_handle = populate_ovstage_from_usd_path(
        stage_handle.file_path, name=name
    )
    return stage_handle


def _publish(stage: Usd.Stage, save_to_file: bool = True) -> StageHandle:
    """Publish an in-memory stage through a temporary file and stage cache.

    Args:
        stage: In-memory stage to export.
        save_to_file: Whether to retain the temporary USD file after caching.

    Returns:
        Handle containing the disk-backed stage and its cache identifiers.

    """
    import os
    import tempfile

    fd, file_path = tempfile.mkstemp(suffix=".usda", prefix="umbrella_test_")
    os.close(fd)
    stage.GetRootLayer().Export(file_path)

    # Open a new stage instance from the file so GetIdentifier() is the path.
    disk_stage = Usd.Stage.Open(file_path)
    cache_id = UsdUtils.StageCache.Get().Insert(disk_stage)

    if not save_to_file:
        # Caller doesn't need the file; clean it up immediately after caching.
        try:
            os.unlink(file_path)
        except OSError:
            pass
        file_path = None

    return StageHandle(
        stage=disk_stage,
        stage_id=cache_id.ToLongInt(),
        cache_id=cache_id,
        file_path=file_path,
    )


def _add_physics_scene(stage: Usd.Stage, gravity_y: float = -9.81) -> None:
    """Add a physics scene with gravity along the Y axis.

    Args:
        stage: Stage that receives the physics scene.
        gravity_y: Signed Y-axis gravitational acceleration.

    """
    scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, gravity_y / abs(gravity_y) if gravity_y else -1.0, 0.0))
    scene.CreateGravityMagnitudeAttr(abs(gravity_y))


def _add_dynamic_cube(
    stage: Usd.Stage,
    path: str,
    position: tuple[float, float, float],
    size: float = 1.0,
    mass: float = 1.0,
) -> UsdGeom.Cube:
    """Add a dynamic cube with collision, rigid-body, and mass schemas.

    Args:
        stage: Stage that receives the cube.
        path: Prim path for the cube.
        position: Cube translation.
        size: Cube side length.
        mass: Cube mass.

    Returns:
        Authored cube schema.

    """
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(size)
    cube.AddTranslateOp().Set(Gf.Vec3d(*position))

    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(cube.GetPrim())
    mass_api.CreateMassAttr(mass)
    return cube


def _add_static_ground(
    stage: Usd.Stage,
    path: str = "/World/Ground",
    extent: float = 50.0,
) -> UsdGeom.Cube:
    """Add a static cube whose top surface lies at ground height.

    Args:
        stage: Stage that receives the ground.
        path: Prim path for the ground.
        extent: Cube side length and ground coverage.

    Returns:
        Authored ground cube schema.

    """
    ground = UsdGeom.Cube.Define(stage, path)
    ground.CreateSizeAttr(extent)
    # Translate down by half the extent so the top sits at y=0.
    ground.AddTranslateOp().Set(Gf.Vec3d(0.0, -extent * 0.5, 0.0))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    return ground


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


def freefall_stage(initial_height: float = 10.0) -> StageHandle:
    """Create a freefall stage with one dynamic cube.

    Args:
        initial_height: Initial Y coordinate of the cube.

    Returns:
        Cached stage handle for a ground-free negative-Y gravity scene.

    """
    stage = Usd.Stage.CreateInMemory("freefall.usda")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    UsdGeom.Xform.Define(stage, "/World")
    _add_physics_scene(stage)
    _add_dynamic_cube(stage, "/World/Cube", (0.0, initial_height, 0.0))
    return _publish(stage)


def cube_on_ground_stage(initial_height: float = 5.0) -> StageHandle:
    """Create a stage with a dynamic cube above static ground.

    Args:
        initial_height: Initial Y coordinate of the cube.

    Returns:
        Cached stage handle for the cube-and-ground scene.

    """
    stage = Usd.Stage.CreateInMemory("cube_on_ground.usda")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    UsdGeom.Xform.Define(stage, "/World")
    _add_physics_scene(stage)
    _add_static_ground(stage)
    _add_dynamic_cube(stage, "/World/Cube", (0.0, initial_height, 0.0))
    return _publish(stage)


def get_cube_world_y(stage: Usd.Stage, cube_path: str = "/World/Cube") -> float | None:
    """Get a cube's world-space Y coordinate.

    Args:
        stage: Stage containing the cube.
        cube_path: Prim path of the cube.

    Returns:
        World-space Y coordinate, or None when the prim is missing.

    """
    prim = stage.GetPrimAtPath(cube_path)
    if not prim.IsValid():
        return None
    xform = UsdGeom.Xformable(prim)
    matrix = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return float(matrix.ExtractTranslation()[1])
