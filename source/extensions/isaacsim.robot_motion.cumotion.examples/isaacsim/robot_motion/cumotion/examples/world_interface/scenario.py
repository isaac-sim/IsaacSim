# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""CumotionWorldInterface example for obstacle discovery and world synchronization."""

from typing import Literal

import omni.kit.app
from isaacsim.core.experimental.objects import Cone, Cube, Cylinder, Mesh
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.core.experimental.utils import prim as prim_utils
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot_motion.cumotion import CumotionWorldInterface
from isaacsim.robot_motion.experimental.motion_generation import (
    ObstacleConfiguration,
    ObstacleStrategy,
    SceneQuery,
    TrackableApi,
    WorldBinding,
)
from pxr import UsdPhysics

_UpdateStyle = Literal["synchronize_transforms", "synchronize_properties", "synchronize"]
_TrackedCollisionApi = Literal["physics", "motion_generation"]

_OBSTACLE_PRIM_PATH = "/World/obstacle"
_PHYSICS_SCENE_PATH = "/World/PhysicsScene"


class CumotionWorldInterfaceExample:
    """CumotionWorldInterface demo.

    Demonstrates how to:
      - Set up a CumotionWorldInterface with WorldBinding
      - Discover obstacles using SceneQuery
      - Configure obstacle representations
      - Synchronize world state with a chosen update style
    """

    def __init__(self) -> None:
        self._world_binding: WorldBinding | None = None
        self._update_style: _UpdateStyle = "synchronize_transforms"
        self._tracked_collision_api: _TrackedCollisionApi = "physics"

    # --------------------------------------------------------------- config

    def set_update_style(self, style: _UpdateStyle) -> None:
        """Set the world-binding update style used by :meth:`update`.

        Args:
            style: One of ``synchronize_transforms``, ``synchronize_properties``, ``synchronize``.
        """
        if style not in ("synchronize_transforms", "synchronize_properties", "synchronize"):
            raise ValueError(
                f"Invalid update style: {style}. "
                "Must be one of: synchronize_transforms, synchronize_properties, synchronize"
            )
        self._update_style = style

    def set_tracked_collision_api(self, api: _TrackedCollisionApi) -> None:
        """Set which collision API is used to discover obstacles.

        Args:
            api: Either ``physics`` or ``motion_generation``.
        """
        if api not in ("physics", "motion_generation"):
            raise ValueError(f"Invalid collision API: {api}. Must be one of: physics, motion_generation")
        self._tracked_collision_api = api

    # ---------------------------------------------------------------- loading

    async def load(self) -> None:
        """Create a fresh stage, place an obstacle, allocate physics, and prime the world binding."""
        await stage_utils.create_new_stage_async(template="default stage")
        stage_utils.set_stage_up_axis("Z")
        stage_utils.set_stage_units(meters_per_unit=1.0)

        # Obstacle cube with both collision and dynamics APIs.
        Cube(_OBSTACLE_PRIM_PATH, sizes=0.1, positions=[0.4, 0.0, 0.65])
        GeomPrim(_OBSTACLE_PRIM_PATH, apply_collision_apis=True)
        RigidPrim(_OBSTACLE_PRIM_PATH, masses=[1.0])

        ViewportManager.set_camera_view(camera="/OmniverseKit_Persp", eye=[2, 1.5, 2], target=[0, 0, 0])

        # Ensure a physics scene exists; allocate physics tensors without stepping.
        stage = stage_utils.get_current_stage()
        if not stage.GetPrimAtPath(_PHYSICS_SCENE_PATH).IsValid():
            UsdPhysics.Scene.Define(stage, _PHYSICS_SCENE_PATH)
        await omni.kit.app.get_app().next_update_async()
        if SimulationManager.get_physics_sim_view() is None:
            SimulationManager.initialize_physics()

        self.setup()

    def setup(self) -> None:
        """Discover obstacles and build the cuMotion world binding."""
        # Drop any prior world binding so its GPU resources can be reclaimed.
        if self._world_binding is not None:
            self._cleanup_debug_prims()
            self._world_binding = None
        self._cleanup_debug_prims()

        scene_query = SceneQuery()
        objects = scene_query.get_prims_in_aabb(
            search_box_origin=[0.0, 0.0, 0.0],
            search_box_minimum=[-100.0, -100.0, -100.0],
            search_box_maximum=[100.0, 100.0, 100.0],
            tracked_api=TrackableApi.PHYSICS_COLLISION,
        )

        # NOTE: try changing the representation to TRIANGULATED_MESH or the safety
        # tolerance to 0.05 / 0.1 / 0.2 to see how cuMotion treats obstacles.
        obstacle_strategy = ObstacleStrategy()
        obstacle_strategy.set_default_safety_tolerance(0.06)
        obstacle_strategy.set_default_configuration(Mesh, ObstacleConfiguration("obb", 0.01))
        obstacle_strategy.set_default_configuration(Cone, ObstacleConfiguration("obb", 0.01))
        obstacle_strategy.set_default_configuration(Cylinder, ObstacleConfiguration("obb", 0.01))

        self._world_binding = WorldBinding(
            world_interface=CumotionWorldInterface(visualize_debug_prims=True),
            obstacle_strategy=obstacle_strategy,
            tracked_prims=objects,
            tracked_collision_api=TrackableApi.PHYSICS_COLLISION,
        )
        self._world_binding.initialize()

    def reset(self) -> None:
        """Rebuild the world binding."""
        self.setup()

    # --------------------------------------------------------------- per-tick

    def step(self, dt: float) -> None:
        """Per-physics-step update.  No-op when no world binding has been built yet."""
        if self._world_binding is None:
            return
        self.update(dt)

    def update(self, dt: float) -> None:
        """Run one synchronization pass using the currently selected update style.

        Args:
            dt: Physics time step in seconds.
        """
        if self._world_binding is None:
            return

        if self._update_style == "synchronize_transforms":
            self._world_binding.synchronize_transforms()
        elif self._update_style == "synchronize_properties":
            self._world_binding.synchronize_properties()
        elif self._update_style == "synchronize":
            self._world_binding.synchronize()
        else:
            raise ValueError(f"Invalid update style: {self._update_style}")

    # --------------------------------------------------------------- teardown

    def cleanup(self) -> None:
        """Drop the world binding so the closing UsdStage can be fully released."""
        if self._world_binding is not None:
            self._cleanup_debug_prims()
            self._world_binding = None

    def _cleanup_debug_prims(self) -> None:
        """Delete all prims under 'CumotionDebug' to clean up old debug visualization."""
        try:
            stage_utils.get_current_stage()
        except ValueError:
            # There is no stage.
            return
        debug_prim_paths = prim_utils.find_matching_prim_paths(".*CumotionDebug.*", traverse=True)
        if not debug_prim_paths:
            return
        # Filter to root-level prims (deleting a parent removes its children).
        debug_prim_paths_set = set(debug_prim_paths)
        root_prim_paths = [p for p in debug_prim_paths if p.rsplit("/", 1)[0] not in debug_prim_paths_set]
        for prim_path in root_prim_paths:
            try:
                stage_utils.delete_prim(prim_path)
            except ValueError:
                pass
