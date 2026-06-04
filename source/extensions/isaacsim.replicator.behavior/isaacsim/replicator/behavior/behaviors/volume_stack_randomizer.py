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

"""Behavior script that randomly drops and stacks assets on top of prim surfaces."""

from __future__ import annotations

import asyncio
from enum import Enum
from typing import Any

import carb
import carb.events
import numpy as np
import omni.kit.app
import omni.usd
from isaacsim.replicator.behavior.global_variables import EXPOSED_ATTR_NS, EXTENSION_NAME, SCOPE_NAME
from isaacsim.replicator.behavior.utils.behavior_utils import (
    check_if_exposed_variables_should_be_removed,
    create_exposed_variables,
    get_exposed_variable,
    remove_empty_scopes,
    remove_exposed_variables,
)
from isaacsim.replicator.behavior.utils.scene_utils import (
    add_colliders,
    add_rigid_body_dynamics,
    apply_forces_and_simulate_async,
    create_asset,
    create_collision_walls,
    create_physics_material,
    disable_colliders,
    disable_rigid_body_dynamics,
    disable_simulation_reset_on_stop,
    get_world_location,
    get_world_rotation,
    reset_simulation_and_enable_reset_on_stop,
    run_simulation_async,
    set_transform_attributes,
)
from isaacsim.storage.native import get_assets_root_path_async
from omni.behavior.scripting.core import BehaviorScript
from pxr import Gf, PhysicsSchemaTools, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils


class BehaviorState(Enum):
    """Enumeration of volume stack randomizer behavior states."""

    INIT = 0
    SETUP = 1
    RUNNING = 2
    FINISHED = 3
    RESET = 4


class VolumeStackRandomizer(BehaviorScript):
    """Behavior script that randomly drops and stacks assets on top of the prim(s) area."""

    BEHAVIOR_NS = "volumeStackRandomizer"
    EVENT_NAME_IN = f"{EXTENSION_NAME}.{BEHAVIOR_NS}.in"
    EVENT_NAME_OUT = f"{EXTENSION_NAME}.{BEHAVIOR_NS}.out"
    ACTION_FUNCTION_MAP = {
        "setup": "_setup_async",
        "run": "_run_behavior_async",
        "reset": "_reset_async",
    }

    VARIABLES_TO_EXPOSE = [
        {
            "attr_name": "includeChildren",
            "attr_type": Sdf.ValueTypeNames.Bool,
            "default_value": True,
            "doc": "Include valid prim children to the behavior.",
        },
        {
            "attr_name": "event:input",
            "attr_type": Sdf.ValueTypeNames.String,
            "default_value": f"{EVENT_NAME_IN}",
            "doc": (
                "Event to subscribe to for controlling the behavior.\n"
                "NOTE: Changing this value will not have any effect since the event subscription is done on init."
            ),
            "lock": True,
        },
        {
            "attr_name": "event:output",
            "attr_type": Sdf.ValueTypeNames.String,
            "default_value": f"{EVENT_NAME_OUT}",
            "doc": "Event name to publish to on behavior update.",
        },
        {
            "attr_name": "assets:assets",
            "attr_type": Sdf.ValueTypeNames.AssetArray,
            "default_value": [],
            "doc": "Assets to spawn.",
        },
        {
            "attr_name": "assets:csv",
            "attr_type": Sdf.ValueTypeNames.String,
            "default_value": (
                "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxC_01.usd,"
                "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxD_01.usd"
            ),
            "doc": "Assets to spawn as CSV paths.",
        },
        {
            "attr_name": "assets:numRange",
            "attr_type": Sdf.ValueTypeNames.Int2,
            "default_value": Gf.Vec2i(4, 8),
            "doc": "Range of number of assets to spawn.",
        },
        {
            "attr_name": "dropHeight",
            "attr_type": Sdf.ValueTypeNames.Float,
            "default_value": 2.0,
            "doc": "Height from which to drop the assets.",
        },
        {
            "attr_name": "renderSimulation",
            "attr_type": Sdf.ValueTypeNames.Bool,
            "default_value": True,
            "doc": "Render the simulations steps when stacking the assets.",
        },
        {
            "attr_name": "removeRigidBodyDynamics",
            "attr_type": Sdf.ValueTypeNames.Bool,
            "default_value": True,
            "doc": "Remove the rigid body dynamics after the simulation.",
        },
        {
            "attr_name": "preserveSimulationState",
            "attr_type": Sdf.ValueTypeNames.Bool,
            "default_value": True,
            "doc": (
                "Keep the final simulation state as the initial state after a play+stop.\n"
                "NOTE: If multiple simulation behaviors are running concurently, this should be managed externally."
            ),
        },
        {
            "attr_name": "seed",
            "attr_type": Sdf.ValueTypeNames.Int,
            "default_value": -1,
            "doc": "Random seed for reproducible randomization. Use -1 for non-deterministic behavior.",
        },
    ]

    def on_init(self) -> None:
        """Called when the script is assigned to a prim."""
        self._rng = None
        self._state = BehaviorState.INIT
        self._event_name_out = self.EVENT_NAME_OUT
        self._drop_height = 2.0
        self._physics_material = None
        self._render_simulation = True
        self._remove_rigid_body_dynamics = True
        self._preserve_simulation_state = False  # keep False at init to avoid physx reset without performed simulation
        self._valid_prims = []
        self._prim_collision_walls = {}
        self._prim_assets = {}
        self._reset_requested = False
        self._physx_dt = 1 / self.stage.GetTimeCodesPerSecond()

        # App event stream, used to listen to incoming control events, and to publish the state of the behavior script
        self._event_stream = carb.eventdispatcher.get_eventdispatcher()

        # Subscribe to the event stream to listen for incoming control events
        self._event_sub = self._event_stream.observe_event(
            event_name=self.EVENT_NAME_IN, on_event=self._on_event, observer_name="VolumeStackRandomizer._event_sub"
        )

        # Expose the variables as USD attributes
        create_exposed_variables(self.prim, EXPOSED_ATTR_NS, self.BEHAVIOR_NS, self.VARIABLES_TO_EXPOSE)

        # Update the current behavior state and publish the new value
        self._set_state_and_publish(BehaviorState.INIT)

    def on_destroy(self) -> None:
        """Called when the script is unassigned from a prim."""
        # Unsubscribe from the event stream
        self._event_sub = None

        asyncio.ensure_future(self._reset_async())

        # Exposed variables should be removed if the script is no longer assigned to the prim
        if check_if_exposed_variables_should_be_removed(self.prim, __file__):
            remove_exposed_variables(self.prim, EXPOSED_ATTR_NS, self.BEHAVIOR_NS, self.VARIABLES_TO_EXPOSE)

    def _on_event(self, event: carb.events.IEvent) -> None:
        # If the specific prim_path is provided, but does not match the prim_path of this script, return
        if (prim_path := event.payload.get("prim_path")) and prim_path != self.prim_path:
            return

        # Get the action from the payload and call the corresponding function from the mapping
        if (action := event.payload.get("action", None)) and action in self.ACTION_FUNCTION_MAP:
            if function_name := self.ACTION_FUNCTION_MAP.get(action, None):
                try:
                    if action == "reset" and self._state == BehaviorState.RUNNING:
                        self._reset_requested = True
                    else:
                        asyncio.ensure_future(getattr(self, function_name)())
                except AttributeError as e:
                    carb.log_error(f"[{self.prim_path}] {function_name} is not a valid function. {e}.")
            else:
                carb.log_warn(
                    f"[{self.prim_path}] Invalid action '{action}', valid actions are: {self.ACTION_FUNCTION_MAP.keys()}"
                )

    def _set_state_and_publish(self, new_state: BehaviorState) -> None:
        # Update the state and publish it to the event stream
        self._state = new_state
        payload_out = {
            "prim_path": str(self.prim_path),
            "state": self._state.value,
            "state_name": self._state.name,
        }
        if self._event_stream:
            self._event_stream.dispatch_event(event_name=self._event_name_out, payload=payload_out)

    async def _setup_async(self) -> None:
        # Fetch the exposed attributes
        include_children = self._get_exposed_variable("includeChildren")
        self._event_name_out = self._get_exposed_variable("event:output")
        asset_list = self._get_exposed_variable("assets:assets")
        assets_csv = self._get_exposed_variable("assets:csv")
        num_assets_range = self._get_exposed_variable("assets:numRange")
        self._drop_height = self._get_exposed_variable("dropHeight")
        self._render_simulation = self._get_exposed_variable("renderSimulation")
        self._remove_rigid_body_dynamics = self._get_exposed_variable("removeRigidBodyDynamics")
        self._preserve_simulation_state = self._get_exposed_variable("preserveSimulationState")
        seed = self._get_exposed_variable("seed")

        # Initialize the random number generator (use seed if valid, otherwise non-deterministic)
        if self._rng is None:
            self._rng = np.random.default_rng(seed if seed >= 0 else None)

        # Set the simulation delta time
        self._physx_dt = 1 / self.stage.GetTimeCodesPerSecond()

        # Get the prims to apply the behavior to
        if include_children:
            self._valid_prims = [prim for prim in Usd.PrimRange(self.prim) if prim.IsA(UsdGeom.Gprim)]
        elif self.prim.IsA(UsdGeom.Gprim):
            self._valid_prims = [self.prim]
        else:
            self._valid_prims = []
            carb.log_warn(f"[{self.prim_path}] No valid prims found.")

        # Store the assets urls
        assets_urls = []

        # Add the assets from the asset list
        for asset in asset_list:
            assets_urls.append(asset.path)

        # Use the assets root path if the asset URLs are relative
        assets_root_path = await get_assets_root_path_async()

        # Add the assets from the CSV
        for asset_url in assets_csv.split(","):
            # Skip empty strings
            if not asset_url:
                continue
            # Check if absolute or relative path
            if asset_url.startswith(("omniverse://", "http://", "https://", "file://")):
                assets_urls.append(asset_url)
            else:
                if not asset_url.startswith("/"):
                    asset_url = "/" + asset_url
                assets_urls.append(assets_root_path + asset_url)

        # Create the simulation environment
        self._create_sim_environment(
            assets_urls=assets_urls, height=self._drop_height, num_assets_range=num_assets_range
        )

        # Update the current behavior state and publish the new value
        self._set_state_and_publish(BehaviorState.SETUP)

    async def _reset_async(self) -> None:
        if self._preserve_simulation_state:
            reset_simulation_and_enable_reset_on_stop()
        self._remove_collision_walls()
        self._remove_assets()
        self._remove_physics_material()

        if self.stage:
            scope_root_prim = self.stage.GetPrimAtPath(f"{SCOPE_NAME}")
            if scope_root_prim:
                remove_empty_scopes(scope_root_prim, self.stage)
        self._valid_prims.clear()
        self._reset_requested = False
        self._rng = None

        # Update the current behavior state and publish the new value
        self._set_state_and_publish(BehaviorState.RESET)

    async def _run_behavior_async(self) -> None:
        # Update the current behavior state and publish the new value
        self._set_state_and_publish(BehaviorState.RUNNING)

        # Disable the simulation reset on stop setting to preserve the simulation state after play+stop
        if self._preserve_simulation_state:
            disable_simulation_reset_on_stop()

        # Drop the assets from random locations at the given height, set the drop interval and settling simulation steps
        await self._drop_assets_async(drop_interval_steps=10, settling_sim_steps=100)
        if self._reset_requested:
            await self._reset_async()
            return

        # Apply directional forces to the assets to try to fill any gaps between the assets
        force_directions = [(0, 1, 0), (1, 0, 0), (0, -1, 0), (-1, 0, 0)]
        for _ in range(2):
            await self._apply_directional_forces_async(
                force_directions, force_intensity=500, sim_steps=25, include_center_force=True
            )
            if self._reset_requested:
                await self._reset_async()
                return

        # Finally, only apply the force towards the center to pull the assets together
        for _ in range(2):
            await self._apply_directional_forces_async(
                force_directions=[(0, 0, 0)], force_intensity=250, sim_steps=20, include_center_force=True
            )
            if self._reset_requested:
                await self._reset_async()
                return

        # Settle the simulation for a few more steps and cleanup the simulation (e.g. remove collision walls)
        await self._finalize_simulation_async()
        if self._reset_requested:
            await self._reset_async()
            return

        # Reset the simulation and change the physics preferences back to the default value
        # this will make sure these prim states will be used after a timline play+stop
        if self._preserve_simulation_state:
            reset_simulation_and_enable_reset_on_stop()

        # Update the current behavior state and publish the new value
        self._set_state_and_publish(BehaviorState.FINISHED)

    def _create_sim_environment(self, assets_urls: list[str], height: float, num_assets_range: Gf.Vec2i) -> None:
        # Early return if no spawn assets urls were provided
        if not assets_urls:
            carb.log_warn(f"[{self.prim_path}] No assets provided to spawn.")
            return

        # Create the physics material to make allow objects to slide on surfaces and not bounce during the simulation
        physics_material_path = omni.usd.get_stage_next_free_path(
            self.stage, f"{SCOPE_NAME}/{self.BEHAVIOR_NS}/PhysicsMaterial", False
        )
        self._physics_material = create_physics_material(
            self.stage, prim_path=physics_material_path, restitution=0, static_friction=0.001, dynamic_friction=0.001
        )

        # Spawn the assets
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])
        for prim in self._valid_prims:
            # Get the random list of assets to spawn
            assets = []
            rand_num = self._rng.integers(num_assets_range[0], num_assets_range[1] + 1)
            rand_assets_urls = self._rng.choice(assets_urls, size=rand_num).tolist()

            # Add the assets to a common root outside of the prim hierarchy to avoid inheriting any parent scaling
            assets_root_path = omni.usd.get_stage_next_free_path(
                self.stage, f"{SCOPE_NAME}/{self.BEHAVIOR_NS}/Assets/{prim.GetName()}", False
            )

            # Use prim location and orientation as default spawn pose
            xform_cache = UsdGeom.XformCache()
            spawn_location = get_world_location(prim, xform_cache)
            spawn_rotation = get_world_rotation(prim, xform_cache)

            # Spawn the assets and bind the physics material
            for i, asset_url in enumerate(rand_assets_urls):
                # Create the asset (Xform with Reference) and bind the physics material
                asset_prim = create_asset(self.stage, asset_url, f"{assets_root_path}/Asset_{i}")

                # Bind the physics material to the asset
                mat_binding_api = UsdShade.MaterialBindingAPI.Apply(asset_prim)
                mat_binding_api.Bind(self._physics_material, UsdShade.Tokens.weakerThanDescendants, "physics")

                # Disable any previously set rigid body dynamics and collisions until simulation starts
                disable_colliders(asset_prim)
                disable_rigid_body_dynamics(asset_prim)

                # Set the spawn location and orientation to match the prim's world transform
                set_transform_attributes(asset_prim, location=spawn_location, orientation=spawn_rotation.GetQuat())

                # Cache the spawned assets for later use
                assets.append(asset_prim)

            # Clear the cache to account for newly added prims and sort the assets by volume to drop large assets first
            bbox_cache.Clear()
            assets.sort(key=lambda asset: bbox_cache.ComputeWorldBound(asset).GetVolume(), reverse=True)

            # Store the assets in the dictionary
            self._prim_assets[prim] = assets

        # Create the collision walls around the top surface of the prims
        for prim in self._valid_prims:
            collision_wall_prims = create_collision_walls(
                self.stage,
                prim,
                prim_path=f"{SCOPE_NAME}/{self.BEHAVIOR_NS}/CollisionWalls/{prim.GetName()}",
                height=height,
                physics_material=self._physics_material,
                bbox_cache=bbox_cache,
                visible=False,
            )
            # Cache the collision wall prims to remove them after the simulation
            self._prim_collision_walls[prim] = collision_wall_prims

    async def _drop_assets_async(self, drop_interval_steps: int, settling_sim_steps: int) -> None:
        # Group the prims and their associated assets into batches to allow parallel simulation between the prims
        prim_asset_batches = self._group_prims_and_assets_into_batches()

        # Spawn the assets at random poses and simulate the drop start for a few frames for each batch of prim-asset pairs
        for prim_asset_batch in prim_asset_batches:
            await self._start_batched_asset_drop_async(
                prim_asset_batch, self._drop_height, sim_steps=drop_interval_steps
            )
            if self._reset_requested:
                return

        # Let the simulation run for additional steps to allow all assets to finish dropping
        await run_simulation_async(
            sim_steps=settling_sim_steps, physx_dt=self._physx_dt, render=self._render_simulation
        )

    async def _start_batched_asset_drop_async(self, prim_asset_batch: list, drop_height: float, sim_steps: int) -> None:
        # For each prim-assset pair calculate the drop area and prepare to drop the asset from a random location
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])
        xform_cache = UsdGeom.XformCache()
        for prim, asset in prim_asset_batch:
            # Compute prim's size, midpoint, and calculate the drop area center at the specified height above the surface
            prim_bound = bbox_cache.ComputeWorldBound(prim)
            prim_scale = Gf.Transform(prim_bound.GetMatrix()).GetScale()

            # NOTE: GetRange() returns the untransformed size of the bounding box
            prim_range_untransformed = bbox_cache.ComputeWorldBound(prim).GetRange()

            # Apply the prim scale to the range to get the transformed size
            prim_width, prim_depth, prim_height = prim_range_untransformed.GetSize()
            prim_width *= prim_scale[0]
            prim_depth *= prim_scale[1]
            prim_height *= prim_scale[2]

            # Calculate the drop area center at the specified height above the surface
            mid_point = prim_range_untransformed.GetMidpoint()
            mid_point[0] *= prim_scale[0]
            mid_point[1] *= prim_scale[1]
            mid_point[2] *= prim_scale[2]
            drop_area_center = mid_point + Gf.Vec3d(0, 0, prim_height / 2 + drop_height)

            # Compute the asset's size and drop margin to avoid overlapping with the prim
            asset_width, asset_depth, asset_height = bbox_cache.ComputeWorldBound(asset).GetRange().GetSize()

            # Use the largest dimension of the asset to calculate a margin for avoiding overlap in any direction
            drop_margin = max(asset_width, asset_depth, asset_height) / 2

            # Adjust the drop area width and depth by subtracting the margin
            drop_area_size = min(prim_width, prim_depth) / 2 - drop_margin

            # Use the asset's largest dimension to calculate the margin needed to avoid overlap with the collision walls
            asset_width, asset_depth, asset_height = bbox_cache.ComputeWorldBound(asset).GetRange().GetSize()
            drop_margin = max(asset_width, asset_depth, asset_height) / 2
            drop_area_size = min(prim_width, prim_depth) / 2 - drop_margin

            # Generate a random location for the asset within the adjusted drop area, ensuring no overlap with the walls
            random_location = drop_area_center + Gf.Vec3d(
                self._rng.uniform(-drop_area_size, drop_area_size),
                self._rng.uniform(-drop_area_size, drop_area_size),
                0,  # No vertical offset in randomization
            )

            # Generate a random orientation with 90-degree steps around the x, y, and z axes
            rotation_choices = [180, 90, 0, -90, -180]
            random_rotation = (
                Gf.Rotation(Gf.Vec3d.XAxis(), float(self._rng.choice(rotation_choices)))
                * Gf.Rotation(Gf.Vec3d.YAxis(), float(self._rng.choice(rotation_choices)))
                * Gf.Rotation(Gf.Vec3d.ZAxis(), float(self._rng.choice(rotation_choices)))
            )

            # Calculate the spawn location and rotation relative to the prim's world transform
            world_location = get_world_location(prim, xform_cache)
            spawn_location = random_location + world_location
            world_rotation = get_world_rotation(prim, xform_cache)
            spawn_rotation = random_rotation * world_rotation

            # Set the drop pose and enable collisions and rigid body dynamics with dampened angular movements
            set_transform_attributes(asset, location=spawn_location, orientation=spawn_rotation.GetQuat())
            add_colliders(asset)
            add_rigid_body_dynamics(asset, angular_damping=10.0, linear_damping=0.01)

            # Early return if a reset was requested during the run
            if self._reset_requested:
                return

        # Start simulating the drop for the given batch of prim-asset pairs
        await run_simulation_async(sim_steps=sim_steps, physx_dt=self._physx_dt, render=self._render_simulation)

    async def _apply_directional_forces_async(
        self, force_directions: list, force_intensity: float, sim_steps: int, include_center_force: bool = False
    ) -> None:
        # Iterate over the force directions and apply the forces to the assets
        stage_id = UsdUtils.StageCache.Get().GetId(self.stage).ToLongInt()
        xform_cache = UsdGeom.XformCache()
        for force_direction in force_directions:
            body_ids = []
            forces = []
            positions = []
            xform_cache.Clear()

            # Apply the directional forces (north, east, south, west) in local frame of each prim to every asset
            for prim, asset_list in self._prim_assets.items():
                # Compute the directional forces relative to the prim's orientation
                prim_rot = get_world_rotation(prim, xform_cache)
                directional_force = Gf.Vec3d(prim_rot.TransformDir(force_direction)) * force_intensity

                # Apply the forces to all assets of the prim
                for asset in asset_list:
                    body_id = PhysicsSchemaTools.sdfPathToInt(asset.GetPath())
                    asset_position = get_world_location(asset, xform_cache)
                    body_ids.append(body_id)

                    total_force = directional_force
                    # Apply an additional force towards the center of the prim to pull the assets together
                    if include_center_force:
                        center_force = (get_world_location(prim, xform_cache) - asset_position) * force_intensity * 0.2
                        total_force += center_force
                    forces.append(total_force)
                    positions.append(asset_position)

            # Early return if a reset was requested during the run
            if self._reset_requested:
                return

            # Apply the calculated forces and simulate the movement for the specified number of steps
            await apply_forces_and_simulate_async(
                stage_id, body_ids, forces, positions, sim_steps, self._physx_dt, self._render_simulation
            )

    async def _finalize_simulation_async(self) -> None:
        # Let the simulation run for a few more steps to allow the assets to settle
        await run_simulation_async(sim_steps=20, physx_dt=self._physx_dt, render=self._render_simulation)

        # If no app updates happened during the simulation, wait an update to ensure the simulation is solved
        if not self._render_simulation:
            await omni.kit.app.get_app().next_update_async()

        # Increase the friction to prevent sliding of the assets on the surface
        if self._physics_material and self._physics_material.GetPrim().IsValid():
            physics_material_api = UsdPhysics.MaterialAPI(self._physics_material.GetPrim())
            physics_material_api.GetStaticFrictionAttr().Set(0.95)
            physics_material_api.GetDynamicFrictionAttr().Set(0.95)

        # Remove the rigid body dynamics properties
        if self._remove_rigid_body_dynamics:
            for assets in self._prim_assets.values():
                for asset in assets:
                    UsdPhysics.RigidBodyAPI(asset).GetRigidBodyEnabledAttr().Set(False)

        # Remove simulation environment setup (collision walls, assets, physics material, physics scenes)
        self._remove_collision_walls()

        # Remove any remaining empty scopes from the behavior's scope
        if self.stage:
            scope_root_prim = self.stage.GetPrimAtPath(f"{SCOPE_NAME}/{self.BEHAVIOR_NS}")
            if scope_root_prim:
                remove_empty_scopes(scope_root_prim, self.stage)

    def _group_prims_and_assets_into_batches(self) -> list:
        # Early return if no valid prims or assets were found
        if not self._prim_assets or not self._valid_prims:
            print(f"[{self.prim_path}] No valid prims or assets found.")
            return []

        # Group prims and their associated assets into batches, where each batch contains one asset for each prim.
        # Useful for parallel simulation of multiple prims with their corresponding assets.
        # from: {'prim1': ['asset1_0', 'asset1_1'], 'prim2': ['asset2_0']}
        # to:   [[(prim1, asset1_0), (prim2, asset2_0)], [(prim1, asset1_1)]]
        prim_asset_batches = []

        # Retrieve all prims and their corresponding asset lists from the dictionary
        prim_list = list(self._prim_assets.keys())
        asset_lists = [self._prim_assets[prim] for prim in prim_list]

        # Find the maximum number of assets assigned to any prim to determine the batching range
        max_asset_count = max(len(asset_list) for asset_list in asset_lists)

        # Create batches of prim-asset pairs, grouping them by asset position (index in the list of assets)
        for asset_index in range(max_asset_count):
            current_batch = []

            # For each prim, pair it with its corresponding asset at the current index (if available)
            for prim, asset_list in zip(prim_list, asset_lists):
                if asset_index < len(asset_list):  # Only add the pair if the prim has an asset at this index
                    current_batch.append((prim, asset_list[asset_index]))

            # Add the batch to the list if it contains at least one valid prim-asset pair
            if current_batch:
                prim_asset_batches.append(current_batch)

        return prim_asset_batches

    def _remove_collision_walls(self) -> None:
        if not self.stage:
            carb.log_warn(f"[{self.prim_path}] Stage is not valid to remove collision walls.")
            return

        # Remove the collision walls through the common xform root parent
        for collision_walls in self._prim_collision_walls.values():
            for wall in collision_walls:
                if wall.IsValid():
                    parent = wall.GetParent()
                    if parent.IsValid():
                        self.stage.RemovePrim(parent.GetPath())
                        break
        self._prim_collision_walls.clear()

    def _remove_assets(self) -> None:
        if not self.stage:
            carb.log_warn(f"[{self.prim_path}] Stage is not valid to remove assets.")
            return

        if self._prim_assets:
            for assets in self._prim_assets.values():
                for asset in assets:
                    if asset.IsValid():
                        self.stage.RemovePrim(asset.GetPath())

        self._prim_assets.clear()

    def _remove_physics_material(self) -> None:
        if not self.stage:
            carb.log_warn(f"[{self.prim_path}] Stage is not valid to remove physics material.")
            return

        if self._physics_material:
            self.stage.RemovePrim(self._physics_material.GetPath())
            self._physics_material = None

    def _get_exposed_variable(self, attr_name: str) -> Any:
        full_attr_name = f"{EXPOSED_ATTR_NS}:{self.BEHAVIOR_NS}:{attr_name}"
        return get_exposed_variable(self.prim, full_attr_name)

    def set_rng(self, rng: np.random.Generator | None = None) -> None:
        """Set the random number generator, overriding the USD seed attribute.

        Args:
            rng: Numpy random generator. If None, creates a new default generator.
        """
        self._rng = rng if rng is not None else np.random.default_rng()
