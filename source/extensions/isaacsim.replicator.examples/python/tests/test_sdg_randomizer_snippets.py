# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import tempfile

import omni.kit
import omni.usd
from isaacsim.test.utils.file_validation import validate_folder_contents


class TestSDGRandomizerSnippets(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def test_randomize_sequential_sphere_scan(self):
        import asyncio
        import itertools
        import os

        import numpy as np
        import omni.replicator.core as rep
        import omni.usd
        from isaacsim.storage.native import get_assets_root_path_async
        from pxr import Gf, Usd, UsdGeom, UsdLux

        sphere_scan_dir = tempfile.mkdtemp(prefix="test_rand_sphere_scan_")
        print(f"Output directory: {sphere_scan_dir}")

        # Fibonacci sphere algorithm: https://arxiv.org/pdf/0912.4540
        def next_point_on_sphere(idx, num_points, radius=1, origin=(0, 0, 0)):
            offset = 2.0 / num_points
            inc = np.pi * (3.0 - np.sqrt(5.0))
            z = ((idx * offset) - 1) + (offset / 2)
            phi = ((idx + 1) % num_points) * inc
            r = np.sqrt(1 - pow(z, 2))
            y = np.cos(phi) * r
            x = np.sin(phi) * r
            return [(x * radius) + origin[0], (y * radius) + origin[1], (z * radius) + origin[2]]

        async def run_randomizations_async(
            num_frames, forklift_path, pallet_path, bin_path, dome_textures, write_data, delay=None
        ):
            assets_root_path = await get_assets_root_path_async()

            await omni.usd.get_context().new_stage_async()
            stage = omni.usd.get_context().get_stage()

            dome_light = UsdLux.DomeLight.Define(stage, "/World/Lights/DomeLight")
            dome_light.GetIntensityAttr().Set(1000)

            forklift_prim = stage.DefinePrim("/World/Forklift", "Xform")
            forklift_prim.GetReferences().AddReference(assets_root_path + forklift_path)
            if not forklift_prim.GetAttribute("xformOp:translate"):
                UsdGeom.Xformable(forklift_prim).AddTranslateOp()
            forklift_prim.GetAttribute("xformOp:translate").Set((-4.5, -4.5, 0))

            pallet_prim = stage.DefinePrim("/World/Pallet", "Xform")
            pallet_prim.GetReferences().AddReference(assets_root_path + pallet_path)
            if not pallet_prim.GetAttribute("xformOp:translate"):
                UsdGeom.Xformable(pallet_prim).AddTranslateOp()
            if not pallet_prim.GetAttribute("xformOp:rotateXYZ"):
                UsdGeom.Xformable(pallet_prim).AddRotateXYZOp()

            bin_prim = stage.DefinePrim("/World/Bin", "Xform")
            bin_prim.GetReferences().AddReference(assets_root_path + bin_path)
            if not bin_prim.GetAttribute("xformOp:translate"):
                UsdGeom.Xformable(bin_prim).AddTranslateOp()
            if not bin_prim.GetAttribute("xformOp:rotateXYZ"):
                UsdGeom.Xformable(bin_prim).AddRotateXYZOp()

            view_cam = stage.DefinePrim("/World/Camera", "Camera")
            if not view_cam.GetAttribute("xformOp:translate"):
                UsdGeom.Xformable(view_cam).AddTranslateOp()
            if not view_cam.GetAttribute("xformOp:orient"):
                UsdGeom.Xformable(view_cam).AddOrientOp()

            dome_textures_full = [assets_root_path + tex for tex in dome_textures]
            textures_cycle = itertools.cycle(dome_textures_full)

            if write_data:
                print(f"Writing data to {sphere_scan_dir}..")
                backend = rep.backends.get("DiskBackend")
                backend.initialize(output_dir=sphere_scan_dir)
                writer = rep.WriterRegistry.get("BasicWriter")
                writer.initialize(backend=backend, rgb=True)
                persp_cam = rep.functional.create.camera(position=(5, 5, 5), look_at=(0, 0, 0), name="PerspCamera")
                rp_persp = rep.create.render_product(persp_cam, (512, 512), name="PerspView")
                rp_view = rep.create.render_product(view_cam, (512, 512), name="SphereView")
                writer.attach([rp_view, rp_persp])

            bb_cache = UsdGeom.BBoxCache(time=Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])
            pallet_size = bb_cache.ComputeWorldBound(pallet_prim).GetRange().GetSize()
            pallet_length = pallet_size.GetLength()
            bin_size = bb_cache.ComputeWorldBound(bin_prim).GetRange().GetSize()

            for i in range(num_frames):
                # Set next background texture every nth frame and run an app update
                if i % 5 == 0:
                    dome_light.GetTextureFileAttr().Set(next(textures_cycle))
                    await omni.kit.app.get_app().next_update_async()

                # Randomize pallet pose
                pallet_prim.GetAttribute("xformOp:translate").Set(
                    Gf.Vec3d(np.random.uniform(-1.5, 1.5), np.random.uniform(-1.5, 1.5), 0)
                )
                rand_z_rot = np.random.uniform(-90, 90)
                pallet_prim.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3d(0, 0, rand_z_rot))
                pallet_tf_mat = omni.usd.get_world_transform_matrix(pallet_prim)
                pallet_rot = pallet_tf_mat.ExtractRotation()
                pallet_pos = pallet_tf_mat.ExtractTranslation()

                # Randomize bin position on top of the rotated pallet area making sure the bin is fully on the pallet
                rand_transl_x = np.random.uniform(
                    -pallet_size[0] / 2 + bin_size[0] / 2, pallet_size[0] / 2 - bin_size[0] / 2
                )
                rand_transl_y = np.random.uniform(
                    -pallet_size[1] / 2 + bin_size[1] / 2, pallet_size[1] / 2 - bin_size[1] / 2
                )

                # Adjust bin position to account for the random rotation of the pallet
                rand_z_rot_rad = np.deg2rad(rand_z_rot)
                rot_adjusted_transl_x = rand_transl_x * np.cos(rand_z_rot_rad) - rand_transl_y * np.sin(rand_z_rot_rad)
                rot_adjusted_transl_y = rand_transl_x * np.sin(rand_z_rot_rad) + rand_transl_y * np.cos(rand_z_rot_rad)
                bin_prim.GetAttribute("xformOp:translate").Set(
                    Gf.Vec3d(
                        pallet_pos[0] + rot_adjusted_transl_x,
                        pallet_pos[1] + rot_adjusted_transl_y,
                        pallet_pos[2] + pallet_size[2] + bin_size[2] / 2,
                    )
                )
                # Keep bin rotation aligned with pallet
                bin_prim.GetAttribute("xformOp:rotateXYZ").Set(pallet_rot.GetAxis() * pallet_rot.GetAngle())

                # Get next camera position on a sphere looking at the bin with a randomized distance
                rand_radius = np.random.normal(3, 0.5) * pallet_length
                bin_pos = omni.usd.get_world_transform_matrix(bin_prim).ExtractTranslation()
                cam_pos = next_point_on_sphere(i, num_points=num_frames, radius=rand_radius, origin=bin_pos)
                view_cam.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*cam_pos))

                eye = Gf.Vec3d(*cam_pos)
                target = Gf.Vec3d(*bin_pos)
                up_axis = Gf.Vec3d(0, 0, 1)
                look_at_quatd = Gf.Matrix4d().SetLookAt(eye, target, up_axis).GetInverse().ExtractRotation().GetQuat()
                view_cam.GetAttribute("xformOp:orient").Set(Gf.Quatf(look_at_quatd))

                if write_data:
                    await rep.orchestrator.step_async(rt_subframes=4, delta_time=0.0)
                else:
                    await omni.kit.app.get_app().next_update_async()
                # Optional delay between frames to better visualize the randomization in the viewport
                if delay is not None and delay > 0:
                    await asyncio.sleep(delay)

            # Wait for the data to be written to disk and cleanup writer and render products
            if write_data:
                await rep.orchestrator.wait_until_complete_async()
                writer.detach()
                rp_persp.destroy()
                rp_view.destroy()

        NUM_FRAMES = 90
        FORKLIFT_PATH = "/Isaac/Props/Forklift/forklift.usd"
        PALLET_PATH = "/Isaac/Props/Pallet/pallet.usd"
        BIN_PATH = "/Isaac/Props/KLT_Bin/small_KLT_visual.usd"
        DOME_TEXTURES = [
            "/NVIDIA/Assets/Skies/Cloudy/champagne_castle_1_4k.hdr",
            "/NVIDIA/Assets/Skies/Clear/evening_road_01_4k.hdr",
            "/NVIDIA/Assets/Skies/Clear/mealie_road_4k.hdr",
            "/NVIDIA/Assets/Skies/Clear/qwantani_4k.hdr",
        ]
        # asyncio.ensure_future(
        #     run_randomizations_async(
        #         NUM_FRAMES, FORKLIFT_PATH, PALLET_PATH, BIN_PATH, DOME_TEXTURES, write_data=True, delay=0.2
        #     )
        # )

        # Test
        test_num_frames = 4
        await run_randomizations_async(
            test_num_frames, FORKLIFT_PATH, PALLET_PATH, BIN_PATH, DOME_TEXTURES, write_data=True
        )

        folder_contents_success = validate_folder_contents(
            path=sphere_scan_dir, expected_counts={"png": test_num_frames * 2}, recursive=True
        )
        self.assertTrue(folder_contents_success, f"Output directory contents validation failed for {sphere_scan_dir}")

    async def test_randomize_physics_based_volume_filling(self):
        import os
        import random
        from itertools import chain

        import carb
        import omni.kit.app
        import omni.physx
        import omni.replicator.core as rep
        import omni.usd
        from isaacsim.storage.native import get_assets_root_path_async
        from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils

        box_stacking_dir = tempfile.mkdtemp(prefix="test_box_stacking_")
        print(f"Output directory: {box_stacking_dir}")

        # Add transformation properties to the prim (if not already present)
        def set_transform_attributes(prim, location=None, orientation=None, rotation=None, scale=None):
            if location is not None:
                if not prim.HasAttribute("xformOp:translate"):
                    UsdGeom.Xformable(prim).AddTranslateOp()
                prim.GetAttribute("xformOp:translate").Set(location)
            if orientation is not None:
                if not prim.HasAttribute("xformOp:orient"):
                    UsdGeom.Xformable(prim).AddOrientOp()
                prim.GetAttribute("xformOp:orient").Set(orientation)
            if rotation is not None:
                if not prim.HasAttribute("xformOp:rotateXYZ"):
                    UsdGeom.Xformable(prim).AddRotateXYZOp()
                prim.GetAttribute("xformOp:rotateXYZ").Set(rotation)
            if scale is not None:
                if not prim.HasAttribute("xformOp:scale"):
                    UsdGeom.Xformable(prim).AddScaleOp()
                prim.GetAttribute("xformOp:scale").Set(scale)

        # Enables collisions with the asset (without rigid body dynamics the asset will be static)
        def add_colliders(prim):
            # Iterate descendant prims (including root) and add colliders to mesh or primitive types
            for desc_prim in Usd.PrimRange(prim):
                if desc_prim.IsA(UsdGeom.Mesh) or desc_prim.IsA(UsdGeom.Gprim):
                    # Physics
                    if not desc_prim.HasAPI(UsdPhysics.CollisionAPI):
                        collision_api = UsdPhysics.CollisionAPI.Apply(desc_prim)
                    else:
                        collision_api = UsdPhysics.CollisionAPI(desc_prim)
                    collision_api.CreateCollisionEnabledAttr(True)

                # Add mesh specific collision properties only to mesh types
                if desc_prim.IsA(UsdGeom.Mesh):
                    if not desc_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                        mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(desc_prim)
                    else:
                        mesh_collision_api = UsdPhysics.MeshCollisionAPI(desc_prim)
                    mesh_collision_api.CreateApproximationAttr().Set("convexHull")

        # Enables rigid body dynamics (physics simulation) on the prim (having valid colliders is recommended)
        def add_rigid_body_dynamics(prim, disable_gravity=False, angular_damping=None):
            # Physics
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(prim)
            else:
                rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
            rigid_body_api.CreateRigidBodyEnabledAttr(True)
            # PhysX
            if not prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
                physx_rigid_body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            else:
                physx_rigid_body_api = PhysxSchema.PhysxRigidBodyAPI(prim)
            physx_rigid_body_api.GetDisableGravityAttr().Set(disable_gravity)
            if angular_damping is not None:
                physx_rigid_body_api.CreateAngularDampingAttr().Set(angular_damping)

        # Create a new prim with the provided asset URL and transform properties
        def create_asset(stage, asset_url, path, location=None, rotation=None, orientation=None, scale=None):
            prim_path = omni.usd.get_stage_next_free_path(stage, path, False)
            prim = stage.DefinePrim(prim_path, "Xform")
            prim.GetReferences().AddReference(asset_url)
            set_transform_attributes(prim, location=location, rotation=rotation, orientation=orientation, scale=scale)
            return prim

        # Create a new prim with the provided asset URL and transform properties including colliders
        def create_asset_with_colliders(
            stage, asset_url, path, location=None, rotation=None, orientation=None, scale=None
        ):
            prim = create_asset(stage, asset_url, path, location, rotation, orientation, scale)
            add_colliders(prim)
            return prim

        # Create collision walls around the top surface of the prim with the given height and thickness
        def create_collision_walls(stage, prim, bbox_cache=None, height=2, thickness=0.3, material=None, visible=False):
            # Use the untransformed axis-aligned bounding box to calculate the prim surface size and center
            if bbox_cache is None:
                bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])
            local_range = bbox_cache.ComputeWorldBound(prim).GetRange()
            width, depth, local_height = local_range.GetSize()
            # Raise the midpoint height to the prim's surface
            mid = local_range.GetMidpoint() + Gf.Vec3d(0, 0, local_height / 2)

            # Define the walls (name, location, size) with the specified thickness added externally to the surface and height
            walls = [
                ("floor", (mid[0], mid[1], mid[2] - thickness / 2), (width, depth, thickness)),
                ("ceiling", (mid[0], mid[1], mid[2] + height + thickness / 2), (width, depth, thickness)),
                (
                    "left_wall",
                    (mid[0] - (width + thickness) / 2, mid[1], mid[2] + height / 2),
                    (thickness, depth, height),
                ),
                (
                    "right_wall",
                    (mid[0] + (width + thickness) / 2, mid[1], mid[2] + height / 2),
                    (thickness, depth, height),
                ),
                (
                    "front_wall",
                    (mid[0], mid[1] + (depth + thickness) / 2, mid[2] + height / 2),
                    (width, thickness, height),
                ),
                (
                    "back_wall",
                    (mid[0], mid[1] - (depth + thickness) / 2, mid[2] + height / 2),
                    (width, thickness, height),
                ),
            ]

            # Use the parent prim path to create the walls as children (use local coordinates)
            prim_path = prim.GetPath()
            collision_walls = []
            for name, location, size in walls:
                prim = stage.DefinePrim(f"{prim_path}/{name}", "Cube")
                scale = (size[0] / 2.0, size[1] / 2.0, size[2] / 2.0)
                set_transform_attributes(prim, location=location, scale=scale)
                add_colliders(prim)
                if not visible:
                    UsdGeom.Imageable(prim).MakeInvisible()
                if material is not None:
                    mat_binding_api = UsdShade.MaterialBindingAPI.Apply(prim)
                    mat_binding_api.Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")
                collision_walls.append(prim)
            return collision_walls

        # Slide the assets independently in perpendicular directions and then pull them all together towards the given center
        async def apply_forces_async(stage, boxes, pallet, strength=550, strength_center_multiplier=2):
            timeline = omni.timeline.get_timeline_interface()
            timeline.play()
            # Get the pallet center and forward vector to apply forces in the perpendicular directions and towards the center
            pallet_tf: Gf.Matrix4d = UsdGeom.Xformable(pallet).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            pallet_center = pallet_tf.ExtractTranslation()
            pallet_rot: Gf.Rotation = pallet_tf.ExtractRotation()
            force_forward = Gf.Vec3d(pallet_rot.TransformDir(Gf.Vec3d(1, 0, 0))) * strength
            force_right = Gf.Vec3d(pallet_rot.TransformDir(Gf.Vec3d(0, 1, 0))) * strength

            physx_simulation_interface = omni.physx.get_physx_simulation_interface()
            stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
            for box_prim in boxes:
                body_path = PhysicsSchemaTools.sdfPathToInt(box_prim.GetPath())
                forces = [force_forward, force_right, -force_forward, -force_right]
                for force in chain(forces, forces):
                    box_tf: Gf.Matrix4d = UsdGeom.Xformable(box_prim).ComputeLocalToWorldTransform(
                        Usd.TimeCode.Default()
                    )
                    box_position = carb.Float3(*box_tf.ExtractTranslation())
                    physx_simulation_interface.apply_force_at_pos(
                        stage_id, body_path, carb.Float3(*force), box_position
                    )
                    for _ in range(10):
                        await omni.kit.app.get_app().next_update_async()

            # Pull all boxes at once to the pallet center
            for box_prim in boxes:
                body_path = PhysicsSchemaTools.sdfPathToInt(box_prim.GetPath())
                box_tf: Gf.Matrix4d = UsdGeom.Xformable(box_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                box_location = box_tf.ExtractTranslation()
                force_to_center = (pallet_center - box_location) * strength * strength_center_multiplier
                physx_simulation_interface.apply_force_at_pos(
                    stage_id,
                    body_path,
                    carb.Float3(*force_to_center),
                    carb.Float3(*box_location),
                )
            for _ in range(20):
                await omni.kit.app.get_app().next_update_async()
            timeline.pause()

        # Create a new stage and and run the example scenario
        async def stack_boxes_on_pallet_async(
            pallet_prim, boxes_urls_and_weights, num_boxes, drop_height=1.5, drop_margin=0.2
        ):
            pallet_path = pallet_prim.GetPath()
            print(f"[BoxStacking] Running scenario for pallet {pallet_path} with {num_boxes} boxes..")
            stage = omni.usd.get_context().get_stage()
            bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])

            # Create a custom physics material to allow the boxes to easily slide into stacking positions
            material_path = f"{pallet_path}/Looks/PhysicsMaterial"
            default_material = UsdShade.Material.Define(stage, material_path)
            physics_material = UsdPhysics.MaterialAPI.Apply(default_material.GetPrim())
            physics_material.CreateRestitutionAttr().Set(0.0)  # Inelastic collision (no bouncing)
            physics_material.CreateStaticFrictionAttr().Set(0.01)  # Small friction to allow sliding of stationary boxes
            physics_material.CreateDynamicFrictionAttr().Set(0.01)  # Small friction to allow sliding of moving boxes

            # Apply the physics material to the pallet
            mat_binding_api = UsdShade.MaterialBindingAPI.Apply(pallet_prim)
            mat_binding_api.Bind(default_material, UsdShade.Tokens.weakerThanDescendants, "physics")

            # Create collision walls around the top of the pallet and apply the physics material to them
            collision_walls = create_collision_walls(
                stage, pallet_prim, bbox_cache, height=drop_height + drop_margin, material=default_material
            )

            # Create the random boxes (without physics) with the specified weights and sort them by size (volume)
            box_urls, box_weights = zip(*boxes_urls_and_weights)
            rand_boxes_urls = random.choices(box_urls, weights=box_weights, k=num_boxes)
            boxes = [
                create_asset(stage, box_url, f"{pallet_path}_Boxes/Box_{i}")
                for i, box_url in enumerate(rand_boxes_urls)
            ]
            boxes.sort(key=lambda box: bbox_cache.ComputeLocalBound(box).GetVolume(), reverse=True)

            # Calculate the drop area above the pallet taking into account the pallet surface, drop height and the margin
            # Note: The boxes can be spawned colliding with the surrounding collision walls as they will be pushed inwards
            pallet_range = bbox_cache.ComputeWorldBound(pallet_prim).GetRange()
            pallet_width, pallet_depth, pallet_heigth = pallet_range.GetSize()
            # Move the spawn center at the given height above the pallet surface
            spawn_center = pallet_range.GetMidpoint() + Gf.Vec3d(0, 0, pallet_heigth / 2 + drop_height)
            spawn_width, spawn_depth = pallet_width / 2 - drop_margin, pallet_depth / 2 - drop_margin

            # Use the pallet local-to-world transform to apply the local random offsets relative to the pallet
            pallet_tf: Gf.Matrix4d = UsdGeom.Xformable(pallet_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            pallet_rot: Gf.Rotation = pallet_tf.ExtractRotation()

            # Simulate dropping the boxes from random poses on the pallet
            timeline = omni.timeline.get_timeline_interface()
            for box_prim in boxes:
                # Create a random location and orientation for the box within the drop area in local frame
                local_loc = spawn_center + Gf.Vec3d(
                    random.uniform(-spawn_width, spawn_width), random.uniform(-spawn_depth, spawn_depth), 0
                )
                axes = [Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1)]
                angles = [random.choice([180, 90, 0, -90, -180]) + random.uniform(-3, 3) for _ in axes]
                local_rot = Gf.Rotation()
                for axis, angle in zip(axes, angles):
                    local_rot *= Gf.Rotation(axis, angle)

                # Transform the local pose to the pallet's world coordinate system
                world_loc = pallet_tf.Transform(local_loc)
                world_quat = Gf.Quatf((pallet_rot * local_rot).GetQuat())

                # Set the spawn pose and enable collisions and rigid body dynamics with dampened angular movements
                set_transform_attributes(box_prim, location=world_loc, orientation=world_quat)
                add_colliders(box_prim)
                add_rigid_body_dynamics(box_prim, angular_damping=0.9)

                # Bind the physics material to the box (allow frictionless sliding)
                mat_binding_api = UsdShade.MaterialBindingAPI.Apply(box_prim)
                mat_binding_api.Bind(default_material, UsdShade.Tokens.weakerThanDescendants, "physics")
                # Wait for an app update to load the new attributes
                await omni.kit.app.get_app().next_update_async()

                # Play simulation for a few frames for each box
                timeline.play()
                for _ in range(20):
                    await omni.kit.app.get_app().next_update_async()
                timeline.pause()

            # Iteratively apply forces to the boxes to move them around then pull them all together towards the pallet center
            await apply_forces_async(stage, boxes, pallet_prim)

            # Remove rigid body dynamics of the boxes until all other scenarios are completed
            for box in boxes:
                UsdPhysics.RigidBodyAPI(box).GetRigidBodyEnabledAttr().Set(False)

            # Increase the friction to prevent sliding of the boxes on the pallet before removing the collision walls
            physics_material.CreateStaticFrictionAttr().Set(0.9)
            physics_material.CreateDynamicFrictionAttr().Set(0.9)

            # Remove collision walls
            for wall in collision_walls:
                stage.RemovePrim(wall.GetPath())
            return boxes

        # Run the example scenario
        async def run_box_stacking_scenarios_async(num_pallets, env_url=None, write_data=False):
            # Get assets root path once for all asset loading operations
            assets_root_path = await get_assets_root_path_async()

            # List of pallets and boxes to randomly choose from with their respective weights
            pallets_urls_and_weights = [
                (assets_root_path + "/Isaac/Environments/Simple_Warehouse/Props/SM_PaletteA_01.usd", 0.25),
                (assets_root_path + "/Isaac/Environments/Simple_Warehouse/Props/SM_PaletteA_02.usd", 0.75),
            ]
            boxes_urls_and_weights = [
                (assets_root_path + "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxA_01.usd", 0.02),
                (assets_root_path + "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxB_01.usd", 0.06),
                (assets_root_path + "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxC_01.usd", 0.12),
                (assets_root_path + "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxD_01.usd", 0.80),
            ]

            # Load a predefined or create a new stage
            if env_url is not None:
                env_path = env_url if env_url.startswith("omniverse://") else assets_root_path + env_url
                omni.usd.get_context().open_stage(env_path)
                stage = omni.usd.get_context().get_stage()
            else:
                omni.usd.get_context().new_stage()
                stage = omni.usd.get_context().get_stage()
                distant_light = stage.DefinePrim("/World/Lights/DistantLight", "DistantLight")
                distant_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(400.0)
                if not distant_light.HasAttribute("xformOp:rotateXYZ"):
                    UsdGeom.Xformable(distant_light).AddRotateXYZOp()
                distant_light.GetAttribute("xformOp:rotateXYZ").Set((0, 60, 0))
                dome_light = stage.DefinePrim("/World/Lights/DomeLight", "DomeLight")
                dome_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(500.0)

            # Spawn the pallets
            pallets = []
            pallets_urls, pallets_weights = zip(*pallets_urls_and_weights)
            rand_pallet_urls = random.choices(pallets_urls, weights=pallets_weights, k=num_pallets)
            # Custom pallet poses for the evnironment
            custom_pallet_locations = [
                (-9.3, 5.3, 1.3),
                (-9.3, 7.3, 1.3),
                (-9.3, -0.6, 1.3),
            ]
            random.shuffle(custom_pallet_locations)
            for i, pallet_url in enumerate(rand_pallet_urls):
                # Use a custom location for every other pallet
                if env_url is not None:
                    if i % 2 == 0 and custom_pallet_locations:
                        rand_loc = Gf.Vec3d(*custom_pallet_locations.pop())
                    else:
                        rand_loc = Gf.Vec3d(-6.5, i * 1.75, 0) + Gf.Vec3d(
                            random.uniform(-0.2, 0.2), random.uniform(0, 0.2), 0
                        )
                else:
                    rand_loc = Gf.Vec3d(i * 1.5, 0, 0) + Gf.Vec3d(random.uniform(0, 0.2), random.uniform(-0.2, 0.2), 0)
                rand_rot = (0, 0, random.choice([180, 90, 0, -90, -180]) + random.uniform(-15, 15))
                pallet_prim = create_asset_with_colliders(
                    stage, pallet_url, f"/World/Pallet_{i}", location=rand_loc, rotation=rand_rot
                )
                pallets.append(pallet_prim)

            # Stack the boxes on the pallets
            total_boxes = []
            for pallet in pallets:
                if env_url is not None:
                    rand_num_boxes = random.randint(8, 15)
                    stacked_boxes = await stack_boxes_on_pallet_async(
                        pallet, boxes_urls_and_weights, num_boxes=rand_num_boxes, drop_height=1.0
                    )
                else:
                    rand_num_boxes = random.randint(12, 20)
                    stacked_boxes = await stack_boxes_on_pallet_async(
                        pallet, boxes_urls_and_weights, num_boxes=rand_num_boxes
                    )
                total_boxes.extend(stacked_boxes)

            # Re-enable rigid body dynamics of the boxes and run the simulation for a while
            for box in total_boxes:
                UsdPhysics.RigidBodyAPI(box).GetRigidBodyEnabledAttr().Set(True)
            timeline = omni.timeline.get_timeline_interface()
            timeline.play()
            for _ in range(200):
                await omni.kit.app.get_app().next_update_async()
            timeline.pause()

            if write_data:
                print(f"Writing data to {box_stacking_dir}..")
                backend = rep.backends.get("DiskBackend")
                backend.initialize(output_dir=box_stacking_dir)
                writer = rep.WriterRegistry.get("BasicWriter")
                writer.initialize(backend=backend, rgb=True)
                cam = rep.functional.create.camera(position=(5, -5, 2), look_at=(0, 0, 0), name="PalletCamera")
                rp = rep.create.render_product(cam, resolution=(512, 512))
                writer.attach(rp)

                # Capture the data and wait for the data to be written to disk
                await rep.orchestrator.step_async(rt_subframes=8)

                # Wait for the data to be written to disk and cleanup
                await rep.orchestrator.wait_until_complete_async()
                writer.detach()
                rp.destroy()

        # asyncio.ensure_future(run_box_stacking_scenarios_async(num_pallets=1, write_data=True))
        # asyncio.ensure_future(
        #     run_box_stacking_scenarios_async(
        #         num_pallets=6, env_url="/Isaac/Environments/Simple_Warehouse/warehouse.usd", write_data=True
        #     )
        # )

        # Test
        await run_box_stacking_scenarios_async(num_pallets=1, write_data=True)

        folder_contents_success = validate_folder_contents(path=box_stacking_dir, expected_counts={"png": 1})
        self.assertTrue(folder_contents_success, f"Output directory contents validation failed for {box_stacking_dir}")
