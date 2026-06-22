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

"""Verify the scene-based forklift SDG workflow writes the expected multi-camera dataset files."""

import tempfile
from typing import Any

import carb.settings
import omni.kit
import omni.usd
from isaacsim.test.utils.file_validation import validate_folder_contents


class TestSceneBasedSDG(omni.kit.test.AsyncTestCase):
    """Runs the warehouse scene SDG example with physics settling and graph randomizers."""

    async def setUp(self) -> None:
        """Open a clean stage and preserve the DLSS setting used by the SDG capture."""
        await omni.kit.app.get_app().next_update_async()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self.original_dlss_exec_mode = carb.settings.get_settings().get("rtx/post/dlss/execMode")

    async def tearDown(self) -> Any:
        """Close the stage, wait for asset loading to finish, and restore the DLSS setting.

        Returns:
            None.
        """
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()
        carb.settings.get_settings().set("rtx/post/dlss/execMode", self.original_dlss_exec_mode)

    async def test_scene_based_sdg(self) -> Any:
        """Generate forklift, pallet, box, cone, lighting, and camera randomizations and validate outputs.

        Returns:
            None.
        """
        import math
        import os

        import carb.settings
        import numpy as np
        import omni.kit.app
        import omni.replicator.core as rep
        import omni.usd
        from isaacsim.core.experimental.prims import GeomPrim, RigidPrim, XformPrim
        from isaacsim.core.experimental.utils.bounds import (
            compute_combined_aabb,
            compute_obb,
            create_bbox_cache,
            get_obb_corners,
        )
        from isaacsim.core.experimental.utils.semantics import add_labels, remove_all_labels
        from isaacsim.core.experimental.utils.stage import add_reference_to_stage, define_prim
        from isaacsim.core.experimental.utils.transform import euler_angles_to_quaternion, quaternion_to_euler_angles
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.storage.native import get_assets_root_path_async
        from pxr import Gf, Usd, UsdGeom

        def _create_prim(
            prim_path: Any,
            position: Any | None = None,
            orientation: Any | None = None,
            usd_path: Any | None = None,
            semantic_label: Any | None = None,
        ) -> Any:
            if usd_path is not None:
                prim = add_reference_to_stage(usd_path, prim_path)
            else:
                prim = define_prim(prim_path, "Xform")
            if semantic_label is not None:
                add_labels(prim, labels=[semantic_label], taxonomy="class")
            if position is not None or orientation is not None:
                XformPrim(prim_path, positions=position, orientations=orientation, reset_xform_op_properties=True)
            return prim

        def setup_writer(config: dict) -> rep.Writer | None:
            """Setup and initialize writer with optional backend support.

            Args:
                config: Writer and backend configuration dictionary.

            Returns:
                Initialized writer, or None if the writer or backend configuration is invalid.
            """

            def normalize_output_dir(params: dict[str, Any]) -> None:
                if "output_dir" in params and not os.path.isabs(params["output_dir"]):
                    params["output_dir"] = os.path.join(os.getcwd(), params["output_dir"])

            writer_type = config.get("writer", "BasicWriter")
            if writer_type not in rep.WriterRegistry.get_writers():
                print(f"[SDG] Writer type '{writer_type}' not found in registry.")
                return None

            writer = rep.WriterRegistry.get(writer_type)
            writer_kwargs = dict(config.get("writer_config", {}))
            normalize_output_dir(writer_kwargs)

            backend_type = config.get("backend_type")
            backend = None
            if backend_type:
                try:
                    backend = rep.backends.get(backend_type)
                except Exception as e:
                    print(f"[SDG] Backend '{backend_type}' not found: {e}")
                    return None

                backend_params = dict(config.get("backend_params", {}))
                normalize_output_dir(backend_params)

                try:
                    print(f"[SDG] Backend: {backend_type} | Params: {backend_params}")
                    backend.initialize(**backend_params)
                except TypeError as e:
                    print(f"[SDG] Invalid backend params: {e}")
                    return None

            if "output_dir" in writer_kwargs:
                print(f"[SDG] Output: {writer_kwargs['output_dir']}")

            backend_info = f" + {backend_type}" if backend else ""
            print(f"[SDG] Writer: {writer_type}{backend_info} | Config: {writer_kwargs}")

            try:
                if backend:
                    writer.initialize(backend=backend, **writer_kwargs)
                else:
                    writer.initialize(**writer_kwargs)
            except TypeError as e:
                print(f"[SDG] Invalid writer params: {e}")
                return None

            return writer

        async def simulate_falling_objects_async(
            forklift_prim: Usd.Prim,
            assets_root_path: str,
            config: dict,
            max_sim_steps: int = 250,
            num_boxes: int = 8,
            rng: np.random.Generator | None = None,
        ) -> None:
            """Run physics simulation to drop boxes on pallet near forklift.

            Args:
                forklift_prim: Forklift prim used as the placement reference.
                assets_root_path: Root URL or path for Isaac Sim assets.
                config: SDG asset configuration dictionary.
                max_sim_steps: Maximum number of physics steps to simulate.
                num_boxes: Number of boxes to spawn and drop.
                rng: Random number generator for placement values, or None to create one.
            """
            if rng is None:
                rng = np.random.default_rng()

            forklift_transform = omni.usd.get_world_transform_matrix(forklift_prim)
            sim_pallet_offset = Gf.Matrix4d().SetTranslate(Gf.Vec3d(rng.uniform(-1, 1), rng.uniform(-4, -3.6), 0))
            sim_pallet_position = list((sim_pallet_offset * forklift_transform).ExtractTranslation())
            sim_pallet_rotation = euler_angles_to_quaternion([0, 0, rng.uniform(0, math.pi)]).numpy()

            sim_pallet = _create_prim(
                prim_path="/World/SimulatedPallet",
                position=sim_pallet_position,
                orientation=sim_pallet_rotation,
                usd_path=assets_root_path + config["pallet"]["url"],
                semantic_label=config["pallet"]["class"],
            )
            sim_pallet_geom = GeomPrim(f"{str(sim_pallet.GetPrimPath())}/.*", apply_collision_apis=True)
            sim_pallet_geom.set_collision_approximations("boundingCube")

            bbox_cache = create_bbox_cache()
            current_height = bbox_cache.ComputeLocalBound(sim_pallet).GetRange().GetSize()[2] * 1.1

            sim_box_rigid_prims = []
            for box_index in range(num_boxes):
                box_xy_offset = [rng.uniform(-0.2, 0.2), rng.uniform(-0.2, 0.2), current_height]
                sim_box = _create_prim(
                    prim_path=f"/World/SimulatedCardbox_{box_index}",
                    position=[a + b for a, b in zip(sim_pallet_position, box_xy_offset)],
                    orientation=sim_pallet_rotation,
                    usd_path=assets_root_path + config["cardbox"]["url"],
                    semantic_label=config["cardbox"]["class"],
                )
                current_height += bbox_cache.ComputeLocalBound(sim_box).GetRange().GetSize()[2] * 1.1

                sim_box_geom = GeomPrim(f"{str(sim_box.GetPrimPath())}/.*", apply_collision_apis=True)
                sim_box_geom.set_collision_approximations("convexHull")
                sim_box_rigid_prims.append(RigidPrim(str(sim_box.GetPrimPath())))

            SimulationManager.set_physics_dt(1.0 / 90.0)
            SimulationManager.initialize_physics()

            velocity_threshold = 0.01
            for step in range(max_sim_steps):
                SimulationManager.step()
                if sim_box_rigid_prims:
                    top_box_velocity = sim_box_rigid_prims[-1].get_velocities(indices=[0])[0].numpy()
                    if np.linalg.norm(top_box_velocity) < velocity_threshold:
                        print(f"[SDG] Simulation settled at step {step}")
                        break
                await omni.kit.app.get_app().next_update_async()

        def setup_camera_bounds(
            pallet_prim: Usd.Prim, forklift_prim: Usd.Prim, pallet_tf: Gf.Matrix4d, forklift_tf: Gf.Matrix4d
        ) -> dict[str, dict[str, tuple[float, float, float]]]:
            """Calculate camera randomization bounds for pallet, top view, and driver cameras.

            Args:
                pallet_prim: Pallet prim used to calculate pallet camera bounds.
                forklift_prim: Forklift prim used to calculate top and driver camera bounds.
                pallet_tf: World transform of the pallet.
                forklift_tf: World transform of the forklift.

            Returns:
                Camera names mapped to minimum and maximum position bounds.
            """
            pallet_pos = pallet_tf.ExtractTranslation()
            pallet_cam_bounds = {
                "min": (pallet_pos[0] - 2, pallet_pos[1] - 2, 2),
                "max": (pallet_pos[0] + 2, pallet_pos[1] + 2, 4),
            }

            forklift_pos = forklift_tf.ExtractTranslation()
            top_cam_bounds = {
                "min": (forklift_pos[0], forklift_pos[1], 9),
                "max": (forklift_pos[0], forklift_pos[1], 11),
            }

            driver_cam_pos = forklift_pos + Gf.Vec3d(0.0, 0.0, 1.9)
            driver_cam_bounds = {
                "min": (driver_cam_pos[0], driver_cam_pos[1], driver_cam_pos[2] - 0.25),
                "max": (driver_cam_pos[0], driver_cam_pos[1], driver_cam_pos[2] + 0.25),
            }

            return {
                "pallet_cam": pallet_cam_bounds,
                "top_cam": top_cam_bounds,
                "driver_cam": driver_cam_bounds,
            }

        def create_scatter_plane_for_prim(
            prim: Usd.Prim, prim_tf: Gf.Matrix4d, parent_path: str, scale_factor: float = 0.8, visible: bool = False
        ) -> Usd.Prim:
            """Create scatter plane sized and aligned to prim surface.

            Args:
                prim: Prim whose bounds define the scatter plane.
                prim_tf: World transform of the prim.
                parent_path: Parent prim path for the scatter plane.
                scale_factor: Multiplier applied to the prim footprint.
                visible: Whether the scatter plane should be visible.

            Returns:
                Created scatter plane prim.
            """
            bb_cache = create_bbox_cache()
            prim_bbox = bb_cache.ComputeLocalBound(prim)
            prim_bbox.Transform(prim_tf)
            prim_size = prim_bbox.GetRange().GetSize()

            prim_quat = prim_tf.ExtractRotation().GetQuaternion()
            prim_quat_xyzw = (prim_quat.GetReal(), *prim_quat.GetImaginary())
            prim_rotation_deg = quaternion_to_euler_angles(np.array(prim_quat_xyzw), degrees=True).numpy()

            prim_pos = prim_tf.ExtractTranslation()
            scatter_plane_scale = (prim_size[0] * scale_factor, prim_size[1] * scale_factor, 1)
            scatter_plane_pos = prim_pos + Gf.Vec3d(0, 0, prim_size[2])

            scatter_plane = rep.functional.create.plane(
                scale=scatter_plane_scale,
                position=tuple(scatter_plane_pos),
                rotation=tuple(prim_rotation_deg),
                visible=visible,
                parent=parent_path,
            )

            return scatter_plane

        def setup_cone_placement_corners(
            forklift_prim: Usd.Prim, bb_cache: Any | None = None, scale_factor: float = 1.3
        ) -> tuple[list[list[float]], tuple[float, float, float]]:
            """Calculate forklift OBB corners for cone placement.

            Args:
                forklift_prim: Forklift prim used to compute an oriented bounding box.
                bb_cache: Bounding box cache to reuse, or None to create one.
                scale_factor: Multiplier applied to the forklift footprint.

            Returns:
                Cone placement corner positions and forklift rotation in degrees.
            """
            if bb_cache is None:
                bb_cache = create_bbox_cache()

            forklift_obb_center, forklift_obb_axes, forklift_obb_extent = compute_obb(
                forklift_prim, bbox_cache=bb_cache
            )
            enlarged_extent = (
                forklift_obb_extent[0] * scale_factor,
                forklift_obb_extent[1] * scale_factor,
                forklift_obb_extent[2],
            )
            forklift_obb_corners = get_obb_corners(forklift_obb_center, forklift_obb_axes, enlarged_extent)

            cone_placement_corners = [
                forklift_obb_corners[0].tolist(),
                forklift_obb_corners[2].tolist(),
                forklift_obb_corners[4].tolist(),
                forklift_obb_corners[6].tolist(),
            ]

            forklift_obb_quat = Gf.Matrix3d(forklift_obb_axes).ExtractRotation().GetQuaternion()
            forklift_obb_quat_xyzw = (forklift_obb_quat.GetReal(), *forklift_obb_quat.GetImaginary())
            forklift_rotation_deg = quaternion_to_euler_angles(np.array(forklift_obb_quat_xyzw), degrees=True).numpy()

            return cone_placement_corners, forklift_rotation_deg

        def register_lights_graph_randomizer(forklift_prim: Usd.Prim, pallet_prim: Usd.Prim, event_name: str) -> None:
            """Register graph randomizer for sphere lights.

            Args:
                forklift_prim: Forklift prim included in the lighting bounds.
                pallet_prim: Pallet prim included in the lighting bounds.
                event_name: Custom event name that triggers the randomizer.
            """
            bb_cache = create_bbox_cache()
            combined_bounds = compute_combined_aabb([forklift_prim, pallet_prim], bbox_cache=bb_cache)
            light_pos_min = (combined_bounds[0], combined_bounds[1], 6)
            light_pos_max = (combined_bounds[3], combined_bounds[4], 7)

            with rep.trigger.on_custom_event(event_name):
                rep.create.light(
                    light_type="Sphere",
                    color=rep.distribution.uniform((0.2, 0.1, 0.1), (0.9, 0.8, 0.8)),
                    intensity=rep.distribution.uniform(2000, 4000),
                    position=rep.distribution.uniform(light_pos_min, light_pos_max),
                    scale=rep.distribution.uniform(1, 4),
                    count=3,
                )

        def register_cardboxes_materials_graph_randomizer(
            cardboxes: list[Usd.Prim], cardbox_material_urls: list[str], event_name: str
        ) -> None:
            """Register graph randomizer for cardbox materials.

            Args:
                cardboxes: Cardbox prims whose mesh children receive randomized materials.
                cardbox_material_urls: Material URLs to sample from.
                event_name: Custom event name that triggers the randomizer.
            """
            cardbox_mesh_paths = []
            for cardbox in cardboxes:
                meshes = [child for child in cardbox.GetChildren() if child.IsA(UsdGeom.Mesh)]
                cardbox_mesh_paths.extend([mesh.GetPrimPath() for mesh in meshes])

            with rep.trigger.on_custom_event(event_name):
                cardbox_mesh_group_node = rep.create.group(cardbox_mesh_paths)
                with cardbox_mesh_group_node:
                    rep.randomizer.materials(cardbox_material_urls)

        async def run_example_async(config: Any) -> None:
            assets_root_path = await get_assets_root_path_async()
            if assets_root_path is None:
                print("[SDG] Could not get nucleus server path")
                return

            # Load environment stage
            env_url = config.get("env_url", "/Isaac/Environments/Grid/default_environment.usd")
            env_path = env_url if env_url.startswith("omniverse://") else assets_root_path + env_url
            print(f"[SDG] Loading Stage {env_url}")
            omni.usd.get_context().open_stage(env_path)
            stage = omni.usd.get_context().get_stage()

            await omni.kit.app.get_app().next_update_async()

            # Initialize randomization
            rep.set_global_seed(42)
            rng = np.random.default_rng(42)

            # Configure replicator for manual triggering
            rep.orchestrator.set_capture_on_play(False)

            # Set DLSS to Quality mode for best SDG results
            carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

            # Clear previous semantic labels
            if config.get("clear_previous_semantics", True):
                for prim in stage.Traverse():
                    remove_all_labels(prim, include_descendants=True)

            # Create SDG scope for organizing all generated objects
            sdg_scope = stage.DefinePrim("/SDG", "Scope")

            # Spawn forklift at random pose
            forklift_prim = _create_prim(
                prim_path="/SDG/Forklift",
                position=(rng.uniform(-20, -2), rng.uniform(-1, 3), 0),
                orientation=euler_angles_to_quaternion([0, 0, rng.uniform(0, math.pi)]).numpy(),
                usd_path=assets_root_path + config["forklift"]["url"],
                semantic_label=config["forklift"]["class"],
            )

            # Spawn pallet in front of forklift with random offset
            forklift_tf = omni.usd.get_world_transform_matrix(forklift_prim)
            pallet_offset_tf = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, rng.uniform(-1.8, -1.2), 0))
            pallet_pos = list((pallet_offset_tf * forklift_tf).ExtractTranslation())
            forklift_quat = forklift_tf.ExtractRotationQuat()
            forklift_quat_xyzw = (forklift_quat.GetReal(), *forklift_quat.GetImaginary())

            pallet_prim = _create_prim(
                prim_path="/SDG/Pallet",
                position=pallet_pos,
                orientation=forklift_quat_xyzw,
                usd_path=assets_root_path + config["pallet"]["url"],
                semantic_label=config["pallet"]["class"],
            )

            # Create cardboxes for pallet scattering
            cardboxes = []
            for i in range(5):
                cardbox = _create_prim(
                    prim_path=f"/SDG/CardBox_{i}",
                    usd_path=assets_root_path + config["cardbox"]["url"],
                    semantic_label=config["cardbox"]["class"],
                )
                cardboxes.append(cardbox)

            # Create traffic cone for corner placement
            cone = _create_prim(
                prim_path="/SDG/Cone",
                usd_path=assets_root_path + config["cone"]["url"],
                semantic_label=config["cone"]["class"],
            )

            # Create cameras
            rep.functional.create.scope(name="Cameras", parent="/SDG")
            driver_cam = rep.functional.create.camera(
                focus_distance=400.0,
                focal_length=24.0,
                clipping_range=(0.1, 10000000.0),
                name="DriverCam",
                parent="/SDG/Cameras",
            )
            pallet_cam = rep.functional.create.camera(name="PalletCam", parent="/SDG/Cameras")
            top_view_cam = rep.functional.create.camera(
                clipping_range=(6.0, 1000000.0), name="TopCam", parent="/SDG/Cameras"
            )

            await omni.kit.app.get_app().next_update_async()

            # Setup render products
            resolution = config.get("resolution", (512, 512))
            forklift_rp = rep.create.render_product(top_view_cam, resolution, name="TopView")
            driver_rp = rep.create.render_product(driver_cam, resolution, name="DriverView")
            pallet_rp = rep.create.render_product(pallet_cam, resolution, name="PalletView")

            render_products = [forklift_rp, driver_rp, pallet_rp]
            for render_product in render_products:
                render_product.hydra_texture.set_updates_enabled(False)

            # Initialize writer and attach to render products
            writer = setup_writer(config)
            if not writer:
                print("[SDG] Failed to setup writer")
                return

            writer.attach(render_products)

            for render_product in render_products:
                render_product.hydra_texture.set_updates_enabled(True)

            rt_subframes = config.get("rt_subframes", -1)

            # Calculate camera randomization bounds
            pallet_tf = omni.usd.get_world_transform_matrix(pallet_prim)
            camera_bounds = setup_camera_bounds(pallet_prim, forklift_prim, pallet_tf, forklift_tf)
            pallet_cam_bounds_min = camera_bounds["pallet_cam"]["min"]
            pallet_cam_bounds_max = camera_bounds["pallet_cam"]["max"]
            top_cam_bounds_min = camera_bounds["top_cam"]["min"]
            top_cam_bounds_max = camera_bounds["top_cam"]["max"]
            driver_cam_bounds_min = camera_bounds["driver_cam"]["min"]
            driver_cam_bounds_max = camera_bounds["driver_cam"]["max"]

            # Setup scatter plane and cone placement
            scatter_plane = create_scatter_plane_for_prim(pallet_prim, pallet_tf, parent_path="/SDG", scale_factor=0.8)
            cone_placement_corners, forklift_rotation_deg = setup_cone_placement_corners(forklift_prim)

            # Register graph-based randomizers for lights and materials
            register_lights_graph_randomizer(forklift_prim, pallet_prim, event_name="randomize_lights")

            cardbox_material_urls = [
                f"{assets_root_path}/Isaac/Environments/Simple_Warehouse/Materials/MI_PaperNotes_01.mdl",
                f"{assets_root_path}/Isaac/Environments/Simple_Warehouse/Materials/MI_CardBoxB_05.mdl",
            ]
            register_cardboxes_materials_graph_randomizer(
                cardboxes, cardbox_material_urls, event_name="randomize_cardboxes_materials"
            )

            # Run physics simulation to settle boxes on pallet
            await simulate_falling_objects_async(forklift_prim, assets_root_path, config, rng=rng)

            # SDG loop - generate frames with randomizations
            num_frames = config.get("num_frames", 10)
            print(f"[SDG] Running SDG for {num_frames} frames")
            for i in range(num_frames):
                print(f"[SDG] Frame {i}/{num_frames}")

                print(f"[SDG]  Randomizing boxes on pallet.")
                rep.functional.randomizer.scatter_2d(
                    prims=cardboxes, surface_prims=scatter_plane, check_for_collisions=True, rng=rng
                )

                print(f"[SDG]  Randomizing boxes materials.")
                rep.utils.send_og_event(event_name="randomize_cardboxes_materials")
                print(f"[SDG]  Randomizing lights.")
                rep.utils.send_og_event(event_name="randomize_lights")

                print(f"[SDG]  Randomizing pallet camera.")
                rep.functional.modify.pose(
                    pallet_cam,
                    position_value=rng.uniform(pallet_cam_bounds_min, pallet_cam_bounds_max),
                    look_at_value=pallet_prim,
                    look_at_up_axis=(0, 0, 1),
                )

                print(f"[SDG]  Randomizing driver camera.")
                rep.functional.modify.pose(
                    driver_cam,
                    position_value=rng.uniform(driver_cam_bounds_min, driver_cam_bounds_max),
                    look_at_value=pallet_prim,
                    look_at_up_axis=(0, 0, 1),
                )

                if i % 2 == 0:
                    print(f"[SDG]  Randomizing cone position.")
                    selected_corner = cone_placement_corners[rng.integers(0, len(cone_placement_corners))]
                    rep.functional.modify.pose(
                        cone,
                        position_value=selected_corner,
                    )

                if i % 4 == 0:
                    print(f"[SDG]  Randomizing top view camera.")
                    roll_angle = rng.uniform(0, 2 * np.pi)
                    rep.functional.modify.pose(
                        top_view_cam,
                        position_value=rng.uniform(top_cam_bounds_min, top_cam_bounds_max),
                        look_at_value=forklift_prim,
                        look_at_up_axis=(np.cos(roll_angle), np.sin(roll_angle), 0.0),
                    )

                print(f"[SDG]  Capturing frame with rt_subframes={rt_subframes}")
                await rep.orchestrator.step_async(delta_time=0.0, rt_subframes=rt_subframes)

            # Cleanup
            await rep.orchestrator.wait_until_complete_async()
            writer.detach()
            for render_product in render_products:
                render_product.destroy()

            print("[SDG] Complete")

        config = {
            "resolution": [512, 512],
            "rt_subframes": 32,
            "num_frames": 10,
            "env_url": "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
            "writer": "BasicWriter",
            "backend_type": "DiskBackend",
            "backend_params": {
                "output_dir": "_out_scene_based_sdg",
            },
            "writer_config": {
                "rgb": True,
                "bounding_box_2d_tight": True,
                "semantic_segmentation": True,
                "distance_to_image_plane": True,
                "bounding_box_3d": True,
                "occlusion": True,
            },
            "clear_previous_semantics": True,
            "forklift": {
                "url": "/Isaac/Props/Forklift/forklift.usd",
                "class": "forklift",
            },
            "cone": {
                "url": "/Isaac/Environments/Simple_Warehouse/Props/S_TrafficCone.usd",
                "class": "traffic_cone",
            },
            "pallet": {
                "url": "/Isaac/Environments/Simple_Warehouse/Props/SM_PaletteA_01.usd",
                "class": "pallet",
            },
            "cardbox": {
                "url": "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxD_04.usd",
                "class": "cardbox",
            },
        }

        # asyncio.ensure_future(run_example_async(config))

        # Test parameters
        num_frames = 4
        env_url = "/Isaac/Environments/Grid/default_environment.usd"
        config["num_frames"] = num_frames
        config["env_url"] = env_url
        out_dir = tempfile.mkdtemp(prefix="test_scene_based_sdg_")
        print(f"Output directory: {out_dir}")
        config["backend_params"]["output_dir"] = out_dir
        await run_example_async(config)

        # Validate that all expected files were written to disk
        # pngs: num_frames * 3 (cameras) * ( 1 (rgb) + 1 (semantic segmentation))
        # json: num_frames * 3 (cameras) * ( 2 (bounding_box_2d_tight) + 2 (bounding_box_3d) + 1 (semantic segmentation) )
        # npy: num_frames * 3 (cameras) * ( 1 (bounding_box_2d_tight) + 1 (bounding_box_3d) +  1 (distance to image plane) + 1 (occlusion))
        expected_pngs = num_frames * 3 * (1 + 1)
        expected_jsons = num_frames * 3 * (2 + 2 + 1)
        expected_npy = num_frames * 3 * (1 + 1 + 1 + 1)
        all_data_written = validate_folder_contents(
            out_dir, {"png": expected_pngs, "json": expected_jsons, "npy": expected_npy}, recursive=True
        )
        self.assertTrue(all_data_written, f"Not all files were written in to: {out_dir}")
