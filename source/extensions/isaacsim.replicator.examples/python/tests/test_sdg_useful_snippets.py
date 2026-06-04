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

import tempfile

import carb.settings
import omni.kit
import omni.usd
from isaacsim.test.utils.file_validation import validate_folder_contents


class TestSDGUsefulSnippets(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()
        self.original_dlss_exec_mode = carb.settings.get_settings().get("rtx/post/dlss/execMode")

    async def tearDown(self):
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()
        carb.settings.get_settings().set("rtx/post/dlss/execMode", self.original_dlss_exec_mode)

    async def test_sdg_snippet_multi_camera(self):
        import os

        import carb.settings
        import omni.replicator.core as rep
        import omni.usd
        from omni.replicator.core import Writer
        from omni.replicator.core.backends import DiskBackend
        from omni.replicator.core.functional import write_image

        NUM_FRAMES = 5
        mc_test_root = tempfile.mkdtemp(prefix="test_mc_")
        print(f"Test output root: {mc_test_root}")

        # Randomize cube color every frame using a graph-based replicator randomizer
        def cube_color_randomizer():
            cube_prims = rep.get.prims(path_pattern="Cube")
            with cube_prims:
                rep.randomizer.color(colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))
            return cube_prims.node

        # Example of custom writer class to access the annotator data
        class MyWriter(Writer):
            def __init__(self, rgb: bool = True):
                # Organize data from render product perspective (legacy, annotator, renderProduct)
                self.data_structure = "renderProduct"
                self.annotators = []
                self._frame_id = 0
                if rgb:
                    # Create a new rgb annotator and add it to the writer's list of annotators
                    self.annotators.append(rep.annotators.get("rgb"))
                # Create writer output directory and initialize DiskBackend
                writer_dir = os.path.join(mc_test_root, "writer")
                print(f"Writing writer data to {writer_dir}")
                self.backend = DiskBackend(output_dir=writer_dir, overwrite=True)

            def write(self, data):
                if "renderProducts" in data:
                    for rp_name, rp_data in data["renderProducts"].items():
                        if "rgb" in rp_data:
                            file_path = f"{rp_name}_frame_{self._frame_id}.png"
                            self.backend.schedule(write_image, data=rp_data["rgb"]["data"], path=file_path)
                self._frame_id += 1

        rep.WriterRegistry.register(MyWriter)

        # Create a new stage
        omni.usd.get_context().new_stage()

        # Set global random seed for the replicator randomizer
        rep.set_global_seed(11)

        # Disable capture on play to capture data manually using step
        rep.orchestrator.set_capture_on_play(False)

        # Set DLSS to Quality mode (2) for best SDG results , options: 0 (Performance), 1 (Balanced), 2 (Quality), 3 (Auto)
        carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

        # Setup stage
        rep.functional.create.xform(name="World")
        rep.functional.create.dome_light(intensity=900, parent="/World", name="DomeLight")
        cube = rep.functional.create.cube(parent="/World", name="Cube", semantics={"class": "my_cube"})

        # Register the graph-based cube color randomizer to trigger on every frame
        rep.randomizer.register(cube_color_randomizer)
        with rep.trigger.on_frame():
            rep.randomizer.cube_color_randomizer()

        # Create cameras
        cam_top = rep.functional.create.camera(position=(0, 0, 5), look_at=(0, 0, 0), parent="/World", name="CamTop")
        cam_side = rep.functional.create.camera(position=(2, 2, 0), look_at=(0, 0, 0), parent="/World", name="CamSide")
        cam_persp = rep.functional.create.camera(
            position=(5, 5, 5), look_at=(0, 0, 0), parent="/World", name="CamPersp"
        )

        # Create the render products
        rp_top = rep.create.render_product(cam_top, resolution=(320, 320), name="RpTop")
        rp_side = rep.create.render_product(cam_side, resolution=(640, 640), name="RpSide")
        rp_persp = rep.create.render_product(cam_persp, resolution=(1024, 1024), name="RpPersp")

        # Example of accessing the data through a custom writer
        writer = rep.WriterRegistry.get("MyWriter")
        writer.initialize(rgb=True)
        writer.attach([rp_top, rp_side, rp_persp])

        # Example of accessing the data directly through annotators
        rgb_annotators = []
        for rp in [rp_top, rp_side, rp_persp]:
            # Create a new rgb annotator for each render product
            rgb = rep.annotators.get("rgb")
            # Attach the annotator to the render product
            rgb.attach(rp)
            rgb_annotators.append(rgb)

        # Create annotator output directory
        output_dir_annot = os.path.join(mc_test_root, "annot")
        print(f"Writing annotator data to {output_dir_annot}")
        os.makedirs(output_dir_annot)

        async def run_example_async():
            for i in range(NUM_FRAMES):
                print(f"Step {i}")
                # The step function triggers registered graph-based randomizers, collects data from annotators,
                # and invokes the write function of attached writers with the annotator data
                await rep.orchestrator.step_async(rt_subframes=32)
                for j, rgb_annot in enumerate(rgb_annotators):
                    file_path = os.path.join(output_dir_annot, f"rp{j}_step_{i}.png")
                    write_image(path=file_path, data=rgb_annot.get_data())

            # Wait for the data to be written and release resources
            await rep.orchestrator.wait_until_complete_async()
            writer.detach()
            for annot in rgb_annotators:
                annot.detach()
            for rp in [rp_top, rp_side, rp_persp]:
                rp.destroy()

        # asyncio.ensure_future(run_example_async())
        await run_example_async()

        # Validate the output directory contents
        annot_output_dir = os.path.join(mc_test_root, "annot")
        folder_contents_success_annot = validate_folder_contents(
            path=annot_output_dir, expected_counts={"png": NUM_FRAMES * 3}
        )
        self.assertTrue(
            folder_contents_success_annot, f"Output directory contents validation failed for {annot_output_dir}"
        )
        writer_output_dir = os.path.join(mc_test_root, "writer")
        folder_contents_success_writer = validate_folder_contents(
            path=writer_output_dir, expected_counts={"png": NUM_FRAMES * 3}
        )
        self.assertTrue(
            folder_contents_success_writer, f"Output directory contents validation failed for {writer_output_dir}"
        )

    async def test_sdg_snippet_simulation_get_data(self):
        import os

        import carb.settings
        import numpy as np
        import omni
        import omni.replicator.core as rep
        from isaacsim.core.experimental.objects import GroundPlane
        from isaacsim.core.simulation_manager import SimulationManager
        from omni.replicator.core.functional import write_image, write_json
        from pxr import UsdPhysics

        # Util function to save semantic segmentation annotator data
        def write_sem_data(sem_data, file_path):
            id_to_labels = sem_data["info"]["idToLabels"]
            write_json(path=file_path + ".json", data=id_to_labels)
            sem_image_data = sem_data["data"]
            write_image(path=file_path + ".png", data=sem_image_data)

        # Create a new stage
        omni.usd.get_context().new_stage()

        # Setting capture on play to False will prevent the replicator from capturing data each frame
        rep.orchestrator.set_capture_on_play(False)

        # Set DLSS to Quality mode (2) for best SDG results , options: 0 (Performance), 1 (Balanced), 2 (Quality), 3 (Auto)
        carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

        # Add a dome light and a ground plane
        rep.functional.create.xform(name="World")
        rep.functional.create.dome_light(intensity=500, parent="/World", name="DomeLight")
        ground_plane = GroundPlane("/World/GroundPlane")
        rep.functional.modify.semantics(ground_plane.prims, {"class": "ground_plane"}, mode="add")

        # Create a camera and render product to collect the data from
        rep.functional.create.xform(name="World")
        cam = rep.functional.create.camera(position=(5, 5, 5), look_at=(0, 0, 0), parent="/World", name="Camera")
        rp = rep.create.render_product(cam, resolution=(512, 512), name="MyRenderProduct")

        # Set the output directory for the data
        out_dir = tempfile.mkdtemp(prefix="test_sim_event_")
        writer_dir = os.path.join(out_dir, "writer")
        annotator_dir = os.path.join(out_dir, "annotator")

        os.makedirs(writer_dir)
        os.makedirs(annotator_dir)

        print(f"Outputting data to {out_dir}..")
        backend = rep.backends.get("DiskBackend")
        backend.initialize(output_dir=writer_dir)

        # Example of using a writer to save the data
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(backend=backend, rgb=True, semantic_segmentation=True, colorize_semantic_segmentation=True)
        writer.attach(rp)

        # Example of accesing the data directly from annotators
        rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb_annot.attach(rp)
        sem_annot = rep.AnnotatorRegistry.get_annotator("semantic_segmentation", init_params={"colorize": True})
        sem_annot.attach(rp)

        # Initialize the simulation manager
        SimulationManager.initialize_physics()

        async def run_example_async():
            # Spawn and drop a few cubes, capture data when they stop moving
            for i in range(5):
                cube = rep.functional.create.cube(name=f"Cuboid_{i}", parent="/World")
                rep.functional.modify.position(cube, (0, 0, 10 + i))
                rep.functional.modify.semantics(cube, {"class": "cuboid"}, mode="add")
                rep.functional.physics.apply_rigid_body(cube, with_collider=True)
                physics_rigid_body_api = UsdPhysics.RigidBodyAPI(cube)

                for s in range(500):
                    SimulationManager.step()
                    linear_velocity = physics_rigid_body_api.GetVelocityAttr().Get()
                    speed = np.linalg.norm(linear_velocity)

                    if speed < 0.1:
                        print(f"Cube_{i} stopped moving after {s} simulation steps, writing data..")
                        # Tigger the writer and update the annotators with new data
                        await rep.orchestrator.step_async(rt_subframes=4, delta_time=0.0, pause_timeline=False)
                        rgb_path = os.path.join(annotator_dir, f"Cube_{i}_step_{s}_rgb.png")
                        sem_path = os.path.join(annotator_dir, f"Cube_{i}_step_{s}_sem")
                        write_image(path=rgb_path, data=rgb_annot.get_data())
                        write_sem_data(sem_annot.get_data(), sem_path)
                        break

            # Wait for the data to be written to disk and clean up resources
            await rep.orchestrator.wait_until_complete_async()
            rgb_annot.detach()
            sem_annot.detach()
            writer.detach()
            rp.destroy()

        # asyncio.ensure_future(run_example_async())
        await run_example_async()

        # Validate the output directory contents
        folder_contents_success_annot = validate_folder_contents(
            path=annotator_dir, expected_counts={"png": 10, "json": 5}
        )
        folder_contents_success_writer = validate_folder_contents(
            path=writer_dir, expected_counts={"png": 10, "json": 5}
        )
        self.assertTrue(
            folder_contents_success_annot, f"Output directory contents validation failed for {annotator_dir}"
        )
        self.assertTrue(folder_contents_success_writer, f"Output directory contents validation failed for {writer_dir}")

    async def test_sdg_snippet_custom_event_and_write(self):
        import os

        import carb.settings
        import omni.replicator.core as rep
        import omni.usd

        omni.usd.get_context().new_stage()

        # Set global random seed for the replicator randomizer to ensure reproducibility
        rep.set_global_seed(11)

        # Setting capture on play to False will prevent the replicator from capturing data each frame
        rep.orchestrator.set_capture_on_play(False)

        # Set DLSS to Quality mode (2) for best SDG results , options: 0 (Performance), 1 (Balanced), 2 (Quality), 3 (Auto)
        carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

        rep.functional.create.xform(name="World")
        rep.functional.create.distant_light(intensity=4000, rotation=(315, 0, 0), parent="/World", name="DistantLight")
        small_cube = rep.functional.create.cube(scale=0.75, position=(-1.5, 1.5, 0), parent="/World", name="SmallCube")
        large_cube = rep.functional.create.cube(scale=1.25, position=(1.5, -1.5, 0), parent="/World", name="LargeCube")

        # Graph-based randomizations triggered on custom events
        with rep.trigger.on_custom_event(event_name="randomize_small_cube"):
            small_cube_node = rep.get.prim_at_path(small_cube.GetPath())
            with small_cube_node:
                rep.randomizer.rotation()

        with rep.trigger.on_custom_event(event_name="randomize_large_cube"):
            large_cube_node = rep.get.prim_at_path(large_cube.GetPath())
            with large_cube_node:
                rep.randomizer.rotation()

        # Use the disk backend to write the data to disk
        out_dir = tempfile.mkdtemp(prefix="test_custom_event_")
        print(f"Writing data to {out_dir}")
        backend = rep.backends.get("DiskBackend")
        backend.initialize(output_dir=out_dir)

        cam = rep.functional.create.camera(position=(5, 5, 5), look_at=(0, 0, 0), parent="/World", name="Camera")
        rp = rep.create.render_product(cam, (512, 512))
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(backend=backend, rgb=True)
        writer.attach(rp)

        async def run_example_async():
            print(f"Capturing at original positions")
            await rep.orchestrator.step_async(rt_subframes=8)

            print("Randomizing small cube rotation (graph-based) and capturing...")
            rep.utils.send_og_event(event_name="randomize_small_cube")
            await rep.orchestrator.step_async(rt_subframes=8)

            print("Moving small cube position (USD API) and capturing...")
            small_cube.GetAttribute("xformOp:translate").Set((-1.5, 1.5, -2))
            await rep.orchestrator.step_async(rt_subframes=8)

            print("Randomizing large cube rotation (graph-based) and capturing...")
            rep.utils.send_og_event(event_name="randomize_large_cube")
            await rep.orchestrator.step_async(rt_subframes=8)

            print("Moving large cube position (USD API) and capturing...")
            large_cube.GetAttribute("xformOp:translate").Set((1.5, -1.5, 2))
            await rep.orchestrator.step_async(rt_subframes=8)

            # Wait until all the data is saved to disk and cleanup writer and render product
            await rep.orchestrator.wait_until_complete_async()
            writer.detach()
            rp.destroy()

        # asyncio.ensure_future(run_example_async())
        await run_example_async()

        # Validate the output directory contents
        folder_contents_success = validate_folder_contents(path=out_dir, expected_counts={"png": 5})
        self.assertTrue(folder_contents_success, f"Output directory contents validation failed for {out_dir}")

    async def test_sdg_snippet_motion_blur_short(self):
        import os

        import carb.settings
        import omni.kit.app
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from isaacsim.storage.native import get_assets_root_path
        from pxr import PhysxSchema, UsdPhysics

        motion_blur_root = tempfile.mkdtemp(prefix="test_motion_blur_")
        print(f"Test output root: {motion_blur_root}")

        # Paths to the animated and physics-ready assets
        PHYSICS_ASSET_URL = "/Isaac/Props/YCB/Axis_Aligned_Physics/003_cracker_box.usd"
        ANIM_ASSET_URL = "/Isaac/Props/YCB/Axis_Aligned/003_cracker_box.usd"

        # -z velocities and start locations of the animated (left side) and physics (right side) assets (stage units/s)
        ASSET_VELOCITIES = [0, 5, 10]
        ASSET_X_MIRRORED_LOCATIONS = [(0.5, 0, 0.3), (0.3, 0, 0.3), (0.1, 0, 0.3)]

        # Used to calculate how many frames to animate the assets to maintain the same velocity as the physics assets
        ANIMATION_DURATION = 10

        # Number of frames to capture for each scenario
        NUM_FRAMES = 3

        # Configuration for motion blur examples
        DELTA_TIMES = [None, 1 / 30, 1 / 60, 1 / 240]
        SAMPLES_PER_PIXEL = [32, 128]
        MOTION_BLUR_SUBSAMPLES = [4, 16]

        def setup_stage():
            """Create a new USD stage with animated and physics-enabled assets with synchronized motion."""
            omni.usd.get_context().new_stage()
            settings = carb.settings.get_settings()
            # Set DLSS to Quality mode (2) for best SDG results , options: 0 (Performance), 1 (Balanced), 2 (Quality), 3 (Auto)
            settings.set("rtx/post/dlss/execMode", 2)

            # Capture data only on request
            rep.orchestrator.set_capture_on_play(False)

            stage = omni.usd.get_context().get_stage()
            timeline = omni.timeline.get_timeline_interface()
            timeline.set_end_time(ANIMATION_DURATION)

            # Create lights
            rep.functional.create.xform(name="World")
            rep.functional.create.dome_light(intensity=100, parent="/World", name="DomeLight")
            rep.functional.create.distant_light(
                intensity=2500, rotation=(315, 0, 0), parent="/World", name="DistantLight"
            )

            # Setup the physics assets with gravity disabled and the requested velocity
            assets_root_path = get_assets_root_path()
            physics_asset_url = assets_root_path + PHYSICS_ASSET_URL
            for location, velocity in zip(ASSET_X_MIRRORED_LOCATIONS, ASSET_VELOCITIES):
                prim = rep.functional.create.reference(
                    usd_path=physics_asset_url,
                    parent="/World",
                    name=f"physics_asset_{int(abs(velocity))}",
                    position=location,
                )
                physics_rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
                physics_rigid_body_api.GetVelocityAttr().Set((0, 0, -velocity))
                physx_rigid_body_api = PhysxSchema.PhysxRigidBodyAPI(prim)
                physx_rigid_body_api.GetDisableGravityAttr().Set(True)
                physx_rigid_body_api.GetAngularDampingAttr().Set(0.0)
                physx_rigid_body_api.GetLinearDampingAttr().Set(0.0)

            # Setup animated assets maintaining the same velocity as the physics assets
            anim_asset_url = assets_root_path + ANIM_ASSET_URL
            for location, velocity in zip(ASSET_X_MIRRORED_LOCATIONS, ASSET_VELOCITIES):
                start_location = (-location[0], location[1], location[2])
                prim = rep.functional.create.reference(
                    usd_path=anim_asset_url,
                    parent="/World",
                    name=f"anim_asset_{int(abs(velocity))}",
                    position=start_location,
                )
                animation_distance = velocity * ANIMATION_DURATION
                end_location = (start_location[0], start_location[1], start_location[2] - animation_distance)
                end_keyframe_time = timeline.get_time_codes_per_seconds() * ANIMATION_DURATION
                # Timesampled keyframe (animated) translation
                prim.GetAttribute("xformOp:translate").Set(start_location, time=0)
                prim.GetAttribute("xformOp:translate").Set(end_location, time=end_keyframe_time)

        async def run_motion_blur_example_async(
            num_frames, delta_time=None, use_path_tracing=True, motion_blur_subsamples=8, samples_per_pixel=64
        ):
            """Capture motion blur frames with the given delta time step and render mode."""
            setup_stage()
            stage = omni.usd.get_context().get_stage()
            settings = carb.settings.get_settings()

            # Enable motion blur capture
            settings.set("/omni/replicator/captureMotionBlur", True)

            # Set motion blur settings based on the render mode
            if use_path_tracing:
                print("[MotionBlur] Setting PathTracing render mode motion blur settings")
                settings.set("/rtx/rendermode", "PathTracing")
                # (int): Total number of samples for each rendered pixel, per frame.
                settings.set("/rtx/pathtracing/spp", samples_per_pixel)
                # (int): Maximum number of samples to accumulate per pixel. When this count is reached the rendering stops until a scene or setting change is detected, restarting the rendering process. Set to 0 to remove this limit.
                settings.set("/rtx/pathtracing/totalSpp", samples_per_pixel)
                settings.set("/rtx/pathtracing/optixDenoiser/enabled", 0)
                # Number of sub samples to render if in PathTracing render mode and motion blur is enabled.
                settings.set("/omni/replicator/pathTracedMotionBlurSubSamples", motion_blur_subsamples)
            else:
                print("[MotionBlur] Setting RealTimePathTracing render mode motion blur settings")
                settings.set("/rtx/rendermode", "RealTimePathTracing")
                # 0: Disabled, 1: TAA, 2: FXAA, 3: DLSS, 4:RTXAA
                settings.set("/rtx/post/aa/op", 2)
                # (float): The fraction of the largest screen dimension to use as the maximum motion blur diameter.
                settings.set("/rtx/post/motionblur/maxBlurDiameterFraction", 0.02)
                # (float): Exposure time fraction in frames (1.0 = one frame duration) to sample.
                settings.set("/rtx/post/motionblur/exposureFraction", 1.0)
                # (int): Number of samples to use in the filter. A higher number improves quality at the cost of performance.
                settings.set("/rtx/post/motionblur/numSamples", 8)

            # Setup backend
            mode_str = f"pt_subsamples_{motion_blur_subsamples}_spp_{samples_per_pixel}" if use_path_tracing else "rt"
            delta_time_str = "None" if delta_time is None else f"{delta_time:.4f}"
            output_directory = os.path.join(motion_blur_root, f"dt_{delta_time_str}_{mode_str}")
            os.makedirs(output_directory)
            print(f"[MotionBlur] Output directory: {output_directory}")
            backend = rep.backends.get("DiskBackend")
            backend.initialize(output_dir=output_directory)

            # Setup writer and render product
            camera = rep.functional.create.camera(
                position=(0, 1.5, 0), look_at=(0, 0, 0), parent="/World", name="MotionBlurCam"
            )
            render_product = rep.create.render_product(camera, (1280, 720))
            writer = rep.WriterRegistry.get("BasicWriter")
            writer.initialize(backend=backend, rgb=True)
            writer.attach(render_product)

            # Run a few updates to make sure all materials are fully loaded for capture
            for _ in range(5):
                await omni.kit.app.get_app().next_update_async()

            # Create or get the physics scene
            rep.functional.physics.create_physics_scene(path="/PhysicsScene")
            physx_scene = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/PhysicsScene"))

            # Check the target physics depending on the delta time and the render mode
            target_physics_fps = stage.GetTimeCodesPerSecond() if delta_time is None else 1 / delta_time
            if use_path_tracing:
                target_physics_fps *= motion_blur_subsamples

            # Check if the physics FPS needs to be increased to match the delta time
            original_physics_fps = physx_scene.GetTimeStepsPerSecondAttr().Get()
            if target_physics_fps > original_physics_fps:
                print(f"[MotionBlur] Changing physics FPS from {original_physics_fps} to {target_physics_fps}")
                physx_scene.GetTimeStepsPerSecondAttr().Set(target_physics_fps)

            # Start the timeline for physics updates in the step function
            timeline = omni.timeline.get_timeline_interface()
            timeline.play()

            # Capture frames
            for i in range(num_frames):
                print(f"[MotionBlur] \tCapturing frame {i}")
                await rep.orchestrator.step_async(delta_time=delta_time)

            # Restore the original physics FPS
            if target_physics_fps > original_physics_fps:
                print(f"[MotionBlur] Restoring physics FPS from {target_physics_fps} to {original_physics_fps}")
                physx_scene.GetTimeStepsPerSecondAttr().Set(original_physics_fps)

            # Switch back to the raytracing render mode
            if use_path_tracing:
                print("[MotionBlur] Restoring render mode to RealTimePathTracing")
                settings.set("/rtx/rendermode", "RealTimePathTracing")

            # Wait until the data is fully written
            await rep.orchestrator.wait_until_complete_async()

            # Cleanup
            writer.detach()
            render_product.destroy()

        async def run_motion_blur_examples_async(num_frames, delta_times, samples_per_pixel, motion_blur_subsamples):
            print(
                f"[MotionBlur] Running with delta_times={delta_times}, samples_per_pixel={samples_per_pixel}, motion_blur_subsamples={motion_blur_subsamples}"
            )

            for delta_time in delta_times:
                # RayTracing examples
                await run_motion_blur_example_async(
                    num_frames=num_frames, delta_time=delta_time, use_path_tracing=False
                )
                # PathTracing examples
                for motion_blur_subsample in motion_blur_subsamples:
                    for samples_per_pixel_value in samples_per_pixel:
                        await run_motion_blur_example_async(
                            num_frames=num_frames,
                            delta_time=delta_time,
                            use_path_tracing=True,
                            motion_blur_subsamples=motion_blur_subsample,
                            samples_per_pixel=samples_per_pixel_value,
                        )

        # asyncio.ensure_future(
        #     run_motion_blur_examples_async(
        #         num_frames=NUM_FRAMES,
        #         delta_times=DELTA_TIMES,
        #         samples_per_pixel=SAMPLES_PER_PIXEL,
        #         motion_blur_subsamples=MOTION_BLUR_SUBSAMPLES,
        #     )
        # )

        # Test scenario with less examples
        test_delta_times = [None, 1 / 240]
        test_samples_per_pixel = [32]
        test_motion_blur_subsamples = [4]
        await run_motion_blur_examples_async(
            num_frames=NUM_FRAMES,
            delta_times=test_delta_times,
            samples_per_pixel=test_samples_per_pixel,
            motion_blur_subsamples=test_motion_blur_subsamples,
        )

        # Validate output directories
        for delta_time in test_delta_times:
            delta_time_str = "None" if delta_time is None else f"{delta_time:.4f}"

            # RayTracing output directory
            out_dir = os.path.join(motion_blur_root, f"dt_{delta_time_str}_rt")
            folder_contents_success = validate_folder_contents(path=out_dir, expected_counts={"png": NUM_FRAMES})
            self.assertTrue(folder_contents_success, f"Output directory contents validation failed for {out_dir}")

            # PathTracing output directories for all combinations of subsamples and samples_per_pixel
            for motion_blur_subsample in test_motion_blur_subsamples:
                for spp in test_samples_per_pixel:
                    mode_str = f"pt_subsamples_{motion_blur_subsample}_spp_{spp}"
                    out_dir = os.path.join(motion_blur_root, f"dt_{delta_time_str}_{mode_str}")
                    folder_contents_success = validate_folder_contents(
                        path=out_dir, expected_counts={"png": NUM_FRAMES}
                    )
                    self.assertTrue(
                        folder_contents_success, f"Output directory contents validation failed for {out_dir}"
                    )
