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

"""Verify the getting-started Replicator SDG examples and their expected output files."""

import tempfile
from typing import Any

import carb.settings
import omni.kit
import omni.usd
from isaacsim.test.utils.file_validation import validate_folder_contents


class TestSDGGettingStarted(omni.kit.test.AsyncTestCase):
    """Covers BasicWriter, PoseWriter, randomizers, physics capture, and Fabric write examples."""

    async def setUp(self) -> None:
        """Create a clean stage and save settings changed by the getting-started examples."""
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()
        self.original_dlss_exec_mode = carb.settings.get_settings().get("rtx/post/dlss/execMode")
        self.original_write_to_fabric = carb.settings.get_settings().get(
            "/exts/omni.replicator.core/enableWriteToFabric"
        )

    async def tearDown(self) -> None:
        """Close the stage, wait for pending loads, and restore render and Fabric settings."""
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()
        carb.settings.get_settings().set("rtx/post/dlss/execMode", self.original_dlss_exec_mode)
        if self.original_write_to_fabric is not None:
            carb.settings.get_settings().set(
                "/exts/omni.replicator.core/enableWriteToFabric", self.original_write_to_fabric
            )

    async def test_sdg_getting_started_01(self) -> None:
        """Capture a labeled cube with BasicWriter RGB and tight 2D bounding-box annotators."""
        import carb.settings
        import omni.replicator.core as rep
        import omni.usd

        out_dir = tempfile.mkdtemp(prefix="test_sdg_basic_writer_")
        print(f"Output directory: {out_dir}")

        async def run_example_async() -> None:
            """Create the cube scene, step three captures, and flush BasicWriter output."""
            # Create a new stage and disable capture on play
            omni.usd.get_context().new_stage()
            rep.orchestrator.set_capture_on_play(False)

            # Set DLSS to Quality mode (2) for best SDG results (Options: 0 (Performance), 1 (Balanced), 2 (Quality), 3 (Auto)
            carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

            # Setup the stage with a dome light and a cube
            rep.functional.create.xform(name="World")
            rep.functional.create.dome_light(intensity=500, parent="/World", name="DomeLight")
            cube = rep.functional.create.cube(parent="/World", name="Cube")
            rep.functional.modify.semantics(cube, {"class": "my_cube"}, mode="add")

            # Create a render product using the viewport perspective camera
            cam = rep.functional.create.camera(position=(5, 5, 5), look_at=(0, 0, 0), parent="/World", name="Camera")
            rp = rep.create.render_product(cam, (512, 512), name="MyRenderProduct")

            # Write data using the basic writer with the rgb and bounding box annotators
            backend = rep.backends.get("DiskBackend")
            backend.initialize(output_dir=out_dir)
            writer = rep.writers.get("BasicWriter")
            writer.initialize(backend=backend, rgb=True, bounding_box_2d_tight=True)
            writer.attach(rp)

            # Trigger a data capture request (data will be written to disk by the writer)
            for i in range(3):
                print(f"Step {i}")
                await rep.orchestrator.step_async()

            # Wait for the data to be written to disk and clean up resources
            await rep.orchestrator.wait_until_complete_async()
            writer.detach()
            rp.destroy()

        # Run the example
        # asyncio.ensure_future(run_example_async())
        await run_example_async()

        # Validate the output directory contents
        folder_contents_success = validate_folder_contents(
            path=out_dir, expected_counts={"png": 3, "json": 6, "npy": 3}
        )
        self.assertTrue(folder_contents_success, f"Output directory contents validation failed for {out_dir}")

    async def test_sdg_getting_started_02(self) -> None:
        """Capture two camera views with direct RGB annotators, a custom writer, and PoseWriter."""
        import carb.settings
        import omni.replicator.core as rep
        import omni.usd
        from omni.replicator.core import Writer

        out_dir = tempfile.mkdtemp(prefix="test_sdg_pose_writer_")
        print(f"Output directory: {out_dir}")

        # Create a custom writer to access annotator data
        class MyWriter(Writer):
            def __init__(self, camera_params: bool = True, bounding_box_3d: bool = True) -> None:
                # Organize data from render product perspective (legacy, annotator, renderProduct)
                self.data_structure = "renderProduct"
                self.annotators = []
                if camera_params:
                    self.annotators.append(rep.annotators.get("camera_params"))
                if bounding_box_3d:
                    self.annotators.append(rep.annotators.get("bounding_box_3d"))
                self._frame_id = 0

            def write(self, data: dict) -> None:
                print(f"[MyWriter][{self._frame_id}] data:")
                for key, value in data.items():
                    print(f"  {key}: {value}")
                self._frame_id += 1

        # Register the writer
        rep.writers.register_writer(MyWriter)

        async def run_example_async() -> None:
            """Write pose/debug outputs while reading RGB data directly from two render products."""
            # Create a new stage and disable capture on play
            omni.usd.get_context().new_stage()
            rep.orchestrator.set_capture_on_play(False)

            # Set DLSS to Quality mode (2) for best SDG results , options: 0 (Performance), 1 (Balanced), 2 (Quality), 3 (Auto)
            carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

            # Setup stage
            rep.functional.create.xform(name="World")
            rep.functional.create.dome_light(intensity=500, parent="/World", name="DomeLight")
            cube = rep.functional.create.cube(parent="/World", name="Cube")
            rep.functional.modify.semantics(cube, {"class": "my_cube"}, mode="add")

            # Capture from two perspectives, a custom camera and a perspective camera
            top_cam = rep.functional.create.camera(
                position=(0, 0, 5), look_at=(0, 0, 0), parent="/World", name="TopCamera"
            )
            persp_cam = rep.functional.create.camera(
                position=(5, 5, 5), look_at=(0, 0, 0), parent="/World", name="PerspCamera"
            )

            # Create the render products
            rp_top = rep.create.render_product(top_cam.GetPath(), (400, 400), name="top_view")
            rp_persp = rep.create.render_product(persp_cam.GetPath(), (512, 512), name="persp_view")

            # Use the annotators to access the data directly, each annotator is attached to a render product
            rgb_annotator_top = rep.annotators.get("rgb")
            rgb_annotator_top.attach(rp_top)
            rgb_annotator_persp = rep.annotators.get("rgb")
            rgb_annotator_persp.attach(rp_persp)

            # Use the custom writer to access the annotator data
            custom_writer = rep.writers.get("MyWriter")
            custom_writer.initialize(camera_params=True, bounding_box_3d=True)
            custom_writer.attach([rp_top, rp_persp])

            # Use the pose writer to write the data to disk
            pose_writer = rep.WriterRegistry.get("PoseWriter")
            pose_writer.initialize(output_dir=out_dir, write_debug_images=True)
            pose_writer.attach([rp_top, rp_persp])

            # Trigger a data capture request (data will be written to disk by the writer)
            for i in range(3):
                print(f"Step {i}")
                await rep.orchestrator.step_async()

                # Get the data from the annotators
                rgb_data_cam = rgb_annotator_top.get_data()
                rgb_data_persp = rgb_annotator_persp.get_data()
                print(f"[Annotator][Cam][{i}] rgb_data_cam shape: {rgb_data_cam.shape}")
                print(f"[Annotator][Persp][{i}] rgb_data_persp shape: {rgb_data_persp.shape}")

            # Wait for the data to be written to disk and clean up resources
            await rep.orchestrator.wait_until_complete_async()
            pose_writer.detach()
            custom_writer.detach()
            rgb_annotator_top.detach()
            rgb_annotator_persp.detach()
            rp_top.destroy()
            rp_persp.destroy()

        # Run the example
        # asyncio.ensure_future(run_example_async())
        await run_example_async()

        # Validate the output directory contents
        folder_contents_success = validate_folder_contents(path=out_dir, expected_counts={"png": 12, "json": 6})
        self.assertTrue(folder_contents_success, f"Output directory contents validation failed for {out_dir}")

    async def test_sdg_getting_started_03(self) -> None:
        """Mix a Python position randomizer with a custom-event light randomizer and validate output."""
        import random

        import carb.settings
        import omni.replicator.core as rep
        import omni.usd

        out_dir = tempfile.mkdtemp(prefix="test_sdg_basic_writer_rand_")
        print(f"Output directory: {out_dir}")

        # Randomize the location of a prim without the graph-based randomizer
        def randomize_location(prim: Any) -> None:
            random_pos = (random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
            rep.functional.modify.position(prim, random_pos)

        async def run_example_async() -> None:
            """Randomize cube pose each frame, trigger light changes on selected frames, and capture."""
            # Create a new stage and disable capture on play
            omni.usd.get_context().new_stage()
            rep.orchestrator.set_capture_on_play(False)
            random.seed(42)
            rep.set_global_seed(42)

            # Set DLSS to Quality mode (2) for best SDG results , options: 0 (Performance), 1 (Balanced), 2 (Quality), 3 (Auto)
            carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

            # Setup stage
            rep.functional.create.xform(name="World")
            cube = rep.functional.create.cube(parent="/World", name="Cube")
            rep.functional.modify.semantics(cube, {"class": "my_cube"}, mode="add")

            # Create a replicator randomizer with custom event trigger
            with rep.trigger.on_custom_event(event_name="randomize_dome_light_color"):
                rep.create.light(light_type="Dome", color=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))

            # Create a render product using the viewport perspective camera
            cam = rep.functional.create.camera(position=(5, 5, 5), look_at=(0, 0, 0), parent="/World", name="Camera")
            rp = rep.create.render_product(cam, (512, 512))

            # Write data using the basic writer with the rgb and bounding box annotators
            backend = rep.backends.get("DiskBackend")
            backend.initialize(output_dir=out_dir)
            writer = rep.writers.get("BasicWriter")
            writer.initialize(
                backend=backend, rgb=True, semantic_segmentation=True, colorize_semantic_segmentation=True
            )
            writer.attach(rp)

            # Trigger a data capture request (data will be written to disk by the writer)
            for i in range(3):
                print(f"Step {i}")
                # Trigger the custom graph-based event randomizer every second step
                if i % 2 == 1:
                    rep.utils.send_og_event(event_name="randomize_dome_light_color")

                # Run the custom USD API location randomizer on the prims
                randomize_location(cube)

                # Since the replicator randomizer is set to trigger on custom events, step will only trigger the writer
                await rep.orchestrator.step_async(rt_subframes=32)

            # Wait for the data to be written to disk and clean up resources
            await rep.orchestrator.wait_until_complete_async()
            writer.detach()
            rp.destroy()

        # Run the example
        # asyncio.ensure_future(run_example_async())
        await run_example_async()

        # Validate the output directory contents
        folder_contents_success = validate_folder_contents(path=out_dir, expected_counts={"png": 6, "json": 3})
        self.assertTrue(folder_contents_success, f"Output directory contents validation failed for {out_dir}")

    async def test_sdg_getting_started_04(self) -> None:
        """Capture a falling rigid cube at distance intervals, including hidden-cube segmentation frames."""
        import carb.settings
        import omni.kit.app
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from isaacsim.core.experimental.prims import RigidPrim
        from pxr import UsdGeom

        out_dir = tempfile.mkdtemp(prefix="test_sdg_basic_writer_sim_")
        print(f"Output directory: {out_dir}")

        async def run_example_async() -> None:
            """Run physics, capture at drop-height thresholds, and resume the timeline between captures."""
            # Create a new stage and disable capture on play
            omni.usd.get_context().new_stage()
            rep.orchestrator.set_capture_on_play(False)

            # Set DLSS to Quality mode (2) for best SDG results , options: 0 (Performance), 1 (Balanced), 2 (Quality), 3 (Auto)
            carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

            # Add a light
            rep.functional.create.xform(name="World")
            rep.functional.create.dome_light(intensity=500, parent="/World", name="DomeLight")

            # Create a cube with colliders and rigid body dynamics at a specific location
            cube = rep.functional.create.cube(name="Cube", parent="/World")
            rep.functional.modify.position(cube, (0, 0, 2))
            rep.functional.modify.semantics(cube, {"class": "my_cube"}, mode="add")
            rep.functional.physics.apply_rigid_body(cube, with_collider=True)

            # Createa a sphere with colliders and rigid body dynamics next to the cube
            sphere = rep.functional.create.sphere(name="Sphere", parent="/World")
            rep.functional.modify.position(sphere, (-1, -1, 2))
            rep.functional.modify.semantics(sphere, {"class": "my_sphere"}, mode="add")
            rep.functional.physics.apply_rigid_body(sphere, with_collider=True)

            # Create a render product using the viewport perspective camera
            cam = rep.functional.create.camera(position=(5, 5, 5), look_at=(0, 0, 0), parent="/World", name="Camera")
            rp = rep.create.render_product(cam, (512, 512))

            # Write data using the basic writer with the rgb and bounding box annotators
            backend = rep.backends.get("DiskBackend")
            backend.initialize(output_dir=out_dir)
            writer = rep.writers.get("BasicWriter")
            writer.initialize(
                backend=backend, rgb=True, semantic_segmentation=True, colorize_semantic_segmentation=True
            )
            writer.attach(rp)

            # Start the timeline (will only advance with app update)
            timeline = omni.timeline.get_timeline_interface()
            timeline.play()

            # Wrap the cube with as a RigidPrim for easy access to its world poses and velocities
            cube_rigid = RigidPrim(str(cube.GetPrimPath()))

            # Wrap the cube as an Imageable object to toggle visibility during capture
            cube_imageable = UsdGeom.Imageable(cube)

            # Define the capture interval in meters
            capture_interval_meters = 0.5
            cube_pos = cube_rigid.get_world_poses(indices=[0])[0].numpy()
            previous_capture_height = cube_pos[0, 2]

            # Update the app which will advance the timeline (and implicitly the simulation)
            for i in range(100):
                await omni.kit.app.get_app().next_update_async()
                cube_pos = cube_rigid.get_world_poses(indices=[0])[0].numpy()
                current_height = cube_pos[0, 2]
                distance_dropped = previous_capture_height - current_height
                print(f"Step {i}; cube height: {current_height:.3f}; drop since last capture: {distance_dropped:.3f}")

                # Stop the simulation if the cube falls below the ground
                if current_height < 0:
                    print(f"\t Cube fell below the ground at height {current_height:.3f}, stopping simulation..")
                    break

                # Capture every time the cube drops by the threshold distance
                if distance_dropped >= capture_interval_meters:
                    print(f"\t Capturing at height {current_height:.3f}")
                    previous_capture_height = current_height

                    # Setting delta_time to 0.0 will make sure the timeline is not advanced during capture
                    await rep.orchestrator.step_async(delta_time=0.0)

                    # Capture again with the cube hidden
                    print("\t Capturing with cube hidden")
                    cube_imageable.MakeInvisible()
                    await rep.orchestrator.step_async(delta_time=0.0)
                    cube_imageable.MakeVisible()

                    # Resume the timeline to continue the simulation
                    timeline.play()

            # Pause the simulation
            timeline.pause()

            # Wait for the data to be written to disk and clean up resources
            await rep.orchestrator.wait_until_complete_async()
            writer.detach()
            rp.destroy()

        # Run the example
        # asyncio.ensure_future(run_example_async())
        await run_example_async()

        # Validate the output directory contents
        folder_contents_success = validate_folder_contents(path=out_dir, expected_counts={"png": 12, "json": 6})
        self.assertTrue(folder_contents_success, f"Output directory contents validation failed for {out_dir}")

    async def test_sdg_getting_started_05(self) -> None:
        """Compare capture output for wait-for-render and Fabric-write randomization configurations."""
        import os
        import time

        import carb.settings
        import omni.replicator.core as rep
        import omni.usd

        NUM_CUBES = 100
        NUM_CAPTURES = 10
        test_root = tempfile.mkdtemp(prefix="test_sdg_fabric_")
        print(f"Test output root: {test_root}")

        async def run_example_async(wait_for_render: Any, write_to_fabric: Any) -> None:
            print(f"\n[SDG] Running with wait_for_render={wait_for_render}, write_to_fabric={write_to_fabric}")
            omni.usd.get_context().new_stage()
            rep.orchestrator.set_capture_on_play(False)

            settings = carb.settings.get_settings()
            settings.set("rtx/post/dlss/execMode", 2)
            settings.set("/exts/omni.replicator.core/enableWriteToFabric", write_to_fabric)

            rng = rep.rng.ReplicatorRNG(seed=42)

            # Setup stage with a dome light and batch-created cubes
            rep.functional.create.xform(name="World")
            rep.functional.create.dome_light(intensity=500, parent="/World", name="DomeLight")
            cubes = rep.functional.create_batch.cube(
                count=NUM_CUBES,
                parent="/World",
                name="Cube",
                semantics={"class": "my_cube"},
            )
            rep.functional.modify.scale(cubes, (0.2, 0.2, 0.2))

            # Create the camera and render product
            cam = rep.functional.create.camera(position=(5, 5, 5), look_at=(0, 0, 0), parent="/World", name="Camera")
            rp = rep.create.render_product(cam, (512, 512))

            # Write data using BasicWriter with rgb annotator
            backend = rep.backends.get("DiskBackend")
            out_dir = os.path.join(test_root, f"fabric_{write_to_fabric}_wait_{wait_for_render}")
            os.makedirs(out_dir)
            backend.initialize(output_dir=out_dir)
            print(f"[SDG] Output directory: {out_dir}")
            writer = rep.writers.get("BasicWriter")
            writer.initialize(backend=backend, rgb=True)
            writer.attach(rp)

            # Randomize and capture, measuring timing for each phase
            randomization_times_ms = []
            capture_times_ms = []
            total_start = time.perf_counter()

            for i in range(NUM_CAPTURES):
                random_positions = rng.generator.uniform((-3.0, -3.0, -3.0), (3.0, 3.0, 3.0), size=(NUM_CUBES, 3))
                random_rotations = rng.generator.uniform((0.0, 0.0, 0.0), (360.0, 360.0, 360.0), size=(NUM_CUBES, 3))
                random_scales = rng.generator.uniform(0.1, 0.4, size=(NUM_CUBES, 3))

                rand_start = time.perf_counter()
                rep.functional.modify.pose(
                    cubes,
                    position_value=random_positions,
                    rotation_value=random_rotations,
                    scale_value=random_scales,
                )
                rep.functional.randomizer.display_color(cubes, rng=rng)
                rand_ms = (time.perf_counter() - rand_start) * 1000.0
                randomization_times_ms.append(rand_ms)

                cap_start = time.perf_counter()
                await rep.orchestrator.step_async(wait_for_render=wait_for_render)
                cap_ms = (time.perf_counter() - cap_start) * 1000.0
                capture_times_ms.append(cap_ms)

                print(f"[SDG] Step {i}: randomization {rand_ms:.1f} ms, capture {cap_ms:.1f} ms")

            # Wait for all data to be written to disk
            print("[SDG] Waiting for all data to be written to disk..")
            await rep.orchestrator.wait_until_complete_async()
            total_ms = (time.perf_counter() - total_start) * 1000.0

            avg_rand = sum(randomization_times_ms) / len(randomization_times_ms)
            avg_cap = sum(capture_times_ms) / len(capture_times_ms)
            print(
                f"[SDG] Avg randomization: {avg_rand:.1f} ms, avg capture: {avg_cap:.1f} ms, total: {total_ms:.1f} ms"
            )

            writer.detach()
            rp.destroy()

        async def run_examples_async() -> None:
            # Run with different configurations to compare performance
            await run_example_async(wait_for_render=True, write_to_fabric=False)
            await run_example_async(wait_for_render=False, write_to_fabric=False)
            await run_example_async(wait_for_render=False, write_to_fabric=True)

        # asyncio.ensure_future(run_examples_async())

        # Test the examples
        await run_examples_async()
        out_dir_1 = os.path.join(test_root, "fabric_False_wait_True")
        out_dir_2 = os.path.join(test_root, "fabric_False_wait_False")
        out_dir_3 = os.path.join(test_root, "fabric_True_wait_False")

        # Validate the output directory contents
        for out_dir in [out_dir_1, out_dir_2, out_dir_3]:
            folder_contents_success = validate_folder_contents(path=out_dir, expected_counts={"png": NUM_CAPTURES})
            self.assertTrue(folder_contents_success, f"Output directory contents validation failed for {out_dir}")
