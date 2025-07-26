# SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import carb
import numpy as np
import omni.kit.app
import omni.kit.test
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
import omni.timeline
from isaacsim.core.utils.prims import get_prim_at_path
from pxr import PhysxSchema, UsdPhysics


def print_unique_values_and_counts(values: np.ndarray, tolerance: float = 1e-6) -> list[tuple[float, int]]:
    sorted_indices = np.argsort(values)
    sorted_values = values[sorted_indices]

    value_counts = []
    current_value = sorted_values[0]
    current_count = 1

    for value in sorted_values[1:]:
        if abs(value - current_value) <= tolerance:
            current_count += 1
        else:
            value_counts.append((current_value, current_count))
            current_value = value
            current_count = 1
    value_counts.append((current_value, current_count))

    sorted_counts = sorted(value_counts, key=lambda x: x[0])
    print(f"  Value counts (tolerance={tolerance}):")
    for value, count in sorted_counts:
        print(f"    {value:.6f}: {count}")

    return sorted_counts


class TestAdvanceTimelineAndPhysics(omni.kit.test.AsyncTestCase):

    async def setUp(self):
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()
        self._initial_stage_fps = omni.timeline.get_timeline_interface().get_time_codes_per_second()
        print(f"[setUp] Default stage FPS: {self._initial_stage_fps}")

    async def tearDown(self):
        omni.timeline.get_timeline_interface().set_time_codes_per_second(self._initial_stage_fps)
        await omni.kit.app.get_app().next_update_async()
        print(f"[tearDown] Restored stage FPS: {omni.timeline.get_timeline_interface().get_time_codes_per_second()}")
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def run_timeline_and_get_delta_times(
        self, num_frames: int, stage_fps: float = None, physx_fps: float = None, verbose: bool = False
    ) -> list[float]:
        # Create a new stage
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        # Set a custom physics FPS if provided
        if physx_fps is not None:
            stage = omni.usd.get_context().get_stage()
            physx_scene = None
            for prim in stage.Traverse():
                if prim.IsA(UsdPhysics.Scene):
                    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(prim)
                    break
            if physx_scene is None:
                physics_scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
                physx_scene = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
            physx_scene.GetTimeStepsPerSecondAttr().Set(physx_fps)
            print(
                f"  Physics FPS: physx_scene.GetTimeStepsPerSecondAttr(): {physx_scene.GetTimeStepsPerSecondAttr().Get()}"
            )

        # Set a custom stage FPS if provided
        timeline = omni.timeline.get_timeline_interface()
        if stage_fps:
            timeline.set_time_codes_per_second(stage_fps)
            print(f"  Stage FPS: timeline.get_time_codes_per_second(): {timeline.get_time_codes_per_second()}")

        # Play the timeline
        if verbose:
            print(f"  Starting timeline at time: {timeline.get_current_time()} and running for {num_frames} frames:")
        timeline.play()

        # Get the delta times between frames
        delta_times = []
        prev_time = timeline.get_current_time()
        for i in range(num_frames):
            await omni.kit.app.get_app().next_update_async()
            delta_time = timeline.get_current_time() - prev_time
            if verbose:
                print(f"    {i}: {delta_time:.6f}")
            delta_times.append(delta_time)
            prev_time = timeline.get_current_time()
        return delta_times

    async def test_timeline_delta_times_with_default_fps(self):
        num_frames = 11
        stage_fps = None
        physx_fps = None
        print(f"*** Running tests with:\n  num_frames={num_frames}\n  stage_fps={stage_fps}\n  physx_fps={physx_fps}")
        delta_times = await self.run_timeline_and_get_delta_times(
            num_frames, stage_fps=stage_fps, physx_fps=physx_fps, verbose=True
        )

        # Check if all delta times are equal, skipping the first value as it is always different from the others
        delta_times_without_first = np.array(delta_times[1:])
        print_unique_values_and_counts(delta_times_without_first)
        all_equal = np.all(np.abs(delta_times_without_first - delta_times_without_first[0]) < 1e-6)
        self.assertTrue(all_equal, f"All delta times should be equal, stage_fps={stage_fps}, physx_fps={physx_fps}")

    async def test_timeline_delta_times_with_matching_fps(self):
        num_frames = 11
        stage_fps = 100
        physx_fps = 100
        print(f"*** Running tests with:\n  num_frames={num_frames}\n  stage_fps={stage_fps}\n  physx_fps={physx_fps}")
        delta_times = await self.run_timeline_and_get_delta_times(
            num_frames, stage_fps=stage_fps, physx_fps=physx_fps, verbose=True
        )

        # Check if all delta times are equal, skipping the first value as it is always different from the others
        delta_times_without_first = np.array(delta_times[1:])
        print_unique_values_and_counts(delta_times_without_first)
        all_equal = np.all(np.abs(delta_times_without_first - delta_times_without_first[0]) < 1e-6)
        self.assertTrue(all_equal, f"All delta times should be equal, stage_fps={stage_fps}, physx_fps={physx_fps}")

    async def test_timeline_delta_times_with_stage_fps_higher_than_physics(self):
        num_frames = 11
        stage_fps = 100
        physx_fps = 75
        print(f"*** Running tests with:\n  num_frames={num_frames}\n  stage_fps={stage_fps}\n  physx_fps={physx_fps}")
        delta_times = await self.run_timeline_and_get_delta_times(
            num_frames, stage_fps=stage_fps, physx_fps=physx_fps, verbose=True
        )

        # Check if all delta times are equal, skipping the first value as it is always different from the others
        delta_times_without_first = np.array(delta_times[1:])
        print_unique_values_and_counts(delta_times_without_first)
        all_equal = np.all(np.abs(delta_times_without_first - delta_times_without_first[0]) < 1e-6)
        self.assertTrue(all_equal, f"All delta times should be equal, stage_fps={stage_fps}, physx_fps={physx_fps}")

    async def test_timeline_delta_times_with_stage_fps_lower_than_physics(self):
        num_frames = 11
        stage_fps = 75
        physx_fps = 100
        print(f"*** Running tests with:\n  num_frames={num_frames}\n  stage_fps={stage_fps}\n  physx_fps={physx_fps}")
        delta_times = await self.run_timeline_and_get_delta_times(
            num_frames, stage_fps=stage_fps, physx_fps=physx_fps, verbose=True
        )

        # Check if all delta times are equal, skipping the first value as it is always different from the others
        delta_times_without_first = np.array(delta_times[1:])
        print_unique_values_and_counts(delta_times_without_first)
        all_equal = np.all(np.abs(delta_times_without_first - delta_times_without_first[0]) < 1e-6)
        self.assertTrue(all_equal, f"All delta times should be equal, stage_fps={stage_fps}, physx_fps={physx_fps}")

    async def step_simulation(self, step_method: str = "timeline_forward"):
        """Step the simulation using the specified method.

        Args:
            step_method: The method to use for stepping the simulation.
                Options are:
                - "rep_orchestrator": Use rep.orchestrator.step_async
                - "timeline_api": Use timeline API methods
                - "timeline_forward": Use timeline.play() and timeline.pause()
        """
        timeline = omni.timeline.get_timeline_interface()
        time_before = timeline.get_current_time()
        frame_before = time_before * timeline.get_time_codes_per_seconds()
        print(f"step_method: {step_method}")
        print(f"Before - Time: {time_before:.4f}, Frame: {frame_before:.2f}")
        is_timeline_auto_updating = timeline.is_auto_updating()

        if step_method == "rep_orchestrator":
            await rep.orchestrator.step_async(delta_time=0, pause_timeline=True)
            await rep.orchestrator.step_async(delta_time=None, pause_timeline=True)
        elif step_method == "timeline_two_steps":
            timeline.play()
            timeline.commit()
            await omni.kit.app.get_app().next_update_async()
            timeline.set_auto_update(False)
            timeline.commit_silently()
            await omni.kit.app.get_app().next_update_async()
            timeline.set_auto_update(True)
            timeline.commit_silently()
            timeline.pause()
            timeline.commit()
        elif step_method == "timeline_one_step":
            timeline.play()
            timeline.commit()
            await omni.kit.app.get_app().next_update_async()
            timeline.pause()
            timeline.commit()
        else:
            raise ValueError(
                f"Unknown step_method: {step_method}. Valid options are: 'rep_orchestrator', 'timeline_api', 'timeline_forward'"
            )

        time_after = timeline.get_current_time()
        frame_after = time_after * timeline.get_time_codes_per_seconds()
        print(f"After Pause - Time: {time_after:.4f}, Frame: {frame_after:.2f}")
        self.assertAlmostEqual(frame_after, 1.0, 3, msg=f"Frame after should be 1.0, got {frame_after}")
        self.assertAlmostEqual(time_after, 0.0167, 3, msg=f"Time after should be 0.0167, got {time_after}")
        self.assertFalse(timeline.is_playing())
        # Run some number of frames
        for i in range(3):
            await omni.kit.app.get_app().next_update_async()

        time_after = timeline.get_current_time()
        frame_after = time_after * timeline.get_time_codes_per_seconds()
        print(f"After app update- Time: {time_after:.4f}, Frame: {frame_after:.2f}")
        self.assertAlmostEqual(frame_after, 1.0, 3, msg=f"Frame after should be 1.0, got {frame_after}")
        self.assertAlmostEqual(time_after, 0.0167, 3, msg=f"Time after should be 0.0167, got {time_after}")

    async def step_simulation_method(self, step_method: str, no_replicator: bool = False):
        # Wait for a few frames to ensure all extensions are loaded.
        for i in range(5):
            await omni.kit.app.get_app().next_update_async()

        camera_prim_path = "/OmniverseKit_Persp"
        camera_prim = get_prim_at_path(camera_prim_path)
        if not camera_prim.IsValid():
            carb.log_error(f"Error: Camera prim not found at {camera_prim_path}")
            return

        carb.log_info(f"Found camera prim at {camera_prim_path}")

        render_product = rep.create.render_product(camera_prim.GetPath().pathString, (1024, 1024))
        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
        writer_name = f"{rv}ROS2PublishImage"

        writer = rep.writers.get(writer_name)
        if not no_replicator:
            writer.attach([render_product])
        await omni.kit.app.get_app().next_update_async()
        await self.step_simulation(step_method=step_method)

    async def test_rep_orchestrator(self):
        await self.step_simulation_method(step_method="rep_orchestrator")

    async def test_timeline_two_steps(self):
        await self.step_simulation_method(step_method="timeline_two_steps")

    async def test_timeline_one_step(self):
        await self.step_simulation_method(step_method="timeline_one_step")

    async def test_timeline_one_step_no_replicator(self):
        await self.step_simulation_method(step_method="timeline_one_step", no_replicator=True)
