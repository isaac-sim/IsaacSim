# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os

import carb.settings
import omni.kit.app
import omni.replicator.core as rep
import omni.timeline
import omni.usd

# NOTE: To avoid FPS delta misses make sure the sensor framerate is divisible by the timeline framerate
STAGE_FPS = 100.0
SENSOR_FPS = 10.0
SENSOR_DT = 1.0 / SENSOR_FPS


def run_custom_fps_example(duration_seconds):
    # Create a new stage
    omni.usd.get_context().new_stage()

    # Disable capture on play (data will only be accessed at custom times)
    carb.settings.get_settings().set("/omni/replicator/captureOnPlay", False)

    # Make sure fixed time stepping is set (the timeline will be advanced with the same delta time)
    carb.settings.get_settings().set("/app/player/useFixedTimeStepping", True)

    # Set the timeline parameters
    timeline = omni.timeline.get_timeline_interface()
    timeline.set_looping(False)
    timeline.set_current_time(0.0)
    timeline.set_end_time(10)
    timeline.set_time_codes_per_second(STAGE_FPS)
    timeline.play()
    timeline.commit()

    # Create a light and a semantically annoated cube
    rep.create.light()
    rep.create.cube(semantics=[("class", "cube")])

    # Create a render product and disable it (it will re-enabled when data is needed)
    rp = rep.create.render_product("/OmniverseKit_Persp", (512, 512), name="rp")
    rp.hydra_texture.set_updates_enabled(False)

    # Create a writer and an annotator as different ways to access the data
    out_dir_rgb = os.path.join(os.getcwd(), "_out_writer_fps_rgb")
    print(f"Writer data will be written to: {out_dir_rgb}")
    writer_rgb = rep.WriterRegistry.get("BasicWriter")
    writer_rgb.initialize(output_dir=out_dir_rgb, rgb=True)
    writer_rgb.attach(rp)
    annot_depth = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
    annot_depth.attach(rp)

    # Run the simulation for the given number of frames and access the data at the desired framerates
    written_frames = 0
    previous_time = timeline.get_current_time()
    elapsed_time = 0.0
    loop_iteration_count = 0
    while timeline.get_current_time() < duration_seconds:
        current_time = timeline.get_current_time()
        delta_time = current_time - previous_time
        elapsed_time += delta_time
        print(
            f"[{loop_iteration_count}] current_time={current_time:.4f}; delta_time={delta_time:.4f}; elapsed_time={elapsed_time:.4f}/{SENSOR_DT:.4f};"
        )

        # Check if enough time has passed to trigger the sensor
        if elapsed_time >= SENSOR_DT:
            # Reset the elapsed time with the difference to the optimal trigger time (when the timeline fps is not divisible by the sensor framerate)
            elapsed_time = elapsed_time - SENSOR_DT

            # Enable render products for data access
            rp.hydra_texture.set_updates_enabled(True)

            # Step needs to be called after scheduling the write
            rep.orchestrator.step(delta_time=0.0, pause_timeline=False)

            # After step, the annotator data is available and in sync with the stage
            annot_data = annot_depth.get_data()

            # Count the number of frames written
            print(f"\t Writing frame {written_frames}; annotator data shape={annot_data.shape};")
            written_frames += 1

            # Disable render products to avoid unnecessary rendering
            rp.hydra_texture.set_updates_enabled(False)

        previous_time = current_time
        # Advance the app (timeline) by one frame
        simulation_app.update()
        loop_iteration_count += 1

    # Make sure the writer finishes writing the data to disk
    rep.orchestrator.wait_until_complete()


# Run the example.
# NOTE: The simulation duration is calculated to ensure 'target_num_writes' are captured.
# It includes the time for all sensor intervals plus a small buffer of a few stage frames.
target_num_writes = 6
duration_for_target_writes = (target_num_writes * SENSOR_DT) + (5.0 / STAGE_FPS)
run_custom_fps_example(duration_seconds=duration_for_target_writes)

# Close the application
simulation_app.close()
