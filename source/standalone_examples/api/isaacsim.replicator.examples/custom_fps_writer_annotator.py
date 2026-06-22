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

"""Demonstrate custom FPS data capture using writers and annotators."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os

import carb.settings
import omni.kit.app
import omni.replicator.core as rep
import omni.timeline
import omni.usd

# Configuration
NUM_CAPTURES = 6
VERBOSE = True

# NOTE: To avoid FPS delta misses make sure the sensor framerate is divisible by the timeline framerate
STAGE_FPS = 100.0
SENSOR_FPS = 10.0
SENSOR_DT = 1.0 / SENSOR_FPS


def run_custom_fps_example(duration_seconds: float) -> None:
    """Run a simulation capturing data at a custom sensor framerate."""
    # Create a new stage
    omni.usd.get_context().new_stage()

    # Disable capture on play to capture data manually using step
    rep.orchestrator.set_capture_on_play(False)

    # Set DLSS to Quality mode (2) for best SDG results , options: 0 (Performance), 1 (Balanced), 2 (Quality), 3 (Auto)
    carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

    # Enable fixed time stepping: the timeline will advance by `1 / timeCodesPerSecond`
    # per accepted tick, ignoring the run loop's measured wall-clock dt.
    carb.settings.get_settings().set("/app/player/useFixedTimeStepping", True)

    # Create scene with a semantically annotated cube with physics
    rep.functional.create.xform(name="World")
    rep.functional.create.dome_light(intensity=250, parent="/World", name="DomeLight")
    cube = rep.functional.create.cube(position=(0, 0, 2), parent="/World", name="Cube", semantics={"class": "cube"})
    rep.functional.physics.apply_collider(cube)
    rep.functional.physics.apply_rigid_body(cube)

    # Create render product (disabled until data capture is needed)
    cam = rep.functional.create.camera(position=(5, 5, 5), look_at=(0, 0, 0), parent="/World", name="Camera")
    rp = rep.create.render_product(cam, resolution=(512, 512), name="rp")
    rp.hydra_texture.set_updates_enabled(False)

    # Create the backend for the writer
    out_dir_rgb = os.path.join(os.getcwd(), "_out_writer_fps_rgb")
    print(f"Writer data will be written to: {out_dir_rgb}")
    backend = rep.backends.get("DiskBackend")
    backend.initialize(output_dir=out_dir_rgb)

    # Create a writer and an annotator as examples of different ways of accessing data
    writer_rgb = rep.WriterRegistry.get("BasicWriter")
    writer_rgb.initialize(backend=backend, rgb=True)
    writer_rgb.attach(rp)

    # Create an annotator to access the data directly
    annot_depth = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
    annot_depth.attach(rp)

    # Run the simulation for the given number of frames and access the data at the desired framerates
    print(
        f"Starting simulation: {duration_seconds:.2f}s duration, {SENSOR_FPS:.0f} FPS sensor, {STAGE_FPS:.0f} FPS timeline"
    )

    # Set the timeline parameters
    timeline = omni.timeline.get_timeline_interface()
    timeline.set_looping(False)
    timeline.set_current_time(0.0)
    timeline.set_end_time(10)
    timeline.set_time_codes_per_second(STAGE_FPS)
    timeline.play()
    timeline.commit()

    # Run the simulation for the given number of frames and access the data at the desired framerates
    frame_count = 0
    previous_time = timeline.get_current_time()
    elapsed_time = 0.0
    iteration = 0

    while timeline.get_current_time() < duration_seconds:
        current_time = timeline.get_current_time()
        delta_time = current_time - previous_time
        elapsed_time += delta_time

        # Simulation progress
        if VERBOSE:
            print(f"Step {iteration}: timeline time={current_time:.3f}s, elapsed time={elapsed_time:.3f}s")

        # Trigger sensor at desired framerate (use small epsilon for floating point comparison)
        if elapsed_time >= SENSOR_DT - 1e-9:
            elapsed_time -= SENSOR_DT  # Reset with remainder to maintain accuracy

            rp.hydra_texture.set_updates_enabled(True)
            rep.orchestrator.step(delta_time=0.0, pause_timeline=False, rt_subframes=16)
            annot_data = annot_depth.get_data()

            print(f"\n  >> Capturing frame {frame_count} at time={current_time:.3f}s | shape={annot_data.shape}\n")
            frame_count += 1

            rp.hydra_texture.set_updates_enabled(False)

        previous_time = current_time
        # Advance the app (timeline) by one frame
        simulation_app.update()
        iteration += 1

    # Wait for writer to finish
    rep.orchestrator.wait_until_complete()

    # Cleanup
    timeline.pause()
    writer_rgb.detach()
    annot_depth.detach()
    rp.destroy()


# Run example with duration for all captures plus a buffer of 5 frames
duration = (NUM_CAPTURES * SENSOR_DT) + (5.0 / STAGE_FPS)
run_custom_fps_example(duration_seconds=duration)

# <start-custom-fps-writer-annotator-test>
import argparse
import sys

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.test.utils")
from isaacsim.test.utils.file_validation import validate_folder_contents

parser = argparse.ArgumentParser()
parser.add_argument(
    "--test",
    action="store_true",
    help="Validate captured output files against expected counts and exit.",
)
args, _ = parser.parse_known_args()

if args.test:
    # BasicWriter rgb-only writes 1 png per sensor capture.
    out_dir = os.path.join(os.getcwd(), "_out_writer_fps_rgb")
    ok = validate_folder_contents(
        path=out_dir,
        recursive=True,
        expected_counts={"png": NUM_CAPTURES},
        fail_on_empty_files=True,
    )
    if not ok:
        print(f"[SDG][Test][FAIL] Output validation failed for {out_dir}")
        sys.exit(1)
    print(f"[SDG][Test][PASS] Output validation succeeded for {out_dir}")
# <end-custom-fps-writer-annotator-test>

simulation_app.close()
