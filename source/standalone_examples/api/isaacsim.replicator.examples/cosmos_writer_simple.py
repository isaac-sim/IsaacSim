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

"""Demonstrate synthetic data generation using the CosmosWriter."""

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import os

import carb.settings
import omni.replicator.core as rep
import omni.timeline
import omni.usd

SEGMENTATION_MAPPING = {
    "plane": [0, 0, 255, 255],
    "cube": [255, 0, 0, 255],
    "sphere": [0, 255, 0, 255],
}
NUM_FRAMES = 60


def run_cosmos_example(num_frames, segmentation_mapping=None):
    """Run a CosmosWriter example capturing physics simulation frames."""
    # Create a new stage
    omni.usd.get_context().new_stage()

    # Set DLSS to Quality mode (2) for best SDG results , options: 0 (Performance), 1 (Balanced), 2 (Quality), 3 (Auto)
    carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

    # CosmosWriter requires script nodes to be enabled
    carb.settings.get_settings().set_bool("/app/omni.graph.scriptnode/opt_in", True)

    # Disable capture on play, data is captured manually using the step function
    rep.orchestrator.set_capture_on_play(False)

    # Set the stage properties
    rep.settings.set_stage_up_axis("Z")
    rep.settings.set_stage_meters_per_unit(1.0)
    rep.functional.create.dome_light(intensity=500)

    # Create the scenario with a ground plane and a falling sphere and cube.
    plane = rep.functional.create.plane(position=(0, 0, 0), scale=(10, 10, 1), semantics={"class": "plane"})
    rep.functional.physics.apply_collider(plane)

    sphere = rep.functional.create.sphere(position=(0, 0, 3), semantics={"class": "sphere"})
    rep.functional.physics.apply_collider(sphere)
    rep.functional.physics.apply_rigid_body(sphere)

    cube = rep.functional.create.cube(position=(1, 1, 2), scale=0.5, semantics={"class": "cube"})
    rep.functional.physics.apply_collider(cube)
    rep.functional.physics.apply_rigid_body(cube)

    # Set up the writer
    camera = rep.functional.create.camera(position=(5, 5, 3), look_at=(0, 0, 0))
    rp = rep.create.render_product(camera, (1280, 720))
    out_dir = os.path.join(os.getcwd(), "_out_cosmos_simple")
    print(f"Output directory: {out_dir}")
    backend = rep.backends.get("DiskBackend")
    backend.initialize(output_dir=out_dir)
    cosmos_writer = rep.WriterRegistry.get("CosmosWriter")
    cosmos_writer.initialize(backend=backend, segmentation_mapping=segmentation_mapping)
    cosmos_writer.attach(rp)

    # Start the simulation
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    # Capture a frame every app update
    for i in range(num_frames):
        print(f"Frame {i+1}/{num_frames}")
        simulation_app.update()
        rep.orchestrator.step(delta_time=0.0, pause_timeline=False)
    timeline.pause()

    # Wait for all data to be written
    rep.orchestrator.wait_until_complete()
    print("Data generation complete!")
    cosmos_writer.detach()
    rp.destroy()


run_cosmos_example(num_frames=NUM_FRAMES, segmentation_mapping=SEGMENTATION_MAPPING)

# <start-cosmos-writer-simple-test>
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
    # CosmosWriter writes one PNG per frame and one MP4 video per modality
    # (rgb, shaded_seg, segmentation, depth, edges) under clip_0000/.
    out_dir = os.path.join(os.getcwd(), "_out_cosmos_simple")
    num_modalities = 5
    ok = validate_folder_contents(
        path=out_dir,
        recursive=True,
        expected_counts={"png": NUM_FRAMES * num_modalities, "mp4": num_modalities},
        fail_on_empty_files=True,
    )
    if not ok:
        print(f"[SDG][Test][FAIL] Output validation failed for {out_dir}")
        sys.exit(1)
    print(f"[SDG][Test][PASS] Output validation succeeded for {out_dir}")
# <end-cosmos-writer-simple-test>

simulation_app.close()
