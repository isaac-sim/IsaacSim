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

"""Demonstrate simulation-event-driven data capture with writers and annotators."""

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import os

import carb.settings
import numpy as np
import omni
import omni.replicator.core as rep
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.simulation_manager import SimulationManager
from omni.replicator.core.functional import write_image, write_json
from pxr import UsdPhysics


def write_sem_data(sem_data, file_path):
    """Save semantic segmentation data as JSON labels and PNG image."""
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
cam = rep.functional.create.camera(position=(5, 5, 5), look_at=(0, 0, 0), parent="/World", name="Camera")
rp = rep.create.render_product(cam, resolution=(512, 512), name="MyRenderProduct")

# Set the output directory for the data
out_dir = os.path.join(os.getcwd(), "_out_sim_event")
writer_dir = os.path.join(out_dir, "writer")
annotator_dir = os.path.join(out_dir, "annotator")

os.makedirs(out_dir, exist_ok=True)
os.makedirs(writer_dir, exist_ok=True)
os.makedirs(annotator_dir, exist_ok=True)

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
            rep.orchestrator.step(rt_subframes=4, delta_time=0.0, pause_timeline=False)
            rgb_path = os.path.join(annotator_dir, f"Cube_{i}_step_{s}_rgb.png")
            write_image(path=rgb_path, data=rgb_annot.get_data())
            sem_path = os.path.join(annotator_dir, f"Cube_{i}_step_{s}_sem")
            write_sem_data(sem_annot.get_data(), sem_path)
            break

# Wait for the data to be written to disk and clean up resources
rep.orchestrator.wait_until_complete()
rgb_annot.detach()
sem_annot.detach()
writer.detach()
rp.destroy()

# <start-simulation-get-data-test>
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
    # 5 cubes, each captured once: BasicWriter writes rgb + colorized sem_seg + json (2 png + 1 json),
    # annotator branch writes rgb + sem_seg pngs + sem_seg json (2 png + 1 json).
    num_captures = 5
    expected = {"png": num_captures * 4, "json": num_captures * 2}
    if not validate_folder_contents(
        path=out_dir,
        recursive=True,
        expected_counts=expected,
        fail_on_empty_files=True,
    ):
        print(f"[SDG][Test][FAIL] Output validation failed for {out_dir}")
        sys.exit(1)
    print(f"[SDG][Test][PASS] Output validation succeeded for {out_dir}")
# <end-simulation-get-data-test>

simulation_app.close()
